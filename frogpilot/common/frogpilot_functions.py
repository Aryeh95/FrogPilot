#!/usr/bin/env python3
import datetime
import filecmp
import json
import os
import shutil
import subprocess
import tarfile
import threading
import time
import zstandard as zstd

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.time import system_time_valid
from openpilot.system.athena.registration import UNREGISTERED_DONGLE_ID, register
from openpilot.system.hardware import HARDWARE, PC

from openpilot.frogpilot.system.model_manager import ModelManager
from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common import frogpilot_paths, frogpilot_utilities, frogpilot_variables

def backup_directory(backup, destination, success_message, fail_message, minimum_backup_size=0, compressed=False):
  in_progress_destination = destination.parent / (destination.name + "_in_progress")
  in_progress_destination.mkdir(parents=True, exist_ok=True)

  if compressed:
    destination_compressed = destination.parent / (destination.name + ".tar.zst")

    if frogpilot_utilities.run_cmd(["sudo", "rsync", "-avq", f"{backup}/.", in_progress_destination], "", fail_message, report=False) is None:
      frogpilot_utilities.delete_file(in_progress_destination, report=False)
      return

    compressed_file = destination.parent / (destination.name + "_in_progress.tar.zst")
    with open(compressed_file, "wb") as f:
      cctx = zstd.ZstdCompressor(level=2)
      with cctx.stream_writer(f) as compressor:
        with tarfile.open(fileobj=compressor, mode="w|") as tar:
          tar.add(in_progress_destination, arcname=destination.name)

    frogpilot_utilities.delete_file(in_progress_destination, report=False)

    try:
      compressed_file.rename(destination_compressed)
    except FileNotFoundError:
      print(fail_message)
      return
    print(f"Backup saved: {destination_compressed}")

    compressed_backup_size = destination_compressed.stat().st_size
    if minimum_backup_size == 0 or compressed_backup_size < minimum_backup_size:
      frogpilot_variables.params.put_int("MinimumBackupSize", compressed_backup_size)
  else:
    if frogpilot_utilities.run_cmd(["sudo", "rsync", "-avq", f"{backup}/.", in_progress_destination], success_message, fail_message, report=False) is None:
      frogpilot_utilities.delete_file(in_progress_destination, report=False)
      return

    if destination.exists():
      frogpilot_utilities.delete_file(destination, report=False)
    in_progress_destination.rename(destination)

def cleanup_screen_recordings(limit_bytes):
  recordings = sorted(frogpilot_paths.SCREEN_RECORDINGS_PATH.glob("*.mp4"), key=lambda recording: recording.stat().st_mtime, reverse=True)

  total = 0
  for index, recording in enumerate(recordings):
    total += recording.stat().st_size
    if index == 0 or total <= limit_bytes:
      continue

    for suffix in frogpilot_variables.SCREEN_RECORDING_COMPANION_SUFFIXES:
      frogpilot_utilities.delete_file(recording.with_suffix(suffix), report=False)
    frogpilot_utilities.delete_file(recording, report=False)

def cleanup_backups(directory, limit):
  directory.mkdir(parents=True, exist_ok=True)

  for in_progress in directory.glob("*_in_progress*"):
    frogpilot_utilities.delete_file(in_progress, report=False)

  backups = sorted(directory.glob("*_auto*"), key=lambda x: x.stat().st_mtime, reverse=True)
  for oldest_backup in backups[limit:]:
    frogpilot_utilities.delete_file(oldest_backup, report=False)

def backup_frogpilot(build_metadata):
  maximum_backups = 3
  cleanup_backups(frogpilot_paths.FROGPILOT_BACKUPS, maximum_backups)

  directory = Path(BASEDIR)
  destination_directory = frogpilot_paths.FROGPILOT_BACKUPS / f"{build_metadata.channel}_{build_metadata.openpilot.git_commit_date[12:-16]}_auto"
  if (frogpilot_paths.FROGPILOT_BACKUPS / (destination_directory.name + ".tar.zst")).exists():
    print("Backup already exists. Aborting...")
    return

  du_output = subprocess.run(["sudo", "du", "-sb", directory], capture_output=True, check=True, text=True).stdout
  install_size = int(du_output.split()[0])

  _, _, free = shutil.disk_usage(frogpilot_paths.FROGPILOT_BACKUPS)
  minimum_backup_size = frogpilot_variables.params.get_int("MinimumBackupSize")
  if free >= install_size + minimum_backup_size * maximum_backups:
    backup_directory(directory, destination_directory, f"Successfully backed up FrogPilot to {destination_directory}", f"Failed to backup FrogPilot to {destination_directory}", minimum_backup_size, compressed=True)

def backup_settings(destination):
  for key in frogpilot_utilities.settings_backup_keys():
    (destination / key).write_bytes(frogpilot_variables.params.get(key) or b"")

def backup_toggles(params_cache):
  params_backup = Params(str(frogpilot_paths.PARAMS_BACKUP_PATH))

  changes_found = False
  for key, _, _, _ in frogpilot_variables.frogpilot_default_params:
    new_value = frogpilot_variables.params.get(key)
    current_value = params_backup.get(key)

    if new_value != current_value:
      if new_value is not None:
        params_backup.put(key, new_value)
        params_cache.put(key, new_value)
      else:
        params_backup.remove(key)

      if key not in frogpilot_variables.EXCLUDED_KEYS:
        changes_found = True

  maximum_backups = 5

  cleanup_backups(frogpilot_paths.TOGGLE_BACKUPS, maximum_backups)

  existing_backups = list(frogpilot_paths.TOGGLE_BACKUPS.glob("*"))
  if not changes_found and existing_backups:
    print("Toggles are identical to the previous backup. Aborting...")
    return

  directory = frogpilot_paths.PARAMS_BACKUP_PATH / "d"
  destination_directory = frogpilot_paths.TOGGLE_BACKUPS / f"{datetime.datetime.now().strftime('%Y-%m-%d_%I-%M%p').lower()}_auto"
  backup_directory(directory, destination_directory, f"Successfully backed up toggles to {destination_directory}", f"Failed to backup toggles to {destination_directory}")

def convert_params(params_cache):
  print("Starting to convert params")

  for param_store in (frogpilot_variables.params, params_cache):
    value = param_store.get("MaxLateralAcceleration")
    if value is not None:
      try:
        value = json.loads(value)
      except (json.JSONDecodeError, TypeError):
        value = None

      if not isinstance(value, dict):
        param_store.remove("MaxLateralAcceleration")

  for param_store in (frogpilot_variables.params, params_cache, Params(str(frogpilot_paths.PARAMS_BACKUP_PATH))):
    for old_key, new_key in frogpilot_variables.RENAMED_KEYS.items():
      value = param_store.get(old_key)
      if value is not None:
        param_store.put(new_key, value)
        param_store.remove(old_key)

  stop_time = frogpilot_variables.params.get("CEModelStopTime")
  if stop_time is not None:
    tuning_level = frogpilot_variables.TUNING_LEVELS["DEVELOPER"]
    if frogpilot_variables.params.get_bool("TuningLevelConfirmed"):
      tuning_level = frogpilot_variables.params.get_int("TuningLevel")

    if tuning_level >= frogpilot_variables.TUNING_LEVELS["DEVELOPER"]:
      frogpilot_variables.params.put_bool("CEStopLights", stop_time != b"0")

  frogpilot_variables.params.remove("CEModelStopTime")

  if frogpilot_variables.params.get("SpeedLimitFillerShareData") == b"0":
    frogpilot_variables.params.put_bool("SpeedLimitFiller", False)

  frogpilot_variables.params.remove("SpeedLimitFillerShareData")

  if frogpilot_variables.params.get_bool("IsMetric"):
    metric_conversions = {
      "CESignalSpeed": (CV.MPH_TO_KPH, 0),
      "LaneLinesWidth": (CV.INCH_TO_CM, 0),
      "MinimumLaneChangeSpeed": (CV.MPH_TO_KPH, 0),
      "Offset1": (CV.MPH_TO_KPH, 0),
      "Offset2": (CV.MPH_TO_KPH, 0),
      "Offset3": (CV.MPH_TO_KPH, 0),
      "Offset4": (CV.MPH_TO_KPH, 0),
      "Offset5": (CV.MPH_TO_KPH, 0),
      "Offset6": (CV.MPH_TO_KPH, 0),
      "Offset7": (CV.MPH_TO_KPH, 0),
      "PathWidth": (CV.FOOT_TO_METER, 1),
      "RoadEdgesWidth": (CV.INCH_TO_CM, 0),
    }

    for key, (conversion, decimals) in metric_conversions.items():
      if frogpilot_variables.params.get(key) is None and params_cache.get(key) is None:
        frogpilot_variables.params.put(key, str(round(float(frogpilot_variables.get_default_value(key)) * conversion, decimals)))

  if frogpilot_paths.TRACKING_PATH.exists():
    params_tracking = Params(str(frogpilot_paths.TRACKING_PATH))

    frogpilot_stats = json.loads(frogpilot_variables.params.get("FrogPilotStats") or "{}")
    frogpilot_stats["FrogPilotDrives"] = params_tracking.get_int("FrogPilotDrives")
    frogpilot_stats["FrogPilotMeters"] = params_tracking.get_float("FrogPilotKilometers") * 1000
    frogpilot_stats["FrogPilotSeconds"] = params_tracking.get_float("FrogPilotMinutes") * 60

    frogpilot_variables.params.put("FrogPilotStats", json.dumps(frogpilot_stats))

    frogpilot_utilities.delete_file(frogpilot_paths.TRACKING_PATH)

  print("Param conversion completed")

def frogpilot_boot_functions(build_metadata):
  if frogpilot_variables.params.get_int("TetheringEnabled") == 3:
    frogpilot_variables.params.remove("TetheringEnabled")

  variables = frogpilot_variables.FrogPilotVariables()

  flag_files_changed = sync_flag_file(frogpilot_paths.HD_PATH, variables.frogpilot_toggles.use_higher_bitrate)
  flag_files_changed |= sync_flag_file(frogpilot_paths.KONIK_PATH, variables.frogpilot_toggles.use_konik_server)

  if flag_files_changed:
    HARDWARE.reboot()

  ModelManager(boot_run=True)
  variables.update(holiday_theme="stock", started=False)
  frogpilot_toggles = variables.frogpilot_toggles

  ThemeManager.copy_default_theme()
  ThemeManager(frogpilot_variables.params).update_active_theme(time_validated=system_time_valid(), frogpilot_toggles=frogpilot_toggles, boot_run=True)

  if frogpilot_paths.VIDEO_CACHE_PATH.exists():
    for cached_file in frogpilot_paths.VIDEO_CACHE_PATH.iterdir():
      frogpilot_utilities.delete_file(cached_file)

  if frogpilot_utilities.use_konik_server():
    konik_dongle_id = frogpilot_variables.params.get("KonikDongleId", encoding="utf8")
    if konik_dongle_id in (None, UNREGISTERED_DONGLE_ID):
      konik_dongle_id = register(show_spinner=True, register_konik=True)

    if konik_dongle_id != UNREGISTERED_DONGLE_ID:
      frogpilot_variables.params.put("KonikDongleId", konik_dongle_id)
    frogpilot_variables.params.put("DongleId", konik_dongle_id)
  elif frogpilot_variables.params.get("DongleId", encoding="utf8") == frogpilot_variables.params.get("KonikDongleId", encoding="utf8"):
    frogpilot_variables.params.remove("DongleId")

  shutil.rmtree(frogpilot_paths.RESTORE_TEMP_PATH, ignore_errors=True)

  def boot_thread():
    while not system_time_valid():
      print("Waiting for system time to become valid...")
      time.sleep(1)

    backup_frogpilot(build_metadata)

  threading.Thread(target=boot_thread, daemon=True).start()

def remove_tailscale_service():
  legacy_unit = [
    "[Unit]",
    "Description=Tailscale node agent",
    "After=network.target",
    "[Service]",
    f"ExecStart={frogpilot_paths.TAILSCALE_PATH}/tailscaled \\",
    "--tun=userspace-networking \\",
    "--socks5-server=localhost:1055 \\",
    f"--state={frogpilot_paths.TAILSCALE_STATE_PATH}/tailscaled.state \\",
    f"--socket={frogpilot_paths.TAILSCALE_SOCKET_PATH} \\",
    f"--statedir={frogpilot_paths.TAILSCALE_STATE_PATH}",
    "Restart=on-failure",
    "RestartSec=5",
    "[Install]",
    "WantedBy=multi-user.target",
  ]
  unit = [line.strip() for line in frogpilot_paths.UNIT_PATH.read_text().splitlines() if line.strip()]
  if unit != legacy_unit:
    return

  stock_mount_options = subprocess.run(["findmnt", "-no", "OPTIONS", "/"], capture_output=True, text=True, check=True).stdout.strip()

  frogpilot_utilities.run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount / as read-write")
  frogpilot_utilities.run_cmd(["sudo", "systemctl", "disable", "--now", "tailscaled"],
                              "Successfully stopped the Tailscale service", "Failed to stop the Tailscale service")
  frogpilot_utilities.run_cmd(["sudo", "rm", str(frogpilot_paths.UNIT_PATH)],
                              "Successfully removed the Tailscale service", "Failed to remove the Tailscale service")
  frogpilot_utilities.run_cmd(["sudo", "mount", "-o", f"remount,{stock_mount_options}", "/"],
                              "Successfully restored stock mount options", "Failed to restore stock mount options")

  frogpilot_utilities.run_cmd(["sudo", "systemctl", "daemon-reload"], "Successfully reloaded systemd", "Failed to reload systemd")
  frogpilot_utilities.run_cmd(["sudo", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(frogpilot_paths.TAILSCALE_PATH)],
                              "Successfully took ownership of Tailscale", "Failed to take ownership of Tailscale")

def restore_settings(source):
  allowed_keys = frogpilot_utilities.settings_backup_keys()

  for backup_file in source.iterdir():
    key = frogpilot_variables.RENAMED_KEYS.get(backup_file.name, backup_file.name)
    if key in allowed_keys:
      frogpilot_variables.params.put(key, backup_file.read_bytes())

def setup_frogpilot(build_metadata):
  frogpilot_paths.ERROR_LOGS_PATH.mkdir(parents=True, exist_ok=True)
  frogpilot_paths.HD_LOGS_PATH.mkdir(parents=True, exist_ok=True)
  frogpilot_paths.KONIK_LOGS_PATH.mkdir(parents=True, exist_ok=True)
  frogpilot_paths.MODELS_PATH.mkdir(parents=True, exist_ok=True)
  frogpilot_paths.SCREEN_RECORDINGS_PATH.mkdir(parents=True, exist_ok=True)
  frogpilot_paths.THEME_SAVE_PATH.mkdir(parents=True, exist_ok=True)

  cleanup_screen_recordings(10 * 1024 * 1024 * 1024)

  if frogpilot_paths.UNIT_PATH.is_file():
    remove_tailscale_service()

  update_boot_logo(frogpilot_paths.FROGPILOT_BOOT_LOGO_PATH)

  if build_metadata.channel == frogpilot_variables.DEVELOPMENT_BRANCH and frogpilot_paths.FROGS_GO_MOO_PATH.is_file():
    frogpilot_utilities.run_cmd(["sudo", "mount", "-o", "remount,rw", "/persist"], "Successfully remounted /persist as read-write", "Failed to remount /persist")
    frogpilot_utilities.run_cmd(["sudo", "python3", frogpilot_paths.FROGS_GO_MOO_PATH], "Ran frogsgomoo.py", "Failed to run frogsgomoo.py")

def sync_flag_file(flag_path, enabled):
  if flag_path.is_file() == enabled:
    return False

  if enabled:
    flag_path.touch()
  else:
    flag_path.unlink()

  return True

def soft_reboot():
  # restart the openpilot service without rebooting the device
  Path("/tmp/booted").touch()

  subprocess.check_call(["sudo", "systemctl", "restart", "--no-block", "comma"])


def uninstall_frogpilot():
  update_boot_logo(frogpilot_paths.STOCK_BOOT_LOGO_PATH)

  HARDWARE.uninstall()

def update_boot_logo(target_logo):
  if PC:
    return

  if not filecmp.cmp(target_logo, frogpilot_paths.BOOT_LOGO_PATH, shallow=False):
    stock_mount_options = subprocess.run(["findmnt", "-no", "OPTIONS", "/"], capture_output=True, text=True, check=True).stdout.strip()

    frogpilot_utilities.run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount / as read-write")
    frogpilot_utilities.run_cmd(["sudo", "cp", target_logo, frogpilot_paths.BOOT_LOGO_PATH], "Successfully replaced boot logo", "Failed to replace boot logo")
    frogpilot_utilities.run_cmd(["sudo", "mount", "-o", f"remount,{stock_mount_options}", "/"], "Successfully restored stock mount options", "Failed to restore stock mount options")
