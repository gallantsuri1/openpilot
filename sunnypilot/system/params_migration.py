"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.swaglog import cloudlog

ONROAD_BRIGHTNESS_MIGRATION_VERSION: str = "1.0"
ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION: str = "1.0"
TIMEZONE_MIGRATION_VERSION: str = "1.0"

# index → seconds mapping for OnroadScreenOffTimer (SSoT)
ONROAD_BRIGHTNESS_TIMER_VALUES = {0: 3, 1: 5, 2: 7, 3: 10, 4: 15, 5: 30, **{i: (i - 5) * 60 for i in range(6, 16)}}
VALID_TIMER_VALUES = set(ONROAD_BRIGHTNESS_TIMER_VALUES.values())

# List of valid timezones (index → timezone mapping)
TIMEZONE_VALUES = [
  "America/New_York",
  "America/Chicago",
  "America/Denver",
  "America/Los_Angeles",
  "America/Anchorage",
  "Pacific/Honolulu",
  "America/Sao_Paulo",
  "Europe/London",
  "Europe/Paris",
  "Europe/Moscow",
  "Asia/Dubai",
  "Asia/Kolkata",
  "Asia/Shanghai",
  "Asia/Tokyo",
  "Asia/Seoul",
  "Australia/Sydney",
  "Pacific/Auckland",
]

# Display labels for timezones
TIMEZONE_LABELS = {
  "America/New_York": "Eastern Time (US & Canada)",
  "America/Chicago": "Central Time (US & Canada)",
  "America/Denver": "Mountain Time (US & Canada)",
  "America/Los_Angeles": "Pacific Time (US & Canada)",
  "America/Anchorage": "Alaska",
  "Pacific/Honolulu": "Hawaii",
  "America/Sao_Paulo": "Brasilia",
  "Europe/London": "London",
  "Europe/Paris": "Paris, Berlin",
  "Europe/Moscow": "Moscow",
  "Asia/Dubai": "Dubai",
  "Asia/Kolkata": "Mumbai, Kolkata, New Delhi",
  "Asia/Shanghai": "Beijing, Shanghai",
  "Asia/Tokyo": "Tokyo",
  "Asia/Seoul": "Seoul",
  "Australia/Sydney": "Sydney",
  "Pacific/Auckland": "Auckland",
}

VALID_TIMEZONES = set(TIMEZONE_VALUES)
DEFAULT_TIMEZONE = "America/Chicago"


def run_migration(_params):
  # migrate OnroadScreenOffBrightness
  if _params.get("OnroadScreenOffBrightnessMigrated") != ONROAD_BRIGHTNESS_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffBrightness", return_default=True)
      if val >= 2:  # old: 5%, new: Screen Off
        new_val = val + 1
        _params.put("OnroadScreenOffBrightness", new_val)
        log_str = f"Successfully migrated OnroadScreenOffBrightness from {val} to {new_val}."
      else:
        log_str = "Migration not required for OnroadScreenOffBrightness."

      _params.put("OnroadScreenOffBrightnessMigrated", ONROAD_BRIGHTNESS_MIGRATION_VERSION)
      cloudlog.info(log_str + f" Setting OnroadScreenOffBrightnessMigrated to {ONROAD_BRIGHTNESS_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffBrightness: {e}")

  # migrate OnroadScreenOffTimer
  if _params.get("OnroadScreenOffTimerMigrated") != ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffTimer", return_default=True)
      if val not in VALID_TIMER_VALUES:
        _params.put("OnroadScreenOffTimer", 15)
        log_str = f"Successfully migrated OnroadScreenOffTimer from {val} to 15 (default)."
      else:
        log_str = "Migration not required for OnroadScreenOffTimer."

      _params.put("OnroadScreenOffTimerMigrated", ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION)
      cloudlog.info(log_str + f" Setting OnroadScreenOffTimerMigrated to {ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffTimer: {e}")

  # migrate TimeZone - set default if not exists or invalid
  if _params.get("TimeZoneMigrated") != TIMEZONE_MIGRATION_VERSION:
    try:
      val = _params.get("TimeZone", return_default=True)
      if val not in VALID_TIMEZONES:
        _params.put("TimeZone", DEFAULT_TIMEZONE)
        log_str = f"Successfully migrated TimeZone from {val} to {DEFAULT_TIMEZONE} (default)."
      else:
        log_str = "Migration not required for TimeZone."

      _params.put("TimeZoneMigrated", TIMEZONE_MIGRATION_VERSION)
      cloudlog.info(log_str + f" Setting TimeZoneMigrated to {TIMEZONE_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating TimeZone: {e}")
