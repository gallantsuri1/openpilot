from datetime import datetime
from zoneinfo import ZoneInfo
import pyray as rl
from timezonefinder import TimezoneFinder
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets import Widget

# Position constants - bottom right corner
DATE_TIME_MARGIN = 30
DATE_TIME_FONT_SIZE = 45
DATE_TIME_RIGHT_OFFSET = 60
GPS_FONT_SIZE = 40
ROW_SPACING = 8  # Space between rows

# Param key for persisting last known timezone
LAST_TIMEZONE_PARAM = "LastTimezone"

# Timezone finder (singleton, initialized once)
_tf = None


class DateTimeDisplay(Widget):
  """Displays GPS location, GPS time, and local time at the bottom right corner when engaged."""

  def __init__(self):
    super().__init__()
    self.font = gui_app.font(FontWeight.BOLD)
    self.font_size = DATE_TIME_FONT_SIZE
    self.gps_font = gui_app.font(FontWeight.MEDIUM)
    self.gps_font_size = GPS_FONT_SIZE
    self.text_color = rl.Color(255, 255, 255, 255)
    self.text_color_shadow = rl.Color(0, 0, 0, 180)
    self.params = Params()
    self._gps_location_service = get_gps_location_service(self.params)

    # Initialize timezone finder
    global _tf
    if _tf is None:
      _tf = TimezoneFinder()

    # Load timezone from params (default: "America/Chicago" from params_keys.h)
    self._local_tz_str = self.params.get(LAST_TIMEZONE_PARAM)
    self._local_tz = ZoneInfo(self._local_tz_str)

    # Cache for GPS data to detect changes
    self._prev_latitude = None
    self._prev_longitude = None
    self._gps_location_text = None
    self._timezone_str = None

  def _update_state(self):
    """Update GPS coordinates and timezone from GPS if available."""
    try:
      # Check if GPS data is valid
      if not ui_state.sm.valid[self._gps_location_service]:
        self._gps_location_text = None
        return

      gps_data = ui_state.sm[self._gps_location_service]

      # Check if we have a valid fix
      if not gps_data.hasFix:
        self._gps_location_text = None
        return

      latitude = gps_data.latitude
      longitude = gps_data.longitude

      # Only update if coordinates changed
      if latitude != self._prev_latitude or longitude != self._prev_longitude:
        self._prev_latitude = latitude
        self._prev_longitude = longitude

        # Format coordinates
        lat_dir = "N" if latitude >= 0 else "S"
        lon_dir = "E" if longitude >= 0 else "W"

        lat_abs = abs(latitude)
        lon_abs = abs(longitude)

        # Format as DD°MM.MMM'
        lat_deg = int(lat_abs)
        lat_min = (lat_abs - lat_deg) * 60

        lon_deg = int(lon_abs)
        lon_min = (lon_abs - lon_deg) * 60

        self._gps_location_text = f"{lat_deg}°{lat_min:06.3f}'{lat_dir}  {lon_deg}°{lon_min:06.3f}'{lon_dir}"

        # Get timezone from coordinates and update if different from persisted value
        try:
          tz_str = _tf.timezone_at(lat=latitude, lng=longitude)
          if tz_str and tz_str != self._local_tz_str:
            # Update local_tz and persist to params
            self._local_tz_str = tz_str
            self._local_tz = ZoneInfo(tz_str)
            self._timezone_str = tz_str
            self.params.put(LAST_TIMEZONE_PARAM, tz_str)
          elif tz_str:
            # Same as persisted, just set for display
            self._timezone_str = tz_str
          else:
            self._timezone_str = None
        except Exception:
          self._timezone_str = None
    except Exception:
      self._gps_location_text = None

  def _render(self, rect: rl.Rectangle):
    # Only render when engaged
    if not ui_state.engaged:
      return

    # Get GPS location text (already updated in _update_state)
    gps_location_text = self._gps_location_text

    # Prepare rows
    rows = []

    # Row 1: Timezone (or GPS not loaded message)
    if self._timezone_str:
      rows.append({
        'text': self._timezone_str,
        'font': self.font,
        'font_size': self.gps_font_size,
      })
    else:
      rows.append({
        'text': "Loading GPS...",
        'font': self.font,
        'font_size': self.gps_font_size,
      })

    # Row 2: GPS coordinates (or Loading GPS message)
    if gps_location_text:
      rows.append({
        'text': gps_location_text,
        'font': self.font,
        'font_size': self.gps_font_size,
      })
    else:
      rows.append({
        'text': "Loading GPS...",
        'font': self.font,
        'font_size': self.gps_font_size,
      })

    # Row 3: Local time (using timezone from params/GPS)
    local_dt = datetime.now(self._local_tz)

    date_str = local_dt.strftime("%a, %b %d")
    time_str = local_dt.strftime("%I:%M %p")
    local_time_text = f"{date_str}  |  {time_str}"

    rows.append({
      'text': local_time_text,
      'font': self.font,
      'font_size': self.font_size,
    })

    # Calculate total height needed
    total_height = sum(row['font_size'] for row in rows) + (len(rows) - 1) * ROW_SPACING

    # Position at bottom right corner
    text_x_base = rect.x + rect.width - DATE_TIME_RIGHT_OFFSET
    current_y = rect.y + rect.height - DATE_TIME_MARGIN - total_height

    # Draw each row
    for row in rows:
      # Calculate text dimensions
      text_dims = rl.measure_text_ex(row['font'], row['text'], row['font_size'], 0)
      text_x = text_x_base - text_dims.x

      # Draw shadow
      rl.draw_text_ex(
        row['font'],
        row['text'],
        rl.Vector2(int(text_x) + 2, int(current_y) + 2),
        row['font_size'],
        0,
        self.text_color_shadow
      )

      # Draw main text
      rl.draw_text_ex(
        row['font'],
        row['text'],
        rl.Vector2(int(text_x), int(current_y)),
        row['font_size'],
        0,
        self.text_color
      )

      # Move to next row position
      current_y += row['font_size'] + ROW_SPACING
