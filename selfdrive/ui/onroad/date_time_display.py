from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import pyray as rl
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

# Default timezone
DEFAULT_TIMEZONE = "America/Chicago"


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
    self.default_tz = ZoneInfo(DEFAULT_TIMEZONE)
    
    # Cache for GPS data to detect changes
    self._prev_latitude = None
    self._prev_longitude = None
    self._gps_location_text = None

  def _update_state(self):
    """Update GPS coordinates whenever new data is available."""
    try:
      # Check if GPS data is valid and updated
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
    except Exception:
      self._gps_location_text = None

  def _get_gps_datetime(self) -> datetime | None:
    """Get current datetime from GPS if available (similar to timed.py)."""
    try:
      # Check if GPS data is valid
      if not ui_state.sm.valid[self._gps_location_service]:
        return None
      
      gps_data = ui_state.sm[self._gps_location_service]
      
      # Check if we have a valid fix
      if not gps_data.hasFix:
        return None
      
      # Get time from GPS (unixTimestampMillis is in UTC) - same as timed.py
      gps_time = datetime.fromtimestamp(gps_data.unixTimestampMillis / 1000., tz=timezone.utc)
      
      # Check if GPS time is reasonable (not too old)
      time_diff = abs((datetime.now(timezone.utc) - gps_time).total_seconds())
      if time_diff > 60:  # More than 60 seconds old
        return None
      
      return gps_time
    except Exception:
      return None

  def _render(self, rect: rl.Rectangle):
    # Only render when engaged
    if not ui_state.engaged:
      return

    # Get GPS location text (already updated in _update_state)
    gps_location_text = self._gps_location_text
    
    # Get GPS time (always fetch fresh for accurate time)
    gps_dt = self._get_gps_datetime()
    
    # Prepare the three rows
    rows = []
    
    # Row 1: GPS coordinates (if available)
    if gps_location_text:
      rows.append({
        'text': gps_location_text,
        'font': self.gps_font,
        'font_size': self.gps_font_size,
      })
    
    # Row 2: GPS time in UTC (if available)
    if gps_dt is not None:
      gps_time_str = gps_dt.strftime("%I:%M %p UTC")
      rows.append({
        'text': gps_time_str,
        'font': self.font,
        'font_size': self.font_size,
      })
    
    # Row 3: Local time (America/Chicago default timezone)
    # Always use system time in default timezone (independent of GPS)
    local_dt = datetime.now(self.default_tz)
    
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
