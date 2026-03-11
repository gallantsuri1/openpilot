from datetime import datetime
from zoneinfo import ZoneInfo
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets import Widget

# Position constants - bottom right corner
DATE_TIME_MARGIN = 30
DATE_TIME_FONT_SIZE = 50
DATE_TIME_RIGHT_OFFSET = 60
GPS_FONT_SIZE = 40


class DateTimeDisplay(Widget):
  """Displays GPS coordinates and date/time at the bottom right corner when engaged."""

  def __init__(self):
    super().__init__()
    self.font = gui_app.font(FontWeight.BOLD)
    self.font_size = DATE_TIME_FONT_SIZE
    self.text_color = rl.Color(255, 255, 255, 255)
    self.text_color_shadow = rl.Color(0, 0, 0, 180)
    self.tz = ZoneInfo("America/Chicago")

  def _render(self, rect: rl.Rectangle):
    # Only render when engaged
    if not ui_state.engaged:
      return

    # Get current date and time in selected timezone using datetime
    now = datetime.now(self.tz)
    date_str = now.strftime("%a, %b %d")
    time_str = now.strftime("%I:%M %p")

    # Combine date and time
    text = f"{date_str}  |  {time_str}"

    # Calculate text dimensions
    text_dims = rl.measure_text_ex(self.font, text, self.font_size, 0)

    # Position at bottom right corner of the rect
    text_x = rect.x + rect.width - text_dims.x - DATE_TIME_MARGIN - DATE_TIME_RIGHT_OFFSET
    text_y = rect.y + rect.height - DATE_TIME_MARGIN - self.font_size

    # Draw shadow for better visibility
    rl.draw_text_ex(
      self.font,
      text,
      rl.Vector2(int(text_x) + 2, int(text_y) + 2),
      self.font_size,
      0,
      self.text_color_shadow
    )

    # Draw main text
    rl.draw_text_ex(
      self.font,
      text,
      rl.Vector2(int(text_x), int(text_y)),
      self.font_size,
      0,
      self.text_color
    )
