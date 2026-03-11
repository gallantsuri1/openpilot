"""
Timezone selector widget for sunnypilot settings.
"""
import pyray as rl
from collections.abc import Callable
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.sunnypilot.lib.styles import style
from openpilot.system.ui.widgets.list_view import ItemAction
from openpilot.sunnypilot.system.params_migration import TIMEZONE_VALUES, TIMEZONE_LABELS, DEFAULT_TIMEZONE

# Dimensions and styling constants (matching OptionControlSP)
BUTTON_WIDTH = 150
BUTTON_HEIGHT = 150
LABEL_WIDTH = 450  # Wider to accommodate timezone names
BUTTON_SPACING = 25
VALUE_FONT_SIZE = 45
BUTTON_FONT_SIZE = 60
CONTAINER_PADDING = 20


class TimezoneSelectorSP(ItemAction):
  """A < > control for selecting timezone from a predefined list."""

  def __init__(self, param: str, enabled: bool | Callable[[], bool] = True,
               on_value_changed: Callable[[str], None] | None = None):
    super().__init__(enabled=enabled)
    self.params = Params()
    self.param_key = param
    self.on_value_changed = on_value_changed
    self.current_index = 0
    self._left_enabled = enabled
    self._right_enabled = enabled

    # Load current value from params
    current_tz = self.params.get(self.param_key, return_default=True)
    if current_tz in TIMEZONE_VALUES:
      self.current_index = TIMEZONE_VALUES.index(current_tz)
    else:
      # Default to America/Chicago if invalid or not set
      self.current_index = TIMEZONE_VALUES.index(DEFAULT_TIMEZONE)
      self.params.put(self.param_key, DEFAULT_TIMEZONE)

    # Initialize font and button styles
    self._font = gui_app.font(FontWeight.MEDIUM)

    # Layout rectangles for components
    self.left_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.right_btn_rect = rl.Rectangle(0, 0, 0, 0)

  def get_value(self) -> str:
    """Get the current timezone string."""
    return TIMEZONE_VALUES[self.current_index]

  def get_display_label(self) -> str:
    """Get the display label for the current timezone."""
    return TIMEZONE_LABELS.get(TIMEZONE_VALUES[self.current_index], TIMEZONE_VALUES[self.current_index])

  def set_value(self, index: int):
    """Set the timezone by index."""
    if 0 <= index < len(TIMEZONE_VALUES):
      self.current_index = index
      tz_value = TIMEZONE_VALUES[index]
      self.params.put(self.param_key, tz_value)
      if self.on_value_changed:
        self.on_value_changed(tz_value)

  def _render(self, rect: rl.Rectangle):
    if self._rect.width == 0 or self._rect.height == 0 or not self.is_visible:
      return

    control_width = (BUTTON_WIDTH * 2) + LABEL_WIDTH + (BUTTON_SPACING * 2)
    total_width = control_width + (CONTAINER_PADDING * 2)
    self._rect.width = total_width

    start_x = self._rect.x + self._rect.width - control_width - (CONTAINER_PADDING * 2)
    component_y = rect.y + (rect.height - BUTTON_HEIGHT) / 2
    self.container_rect = rl.Rectangle(start_x, component_y, total_width, BUTTON_HEIGHT)

    # background
    rl.draw_rectangle_rounded(self.container_rect, 0.2, 20, style.OPTION_CONTROL_CONTAINER_BG)

    # left button (<)
    self.left_btn_rect = rl.Rectangle(self.container_rect.x, component_y, BUTTON_WIDTH + CONTAINER_PADDING,
                                       BUTTON_HEIGHT)

    # label
    label_x = self.container_rect.x + CONTAINER_PADDING + BUTTON_WIDTH + BUTTON_SPACING
    self.label_rect = rl.Rectangle(label_x, component_y, LABEL_WIDTH, BUTTON_HEIGHT)

    # right button (>)
    right_x = label_x + LABEL_WIDTH + BUTTON_SPACING
    self.right_btn_rect = rl.Rectangle(right_x, component_y, BUTTON_WIDTH + CONTAINER_PADDING, BUTTON_HEIGHT)

    self._left_enabled = self.enabled and self.current_index > 0
    self._right_enabled = self.enabled and self.current_index < len(TIMEZONE_VALUES) - 1

    self._render_button(self.left_btn_rect, "<", self._left_enabled)
    self._render_value_label()
    self._render_button(self.right_btn_rect, ">", self._right_enabled)

  def _render_button(self, rect: rl.Rectangle, text: str, enabled: bool):
    mouse_pos = rl.get_mouse_position()
    is_pressed = (rl.check_collision_point_rec(mouse_pos, rect) and
                  self._touch_valid() and rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT))

    text_color = style.ITEM_TEXT_COLOR if enabled else style.ITEM_DISABLED_TEXT_COLOR

    # highlight
    if enabled and is_pressed:
      rl.draw_rectangle_rounded(rect, 0.2, 20, style.OPTION_CONTROL_BTN_PRESSED)

    # button text
    text_size = measure_text_cached(self._font, text, BUTTON_FONT_SIZE)
    text_x = rect.x + (rect.width - text_size.x) / 2
    text_y = rect.y + (rect.height - text_size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(text_x, text_y), BUTTON_FONT_SIZE, 0, text_color)

  def _render_value_label(self):
    """Render the current timezone label."""
    text = self.get_display_label()
    text_color = style.ITEM_TEXT_COLOR if self.enabled else style.ITEM_DISABLED_TEXT_COLOR

    text_size = measure_text_cached(self._font, text, VALUE_FONT_SIZE)
    text_x = self.label_rect.x + (self.label_rect.width - text_size.x) / 2
    text_y = self.label_rect.y + (self.label_rect.height - text_size.y) / 2

    rl.draw_text_ex(self._font, text, rl.Vector2(text_x, text_y), VALUE_FONT_SIZE, 0, text_color)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._left_enabled and rl.check_collision_point_rec(mouse_pos, self.left_btn_rect):
      self.current_index -= 1
      self.current_index = max(0, self.current_index)
      self.set_value(self.current_index)
    elif self._right_enabled and rl.check_collision_point_rec(mouse_pos, self.right_btn_rect):
      self.current_index += 1
      self.current_index = min(len(TIMEZONE_VALUES) - 1, self.current_index)
      self.set_value(self.current_index)
