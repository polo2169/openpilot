"""Make the isolated RVV trial's scope visible even when native UI is green."""


def draw_rvv_only(rect, combined=False):
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight, gui_app

  box = rl.Rectangle(rect.x + 12, rect.y + rect.height - 68, 350, 56)
  rl.draw_rectangle_rounded(box, 0.2, 6, rl.Color(0, 0, 0, 190))
  font = gui_app.font(FontWeight.MEDIUM)
  label = 'Direction + adaptation RVV' if combined else 'RVV seul - direction manuelle'
  rl.draw_text_ex(font, label, rl.Vector2(box.x + 10, box.y + 6), 19, 0, rl.WHITE)
  rl.draw_text_ex(font, 'Sans freinage automatique', rl.Vector2(box.x + 10, box.y + 32), 17, 0, rl.ORANGE)
