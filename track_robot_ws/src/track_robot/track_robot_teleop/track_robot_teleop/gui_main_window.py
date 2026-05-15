import math
from pathlib import Path

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class TeleopButton(QPushButton):
    def __init__(self, icon_text: str, active_color: str = "#22C55E"):
        super().__init__(icon_text)
        self._active_color = active_color
        self._is_active = False

        self.setFixedSize(76, 76)
        self.setFocusPolicy(Qt.NoFocus)
        self.setCursor(Qt.PointingHandCursor)
        self.setFont(QFont("Segoe UI Symbol", 25, QFont.Bold))
        self._apply_style()

    def set_active(self, active: bool):
        if self._is_active == active:
            return
        self._is_active = active
        self._apply_style()

    def _apply_style(self):
        if self._is_active:
            background = self._active_color
            border = self._active_color
            text_color = "#F8FAFC"
        else:
            background = "#EEF2F7"
            border = "#CBD5E1"
            text_color = "#111827"

        self.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {background};
                border: 1px solid {border};
                border-radius: 14px;
                color: {text_color};
            }}
            """
        )


class StopButton(TeleopButton):
    def __init__(self):
        super().__init__("\u25A0", active_color="#991B1B")
        self.setFont(QFont("Segoe UI Symbol", 20, QFont.Bold))

    def _apply_style(self):
        if self._is_active:
            background = "#991B1B"
            border = "#991B1B"
            text_color = "#FEF2F2"
        else:
            background = "#FECACA"
            border = "#FCA5A5"
            text_color = "#7F1D1D"

        self.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {background};
                border: 1px solid {border};
                border-radius: 14px;
                color: {text_color};
            }}
            """
        )


class SpeedGauge(QWidget):
    def __init__(self):
        super().__init__()
        self._value = 0.0
        self._max_value = 1.0
        self.setFixedSize(116, 154)

    def set_value(self, value: float, max_value: float):
        self._value = value
        self._max_value = max(0.001, abs(max_value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        outer_rect = self.rect().adjusted(5, 5, -5, -5)
        painter.setPen(QPen(QColor("#CBD5E1"), 1))
        painter.setBrush(QColor("#FFFFFF"))
        painter.drawRoundedRect(outer_rect, 10, 10)

        ratio = max(-1.0, min(1.0, self._value / self._max_value))
        center = QPointF(outer_rect.center().x(), outer_rect.bottom() - 28)
        radius = min(outer_rect.width() * 0.40, outer_rect.height() * 0.58)
        arc_rect = QRectF(center.x() - radius, center.y() - radius, radius * 2.0, radius * 2.0)

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("#E5E7EB"), 8, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(arc_rect, 35 * 16, 110 * 16)
        painter.setPen(QPen(QColor("#CBD5E1"), 8, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(arc_rect, 145 * 16, 70 * 16)

        active_color = QColor("#22C55E") if ratio >= 0.0 else QColor("#38BDF8")
        if abs(ratio) > 0.01:
            painter.setPen(QPen(active_color, 8, Qt.SolidLine, Qt.RoundCap))
            if ratio > 0.0:
                painter.drawArc(arc_rect, 90 * 16, int(-55 * ratio) * 16)
            else:
                painter.drawArc(arc_rect, 90 * 16, int(-55 * ratio) * 16)

        angle_deg = -90.0 + ratio * 55.0
        angle_rad = math.radians(angle_deg)
        needle_end = QPointF(
            center.x() + math.cos(angle_rad) * radius * 0.78,
            center.y() + math.sin(angle_rad) * radius * 0.78,
        )
        painter.setPen(QPen(QColor("#111827"), 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(center, needle_end)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawEllipse(center, 4, 4)

        painter.setPen(QColor("#374151"))
        painter.setFont(QFont("Consolas", 10, QFont.Bold))
        painter.drawText(
            QRectF(outer_rect.left(), outer_rect.bottom() - 26, outer_rect.width(), 18),
            Qt.AlignCenter,
            f"{self._value:+.2f}",
        )


class TrackMotionView(QWidget):
    def __init__(self):
        super().__init__()
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._max_linear = 1.0
        self._max_angular = 1.0
        self._phase = 0.0
        self._robot_pixmap = self._load_robot_pixmap()
        self.setFixedSize(286, 210)

    def _load_robot_pixmap(self):
        package_root = Path(__file__).resolve().parent.parent
        candidate = package_root / "assets" / "robot_top_view.png"
        if candidate.exists():
            pixmap = QPixmap(str(candidate))
            if not pixmap.isNull():
                return pixmap
        return None

    def set_motion(self, linear_x: float, max_linear: float, angular_z: float, max_angular: float):
        self._linear_x = linear_x
        self._angular_z = angular_z
        self._max_linear = max(0.001, abs(max_linear))
        self._max_angular = max(0.001, abs(max_angular))
        self._phase = (self._phase + (abs(linear_x) + abs(angular_z) * 0.25) * 1.7) % 1.0
        self.update()

    def _track_values(self):
        turn_gain = (self._max_linear / self._max_angular) * 0.45
        left_speed = self._linear_x - self._angular_z * turn_gain
        right_speed = self._linear_x + self._angular_z * turn_gain
        max_track_speed = self._max_linear + self._max_angular * turn_gain
        return left_speed, right_speed, max(0.001, max_track_speed)

    def _draw_track(self, painter: QPainter, rect: QRectF, speed: float, max_speed: float):
        ratio = max(-1.0, min(1.0, speed / max_speed))
        strength = abs(ratio)
        idle_green = QColor("#DCFCE7")
        active_green = QColor("#16A34A")
        color = QColor(
            int(idle_green.red() + (active_green.red() - idle_green.red()) * strength),
            int(idle_green.green() + (active_green.green() - idle_green.green()) * strength),
            int(idle_green.blue() + (active_green.blue() - idle_green.blue()) * strength),
        )

        painter.setPen(QPen(QColor("#334155"), 1))
        painter.setBrush(QColor("#F8FAFC"))
        painter.drawRoundedRect(rect, 10, 10)

        painter.setPen(Qt.NoPen)
        segment_count = 5
        segment_h = rect.height() / 9.0
        if abs(ratio) < 0.01:
            offset = 0.0
        else:
            direction = -1.0 if ratio > 0 else 1.0
            offset = self._phase * segment_h * 2.0 * direction
        for i in range(segment_count + 2):
            y = rect.top() + i * segment_h * 1.8 + offset - segment_h
            while y < rect.top() - segment_h:
                y += segment_h * 1.8
            while y > rect.bottom():
                y -= segment_h * 1.8
            segment = QRectF(rect.left() + 5, y, rect.width() - 10, segment_h)
            painter.setBrush(color)
            painter.drawRoundedRect(segment, 5, 5)

    def _draw_robot(self, painter: QPainter, body_rect: QRectF):
        if self._robot_pixmap is not None:
            painter.save()
            painter.translate(body_rect.center())
            painter.rotate(-90.0)
            max_width = body_rect.height() * 0.95
            max_height = body_rect.width() * 0.95
            source_width = float(self._robot_pixmap.width())
            source_height = float(self._robot_pixmap.height())
            scale = min(max_width / source_width, max_height / source_height)
            draw_width = source_width * scale
            draw_height = source_height * scale
            target = QRectF(-draw_width / 2.0, -draw_height / 2.0, draw_width, draw_height)
            painter.drawPixmap(target.toRect(), self._robot_pixmap, self._robot_pixmap.rect())
            painter.restore()
            return

        painter.setPen(QPen(QColor("#475569"), 2))
        painter.setBrush(QColor("#F8FAFC"))
        painter.drawRoundedRect(body_rect, 18, 18)
        center_line = QPainterPath()
        center_line.moveTo(body_rect.center().x(), body_rect.top() + 16)
        center_line.lineTo(body_rect.center().x(), body_rect.bottom() - 16)
        painter.setPen(QPen(QColor("#94A3B8"), 1, Qt.DashLine))
        painter.drawPath(center_line)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        card = self.rect().adjusted(4, 4, -4, -4)
        painter.setPen(QPen(QColor("#CBD5E1"), 1))
        painter.setBrush(QColor("#FFFFFF"))
        painter.drawRoundedRect(card, 12, 12)

        left_speed, right_speed, max_track_speed = self._track_values()
        body = QRectF(card.left() + 78, card.top() + 24, 130, card.height() - 48)
        left_track = QRectF(card.left() + 38, card.top() + 36, 28, card.height() - 72)
        right_track = QRectF(card.right() - 66, card.top() + 36, 28, card.height() - 72)

        self._draw_track(painter, left_track, left_speed, max_track_speed)
        self._draw_track(painter, right_track, right_speed, max_track_speed)
        self._draw_robot(painter, body)


class StatusPill(QLabel):
    def __init__(self, text: str):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont("Segoe UI", 9, QFont.DemiBold))
        self.setStyleSheet(
            """
            QLabel {
                background-color: #E5E7EB;
                border: 1px solid #CBD5E1;
                border-radius: 7px;
                padding: 5px 8px;
            }
            """
        )


class MainWindow(QWidget):
    def __init__(self, input_controller, stop_callback, neutralize_callback):
        super().__init__()
        self.input_controller = input_controller
        self.stop_callback = stop_callback
        self.neutralize_callback = neutralize_callback

        self.setWindowTitle("Track Robot GUI Teleop")
        self.setFixedSize(760, 420)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet(
            """
            QWidget {
                background-color: #F3F4F6;
                color: #111827;
            }
            QLabel {
                color: #111827;
            }
            """
        )

        self.input_label = StatusPill("NEUTRAL")
        self.mode_label = StatusPill("GUI TELEOP")
        self.topic_label = StatusPill("/teleop/input_state")

        self.linear_title = QLabel("speed")
        self.linear_title.setAlignment(Qt.AlignCenter)
        self.linear_title.setFont(QFont("Consolas", 10, QFont.Bold))
        self.linear_bar = SpeedGauge()

        self.motion_title = QLabel("track motion")
        self.motion_title.setAlignment(Qt.AlignCenter)
        self.motion_title.setFont(QFont("Consolas", 10, QFont.Bold))
        self.motion_view = TrackMotionView()

        self.btn_forward = TeleopButton("\u2191")
        self.btn_backward = TeleopButton("\u2193")
        self.btn_left = TeleopButton("\u2190")
        self.btn_right = TeleopButton("\u2192")
        self.btn_stop = StopButton()

        header = QHBoxLayout()
        header.setSpacing(8)
        header.addWidget(self.mode_label)
        header.addWidget(self.input_label, 1)
        header.addWidget(self.topic_label)

        control_grid = QGridLayout()
        control_grid.setHorizontalSpacing(9)
        control_grid.setVerticalSpacing(9)
        control_grid.addWidget(self.btn_forward, 0, 1)
        control_grid.addWidget(self.btn_left, 1, 0)
        control_grid.addWidget(self.btn_stop, 1, 1)
        control_grid.addWidget(self.btn_right, 1, 2)
        control_grid.addWidget(self.btn_backward, 2, 1)

        control_panel = QFrame()
        control_panel.setStyleSheet(
            """
            QFrame {
                background-color: #FFFFFF;
                border: 1px solid #CBD5E1;
                border-radius: 10px;
            }
            """
        )
        control_panel.setLayout(control_grid)

        motion_panel = QVBoxLayout()
        motion_panel.setSpacing(6)
        motion_panel.addWidget(self.motion_title)
        motion_panel.addWidget(self.motion_view)

        linear_panel = QVBoxLayout()
        linear_panel.setSpacing(6)
        linear_panel.addWidget(self.linear_title)
        linear_panel.addWidget(self.linear_bar, 0, Qt.AlignCenter)

        main_row = QHBoxLayout()
        main_row.setSpacing(16)
        main_row.addWidget(control_panel)
        main_row.addLayout(motion_panel)
        main_row.addLayout(linear_panel)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 14, 16, 16)
        layout.setSpacing(14)
        layout.addLayout(header)
        layout.addLayout(main_row)
        self.setLayout(layout)

        self._connect_signals()

    def _connect_signals(self):
        self.btn_forward.pressed.connect(lambda: self.input_controller.set_forward(True))
        self.btn_forward.released.connect(lambda: self.input_controller.set_forward(False))

        self.btn_backward.pressed.connect(lambda: self.input_controller.set_backward(True))
        self.btn_backward.released.connect(lambda: self.input_controller.set_backward(False))

        self.btn_left.pressed.connect(lambda: self.input_controller.set_left(True))
        self.btn_left.released.connect(lambda: self.input_controller.set_left(False))

        self.btn_right.pressed.connect(lambda: self.input_controller.set_right(True))
        self.btn_right.released.connect(lambda: self.input_controller.set_right(False))

        self.btn_stop.pressed.connect(lambda: self.btn_stop.set_active(True))
        self.btn_stop.released.connect(lambda: self.btn_stop.set_active(False))
        self.btn_stop.clicked.connect(self.stop_callback)

    def update_feedback(
        self,
        input_summary: str,
        forward: bool,
        backward: bool,
        left: bool,
        right: bool,
        stop: bool,
        linear_x: float,
        max_linear: float,
        angular_z: float,
        max_angular: float,
    ):
        self.input_label.setText(input_summary)
        self.btn_forward.set_active(forward)
        self.btn_backward.set_active(backward)
        self.btn_left.set_active(left)
        self.btn_right.set_active(right)
        self.btn_stop.set_active(stop)
        self.linear_bar.set_value(linear_x, max_linear)
        self.motion_view.set_motion(linear_x, max_linear, angular_z, max_angular)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()
        if key == Qt.Key_W:
            self.input_controller.set_forward(True)
        elif key == Qt.Key_S:
            self.input_controller.set_backward(True)
        elif key == Qt.Key_A:
            self.input_controller.set_left(True)
        elif key == Qt.Key_D:
            self.input_controller.set_right(True)
        elif key == Qt.Key_Space or key == Qt.Key_P:
            self.stop_callback()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()
        if key == Qt.Key_W:
            self.input_controller.set_forward(False)
        elif key == Qt.Key_S:
            self.input_controller.set_backward(False)
        elif key == Qt.Key_A:
            self.input_controller.set_left(False)
        elif key == Qt.Key_D:
            self.input_controller.set_right(False)

    def focusOutEvent(self, event):
        self.neutralize_callback()
        super().focusOutEvent(event)

    def closeEvent(self, event):
        self.neutralize_callback()
        super().closeEvent(event)
