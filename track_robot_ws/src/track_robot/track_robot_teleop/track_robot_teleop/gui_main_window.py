from pathlib import Path

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
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

        self.setFixedSize(132, 132)
        self.setFocusPolicy(Qt.NoFocus)
        self.setCursor(Qt.PointingHandCursor)
        self.setFont(QFont("Segoe UI Symbol", 34, QFont.Bold))

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
            background = "#E2E8F0"
            border = "#CBD5E1"
            text_color = "#0F172A"

        self.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {background};
                border: 1px solid {border};
                border-radius: 22px;
                color: {text_color};
            }}
            """
        )


class StopButton(TeleopButton):
    def __init__(self):
        super().__init__("\u25A0", active_color="#991B1B")
        self.setFont(QFont("Segoe UI Symbol", 28, QFont.Bold))

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
                border-radius: 22px;
                color: {text_color};
            }}
            """
        )


class LinearVelocityBar(QWidget):
    def __init__(self):
        super().__init__()
        self._value = 0.0
        self._max_value = 1.0
        self.setFixedSize(92, 216)

    def set_value(self, value: float, max_value: float):
        self._value = value
        self._max_value = max(0.001, abs(max_value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        outer_rect = self.rect().adjusted(8, 8, -8, -8)
        painter.setPen(QPen(QColor("#CBD5E1"), 1))
        painter.setBrush(QColor("#F8FAFC"))
        painter.drawRoundedRect(outer_rect, 18, 18)

        center_y = outer_rect.center().y()
        painter.setPen(QPen(QColor("#94A3B8"), 2))
        painter.drawLine(
            QPointF(outer_rect.left() + 12, center_y),
            QPointF(outer_rect.right() - 12, center_y),
        )

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#E2E8F0"))
        track_rect = QRectF(
            outer_rect.left() + 24,
            outer_rect.top() + 14,
            outer_rect.width() - 48,
            outer_rect.height() - 28,
        )
        painter.drawRoundedRect(track_rect, 12, 12)

        ratio = max(-1.0, min(1.0, self._value / self._max_value))
        if abs(ratio) < 0.001:
            return

        fill_width = track_rect.width() - 8
        half_height = track_rect.height() / 2.0
        fill_height = half_height * abs(ratio)
        fill_x = track_rect.left() + 4

        if ratio > 0.0:
            fill_rect = QRectF(
                fill_x,
                track_rect.center().y() - fill_height,
                fill_width,
                fill_height,
            )
            fill_color = QColor("#22C55E")
        else:
            fill_rect = QRectF(
                fill_x,
                track_rect.center().y(),
                fill_width,
                fill_height,
            )
            fill_color = QColor("#38BDF8")

        painter.setBrush(fill_color)
        painter.drawRoundedRect(fill_rect, 10, 10)


class AngularTurnBar(QWidget):
    def __init__(self):
        super().__init__()
        self._value = 0.0
        self._max_value = 1.0
        self.setFixedSize(220, 260)
        self._robot_pixmap = self._load_robot_pixmap()

    def set_value(self, value: float, max_value: float):
        self._value = value
        self._max_value = max(0.001, abs(max_value))
        self.update()

    def _load_robot_pixmap(self):
        package_root = Path(__file__).resolve().parent.parent
        candidate = package_root / "assets" / "robot_top_view.png"
        if candidate.exists():
            pixmap = QPixmap(str(candidate))
            if not pixmap.isNull():
                return pixmap
        return None

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        ratio = max(-1.0, min(1.0, self._value / self._max_value))
        base_blue = 170
        deep_blue = 32
        channel = int(base_blue + (deep_blue - base_blue) * abs(ratio))
        outline_color = QColor(channel, channel + 36, 239)
        outer_rect = self.rect().adjusted(10, 8, -10, -8)
        painter.setPen(QPen(QColor("#CBD5E1"), 1))
        painter.setBrush(QColor("#F8FAFC"))
        painter.drawRoundedRect(outer_rect, 18, 18)

        center = outer_rect.center()
        painter.translate(center)
        max_angle_deg = 70.0
        painter.rotate(-90.0 - ratio * max_angle_deg)

        if self._robot_pixmap is not None:
            max_width = outer_rect.width() * 0.72
            max_height = outer_rect.height() * 0.78
            source_width = float(self._robot_pixmap.width())
            source_height = float(self._robot_pixmap.height())
            scale = min(max_width / source_width, max_height / source_height)
            draw_width = source_width * scale
            draw_height = source_height * scale
            target_rect = QRectF(
                -draw_width / 2.0,
                -draw_height / 2.0,
                draw_width,
                draw_height,
            )
            painter.drawPixmap(
                target_rect.toRect(),
                self._robot_pixmap,
                self._robot_pixmap.rect(),
            )
            return

        rect = QRectF(-22.0, -72.0, 44.0, 144.0)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(outline_color, 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawRoundedRect(rect, 14, 14)


class MainWindow(QWidget):
    def __init__(self, input_controller, stop_callback, neutralize_callback):
        super().__init__()
        self.input_controller = input_controller
        self.stop_callback = stop_callback
        self.neutralize_callback = neutralize_callback

        self.setWindowTitle("Track Robot GUI Teleop")
        self.setMinimumSize(860, 520)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet(
            """
            QWidget {
                background-color: #F8FAFC;
                color: #0F172A;
            }
            QLabel {
                color: #0F172A;
            }
            """
        )

        self.status_label = QLabel("Input state: NEUTRAL")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setFont(QFont("Segoe UI", 13, QFont.DemiBold))
        self.status_label.setStyleSheet(
            """
            QLabel {
                background-color: #E2E8F0;
                border: 1px solid #CBD5E1;
                border-radius: 16px;
                padding: 14px;
            }
            """
        )

        self.linear_title = QLabel("linear.x")
        self.linear_title.setAlignment(Qt.AlignCenter)
        self.linear_title.setFont(QFont("Consolas", 11, QFont.Bold))
        self.linear_bar = LinearVelocityBar()

        self.angular_title = QLabel("angular.z")
        self.angular_title.setAlignment(Qt.AlignCenter)
        self.angular_title.setFont(QFont("Consolas", 11, QFont.Bold))
        self.angular_bar = AngularTurnBar()

        self.btn_forward = TeleopButton("\u2191")
        self.btn_backward = TeleopButton("\u2193")
        self.btn_left = TeleopButton("\u2190")
        self.btn_right = TeleopButton("\u2192")
        self.btn_stop = StopButton()

        top_layout = QHBoxLayout()
        top_layout.setSpacing(20)
        top_layout.addWidget(self.status_label, 1)

        linear_layout = QVBoxLayout()
        linear_layout.setSpacing(10)
        linear_layout.addWidget(self.linear_title)
        linear_layout.addWidget(self.linear_bar, 0, Qt.AlignCenter)
        top_layout.addLayout(linear_layout)

        angular_layout = QVBoxLayout()
        angular_layout.setSpacing(10)
        angular_layout.addWidget(self.angular_title)
        angular_layout.addWidget(self.angular_bar, 0, Qt.AlignCenter)
        top_layout.addLayout(angular_layout)

        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(18)
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)

        layout = QVBoxLayout()
        layout.setContentsMargins(28, 28, 28, 28)
        layout.setSpacing(26)
        layout.addLayout(top_layout)
        layout.addLayout(grid)
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
        self.status_label.setText(f"Input state: {input_summary}")
        self.btn_forward.set_active(forward)
        self.btn_backward.set_active(backward)
        self.btn_left.set_active(left)
        self.btn_right.set_active(right)
        self.btn_stop.set_active(stop)
        self.linear_bar.set_value(linear_x, max_linear)
        self.angular_bar.set_value(angular_z, max_angular)

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
