import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from track_robot_teleop.gui_main_window import MainWindow
from track_robot_teleop.teleop_logic import InputStateController, TeleopConfig, TeleopLogic


def main():
    app = QApplication(sys.argv)

    input_controller = InputStateController()
    visual_logic = TeleopLogic(TeleopConfig())
    publish_hz = 50.0
    dt = 1.0 / publish_hz

    def request_stop():
        input_controller.request_stop()

    def neutralize():
        input_controller.neutralize(stop=True)

    window = MainWindow(
        input_controller=input_controller,
        stop_callback=request_stop,
        neutralize_callback=neutralize,
    )
    window.show()
    window.setFocus()

    timer = QTimer()
    timer.setInterval(max(1, int(1000.0 / publish_hz)))

    def refresh():
        state = input_controller.snapshot()
        linear_x, angular_z = visual_logic.update(state, dt, timed_out=False)
        window.update_feedback(
            input_summary=state.summary(),
            forward=state.forward,
            backward=state.backward,
            left=state.left,
            right=state.right,
            stop=state.stop,
            linear_x=linear_x,
            max_linear=visual_logic.config.max_linear,
            angular_z=angular_z,
            max_angular=visual_logic.config.max_angular,
        )
        if state.stop:
            input_controller.clear_stop()

    timer.timeout.connect(refresh)
    timer.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
