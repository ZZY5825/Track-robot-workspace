from dataclasses import dataclass


def move_towards_zero(value: float, step: float) -> float:
    if step <= 0.0:
        return value

    if value > 0.0:
        return max(0.0, value - step)
    if value < 0.0:
        return min(0.0, value + step)
    return 0.0


@dataclass
class TeleopInputState:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    stop: bool = False

    def clear_motion(self):
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False

    def neutralize(self, stop: bool = False):
        self.clear_motion()
        self.stop = stop

    def copy(self):
        return TeleopInputState(
            forward=self.forward,
            backward=self.backward,
            left=self.left,
            right=self.right,
            stop=self.stop,
        )

    def summary(self) -> str:
        active = []
        if self.forward:
            active.append("FWD")
        if self.backward:
            active.append("BACK")
        if self.left:
            active.append("LEFT")
        if self.right:
            active.append("RIGHT")
        if self.stop:
            active.append("STOP")
        if not active:
            active.append("NEUTRAL")
        return " | ".join(active)


class InputStateController:
    def __init__(self):
        self._state = TeleopInputState()

    def set_forward(self, pressed: bool):
        self._state.forward = pressed
        if pressed:
            self._state.stop = False

    def set_backward(self, pressed: bool):
        self._state.backward = pressed
        if pressed:
            self._state.stop = False

    def set_left(self, pressed: bool):
        self._state.left = pressed
        if pressed:
            self._state.stop = False

    def set_right(self, pressed: bool):
        self._state.right = pressed
        if pressed:
            self._state.stop = False

    def request_stop(self):
        self._state.neutralize(stop=True)

    def neutralize(self, stop: bool = False):
        self._state.neutralize(stop=stop)

    def clear_stop(self):
        self._state.stop = False

    def snapshot(self) -> TeleopInputState:
        return self._state.copy()


@dataclass
class TeleopConfig:
    max_linear: float = 0.6
    max_angular: float = 1.0
    linear_accel: float = 0.4
    angular_accel: float = 0.8
    linear_decay: float = 1.2
    angular_decay: float = 2.0


class TeleopLogic:
    def __init__(self, config: TeleopConfig = None):
        self.config = config if config is not None else TeleopConfig()
        self.linear_x = 0.0
        self.angular_z = 0.0

    def reset(self):
        self.linear_x = 0.0
        self.angular_z = 0.0

    def update(self, input_state: TeleopInputState, dt: float, timed_out: bool = False):
        if timed_out or input_state.stop:
            self.reset()
            return self.linear_x, self.angular_z

        linear_accel = max(0.0, self.config.linear_accel * dt)
        angular_accel = max(0.0, self.config.angular_accel * dt)
        angular_decay = max(0.0, self.config.angular_decay * dt)

        if input_state.forward and not input_state.backward:
            self.linear_x += linear_accel
        elif input_state.backward and not input_state.forward:
            self.linear_x -= linear_accel
        elif not input_state.forward and not input_state.backward:
            self.linear_x = 0.0
        # If both forward and backward are pressed, keep current linear_x unchanged.

        if input_state.left and not input_state.right:
            self.angular_z += angular_accel
        elif input_state.right and not input_state.left:
            self.angular_z -= angular_accel
        elif not input_state.left and not input_state.right:
            self.angular_z = move_towards_zero(self.angular_z, angular_decay)
        # If both left and right are pressed, keep current angular_z unchanged.

        self.linear_x = max(-self.config.max_linear, min(self.linear_x, self.config.max_linear))
        self.angular_z = max(-self.config.max_angular, min(self.angular_z, self.config.max_angular))

        return self.linear_x, self.angular_z
