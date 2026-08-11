from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class DepthFrame:
    stamp_ns: int
    frame_id: str
    image: object


@dataclass(frozen=True)
class DepthMatch:
    frame: DepthFrame
    delta_ns: int


class DepthFrameBuffer:
    def __init__(self, max_frames, max_age_ns):
        if int(max_frames) <= 0 or int(max_age_ns) <= 0:
            raise ValueError('depth buffer bounds must be positive')
        self._frames = deque()
        self._max_frames = int(max_frames)
        self._max_age_ns = int(max_age_ns)

    @property
    def size(self):
        return len(self._frames)

    def clear(self):
        self._frames.clear()

    def push(self, frame):
        if int(frame.stamp_ns) <= 0 or not str(frame.frame_id):
            raise ValueError('depth frame stamp and frame_id are required')
        if self._frames and frame.stamp_ns < self._frames[-1].stamp_ns:
            self.clear()
        self._frames.append(frame)
        newest = frame.stamp_ns
        while self._frames and (
                len(self._frames) > self._max_frames
                or newest - self._frames[0].stamp_ns > self._max_age_ns):
            self._frames.popleft()

    def nearest(self, stamp_ns, maximum_delta_ns):
        stamp_ns = int(stamp_ns)
        maximum_delta_ns = int(maximum_delta_ns)
        if stamp_ns <= 0 or maximum_delta_ns < 0 or not self._frames:
            return None
        selected = min(
            self._frames,
            key=lambda frame: (abs(frame.stamp_ns - stamp_ns), frame.stamp_ns),
        )
        delta_ns = abs(selected.stamp_ns - stamp_ns)
        if delta_ns > maximum_delta_ns:
            return None
        return DepthMatch(selected, delta_ns)
