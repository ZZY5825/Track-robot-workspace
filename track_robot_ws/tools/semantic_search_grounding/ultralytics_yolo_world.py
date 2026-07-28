"""R0C compatibility adapter over the installable YOLO-World backend."""

from track_robot_semantic_search.yolo_world_backend import (
    GroundedDetection,
    YoloWorldBackend,
    YoloWorldDependencies,
    load_yolo_world_dependencies,
)
from track_robot_semantic_search.yolo_world_backend import (
    normalize_yolo_world_result as _normalize_generic_result,
)

from .contracts import TeacherDetection


def _teacher_detection(value: GroundedDetection) -> TeacherDetection:
    return TeacherDetection(
        x1=value.x1,
        y1=value.y1,
        x2=value.x2,
        y2=value.y2,
        score=value.score,
        label=value.label,
    )


def normalize_yolo_world_result(
        result, query, width, height, max_detections):
    return tuple(_teacher_detection(value) for value in
                 _normalize_generic_result(
                     result, query, width, height, max_detections))


class UltralyticsYoloWorld(YoloWorldBackend):
    """Preserve the teacher-runner detection contract for R0C artifacts."""

    def predict(self, image, normalized_query):
        return tuple(_teacher_detection(value) for value in
                     super().predict(image, normalized_query))


__all__ = [
    'UltralyticsYoloWorld',
    'YoloWorldDependencies',
    'load_yolo_world_dependencies',
    'normalize_yolo_world_result',
]
