import hashlib
from pathlib import Path
import sqlite3
from typing import Dict, List

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def evenly_spaced_indices(total: int, sample_count: int) -> List[int]:
    if isinstance(total, bool) or not isinstance(total, int) or total <= 0:
        raise ValueError('total must be a positive integer')
    if (isinstance(sample_count, bool) or not isinstance(sample_count, int) or
            sample_count <= 0):
        raise ValueError('sample_count must be a positive integer')
    count = min(total, sample_count)
    return np.rint(np.linspace(0, total - 1, count)).astype(int).tolist()


def read_sampled_messages(
        database_path: Path,
        topic_name: str,
        expected_type: str,
        sample_count: int) -> List[Dict[str, object]]:
    path = Path(database_path).resolve()
    if not path.is_file():
        raise ValueError('rosbag database does not exist: {}'.format(path))
    connection = sqlite3.connect(
        'file:{}?immutable=1'.format(path), uri=True)
    try:
        topic = connection.execute(
            'select id, type from topics where name = ?',
            (topic_name,)).fetchone()
        if topic is None:
            raise ValueError('rosbag topic does not exist: {}'.format(topic_name))
        if topic[1] != expected_type:
            raise ValueError(
                'rosbag topic type is {}, expected {}'.format(
                    topic[1], expected_type))
        total = int(connection.execute(
            'select count(*) from messages where topic_id = ?',
            (topic[0],)).fetchone()[0])
        if total <= 0:
            raise ValueError('rosbag topic has no messages: {}'.format(topic_name))
        rows = []
        for index in evenly_spaced_indices(total, sample_count):
            row = connection.execute(
                'select timestamp, data from messages where topic_id = ? '
                'order by timestamp, id limit 1 offset ?',
                (topic[0], index)).fetchone()
            payload = bytes(row[1])
            rows.append({
                'index': index,
                'timestamp_ns': int(row[0]),
                'data': payload,
                'message_sha256': hashlib.sha256(payload).hexdigest(),
            })
        return rows
    finally:
        connection.close()


def decode_bgr_image(serialized: bytes) -> np.ndarray:
    message = deserialize_message(serialized, Image)
    if message.width <= 0 or message.height <= 0:
        raise ValueError('image dimensions must be positive')
    channels = {'bgr8': 3, 'bgra8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError('unsupported image encoding: {}'.format(
            message.encoding))
    minimum_step = int(message.width) * channels
    if int(message.step) < minimum_step:
        raise ValueError('image step is smaller than encoded width')
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if raw.size != expected:
        raise ValueError('image data length does not match height and step')
    rows = raw.reshape(int(message.height), int(message.step))
    pixels = rows[:, :minimum_step].reshape(
        int(message.height), int(message.width), channels)
    return pixels[:, :, :3].copy()
