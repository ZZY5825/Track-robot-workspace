import sqlite3

import numpy as np
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image

from track_robot_semantic_search.replay_probe import (
    decode_bgr_image,
    evenly_spaced_indices,
    read_sampled_messages,
)


def image_message(value):
    message = Image()
    message.width = 2
    message.height = 1
    message.encoding = 'bgra8'
    message.step = 8
    message.data = [value, 2, 3, 255, 4, 5, 6, 255]
    return message


def test_evenly_spaced_indices_are_deterministic_and_cover_endpoints():
    assert evenly_spaced_indices(10, 4) == [0, 3, 6, 9]
    assert evenly_spaced_indices(3, 10) == [0, 1, 2]


def test_decode_bgra_image_returns_owned_bgr_pixels():
    pixels = decode_bgr_image(serialize_message(image_message(1)))

    assert pixels.dtype == np.uint8
    assert pixels.shape == (1, 2, 3)
    assert pixels.tolist() == [[[1, 2, 3], [4, 5, 6]]]
    assert pixels.flags['OWNDATA']


def test_read_sampled_messages_uses_stable_database_order(tmp_path):
    database = tmp_path / 'bag.db3'
    connection = sqlite3.connect(str(database))
    connection.executescript(
        'create table topics(id integer primary key, name text, type text);'
        'create table messages(id integer primary key, topic_id integer, '
        'timestamp integer, data blob);')
    connection.execute(
        'insert into topics values(5, ?, ?)',
        ('/camera', 'sensor_msgs/msg/Image'))
    for index in range(5):
        connection.execute(
            'insert into messages values(?, 5, ?, ?)',
            (index + 1, 100 + index,
             serialize_message(image_message(index))))
    connection.commit()
    connection.close()

    rows = read_sampled_messages(
        database, '/camera', 'sensor_msgs/msg/Image', 3)

    assert [row['index'] for row in rows] == [0, 2, 4]
    assert [row['timestamp_ns'] for row in rows] == [100, 102, 104]
    assert [decode_bgr_image(row['data'])[0, 0, 0] for row in rows] == [0, 2, 4]
