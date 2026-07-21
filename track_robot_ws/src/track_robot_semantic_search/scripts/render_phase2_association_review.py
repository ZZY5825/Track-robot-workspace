#!/usr/bin/env python3

import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image


IMAGE_TOPIC = '/zed/zed_node/left/image_rect_color'
CAMERA_INFO_TOPIC = '/zed/zed_node/left/camera_info'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_jsonl(path):
    with path.open('r', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def open_bag_database(bag_directory):
    databases = sorted(bag_directory.glob('*.db3'))
    if len(databases) != 1:
        raise ValueError('review renderer requires exactly one rosbag db3 file')
    return sqlite3.connect(
        'file:{}?mode=ro&immutable=1'.format(databases[0]), uri=True)


def topic_id(connection, name, expected_type):
    row = connection.execute(
        'select id, type from topics where name = ?', (name,)).fetchone()
    if row is None or row[1] != expected_type:
        raise ValueError('missing or incompatible bag topic {}'.format(name))
    return int(row[0])


def decode_image(message):
    channels = {'bgr8': 3, 'bgra8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError('unsupported image encoding {}'.format(message.encoding))
    minimum_step = int(message.width) * channels
    raw = np.frombuffer(message.data, dtype=np.uint8)
    rows = raw.reshape(int(message.height), int(message.step))
    return rows[:, :minimum_step].reshape(
        int(message.height), int(message.width), channels)[:, :, :3].copy()


def nearest_image(connection, image_topic_id, target_stamp_ns):
    rows = connection.execute(
        'select timestamp, data from messages where topic_id = ? '
        'order by abs(timestamp - ?) limit 4',
        (image_topic_id, target_stamp_ns)).fetchall()
    candidates = []
    for _, payload in rows:
        message = deserialize_message(bytes(payload), Image)
        candidates.append((abs(stamp_ns(message.header.stamp) - target_stamp_ns), message))
    if not candidates:
        raise ValueError('bag image topic has no messages')
    return min(candidates, key=lambda item: item[0])


def camera_model(connection, camera_info_topic_id):
    row = connection.execute(
        'select data from messages where topic_id = ? order by timestamp limit 1',
        (camera_info_topic_id,)).fetchone()
    if row is None:
        raise ValueError('bag camera-info topic has no messages')
    info = deserialize_message(bytes(row[0]), CameraInfo)
    return {
        'width': int(info.width),
        'height': int(info.height),
        'fx': float(info.p[0] if info.p[0] > 0.0 else info.k[0]),
        'fy': float(info.p[5] if info.p[5] > 0.0 else info.k[4]),
        'cx': float(info.p[2] if info.p[0] > 0.0 else info.k[2]),
        'cy': float(info.p[6] if info.p[5] > 0.0 else info.k[5]),
    }


def base_to_camera(point):
    # Verified Stage-2C static transform:
    # zed_left_camera_optical_frame <- base_link
    x, y, z = point
    return (-y + 0.06, -z + 0.635, x - 0.26)


def project_box(tracklet, camera):
    position = tracklet['position']
    half = [0.5 * max(0.0, value) for value in tracklet['size']]
    corners = []
    for x in (position[0] - half[0], position[0] + half[0]):
        for y in (position[1] - half[1], position[1] + half[1]):
            for z in (position[2] - half[2], position[2] + half[2]):
                camera_point = base_to_camera((x, y, z))
                if camera_point[2] <= 0.05:
                    continue
                corners.append((
                    camera['fx'] * camera_point[0] / camera_point[2] + camera['cx'],
                    camera['fy'] * camera_point[1] / camera_point[2] + camera['cy']))
    centroid = base_to_camera(position)
    if centroid[2] <= 0.05 or not corners:
        return None
    left = min(point[0] for point in corners)
    top = min(point[1] for point in corners)
    right = max(point[0] for point in corners)
    bottom = max(point[1] for point in corners)
    clipped = (
        max(0.0, left), max(0.0, top),
        min(float(camera['width']), right),
        min(float(camera['height']), bottom))
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None
    return [left, top, right, bottom]


def color_for_tracklet(tracklet_id):
    palette = [
        (255, 180, 0), (0, 220, 255), (255, 80, 180), (80, 255, 80),
        (180, 80, 255), (255, 255, 60), (60, 160, 255), (180, 255, 60),
    ]
    return palette[int(tracklet_id) % len(palette)]


def draw_review(image, visual, lidar_batch, pair_scores, camera):
    output = image.copy()
    x, y, width, height = visual['roi']
    cv2.rectangle(output, (x, y), (x + width, y + height), (0, 0, 255), 5)
    cv2.putText(
        output, 'VISUAL ROI C{}'.format(visual['visual_candidate_id']),
        (max(5, x + 8), max(28, y + 30)), cv2.FONT_HERSHEY_SIMPLEX,
        0.9, (0, 0, 255), 3, cv2.LINE_AA)
    projected = []
    for tracklet in lidar_batch['tracklets']:
        tracklet_id = int(tracklet['tracklet_id'])
        if tracklet_id not in pair_scores:
            continue
        box = project_box(tracklet, camera)
        if box is None:
            continue
        color = color_for_tracklet(tracklet_id)
        left, top, right, bottom = [int(round(value)) for value in box]
        cv2.rectangle(output, (left, top), (right, bottom), color, 3)
        label = 'T{} s={:.2f}'.format(tracklet_id, pair_scores[tracklet_id])
        cv2.putText(
            output, label, (max(0, left), max(20, top - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        projected.append({
            'tracklet_id': tracklet_id,
            'projected_box': box,
            'position': tracklet['position'],
            'size': tracklet['size'],
            'score': pair_scores[tracklet_id],
        })
    return output, projected


def add_card_header(image, text):
    card = np.full((image.shape[0] + 42, image.shape[1], 3), 245, dtype=np.uint8)
    card[42:, :] = image
    cv2.putText(card, text, (8, 29), cv2.FONT_HERSHEY_SIMPLEX,
                0.68, (20, 20, 20), 2, cv2.LINE_AA)
    return card


def write_contact_sheets(cards, output_directory):
    page_size = 12
    columns = 3
    card_width, card_height = 640, 402
    for page_start in range(0, len(cards), page_size):
        page_cards = cards[page_start:page_start + page_size]
        rows = int(math.ceil(len(page_cards) / columns))
        page = np.full((rows * card_height, columns * card_width, 3),
                       235, dtype=np.uint8)
        for index, card in enumerate(page_cards):
            resized = cv2.resize(card, (card_width, card_height))
            row, column = divmod(index, columns)
            page[row * card_height:(row + 1) * card_height,
                 column * card_width:(column + 1) * card_width] = resized
        page_number = page_start // page_size + 1
        cv2.imwrite(str(output_directory / 'contact_sheet_{:02d}.png'.format(
            page_number)), page)


def main():
    parser = argparse.ArgumentParser(
        description='Render human-review overlays for Stage 2C association evidence.')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--context-jsonl', required=True, type=Path)
    parser.add_argument('--debug-jsonl', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    arguments = parser.parse_args()

    context = read_jsonl(arguments.context_jsonl)
    debug = read_jsonl(arguments.debug_jsonl)
    visuals = [row for row in context if row['kind'] == 'visual_observation']
    lidar_batches = [row for row in context if row['kind'] == 'lidar_batch']
    lidar_stamps = [row['batch_stamp_ns'] for row in lidar_batches]
    pair_scores = {}
    for row in debug:
        pair_scores.setdefault(int(row['visual_candidate_id']), {})[
            int(row['lidar_tracklet_id'])] = float(row['total_score'])

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    review_rows = []
    cards = []
    connection = open_bag_database(arguments.bag)
    try:
        image_id = topic_id(connection, IMAGE_TOPIC, 'sensor_msgs/msg/Image')
        info_id = topic_id(
            connection, CAMERA_INFO_TOPIC, 'sensor_msgs/msg/CameraInfo')
        camera = camera_model(connection, info_id)
        for visual in visuals:
            target = int(visual['visual_stamp_ns'])
            image_delta, image_message = nearest_image(connection, image_id, target)
            insert_at = bisect.bisect_left(lidar_stamps, target)
            candidates = lidar_batches[max(0, insert_at - 1):insert_at + 1]
            lidar_batch = min(
                candidates, key=lambda row: abs(row['batch_stamp_ns'] - target))
            scores = pair_scores.get(int(visual['visual_candidate_id']), {})
            rendered, projected = draw_review(
                decode_image(image_message), visual, lidar_batch, scores, camera)
            candidate_id = int(visual['visual_candidate_id'])
            file_name = 'candidate_{:04d}.png'.format(candidate_id)
            cv2.imwrite(str(arguments.output_dir / file_name), rendered)
            header = 'C{}  image dt={:.1f}ms  lidar dt={:.1f}ms'.format(
                candidate_id, image_delta / 1.0e6,
                abs(lidar_batch['batch_stamp_ns'] - target) / 1.0e6)
            cards.append(add_card_header(rendered, header))
            review_rows.append({
                'visual_candidate_id': candidate_id,
                'visual_stamp_ns': target,
                'roi': visual['roi'],
                'image_delta_ns': int(image_delta),
                'lidar_batch_stamp_ns': int(lidar_batch['batch_stamp_ns']),
                'lidar_delta_ns': abs(int(lidar_batch['batch_stamp_ns']) - target),
                'projected_tracklets': projected,
                'image_file': file_name,
            })
    finally:
        connection.close()

    with (arguments.output_dir / 'review_manifest.json').open(
            'w', encoding='utf-8') as stream:
        json.dump({'camera': camera, 'candidates': review_rows}, stream,
                  indent=2, sort_keys=True)
        stream.write('\n')
    write_contact_sheets(cards, arguments.output_dir)


if __name__ == '__main__':
    main()
