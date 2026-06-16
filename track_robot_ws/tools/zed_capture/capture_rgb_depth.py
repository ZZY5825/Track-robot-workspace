#!/usr/bin/env python3

import os
import cv2
import numpy as np
import pyzed.sl as sl
from datetime import datetime


def normalize_depth_for_visualization(depth_np, max_distance=10.0):
    """
    Convert depth map to a visible 8-bit image.
    Invalid depth values are set to 0.
    """
    depth_vis = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)

    depth_vis[depth_vis < 0] = 0
    depth_vis[depth_vis > max_distance] = max_distance

    depth_vis = (depth_vis / max_distance * 255.0).astype(np.uint8)
    return depth_vis


def main():
    base_dir = os.path.expanduser("~/track_robot_ws/dataset/zed2i")
    rgb_dir = os.path.join(base_dir, "rgb")
    depth_dir = os.path.join(base_dir, "depth")
    depth_vis_dir = os.path.join(base_dir, "depth_vis")

    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(depth_vis_dir, exist_ok=True)

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30

    # Depth mode options:
    # PERFORMANCE: faster, lower accuracy
    # QUALITY: better quality, slower
    # ULTRA: highest quality, more GPU load
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

    # Use meters for depth values.
    init_params.coordinate_units = sl.UNIT.METER

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {status}")
        return

    print("ZED camera opened successfully.")
    print(f"Saving dataset to: {base_dir}")
    print("Press 's' to save RGB + depth.")
    print("Press 'q' to quit.")

    image = sl.Mat()
    depth = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    image_count = 0

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

            frame = image.get_data()
            depth_np = depth.get_data()

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            depth_vis = normalize_depth_for_visualization(depth_np, max_distance=10.0)

            cv2.imshow("ZED 2i RGB", frame_bgr)
            cv2.imshow("ZED 2i Depth Visualization", depth_vis)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

                rgb_path = os.path.join(rgb_dir, f"zed2i_rgb_{timestamp}.png")
                depth_path = os.path.join(depth_dir, f"zed2i_depth_{timestamp}.npy")
                depth_vis_path = os.path.join(depth_vis_dir, f"zed2i_depth_vis_{timestamp}.png")

                cv2.imwrite(rgb_path, frame_bgr)
                np.save(depth_path, depth_np)
                cv2.imwrite(depth_vis_path, depth_vis)

                image_count += 1

                center_y = depth_np.shape[0] // 2
                center_x = depth_np.shape[1] // 2
                center_depth = depth_np[center_y, center_x]

                print(f"Saved sample [{image_count}]")
                print(f"  RGB:       {rgb_path}")
                print(f"  Depth:     {depth_path}")
                print(f"  Depth vis: {depth_vis_path}")
                print(f"  Center depth: {center_depth:.3f} m")

            elif key == ord("q"):
                break

    zed.close()
    cv2.destroyAllWindows()

    print(f"Capture finished. Total saved samples: {image_count}")


if __name__ == "__main__":
    main()
