#!/usr/bin/env python3

import os
import cv2
import pyzed.sl as sl
from datetime import datetime


def main():
    save_dir = os.path.expanduser("~/track_robot_ws/dataset/zed2i/rgb")
    os.makedirs(save_dir, exist_ok=True)

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {status}")
        return

    print("ZED camera opened successfully.")
    print(f"Saving RGB images to: {save_dir}")
    print("Press 's' to save image.")
    print("Press 'q' to quit.")

    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    image_count = 0

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()

            # ZED image is usually BGRA. Convert to BGR for OpenCV saving.
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            cv2.imshow("ZED 2i RGB Capture", frame_bgr)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"zed2i_rgb_{timestamp}.png"
                filepath = os.path.join(save_dir, filename)

                cv2.imwrite(filepath, frame_bgr)
                image_count += 1
                print(f"Saved [{image_count}]: {filepath}")

            elif key == ord("q"):
                break

    zed.close()
    cv2.destroyAllWindows()

    print(f"Capture finished. Total saved images: {image_count}")


if __name__ == "__main__":
    main()
