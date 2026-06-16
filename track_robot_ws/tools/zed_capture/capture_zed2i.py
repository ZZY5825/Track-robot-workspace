#!/usr/bin/env python3

import pyzed.sl as sl
import cv2


def main():
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
    print("Press 'q' to quit.")

    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()

            cv2.imshow("ZED 2i Left Image", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
