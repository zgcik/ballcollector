import time
import cv2
import numpy as np
import os
import glob

BOARD_SIZE = (6, 9)
cam = cv2.VideoCapture(0)


def cal_imgs():
    i = 0
    while True:
        _, frame = cam.read()
        debug_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        if ret:
            cv2.drawChessboardCorners(debug_frame, BOARD_SIZE, corners, ret)

        # take pic with 'c'
        if cv2.waitKey(1) == ord("c") and ret:
            # last = int(time.time())
            # if ret and int(time.time()) > last:
            last = int(time.time())
            script_dir = os.path.dirname(__file__)
            img_dir = os.path.join(script_dir, "calibration", "imgs")
            cv2.imwrite(os.path.join(img_dir, f"cal_img_{i}.jpg"), frame)
            print("chessboard image saved...")
            i += 1

        # break with 'q'
        if cv2.waitKey(1) == ord("q"):
            break

        cv2.imshow("camera feed", debug_frame)
    print("done")
    cam.release()
    cv2.destroyAllWindows()


def calibrate_camera_fast():
    script_dir = os.path.dirname(__file__)
    obj_points = []
    img_points = []
    objp = np.zeros((np.prod(BOARD_SIZE), 3), np.float32)
    objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)
    image_shape = None
    i = 0
    last = int(time.time())
    while True:
        _, frame = cam.read()
        debug_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        if ret:
            cv2.drawChessboardCorners(debug_frame, BOARD_SIZE, corners, ret)

        if ret and int(time.time()) > last:
            last = int(time.time())
            i += 1
            print(i)
            obj_points.append(objp)
            img_points.append(corners)
            if image_shape is None:
                image_shape = gray.shape[
                    ::-1
                ]  # Capture the shape from the first valid image

        # break with 'q'
        if cv2.waitKey(1) == ord("q"):
            break

        cv2.imshow("camera feed", debug_frame)
    print("done")
    cam.release()

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_shape, None, None
    )  # type: ignore

    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients:\n", dist_coeffs)

    np.save(os.path.join(script_dir, "calibration", "int_matrix.npy"), camera_matrix)
    np.save(os.path.join(script_dir, "calibration", "dist_matrix.npy"), dist_coeffs)


def calibrate_camera():

    obj_points = []
    img_points = []

    objp = np.zeros((np.prod(BOARD_SIZE), 3), np.float32)
    objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)

    script_dir = os.path.dirname(__file__)
    img_dir = os.path.join(script_dir, "calibration", "imgs", "cal_img_*.jpg")
    imgs = glob.glob(img_dir)
    imgs = [cv2.imread(img) for img in imgs]

    image_shape = None
    for img in imgs:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            if image_shape is None:
                image_shape = gray.shape[
                    ::-1
                ]  # Capture the shape from the first valid image
        else:
            print("chessboard corners not found in one image.")

    if not image_shape:
        print("no valid images for calibration.")
        return

    # Perform camera calibration to get camera matrix and distortion coefficients
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_shape, None, None
    )

    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients:\n", dist_coeffs)

    np.save(os.path.join(script_dir, "calibration", "int_matrix.npy"), camera_matrix)
    np.save(os.path.join(script_dir, "calibration", "dist_matrix.npy"), dist_coeffs)


if __name__ == "__main__":
    # cal_imgs()
    # calibrate_camera()
    calibrate_camera_fast()
