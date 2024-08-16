import cv2
import numpy as np

board_size = (9, 6)

obj_points = []
img_points = []

objp = np.zeros((np.prod(board_size), 3), np.float32)
objp[:,:2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1,2)

imgs = [cv2.imread(f'cal_img_{i}.jpg') for i in range(1,21)]

for img in imgs:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)

    if ret:
        obj_points.append(objp)
        img_points.append(corners)

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

print('Camera Matrix:\n', camera_matrix)
print('Distortion Coefficients:\n', dist_coeffs)
