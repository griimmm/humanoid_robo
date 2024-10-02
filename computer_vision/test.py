import cv2
import numpy as np


# From camera calibration script
camera_matrix = np.array([[631.13444093,  0.        ,321.42210038],
                          [  0.        ,632.04692079,219.63790101],
                          [  0.        ,  0.        ,  1.        ]])

# From camera calibration script
distortion_coefficients = np.array([[ 0.05554114,-0.25243366,-0.01103603,-0.00042716, 0.36204737]])

def get_undistorted_image(img, cam_matrix, dist_coeff):
    h,w = img.shape[:2]
    new_cam_matrix,roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 1, (w,h))
    undist_image = cv2.undistort(img, cam_matrix, dist_coeff, None, new_cam_matrix)
    return undist_image

def get_homography_matrix(img):
    """
    Get homography matrix relating image coordinates to xy-coordinates on the flat surface containing the checkerboard
    """
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # checker_board = (6, 8)
    # checker_board = (1,2)
    checker_board = (3,3)
    ret, all_square_corners = cv2.findChessboardCorners(gray, checker_board, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # Corner order:
    # 1-------2
    # |       |
    # |       |
    # |       |
    # |       |
    # 3-------4

    img_corners = np.array([(int(all_square_corners[0][0][0]), int(all_square_corners[0][0][1])),
                   (int(all_square_corners[checker_board[0]-1][0][0]), int(all_square_corners[checker_board[0]-1][0][1])),
                   (int(all_square_corners[-checker_board[0]][0][0]), int(all_square_corners[-checker_board[0]][0][1])),
                   (int(all_square_corners[-1][0][0]), int(all_square_corners[-1][0][1]))])
    
    # OBS: only test
    for corner in img_corners:
        cv2.circle(img, corner, 3, (0,0,255), -1)
    # end test

    # corners_xy_coords_in_plane = np.array([(801,-299),(801,-120),(581,-120),(581,-299)])
    # corners_xy_coords_in_plane = np.array([(395,-106),(395,-123),(361,-106),(361,-123)]) # height diff. +230mm rel. to table      # 1x2 board
    corners_xy_coords_in_plane = np.array([(412-40+46,-40-109+42),(412-40+46,-40-109+15),(412-40+46-28,-40-109+42),(412-40+46-28,-40-109+15)]) # height diff. +230mm rel. to table        # 3x3 board
    
    if not ret:
        return None
    else:
        homography_matrix, _ = cv2.findHomography(img_corners, corners_xy_coords_in_plane)
        return homography_matrix

def translate_coordinates(x, y, H, scale_factor=1):
    """
    Takes a coordinate, scale factor and its homography matrix and returns corresponding coordinates in the mapped coordinate system.
    """
    point1 = np.array([x, y, scale_factor])
    point1 = point1.reshape(3, 1)
    point2 = np.dot(H, point1)
    point2 = point2/point2[2]
    return float(point2[0]), float(point2[1])

def main():
    for i in range(23):
        img_path = f'styrofoam_imgcopy/img_{i}.jpg'
        img = cv2.imread(img_path)

        h,w = img.shape[:2]
        img = get_undistorted_image(img, camera_matrix, distortion_coefficients)
        cv2.imshow("Undistorted", img)

        copy_img = np.copy(img)
        cv2.rectangle(copy_img, (0,0), (int(w/2),h), (0,0,0), -1)
        horizontal_plane_homography_matrix = get_homography_matrix(copy_img)

        other_copy_img = np.copy(img)
        cv2.rectangle(other_copy_img, (int(w/2),0), (w,h), (0,0,0), -1)
        vertical_plane_homography_matrix = get_homography_matrix(other_copy_img)

        cv2.imshow("Homography window 1", copy_img)
        cv2.imshow("Homography window 2", other_copy_img)

        # TODO: use homography matrices for stuff

        key = cv2.waitKey(0)
        if key == ord('q'):
            break

if __name__=='__main__':
    main()