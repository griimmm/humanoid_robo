import cv2
import numpy as np

def get_homography_matrix(img, corners_xy_coords_in_plane):
    """
    Get homography matrix relating image coordinates to xy-coordinates on the flat surface containing the checkerboard.
    """
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    checker_board = (4,4) # Corners where 4 squares meet
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

    if not ret:
        return None
    else:
        homography_matrix, _ = cv2.findHomography(img_corners, corners_xy_coords_in_plane)
        return homography_matrix

def homography_transform(x, y, H, scale_factor=1):
    """
    Takes an image coordinate, scale factor and its homography matrix and returns corresponding coordinates in the mapped coordinate system.
    """
    point1 = np.array([x, y, scale_factor])
    point1 = point1.reshape(3, 1)
    point2 = np.dot(H, point1)
    point2 = point2/point2[2]
    return float(point2[0]), float(point2[1])


def get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img):
    """
    Get transform for pixel coordinates in vertical plane to world coordinates in base (0) frame on the robot
    """

    h,w = img.shape[:2]

    copy_img = np.copy(img)
    cv2.rectangle(copy_img, (0,0), (int(w/2),h), (0,0,0), -1)
    corners_xy_coords_in_horizontal_plane = np.array([(328,0), (328,-28), (300, 0), (300,-28)]) #np.array([(468,0), (468,-28), (440, 0), (440,-28)])#np.array([(412-40+46,-40-109+42),(412-40+46,-40-109+15),(412-40+46-28,-40-109+42),(412-40+46-28,-40-109+15)]) # height diff. +230mm rel. to table
    horizontal_plane_homography_matrix = get_homography_matrix(copy_img, corners_xy_coords_in_horizontal_plane)

    other_copy_img = np.copy(img)
    cv2.rectangle(other_copy_img, (int(w/2),0), (w,h), (0,0,0), -1)
    corners_xy_coords_in_vertical_plane = np.array([(0,38),(28,38),(0,10),(28,10)])
    vertical_plane_homography_matrix = get_homography_matrix(other_copy_img, corners_xy_coords_in_vertical_plane)

    # Find corners of left checkerboard
    copy_img = np.copy(img)
    cv2.rectangle(copy_img, (int(w/2),0), (w,h), (0,0,0), -1)
    gray = cv2.cvtColor(copy_img,cv2.COLOR_BGR2GRAY)
    checker_board = (4,4) # Corners where 4 squares meet
    ret, all_square_corners = cv2.findChessboardCorners(gray, checker_board, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    img_corners_of_vertical_plane_checkerboard = np.array([(int(all_square_corners[0][0][0]), int(all_square_corners[0][0][1])),
                (int(all_square_corners[checker_board[0]-1][0][0]), int(all_square_corners[checker_board[0]-1][0][1])),
                (int(all_square_corners[-checker_board[0]][0][0]), int(all_square_corners[-checker_board[0]][0][1])),
                (int(all_square_corners[-1][0][0]), int(all_square_corners[-1][0][1]))])
    
    # Corner in contact with horizontal surface, the 2nd one from the left, c3 + (c3-c1)/3
    c1 = img_corners_of_vertical_plane_checkerboard[0]
    c3 = img_corners_of_vertical_plane_checkerboard[2]
    lower_left_corner_img_coords = (int(c3[0]+(c3[0]-c1[0])/3), int(int(c3[1]+(c3[1]-c1[1])/3)))

    # Corner in contact with horizontal surface, the 2nd one from the right, c3 + (c3-c1)/3
    c2 = img_corners_of_vertical_plane_checkerboard[1]
    c4 = img_corners_of_vertical_plane_checkerboard[3]
    lower_right_corner_img_coords = (int(c4[0]+(c4[0]-c2[0])/3), int(int(c4[1]+(c4[1]-c2[1])/3)))

    vertical_plane_origo_coord_in_horizontal_plane = homography_transform(lower_left_corner_img_coords[0], lower_left_corner_img_coords[1], horizontal_plane_homography_matrix)
    vertical_plane_other_coord_in_horizontal_plane = homography_transform(lower_right_corner_img_coords[0], lower_right_corner_img_coords[1], horizontal_plane_homography_matrix)

    def transform(vertical_plane_pixel_x,vertical_plane_pixel_y):
        """Get world coordinates x0,y0,z0 from image coordinates x_pixel,y_pixel that are on the vertical surface"""
        (x_vert, y_vert) = homography_transform(vertical_plane_pixel_x, vertical_plane_pixel_y, vertical_plane_homography_matrix)

        p1 = vertical_plane_origo_coord_in_horizontal_plane
        p2 = vertical_plane_other_coord_in_horizontal_plane
        length_of_vec_p1_to_p2 = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        x_horiz = p1[0] + x_vert*(p2[0]-p1[0])/length_of_vec_p1_to_p2
        y_horiz = p1[1] + x_vert*(p2[1]-p1[1])/length_of_vec_p1_to_p2

        height_of_horizontal_plane_above_ground = 233
        height_of_z0_above_ground = 55
        z = (height_of_horizontal_plane_above_ground-height_of_z0_above_ground) + y_vert

        return x_horiz,y_horiz,z

    return transform

