import cv2
import numpy as np
import get_transform_function_for_vertical_plane_coord_to_base_frame_coord

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

    # cv2.imshow("Thing", gray)
    # cv2.waitKey(0)

    # print("ret = ", ret)
    # print("all_square_corners = ", all_square_corners)
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
    # corners_xy_coords_in_plane = np.array([(412-40+46,-40-109+42),(412-40+46,-40-109+15),(412-40+46-28,-40-109+42),(412-40+46-28,-40-109+15)]) # height diff. +230mm rel. to table        # 3x3 board
    
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

def get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img, vertical_plane_homography_matrix, horizontal_plane_homography_matrix):
    """
    Get transform for pixel coordinates in vertical plane to world coordinates in base (0) frame on the robot
    """

    h,w = img.shape[:2]

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
    
    # TEST:
    # copy_img = cv2.drawChessboardCorners(img, checker_board, all_square_corners, ret)
    # print("Corners: ", img_corners_of_vertical_plane_checkerboard)
    # for i in range(4):
        # cv2.circle(img, img_corners_of_vertical_plane_checkerboard[i], 2, (0,50*(i+1),0), -1)
    
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
    
    # TEST:
    # print("Translated lower left coordinates have x =", vertical_plane_origo_coord_in_horizontal_plane[0], "and", vertical_plane_origo_coord_in_horizontal_plane[1])
    cv2.circle(img, lower_left_corner_img_coords, 2, (255,0,0), -1)
    cv2.circle(img, lower_right_corner_img_coords, 2, (255,0,0), -1)
    cv2.imshow("Lower left and right corners", img)
    cv2.waitKey(0)


    def transform(vertical_plane_pixel_x,vertical_plane_pixel_y):
        """Get world coordinates x0,y0,z0 from image coordinates x_pixel,y_pixel that are on the vertical surface"""
        (x_vert, y_vert) = homography_transform(vertical_plane_pixel_x, vertical_plane_pixel_y, vertical_plane_homography_matrix)

        # TEST:
        # print(f"x_vert is {x_vert} and y_vert is {y_vert}")
        # print(f"Test gives {homography_transform(lower_right_corner_img_coords[0], lower_right_corner_img_coords[1], vertical_plane_homography_matrix)}")
        # print(f"Other test gives {homography_transform(lower_left_corner_img_coords[0], lower_left_corner_img_coords[1], vertical_plane_homography_matrix)}")
        # print(f"Yet another test gives {homography_transform(int(all_square_corners[0][0][0]), int(all_square_corners[0][0][1]), vertical_plane_homography_matrix)}")

        p1 = vertical_plane_origo_coord_in_horizontal_plane
        p2 = vertical_plane_other_coord_in_horizontal_plane
        length_of_vec_p1_to_p2 = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        x_horiz = p1[0] + x_vert*(p2[0]-p1[0])/length_of_vec_p1_to_p2
        y_horiz = p1[1] + x_vert*(p2[1]-p1[1])/length_of_vec_p1_to_p2

        height_of_horizontal_plane_above_ground = 233
        height_of_z0_above_ground = 55
        z = (height_of_horizontal_plane_above_ground-height_of_z0_above_ground) + y_vert

        # TODO: add y_vert to z coord

        # TODO: check angle in horizontal plane between origo coord and other coord, the y_vert coord is simply added to z in world coord,
        # x_vert needs to be scaled by some angle and cos or sin or something

        return x_horiz,y_horiz,z


    # transform = lambda vertical_plane_pixel_x,vertical_plane_pixel_y: ( homography_transform(vertical_plane_pixel_x, vertical_plane_pixel_y))

    return transform

def main():
    for i in range(1):
        img_path = f'img_with_two_checkerboards/img_{i}.jpg'
        img = cv2.imread(img_path)

        # cv2.imshow("Image", img)
        # cv2.waitKey(0)

        # get_homography_matrix(img) # test

        h,w = img.shape[:2]
        img = get_undistorted_image(img, camera_matrix, distortion_coefficients)
        cv2.imshow("Undistorted", img)

        copy_img = np.copy(img)
        cv2.rectangle(copy_img, (0,0), (int(w/2),h), (0,0,0), -1)
        corners_xy_coords_in_horizontal_plane = np.array([(412-40+46,-40-109+42),(412-40+46,-40-109+15),(412-40+46-28,-40-109+42),(412-40+46-28,-40-109+15)]) # height diff. +230mm rel. to table
        horizontal_plane_homography_matrix = get_homography_matrix(copy_img, corners_xy_coords_in_horizontal_plane)

        other_copy_img = np.copy(img)
        cv2.rectangle(other_copy_img, (int(w/2),0), (w,h), (0,0,0), -1)
        corners_xy_coords_in_vertical_plane = np.array([(0,38),(28,38),(0,10),(28,10)])
        vertical_plane_homography_matrix = get_homography_matrix(other_copy_img, corners_xy_coords_in_vertical_plane)

        transform = get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img, vertical_plane_homography_matrix, horizontal_plane_homography_matrix)

        test_pixel_x = 200
        test_pixel_y = 180
        print(f"Pixel coords ({test_pixel_x},{test_pixel_y}) give world coords {transform(test_pixel_x,test_pixel_y)}")

        cv2.circle(img, (test_pixel_x, test_pixel_y), 2, (0,0,255), -1)
        cv2.imshow("Test pixel",img)
        cv2.waitKey(0)


        # # Find corners of left checkerboard
        # other_copy_img = np.copy(img)
        # cv2.rectangle(other_copy_img, (int(w/2),0), (w,h), (0,0,0), -1)
        # gray = cv2.cvtColor(other_copy_img,cv2.COLOR_BGR2GRAY)
        # checker_board = (4,4) # Corners where 4 squares meet
        # ret, all_square_corners = cv2.findChessboardCorners(gray, checker_board, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        # img_corners_of_vertical_plane_checkerboard = np.array([(int(all_square_corners[0][0][0]), int(all_square_corners[0][0][1])),
        #            (int(all_square_corners[checker_board[0]-1][0][0]), int(all_square_corners[checker_board[0]-1][0][1])),
        #            (int(all_square_corners[-checker_board[0]][0][0]), int(all_square_corners[-checker_board[0]][0][1])),
        #            (int(all_square_corners[-1][0][0]), int(all_square_corners[-1][0][1]))])
        
        # # Corner in contact with horizontal surface, the 2nd one from the left, c3 + (c3-c1)/3
        # c1 = img_corners_of_vertical_plane_checkerboard[0]
        # c3 = img_corners_of_vertical_plane_checkerboard[2]
        # lower_left_corner_img_coords = (int(c3[0]+(c3[0]-c1[0])/3), int(int(c3[1]+(c3[1]-c1[1])/3)))

        # tc = homography_transform(lower_left_corner_img_coords[0], lower_left_corner_img_coords[1], horizontal_plane_homography_matrix)
        # print("Translated lower left coordinates have x =", tc[0], "and", tc[1])
        # cv2.circle(img, lower_left_corner_img_coords, 2, (255,0,0), -1)
        # cv2.imshow("Lower left corner", img)
        # cv2.waitKey(0)
        
        
        # TODO: get transform between vertical and horizontal plane using homography matrix

        # TODO: do not forget height (z) difference between

        

        cv2.imshow("Homography window 1", copy_img)
        cv2.imshow("Homography window 2", other_copy_img)

        # TODO: use homography matrices for stuff

        key = cv2.waitKey(0)
        if key == ord('q'):
            break

def other_main():
    for i in range(1):
        img_path = f'img_with_two_checkerboards/img_{i}.jpg'
        img = cv2.imread(img_path)
        img = get_undistorted_image(img, camera_matrix, distortion_coefficients)
        transform = get_transform_function_for_vertical_plane_coord_to_base_frame_coord.get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img)
        test_pixel_x = 200
        test_pixel_y = 180
        print(f"Pixel coords ({test_pixel_x},{test_pixel_y}) give world coords {transform(test_pixel_x,test_pixel_y)}")




if __name__=='__main__':
    main()
    other_main()