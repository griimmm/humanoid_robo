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

def main():
    transform = None
    cap = cv2.VideoCapture(1)
    while True:
        success, img = cap.read()
        img = get_undistorted_image(img, camera_matrix, distortion_coefficients)
        
        cv2.imshow("Image", img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('h'):
            transform = get_transform_function_for_vertical_plane_coord_to_base_frame_coord.get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img)

    cv2.destroyAllWindows()

if __name__=='__main__':
    main()