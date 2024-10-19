import cv2
import numpy as np
from move_ee.get_transform_function_for_vertical_plane_coord_to_base_frame_coord import get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord
from move_ee.detect_foam import find_foam
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class CoordinateSaver:
    def __init__(self):
        self.image_coords = []

    def select_point(self,event,x,y,flags,param):
        """
        Mouse callback function.
        """
        if event == cv2.EVENT_LBUTTONDOWN:#cv2.EVENT_LBUTTONDBLCLK:
            self.image_coords.append((x,y))

# def send_points(world_coords: list):
#     # TODO: how to pass points
#     pass

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher')    
        self.publisher = self.create_publisher(Pose, 'cv_coord', 10)
        timer_period = 0.5  # seconds
        self.world_coord = []
        # From camera calibration script
        self.camera_matrix = np.array([[631.13444093,  0.        ,321.42210038],
                          [  0.        ,632.04692079,219.63790101],
                          [  0.        ,  0.        ,  1.        ]])

        # From camera calibration script
        self.distortion_coefficients = np.array([[ 0.05554114,-0.25243366,-0.01103603,-0.00042716, 0.36204737]])
        self.cv_stuff()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_undistorted_image(self, img, cam_matrix, dist_coeff):
        h,w = img.shape[:2]
        new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 1, (w,h))
        undist_image = cv2.undistort(img, cam_matrix, dist_coeff, None, new_cam_matrix)
        return undist_image
    
    def cv_stuff(self):
        cv2.namedWindow("Image")

        transform = None
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # 0 or 1 usually
        
        while True:
            _, img = cap.read()
            img = self.get_undistorted_image(img, self.camera_matrix, self.distortion_coefficients)
            
            cv2.imshow("Image", img)
            # key = cv2.waitKey(1)
            # if key == ord('q'):
            #     break
            # elif key == ord('h'):
            transform = get_transform_function_for_vertical_plane_pixel_coord_to_base_frame_coord(img)
            break
        
        if transform:
            # coord_saver = CoordinateSaver()

            # TEST:
            # cv2.setMouseCallback("Image", coord_saver.select_point)
            # while True:
            #     cv2.imshow("Image", img)
            #     key = cv2.waitKey(1)
            #     if key == ord('q'):
            #         break
            #     elif key == ord('p'):
            #         p_im = coord_saver.image_coords[-1]
            #         cv2.circle(img,(p_im[0],p_im[1]),3,(255,0,0),-1)
            #         print(f"World coord is {transform(p_im[0],p_im[1])}")
            # if len(coord_saver.image_coord_points)>0:
            #     world_coords = []
            #     for image_coord in coord_saver.image_coords:
            #         world_coord = transform(image_coord[0], image_coord[1])
            #         world_coords.append(world_coord)
            #     send_points(world_coords)
            
            points = find_foam(img, "tree-like")
            world_coords = []
            for image_coord in points:
                self.world_coord = transform(image_coord[0], image_coord[1])
                print(f"Coord is {self.world_coord}")
                world_coords.append(self.world_coord)
            # send_points(world_coords)
            # return self.world_coord
            cv2.destroyAllWindows()
    
    def timer_callback(self):
        msg = Pose()        
        msg.position.x = self.world_coord[0]
        msg.position.y = self.world_coord[1] 
        msg.position.z = self.world_coord[2]
        msg.orientation.w = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()