import cv2
import numpy as np
import time
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

class CvCoordPublisher(Node):

    def __init__(self):
        super().__init__('coord_publisher')    
        self.publisher = self.create_publisher(Pose, 'cv_coord', 10)
        timer_period = 0.5  # seconds
        self.world_coords = []
        # From camera calibration script
        self.camera_matrix = np.array([[631.13444093,  0.        ,321.42210038],
                          [  0.        ,632.04692079,219.63790101],
                          [  0.        ,  0.        ,  1.        ]])

        # From camera calibration script
        self.distortion_coefficients = np.array([[ 0.05554114,-0.25243366,-0.01103603,-0.00042716, 0.36204737]])
        self.cv_stuff()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_undistorted_image(self, img, cam_matrix, dist_coeff):
        h, w = img.shape[:2]
        new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 1, (w,h))
        undist_image = cv2.undistort(img, cam_matrix, dist_coeff, None, new_cam_matrix)
        return undist_image
    
    def cv_stuff(self):
        cv2.namedWindow("Image")

        transform = None
        cap = cv2.VideoCapture(2) # 0 or 1 usually removed cv2.CAP_DSHOW
        
        while True:
            _, img = cap.read()
            img = self.get_undistorted_image(img, self.camera_matrix, self.distortion_coefficients)
        
            cv2.imshow("Image", img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('h'):
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
            # Coord is (np.float64(442.7178472845303), np.float64(-167.3339925776814), -7.407854627085442)
            # Coord is (np.float64(358.38445857872296), np.float64(-14.21845920322437), 56.69584497743831)
            # Coord is (np.float64(313.3054322431425), np.float64(67.62693165912923), 90.961423617485)
            # Coord is (np.float64(284.8703808412699), np.float64(119.25355462237806), 112.57554373381933)
            # Coord is (np.float64(265.5678243259605), np.float64(154.29923797138207), 127.24784972609031)
            # Coord is (np.float64(315.5945402710604), np.float(66.67595888996229), 185.65687896248411)
            # Coord is (np.float64(334.40552879670395), np.floa(66.94061338297345), 183.5302266063786)
            # Coord is (np.float64(354.32271317517814), np.floa(67.22083110394591), 181.2785146866287)
            # Coord is (np.float64(375.44662549628026), np.floa(67.51802645325635), 178.89037767582795)
            # Coord is (np.float64(397.51113034055), np.float64(67.82845513355362), 176.3959031749584)

            # self.world_coords = [[0.3155945402710604,  0.06667595888996229, 0.18565687896248411],
            #                      [0.33440552879670395, 0.06694061338297345, 0.1835302266063786],
            #                      [0.35432271317517814, 0.06722083110394591, 0.1812785146866287],
            #                      [0.37544662549628026, 0.06751802645325635, 0.17889037767582795],
            #                      [0.39751113034055,    0.06782845513355362, 0.1763959031749584]]
            # self.world_coords = [[0.1155945402710604, 0.06667595888996229, 0.18565687896248411],
            #                      [0.1155945402710604, 1.09667595888996229, 0.18565687896248411]]
                                #  [0.3155945402710604, 1.03667595888996229, 0.18565687896248411],
                                #  [0.3155945402710604, 1.06667595888996229, 0.18565687896248411],
                                #  [0.3155945402710604, 1.09667595888996229, 0.18565687896248411]]
            # self.world_coords = [[0.4627178472845303, -0.1673339925776814, 0.07407854627085442],
            #                     [0.35838445857872296, -0.01421845920322437, 0.05669584497743831],
            #                     [0.3133054322431425, -0.06762693165912923, 0.090961423617485],
            #                     [0.2848703808412699, -0.11925355462237806, 0.11257554373381933],
            #                     [0.2655678243259605, -0.15429923797138207, 0.12724784972609031]] tree like
            # self.world_coords = [[0.21453, -0.097988, 0.27391],
            #                     #  [0.27028, 0.019982, 0.31915],
            #                      [0.1609, 0.19887, 0.30545],
            #                      [0.10545, 0.25243, 0.4138],
            #                     #  [0.27028, 0.019982, 0.31915],
            #                      [0.11971, 0.246, 0.41379]]
            
            points = find_foam(img, "level")
            world_coord = []
            for image_coord in points:
                world_coord = transform(image_coord[0], image_coord[1])
                # print(f"Coord is {world_coord}")
                self.world_coords.append(world_coord)
            # send_points(world_coords)
            cv2.destroyAllWindows()
            
    
    def timer_callback(self):
        msg = Pose()        
        for world_coord in self.world_coords:
            msg.position.x = world_coord[0] / 1000 # mm conversion #0.15086 #0.099662# #0.25086 #self.world_coords[-1][0] / 1000
            msg.position.y = world_coord[1] / 1000 #0.25713 #0.67246 # #0.27246 #0.11452 #self.world_coords[-1][1] / 1000
            msg.position.z = world_coord[2] / 1000 #0.2964 #0.29638 #  #0.29638 #self.world_coords[-1][2] / 1000
            #msg.orientation.w = 1.0
            print(f"Publishing coordinates x: {msg.position.x}, y: {msg.position.y}, z: {msg.position.z}")
            self.publisher.publish(msg)
            time.sleep(25)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CvCoordPublisher()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

# if __name__=='__main__':
#     main()