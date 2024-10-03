import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.some_var = []
        self.start = 550 #Ideal 500
        self.range = 1750 #Ideal 1800
        self.subscription = self.create_subscription(
            JointState,
            'topic',
            self.listener_callback,
            10)
        self.publisher_1 = self.create_publisher(UInt16, 'servo_body', 10)
        self.publisher_2 = self.create_publisher(UInt16, 'servo_shoulder', 10)
        self.publisher_3 = self.create_publisher(UInt16, 'servo_elbow', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.some_var = msg.data.position

    def timer_callback(self):
        msg1 = UInt16()
        msg2 = UInt16()
        msg3 = UInt16()
        try:
            msg1.data = ((self.some_var[0] - self.start)/self.range)*180 #Degree conversions
            msg2.data = ((self.some_var[1] - self.start)/self.range)*180 #Degree conversions
            msg3.data = ((self.some_var[2] - self.start)/self.range)*180 #Degree conversions
        except:
            print("")
        self.publisher_1.publish(msg1)
        self.publisher_2.publish(msg1)
        self.publisher_3.publish(msg1)

        # self.get_logger().info('Publishing: "%s"' % msg.data)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    print(minimal_subscriber.some_var)
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     main()
