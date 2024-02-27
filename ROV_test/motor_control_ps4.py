import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4Controller(Node):
    def __init__(self):
        super().__init__('ps4_twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'motor_commands', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, data):
        twist = Twist()

        # Mapping (modify as needed):
        # Left analog stick vertical - linear.x
        # Right analog stick vertical - angular.z
        twist.linear.x = data.axes[1]  # Left stick vertical axis
        twist.angular.z = data.axes[3]  # Right stick vertical axis

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    ps4_controller = PS4Controller()
    rclpy.spin(ps4_controller)
    ps4_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
