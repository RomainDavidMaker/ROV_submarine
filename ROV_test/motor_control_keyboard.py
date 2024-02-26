import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class MotorControlKeyboard(Node):
    def __init__(self):
        super().__init__('motor_control_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'motor_commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.settings = termios.tcgetattr(sys.stdin)

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.read_key()
        twist = Twist()
        # Key mappings
        if key == 'z':
            twist.linear.x = 1.0
        elif key == 's':
            twist.linear.x = -1.0
        elif key == 'd':
            twist.angular.y = 1.0
        elif key == 'q':
            twist.angular.y = -1.0
        # Publish the Twist message
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
