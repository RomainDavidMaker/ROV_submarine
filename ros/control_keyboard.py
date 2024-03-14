import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import sys
import select
import termios
import tty

default_linear_velocity = .8    #propotional to motor control 1:full forward
default_angular_velocity = .4

class MotorControlKeyboard(Node):
    def __init__(self):
        super().__init__('control_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'command', 10)
        self.light_publisher = self.create_publisher(Float32MultiArray, 'control_light', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.settings = termios.tcgetattr(sys.stdin)
        self.light_state = [0.0, 0.0]  #  two lights, initially off

        # Velocity parameters
        self.linear_velocity = self.declare_parameter('linear_velocity', default_linear_velocity).value
        self.angular_velocity = self.declare_parameter('angular_velocity', default_angular_velocity).value

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
        light_msg = Float32MultiArray()

        # Key mappings
        if key == 'z':            # forward
            twist.linear.x = self.linear_velocity
        elif key == 's':          # backward
            twist.linear.x = -self.linear_velocity
        elif key == 'd':          # right
            twist.linear.y = self.linear_velocity
        elif key == 'q':          # left
            twist.linear.y = -self.linear_velocity
        elif key == 'r':          # up
            twist.linear.z = self.linear_velocity
        elif key == 'f':          # down
            twist.linear.z = -self.linear_velocity
        elif key == 'k':          # turn left
            twist.angular.z = self.angular_velocity
        elif key == 'm':          # turn right
            twist.angular.z = -self.angular_velocity

        # Key mapping for light toggle
        if key == ' ':            # toggle the lights
            self.light_state[0] = 1.0 - self.light_state[0]
            self.light_state[1] = 1.0 - self.light_state[1]
            light_msg.data = self.light_state
            self.light_publisher.publish(light_msg)

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
