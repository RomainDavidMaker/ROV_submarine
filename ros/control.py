import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.subscription = self.create_subscription(
            Twist,
            'command',
            self.joystick_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'command_motor', 10)

        # Initialize control mode parameter
        self.declare_parameter('control_mode', 'direct')  # Default mode is 'direct'

    def joystick_callback(self, msg):
        # Read the control mode parameter
        control_mode = self.get_parameter('control_mode').value

        if control_mode == 'direct':
            self.handle_direct_mode(msg)
        elif control_mode == 'angle':
            self.handle_angle_mode(msg)
        elif control_mode == 'horizon':
            self.handle_horizon_mode(msg)

    def handle_direct_mode(self, twist_msg):
        # In direct mode, forward the command directly
        self.publisher_.publish(twist_msg)

    def handle_angle_mode(self, twist_msg):
        # TODO: Implement angle mode control logic
        pass

    def handle_horizon_mode(self, twist_msg):
        # TODO: Implement horizon mode control logic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
