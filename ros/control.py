"""
Create a node 'control' that reads the Twist message from the topic 'command', read the orentation from the topic orientation and send the Twist message to the topic   
'command_motor' that control the motors

type of control :
    - direct (no paramerters needed)
    - horizon (q_horizon and PID values needed)
    - angle (PID values needed)

Parameters:
    - control_mode ('direct','horizon','angle')
    - q_horizon quaternion of the horizontal orientation of the ROV (need to be set !)
    - Px, Py, Pz, Ix ....  PID parameters for xyz rotations


Mode direct :
    forward the command directly

Mode horizon:
    The angle of q_target is proportional to the joystick input, but when the joystick is released, the ROV
    returns to a horizontal orientation.


"""




import numpy as np
import rclpy
import tf
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from simple_pid import PID

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        
        # Subscriptions and Publisher
        self.subscription = self.create_subscription(
            Twist,
            'command',
            self.joystick_callback,
            10)
        self.orientation_subscription = self.create_subscription(
            Quaternion,
            'orientation',
            self.orientation_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'command_motor', 10)

        # Initialize control mode parameter
        self.declare_parameter('control_mode', 'direct')  # Default mode is 'direct'

        # Current orientation placeholder
        self.current_orientation = Quaternion()

        # Initialize PID controllers for x, y, z axes
        self.pid_x = PID(1, 0.1, 0.05, setpoint=0)  # Tune these values
        self.pid_y = PID(1, 0.1, 0.05, setpoint=0)
        self.pid_z = PID(1, 0.1, 0.05, setpoint=0)

    def joystick_callback(self, msg):
        control_mode = self.get_parameter('control_mode').value

        if control_mode == 'direct':
            self.handle_direct_mode(msg)
        elif control_mode == 'angle':
            self.handle_angle_mode(msg)
        elif control_mode == 'horizon':
            self.handle_horizon_mode(msg)

    def orientation_callback(self, msg):
        self.current_orientation = msg

    def compute_error(self, q, q_target):
        q_inv = tf.transformations.quaternion_inverse([q.x, q.y, q.z, q.w])
        q_error = tf.transformations.quaternion_multiply(q_inv, [q_target.x, q_target.y, q_target.z, q_target.w])
        error_euler = tf.transformations.euler_from_quaternion(q_error)
        return error_euler

    def handle_direct_mode(self, twist_msg):
        self.publisher_.publish(twist_msg)

    def handle_angle_mode(self, twist_msg):
        # Implement angle mode control logic
        # Define q_target based on twist_msg and compute error
        pass

    def handle_horizon_mode(self, twist_msg):
        q_horizon = self.get_parameter('q_horizon').value
        q_additional_rotation = self.compute_additional_rotation(twist_msg)
        q_target = tf.transformations.quaternion_multiply(q_horizon, q_additional_rotation)

        q_error = self.compute_error(self.current_orientation, q_target)
        stabilization_twist = self.compute_twist_for_stabilization(q_error)

        combined_twist = Twist()
        combined_twist.linear = twist_msg.linear
        combined_twist.angular.x += stabilization_twist.angular.x
        combined_twist.angular.y += stabilization_twist.angular.y
        combined_twist.angular.z += stabilization_twist.angular.z

        self.publisher_.publish(combined_twist)

    def compute_additional_rotation(self, twist_msg):
        # Convert the angular part of twist_msg to a quaternion
        # Assuming that twist_msg.angular.{x, y, z} are roll, pitch, yaw rates
        roll, pitch, yaw = twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z
        q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return q_rot

    def compute_twist_for_stabilization(self, q_error):
        twist = Twist()
        twist.angular.x = self.pid_x(q_error[0])
        twist.angular.y = self.pid_y(q_error[1])
        twist.angular.z = self.pid_z(q_error[2])
        return twist

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
