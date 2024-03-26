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
import transformations
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
        self.declare_parameter('control_mode', 'direct')  # Default mode is 'direct' other modes: 'horizon'
        print('control_mode: ' + self.get_parameter('control_mode').value)
        # Current orientation placeholder
        self.current_orientation = Quaternion()

        self.q_horizon =  Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)

        # Initialize PID controllers for x, y, z axes
        self.pid_x = PID(1, 0., 0.0, setpoint=0)  # Tune these values
        self.pid_y = PID(1, 0., 0.0, setpoint=0)
        self.pid_z = PID(1, 0., 0.0, setpoint=0)

        # Safety feature turn off motors if no command received for more than .5s
        self.last_msg_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.5, self.check_command_timeout)

        # Safety feature to switch to direct mode if no orientation data for more than .5s
        self.last_orientation_time = self.get_clock().now()

    def joystick_callback(self, msg):
        self.last_msg_time = self.get_clock().now()  # Reset timer on new message
        control_mode = self.get_parameter('control_mode').value

        if control_mode != 'direct':
            # Check for the last time orientation data was received
            current_time = self.get_clock().now()
            if (current_time - self.last_orientation_time).nanoseconds > 0.5 * 1e9:
                # Switch to 'direct' mode if no orientation data for more than 0.5 seconds
                print("lost orientation data, switch to direct mode")
                self.set_parameters([rclpy.parameter.Parameter('control_mode', rclpy.parameter.Parameter.Type.STRING, 'direct')])


        if control_mode == 'direct':
            self.handle_direct_mode(msg)
        elif control_mode == 'angle':
            self.handle_angle_mode(msg)
        elif control_mode == 'horizon':
            self.handle_horizon_mode(msg)

    def check_command_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_msg_time).nanoseconds > 0.5 * 1e9:
            # More than 0.5 seconds have passed since the last message
            print("lost control command, turn off motors")
            self.publisher_.publish(Twist())

    def orientation_callback(self, msg):
        self.current_orientation = msg
        self.last_orientation_time = self.get_clock().now()  # Update orientation timer


    def compute_error(self, q, q_target):
        # Calculate the inverse of the current orientation
        q_inv = transformations.quaternion_inverse([q.x, q.y, q.z, q.w])

        # Compute the error quaternion
        q_error = transformations.quaternion_multiply(q_inv, [q_target.x, q_target.y, q_target.z, q_target.w])

        # Convert the error quaternion to Euler angles (roll, pitch, yaw)
        error_euler = transformations.euler_from_quaternion(q_error)

        return error_euler


    def handle_direct_mode(self, twist_msg):
        self.publisher_.publish(twist_msg)

    def handle_angle_mode(self, twist_msg):
        # Implement angle mode control logic
        # Define q_target based on twist_msg and compute error
        raise NotImplementedError # TODO

    def handle_horizon_mode(self, twist_msg):
        """
        Handles the 'horizon' control mode.
        In this mode, the ROV adjusts its orientation to maintain a horizontal position 
        while also considering the joystick input for direction.
        The function computes a target orientation based on the input command and the 
        desired horizontal orientation, calculates the error from the current orientation, 
        and then applies PID control to stabilize the ROV in the desired orientation.
        """

        # Convert the predefined horizontal quaternion orientation from ROS Quaternion to a list format
        q_horizon_list = [self.q_horizon.x, self.q_horizon.y, self.q_horizon.z, self.q_horizon.w]

        # Compute an additional rotation quaternion based on the joystick input (Twist message)
        q_additional_rotation = self.compute_additional_rotation(twist_msg)

        # Perform quaternion multiplication to find the target orientation
        # This combines the horizontal orientation with the joystick-induced rotation
        q_target_list = transformations.quaternion_multiply(q_horizon_list, q_additional_rotation)

        # Convert the target orientation from list format back to ROS Quaternion message
        q_target = Quaternion()
        q_target.x = q_target_list[0]
        q_target.y = q_target_list[1]
        q_target.z = q_target_list[2]
        q_target.w = q_target_list[3]

        # Calculate the orientation error between the current orientation and the target
        error_euler = self.compute_error(self.current_orientation, q_target)

        # Compute a stabilization twist (angular velocity) using PID controllers based on the orientation error
        stabilization_twist = self.compute_twist_for_stabilization(error_euler)

        # Prepare the final command to be sent to the motors
        # This combines the linear motion command from the joystick with the stabilization twist for angular motion
        combined_twist = Twist()
        combined_twist.linear = twist_msg.linear
        combined_twist.angular.x += stabilization_twist.angular.x
        combined_twist.angular.y += stabilization_twist.angular.y
        combined_twist.angular.z += stabilization_twist.angular.z

        # Publish the combined twist message to control the motors
        self.publisher_.publish(combined_twist)


    def compute_additional_rotation(self, twist_msg):
        # Convert the angular part of twist_msg to a quaternion
        roll, pitch, yaw = twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z
        q_rot = transformations.quaternion_from_euler(roll, pitch, yaw)
        return q_rot  # Ensure this is a list or NumPy array


    def compute_twist_for_stabilization(self, error_euler):
        twist = Twist()
        twist.angular.x = self.pid_x(error_euler[0])
        twist.angular.y = self.pid_y(error_euler[1])
        twist.angular.z = self.pid_z(error_euler[2])
        return twist

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
