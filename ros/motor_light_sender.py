import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import serial


class MotorLightSender(Node):
    def __init__(self):
        super().__init__('motor_light_sender')
        self.rov_command_subscriber = self.create_subscription(
            Twist, 'command_motor', self.rov_command_callback, 10)
        self.light_control_subscriber = self.create_subscription(
            Float32MultiArray, 'control_light', self.light_control_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.commands = [0.0] * 11  # Initialize array for 8 motors + 2 lights + 1 servo

    def rov_command_callback(self, msg):
        motor_commands = self.transform_twist_to_motors(msg)
        self.commands[:8] = motor_commands
        self.send_serial_data()

    def normalize_value(self, value):
        return max(min(value, 1.0), -1.0)


    def transform_twist_to_motors(self, twist_msg):  
        # take the 6 twists values and gives 8 motors output
        motor_commands = [0.0] * 8  # Initialize with zeros or your own logic

        #forward (+x direction): turn forward 12 backward 43
        motor_commands[0] += twist_msg.linear.x
        motor_commands[1] += twist_msg.linear.x
        motor_commands[3] -= twist_msg.linear.x
        motor_commands[2] -= twist_msg.linear.x

        #left (+y direction) : turn forward 23 backward 14
        motor_commands[1] += twist_msg.linear.y
        motor_commands[2] += twist_msg.linear.y
        motor_commands[0] -= twist_msg.linear.y
        motor_commands[3] -= twist_msg.linear.y

        #up (+z direction) : turn forward 5678
        motor_commands[4] += twist_msg.linear.z
        motor_commands[5] += twist_msg.linear.z
        motor_commands[6] += twist_msg.linear.z
        motor_commands[7] += twist_msg.linear.z

        #rotation (+z direction) : turn forward 24 backward 13
        motor_commands[1] += twist_msg.angular.z
        motor_commands[3] += twist_msg.angular.z
        motor_commands[0] -= twist_msg.angular.z
        motor_commands[2] -= twist_msg.angular.z

        #rotation (+x direction) : turn forward 58 backward 67
        motor_commands[4] += twist_msg.angular.x
        motor_commands[7] += twist_msg.angular.x
        motor_commands[5] -= twist_msg.angular.x
        motor_commands[6] -= twist_msg.angular.x

        #rotation (+y direction) : turn forward 78 backward 56
        motor_commands[4] -= twist_msg.angular.y
        motor_commands[5] -= twist_msg.angular.y
        motor_commands[6] += twist_msg.angular.y
        motor_commands[7] += twist_msg.angular.y

        motor_commands = [self.normalize_value(command) for command in motor_commands]

        return motor_commands

    def light_control_callback(self, msg):
        if len(msg.data) >= 2:
            # Update the light commands based on the Float32MultiArray
            self.commands[8] = msg.data[0]  # First light control
            self.commands[9] = msg.data[1]  # Second light control
            self.send_serial_data()
        else:
            self.get_logger().warn('Received less than 2 values for light control')

    def send_serial_data(self):
        data_str = ';'.join(map(str, self.commands)) + '\n'
        self.serial_port.write(data_str.encode('utf-8'))
        self.get_logger().info(f'Sending data to serial: {data_str}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorLightSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


