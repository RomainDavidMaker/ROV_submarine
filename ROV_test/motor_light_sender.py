import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import serial

class MotorLightSender(Node):
    def __init__(self):
        super().__init__('motor_light_sender')
        self.motor_command_subscriber = self.create_subscription(
            Float32MultiArray, 'motor_commands', self.motor_command_callback, 10)
        self.light_floor_subscriber = self.create_subscription(
            Float32, 'light_floor_control', self.light_floor_control_callback, 10)
        self.light_front_subscriber = self.create_subscription(
            Float32, 'light_front_control', self.light_front_control_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.commands = [0.0] * 10  # Initialize array for 8 motors + 2 lights

    def motor_command_callback(self, msg):
        if len(msg.data) != 8:
            self.get_logger().error('Expected 8 motor commands, received {}'.format(len(msg.data)))
            return
        self.commands[:8] = msg.data
        self.send_serial_data()

    def light_floor_control_callback(self, msg):
        self.commands[8] = msg.data
        self.send_serial_data()

    def light_front_control_callback(self, msg):
        self.commands[9] = msg.data
        self.send_serial_data()

    def send_serial_data(self):
        data_str = ','.join(map(str, self.commands)) + '\n'
        self.serial_port.write(data_str.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = MotorLightSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
