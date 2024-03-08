import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Using Float32MultiArray to send multiple float values
import serial
import sys

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        # Publisher for an array of floats
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_values', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            serial_data = self.serial_port.readline().decode('utf-8').rstrip()
            try:
                # Splitting the comma-separated values
                pressure, tension, current, t_ext, t_bat, t_esc, leak, q0, q1, q2, q3, d_sonar = map(float, serial_data.split(';'))
                msg = Float32MultiArray()
                msg.data = [pressure, tension, current, t_ext, t_bat, t_esc, leak, q0, q1, q2, q3, d_sonar]
                self.publisher_.publish(msg)
                self.get_logger().info(
                f'Pressure: {pressure} atm, Tension: {tension} V, Current: {current} mA, '
                f'External Temp: {t_ext} °C, Battery Temp: {t_bat} °C, ESC Temp: {t_esc} °C, '
                f'Leak: {leak}, Quaternion: ({q0}, {q1}, {q2}, {q3}), Sonar Distance: {d_sonar} m'
            )
            except ValueError as e:
                self.get_logger().error('Received invalid data: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
