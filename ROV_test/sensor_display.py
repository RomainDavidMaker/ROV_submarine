import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Import Float32MultiArray

class SensorDisplay(Node):
    def __init__(self):
        super().__init__('sensor_display')
        # Change subscription to Float32MultiArray
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_values',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Handle Float32MultiArray data
        float_data = msg.data
        self.get_logger().info(f'Received data: {float_data}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
