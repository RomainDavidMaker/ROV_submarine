import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # Increase publishing frequency
        self.cap = cv2.VideoCapture(0)

        # Set lower resolution and frame rate
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Lower JPEG quality
            compress_format = '.jpg'
            quality = 10  # Lower JPEG quality
            ret, buffer = cv2.imencode(compress_format, frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])

            if ret:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.format = "jpeg"
                compressed_msg.data = np.array(buffer).tobytes()

                self.publisher_.publish(compressed_msg)
                self.get_logger().info('Publishing compressed video frame')

def main(args=None):
    rclpy.init(args=args)
    video_publisher = CompressedImagePublisher()
    rclpy.spin(video_publisher)

    video_publisher.cap.release()
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
