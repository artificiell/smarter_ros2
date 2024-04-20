import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Image gray scale converter class
class GrayScaleConverter(Node):
    def __init__(self):
        super().__init__('gray_scale_converter')

        # Setup subscriber(s)
        self.image_subscriber = self.create_subscription(
            Image,
            'image',
            self.converter_callback,
            30
        )
        self.subscriptions  # Prevent unused variable warning

        # Setup publisher(s)
        self.image_publisher = self.create_publisher(Image, 'camera1/gray_raw', 1)

        # Bridge for handle conversion between seneor_msks.Image and numpy array (OpenCV)
        self.bridge = CvBridge()

    # Callback function
    def converter_callback(self, msg):
        try:

            # Recive the image
            image = self.bridge.imgmsg_to_cv2(msg)

            # Convert the image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Publish the converted gray image (with 'mono8' encoding)
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
            
        except CvBridgeError as e:
            self.get_logger().error('Bridge error: "%s"' % e)

# Main function (entry point)
def main(args = None):
    rclpy.init(args = args)
    converter = GrayScaleConverter()
    rclpy.spin(converter)

    # Destroy the node explicitly
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
