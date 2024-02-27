
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# for image conversions
from cv_bridge import CvBridge
from PIL import Image

# For depth pipeline
from transformers import pipeline

class DepthAnything(Node):
    """
    Class for converting RGB images to depth using the Depth-Anything model.
    """
    def __init__(self) -> None:
        super().__init__("depth_anything")
         self._subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self._subscription  # prevent unused variable warning
        self._publisher = self.create_publisher(Image, '/processed_image', 10)

        self.bridge = CvBridge()

        # TODO: param for model path
        self._depth_anything = pipeline(task="depth-estimation", model="LiheYoung/depth-anything-small-hf")

        self.get_logger().info("DepthAnything initialized")


    def image_callback(self, msg):
        """Convert the input RGB image into a depth image and republish it."""
        try:
            # Convert ROS Image message to PIL Image
            pil_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            pil_image = PILImage.fromarray(np.uint8(pil_image))

            # Process the image using the defined function
            depth = self._depth_anything(cv_image)["depth"]

            # Convert the processed image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(depth, encoding='passthrough')

            # Publish the processed image
            self.publisher.publish(processed_image_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    image_processing_node = ImageProcessingNode()
    rclpy.spin(image_processing_node)
    image_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()