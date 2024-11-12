import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
import time

class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.subscription = self.create_subscription(
            Image, self.topicNameFrames, self.listener_callback_function, self.queueSize
        )

        # Create a client for the switch camera service
        self.client = self.create_client(SetBool, 'switch_camera')
        self.i = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # OpenCV window
        #cv2.namedWindow("Camera Video", cv2.WINDOW_AUTOSIZE)
        self.switch_in_progress = False  # Flag to manage switch state
        #self.cameraDeviceNumber = 0  # Initial camera index
        #self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

    def listener_callback_function(self, imageMessage):
        self.get_logger().info(f'The image frame is received {self.i}')
        self.i += 1
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        cv2.imshow("Camera Video", openCVImage)

        # Check for key presses
        key = cv2.waitKey(1)  # Wait for 1 millisecond
        if key == 9:  # ASCII code for Tab key
            if not self.switch_in_progress:  # Prevent simultaneous switches
                self.switch_camera()  # Switch camera when Tab is pressed

    def switch_camera(self):
        self.switch_in_progress = True
        request = SetBool.Request()
        request.data = True
        future = self.client.call_async(request)
        try:
            # Optionally, you can also set a timeout on the future
            result = future.result(timeout_sec=5.0)
            if result.success:
                self.get_logger().info(f"Service call succeeded: {result.message}")
            else:
                self.get_logger().error(f"Service call failed: {result.message}")
        except Exception as TimeoutException:
            self.get_logger().error("Service call timed out!")
        self.switch_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    subscriberNode = SubscriberNodeClass()

    try:
        # Enter a spinning loop to process subscriptions and key events
        rclpy.spin(subscriberNode)
    except KeyboardInterrupt:
        pass
    finally:
        # Release the camera and destroy the OpenCV window
        #subscriberNode.camera.release()
        cv2.destroyAllWindows()  # Close OpenCV window
        subscriberNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
