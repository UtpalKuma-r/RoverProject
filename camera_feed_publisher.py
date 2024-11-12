import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
import time

class PublisherNodeClass(Node):
    def __init__(self, camera_indexes):
        super().__init__('publisher_node')
        self.camera_indexes = camera_indexes
        self.camera_index = self.camera_indexes[0]  # Start with the first camera
        self.camera = self.open_camera(self.camera_index)
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.02
        self.timer = self.create_timer(self.periodCommunication, self.timer_callback_function)
        self.i = 0

        # Create a service for switching cameras
        self.srv = self.create_service(SetBool, 'switch_camera', self.switch_camera_callback)

    def open_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open camera {index}.")
            return None
        self.get_logger().info(f"Camera {index} opened successfully.")
        return cap

    def timer_callback_function(self):
        if self.camera is None:
            self.get_logger().error("Camera is not initialized. Exiting callback.")
            return

        success, frame = self.camera.read()
        if success:
            frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)
            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info('Publishing image number %d' % self.i)
            self.i += 1
        else:
            self.get_logger().error("Failed to read frame from camera.")
        time.sleep(.5)

    def switch_camera_callback(self, request, response):
        if self.camera is not None:
            self.camera.release()

        # Switch to the next camera
        self.camera_index = self.camera_indexes[0] if self.camera_index == self.camera_indexes[1] else self.camera_indexes[1] 
        self.camera = self.open_camera(self.camera_index)

        if self.camera is None:
            response.success = False
            response.message = f'Failed to switch to camera {self.camera_index}'
        else:
            response.success = True
            response.message = f'Switched to camera {self.camera_index}'
            time.sleep(0.1)  # Short delay to stabilize

        return response

def main(args=None):
    rclpy.init(args=args)

    # Discover available cameras
    camera_indexes = []
    for index in range(10):  # Check the first 10 camera indices
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            camera_indexes.append(index)
            cap.release()
    print(camera_indexes)

    if len(camera_indexes) < 2:
        print("Connect cameras")
        return

    publisherNode = PublisherNodeClass(camera_indexes)
    
    rclpy.spin(publisherNode)
    publisherNode.camera.release()
    publisherNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
