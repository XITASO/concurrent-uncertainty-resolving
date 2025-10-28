# image_difference_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from datetime import datetime
from typing import Dict, Optional, Tuple

class ImageDifferenceCalculator(Node):
    def __init__(self) -> None:
        super().__init__('image_difference_calculator')
        self.bridge: CvBridge = CvBridge()
        # Initialize images and their timestamps
        self.images: Dict[str, Optional[np.ndarray]] = {
            'topic1': None,
            'topic2': None,
            'topic3': None
        }
        self.timestamps: Dict[str, Optional[datetime]] = {
            'topic1': None,
            'topic2': None,
            'topic3': None
        }
        self.create_subscription(Image, '/rgb_raw', lambda msg: self.callback(msg, 'topic1'), 10)
        self.create_subscription(Image, '/managed_subsystem/rgb_camera', lambda msg: self.callback(msg, 'topic2'), 10)
        self.create_subscription(Image, '/managed_subsystem/rgb_enhanced', lambda msg: self.callback(msg, 'topic3'), 10)

    def callback(self, msg: Image, topic_name: str) -> None:
        try:
            cv_image: np.ndarray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.images[topic_name] = cv_image
            # Extract timestamp from the header and store it
            self.timestamps[topic_name] = self.get_ros_time_as_datetime(msg.header.stamp)

            self.check_and_compare_images()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def get_ros_time_as_datetime(self, stamp) -> datetime:
        # Convert ROS2 time to a datetime object
        return datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9)

    def check_and_compare_images(self) -> None:
        # Check if all images and timestamps are available
        if all(image is not None for image in self.images.values()) and all(self.timestamps.values()):
            # Ensure timestamps are close enough to each other (within a threshold)
            time_diff_1_to_2 = abs((self.timestamps['topic1'] - self.timestamps['topic2']).total_seconds())
            time_diff_2_to_3 = abs((self.timestamps['topic2'] - self.timestamps['topic3']).total_seconds())
            time_diff_3_to_1 = abs((self.timestamps['topic3'] - self.timestamps['topic1']).total_seconds())

            threshold = 0.0 # seconds; adjust as needed
            
            if (time_diff_1_to_2 <= threshold and time_diff_2_to_3 <= threshold and time_diff_3_to_1 <= threshold):
                # Calculate differences between the images
                diff1_to_2: np.ndarray = cv2.absdiff(self.images['topic1'], self.images['topic2'])
                diff2_to_3: np.ndarray = cv2.absdiff(self.images['topic2'], self.images['topic3'])
                diff3_to_1: np.ndarray = cv2.absdiff(self.images['topic3'], self.images['topic1'])

                self.get_logger().info(f"Difference between image 1 and 2: {np.sum(diff1_to_2)}")
                self.get_logger().info(f"Difference between image 2 and 3: {np.sum(diff2_to_3)}")
                self.get_logger().info(f"Difference between image 3 and 1: {np.sum(diff3_to_1)}")

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: ImageDifferenceCalculator = ImageDifferenceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
