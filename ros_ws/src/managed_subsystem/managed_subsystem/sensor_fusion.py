import rclpy
from typing import List
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from system_interfaces.msg import SensorFusion
from system_interfaces.msg import FusionType
import os
from rclpy.lifecycle import LifecycleState

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.sensor_fusion_config import comm_types

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
import time

class SensorFusionNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "sensor_fusion",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the SensorFusionNode class.

        Parameters:
        comm_types (List): A list containing the communication types configuration.
        node_name (str): The name of the node. Defaults to "sensor_fusion_node".
        param_file (str): The parameter file path. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.topic_camera_input = None 
        self.modality = None
        self.delta_t_threshold = None
        self.image_shift = None
        self.do_recalibration = None
        self.do_drop_sensor_fusion = None # should resolve after restart
        self.do_hard_drop_sensor_fusion = None # should resolve after redeploy
        super().__init__(node_name, comm_types, config_file, namespace="/managed_subsystem")

        self.trigger_configure()
        self.trigger_activate()

        self.data_buffer = {
            "depth": [Image(), -1],
            "rgb": [Image(), -1],
        }

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def on_configure(self, state: LifecycleState):
        time.sleep(1)
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState):
        self.do_drop_sensor_fusion = False
        return super().on_activate(state)

    def rgb_callback(self, image: Image) -> None:
        """
        Callback function for RGB image data.

        Parameters:
        image (Image): The RGB image message.
        """
        self.add_to_data_buffer(image, "rgb")

    def depth_callback(self, image: Image) -> None:
        """
        Callback function for depth image data.

        Parameters:
        image (Image): The depth image message.
        """
        self.add_to_data_buffer(image, "depth")

    def add_to_data_buffer(self, image: Image, key: str) -> None:
        """
        Adds incoming image data to the data buffer.

        Parameters:
        image (Image): The image message.
        key (str): The data type key ('rgb' or 'depth').
        """
        time_code = image.header.stamp.sec + image.header.stamp.nanosec / 1e9
        self.data_buffer[key] = [image, time_code]

        if self.modality == FusionType.FUSION:
            if self.check_for_full_batch():
                self.publish_fused_data()
        else:
            self.publish_fused_data()

    def check_for_full_batch(self) -> bool:
        """
        Checks if there is almost the same time stamp on both data sources.
        True by default, if only one data source is used.

        Returns:
        bool: True if the time difference is within the threshold, False otherwise.
        """
        if self.modality == FusionType.FUSION:
            return (
                abs(self.data_buffer["rgb"][1] - self.data_buffer["depth"][1])
                <= self.delta_t_threshold
            )
        return True

    def shift_image_right(self, cv_image: np.ndarray) -> np.ndarray:
        # Get image dimensions
        height, width = cv_image.shape[:2]

        # Create a new array filled with zeros
        shifted_image = np.zeros_like(cv_image)

        # Shift the image
        if self.image_shift < width: # type: ignore
            shifted_image[:, self.image_shift:] = cv_image[:, :-self.image_shift] # type: ignore

        return shifted_image

    def process_ros_image_message(self, ros_image: Image) -> Image:
        # Convert ROS Image to CV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

        # Shift the image to the right
        shifted_cv_image = self.shift_image_right(cv_image)

        # Convert back to ROS Image message
        ros_shifted_image = bridge.cv2_to_imgmsg(shifted_cv_image, encoding="bgr8")

        return ros_shifted_image

    def publish_fused_data(self) -> None:
        """
        Publishes the fused sensor data.
        """
        if self.do_drop_sensor_fusion or self.do_hard_drop_sensor_fusion:
            return

        if self.do_recalibration:
            self.logger.info("Sensor fusion is doing recalibration")
            self.image_shift = 0
            self.do_recalibration = False
        header = Header()
        header.frame_id = f"{self.ns}/sensors_fused"
        if self.modality == FusionType.DEPTH:
            header.stamp = self.data_buffer["depth"][0].header.stamp
        else:
            header.stamp = self.data_buffer["rgb"][0].header.stamp

        out_msg = SensorFusion(
            depth=self.data_buffer["depth"][0],
            rgb=self.data_buffer["rgb"][0] if self.image_shift == 0 else self.process_ros_image_message(self.data_buffer["rgb"][0]),
            modality=FusionType(fusion_type=self.modality),
            header=header,
        )

        publisher = self.get_comm_object("/sensors_fused", comm_type=CommunicationTypes.PUBLISHER)
        publisher.publish(out_msg) # type: ignore



def main() -> None:
    """
    Main function to initiate the SensorFusionNode.
    """
    rclpy.init()
    test = SensorFusionNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
