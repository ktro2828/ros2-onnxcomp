from __future__ import annotations

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.topic_endpoint_info import QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


class OnnxCompareNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        # ROS 2 parameters
        descriptor = ParameterDescriptor(dynamic_typing=True)

        self._onnx_path = (
            self.declare_parameter("onnx_path", descriptor)
            .get_parameter_value()
            .string_value
        )
        self._layer_name = (
            self.declare_parameter("layer_name", descriptor)
            .get_parameter_value()
            .string_value
        )

        self._use_raw = (
            self.declare_parameter("use_raw", False, descriptor)
            .get_parameter_value()
            .bool_value
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._image_subscription = self.create_subscription(
            Image if self._use_raw else CompressedImage,
            "~/input/image",
            self.callback,
            qos_profile,
        )

    def callback(self, msg: Image | CompressedImage) -> None:
        if self._use_raw:
            pass
        else:
            pass

    def visualize(self, image, results) -> None:
        pass
