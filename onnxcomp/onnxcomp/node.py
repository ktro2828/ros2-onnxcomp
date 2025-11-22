from __future__ import annotations

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from numpy._typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.topic_endpoint_info import QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


class OnnxCompareNode(Node):
    def __init__(self) -> None:
        super().__init__("onnx_compare_node")
        # ROS 2 parameters
        descriptor = ParameterDescriptor(dynamic_typing=True)

        self._onnx_path = (
            self.declare_parameter("onnx_path", descriptor=descriptor)
            .get_parameter_value()
            .string_value
        )
        self._use_raw = (
            self.declare_parameter("use_raw", descriptor=descriptor)
            .get_parameter_value()
            .bool_value
        )

        # onnxruntime session
        self._session = ort.InferenceSession(
            self._onnx_path, providers=["CPUExecutionProvider", "CUDAExecutionProvider"]
        )
        self._in_shape = self._session.get_inputs()[0].shape
        self._in_name = self._session.get_inputs()[0].name
        self._out_names = [o.name for o in self._session.get_outputs()]

        # cv bridge
        self._cv_bridge = CvBridge()

        # subscription & publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._input_subscription = self.create_subscription(
            Image if self._use_raw else CompressedImage,
            "~/input/image",
            self.callback,
            qos_profile,
        )

        self._feature_publisher = self.create_publisher(
            Image,
            "~/output/feature",
            qos_profile,
        )

    def callback(self, msg: Image | CompressedImage) -> None:
        image = self.preprocess(msg)

        # inference
        *_, feature = self._session.run(self._out_names, {self._in_name: image})
        feature = np.asarray(feature)

        # publish feature map as Image
        feature = self.postprocess(feature)
        feature_msg = self._cv_bridge.cv2_to_imgmsg(feature)
        self._feature_publisher.publish(feature_msg)

    def preprocess(self, msg: Image | CompressedImage) -> NDArray:
        """Preprocess input image.

        Args:
            msg (Image | CompressedImage): Input image message.

        Returns:
            Preprocessed image as a NumPy array.
        """
        if self._use_raw:
            # convert Image to cv2 image
            image = self._cv_bridge.imgmsg_to_cv2(msg)
        else:
            # convert CompressedImage to cv2 image
            image = self._cv_bridge.compressed_imgmsg_to_cv2(msg)

        # preprocess
        image = cv2.resize(image, (self._in_shape[3], self._in_shape[2]))
        image = image.astype(np.float32) / 255.0
        image = np.expand_dims(image, axis=0)
        image = np.transpose(image, (0, 3, 1, 2))
        return image

    def postprocess(self, feature: NDArray) -> NDArray:
        """Postprocess feature map.

        Args:
            feature (NDArray): Feature map as a NumPy array.

        Returns:
            Postprocessed feature map as a NumPy array.
        """
        # (1, C, H, W) -> (H, W, C) -> (H, W)
        feature = feature[0].transpose(1, 2, 0).mean(axis=2)
        feature = cv2.normalize(feature, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        return feature


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OnnxCompareNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
