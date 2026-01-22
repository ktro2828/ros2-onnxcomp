from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.topic_endpoint_info import QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from torchmetrics.regression import CosineSimilarity, JensenShannonDivergence

from onnxcomp.onnxcomp.inference import InferenceModel
from onnxcomp.onnxcomp.scorer import Scorer


class OnnxCompare2on1Node(Node):
    def __init__(self) -> None:
        super().__init__("compare_2on1_node")
        # ROS 2 parameters
        descriptor = ParameterDescriptor(dynamic_typing=True)

        onnx1 = (
            self.declare_parameter("onnx1", descriptor=descriptor)
            .get_parameter_value()
            .string_value
        )
        onnx2 = (
            self.declare_parameter("onnx2", descriptor=descriptor)
            .get_parameter_value()
            .string_value
        )

        self._use_raw = (
            self.declare_parameter("use_raw", descriptor=descriptor)
            .get_parameter_value()
            .bool_value
        )

        self._model1 = InferenceModel(onnx1)
        self._model2 = InferenceModel(onnx2)

        self._cossim = CosineSimilarity()
        self._jsd = JensenShannonDivergence()

        self._cossim_scorer = Scorer("Cosine Similarity")
        self._jsd_scorer = Scorer("Jensen-Shannon Divergence")

        # cv bridge
        self._cv_bridge = CvBridge()

        # subscription & publisher
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

        self._cossim_publisher = self.create_publisher(
            Float64, "~/output/score/cosine_similarity", 1
        )
        self._jsd_publisher = self.create_publisher(
            Float64, "~/output/score/jensen_shannon_divergence", 1
        )

    def callback(self, msg: Image | CompressedImage) -> None:
        if self._use_raw:
            # convert Image to cv2 image
            image = self._cv_bridge.imgmsg_to_cv2(msg)
        else:
            # convert CompressedImage to cv2 image
            image = self._cv_bridge.compressed_imgmsg_to_cv2(msg)

        p1 = self._model1(image)
        p2 = self._model2(image)

        cossim_score = self._cossim(p1, p2)
        jsd_score = self._jsd(p1, p2)

        self._cossim_scorer.add_score(cossim_score)
        self._jsd_scorer.add_score(jsd_score)

        self.get_logger().info(f"{self._cossim_scorer.metric}: {cossim_score:.4f}")
        self.get_logger().info(f"{self._jsd_scorer.metric}: {jsd_score:.4f}")

        cossim_score_msg = Float64()
        cossim_score_msg.data = cossim_score
        self._cossim_publisher.publish(cossim_score_msg)

        jsd_score_msg = Float64()
        jsd_score_msg.data = jsd_score
        self._jsd_publisher.publish(jsd_score_msg)

    def summarize(self) -> None:
        """Summarize the results."""
        self._cossim_scorer.print_summary()
        self._jsd_scorer.print_summary()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OnnxCompare2on1Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.summarize()

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
