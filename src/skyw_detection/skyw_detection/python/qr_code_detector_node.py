#!/usr/bin/env python3

"""ROS 2 node: QR decode via pyzbar with grayscale + threshold (tutorial-style)."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QrCodeDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("qrcode_detector")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("decoded_topic", "/qr_decoded")
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("binary_threshold", 45)
        self.declare_parameter("publish_only_on_change", True)

        self._bridge = CvBridge()
        self._last_decoded: str | None = None

        cam = str(self.get_parameter("camera_topic").value)
        self._enable_viz = bool(self.get_parameter("enable_visualization").value)
        self._thresh = int(self.get_parameter("binary_threshold").value)
        self._publish_only_on_change = bool(self.get_parameter("publish_only_on_change").value)

        out_topic = str(self.get_parameter("decoded_topic").value)
        self._pub = self.create_publisher(String, out_topic, 10)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, cam, self._on_image, qos)
        self.get_logger().info(f"QR detector: camera={cam} publish={out_topic}")

    def _on_image(self, msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self._thresh, 255, cv2.THRESH_BINARY)

        for obj in decode(thresh):
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([p for p in points], dtype=np.float32))
                points = hull.reshape(-1, 2)
            for j in range(len(points)):
                cv2.line(
                    image,
                    tuple(points[j]),
                    tuple(points[(j + 1) % len(points)]),
                    (0, 255, 0),
                    3,
                )

            x, y = obj.rect.left, obj.rect.top
            try:
                qr_data = obj.data.decode("utf-8")
            except (AttributeError, UnicodeDecodeError):
                qr_data = str(obj.data)

            publish = (not self._publish_only_on_change) or (qr_data != self._last_decoded)
            if publish:
                m = String()
                m.data = qr_data
                self._pub.publish(m)
                self._last_decoded = qr_data
                self.get_logger().info(f"decoded: {qr_data}")

            cv2.putText(
                image,
                qr_data,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        if self._enable_viz:
            # Scale down for a smaller, more compact GUI window
            display_scale = 0.5
            width = int(image.shape[1] * display_scale)
            height = int(image.shape[0] * display_scale)
            small_img = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            
            cv2.imshow("Camera output", small_img)
            cv2.waitKey(1)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QrCodeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
