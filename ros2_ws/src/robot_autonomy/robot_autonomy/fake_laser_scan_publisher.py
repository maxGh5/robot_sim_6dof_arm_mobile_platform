import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FakeLaserScanPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_laser_scan_publisher")
        self.declare_parameter("frame_id", "laser_front")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_rate_hz", 5.0)
        self.declare_parameter("num_points", 360)
        self.declare_parameter("range_m", 2.0)

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self._scan_rate_hz = self.get_parameter("scan_rate_hz").get_parameter_value().double_value
        self._num_points = int(self.get_parameter("num_points").get_parameter_value().integer_value)
        self._range_m = self.get_parameter("range_m").get_parameter_value().double_value

        if self._scan_rate_hz <= 0.0:
            self.get_logger().warn("scan_rate_hz <= 0.0, defaulting to 5.0 Hz")
            self._scan_rate_hz = 5.0

        if self._num_points < 2:
            self.get_logger().warn("num_points < 2, defaulting to 360 points")
            self._num_points = 360

        self._publisher = self.create_publisher(LaserScan, self._scan_topic, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / self._scan_rate_hz, self._publish_scan)

        self.get_logger().info(
            "Publishing fake LaserScan on %s with %d points at %.2f Hz",
            self._scan_topic,
            self._num_points,
            self._scan_rate_hz,
        )

    def _build_ranges(self) -> List[float]:
        return [self._range_m for _ in range(self._num_points)]

    def _publish_scan(self) -> None:
        now = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self._frame_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(self._num_points - 1)
        scan.scan_time = 1.0 / self._scan_rate_hz
        scan.time_increment = scan.scan_time / float(self._num_points)
        scan.range_min = 0.05
        scan.range_max = max(self._range_m, scan.range_min + 0.01)
        scan.ranges = self._build_ranges()
        scan.intensities = [1.0 for _ in range(self._num_points)]

        self._publisher.publish(scan)


def main() -> None:
    rclpy.init()
    node = FakeLaserScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
