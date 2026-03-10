#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WallDistancePlotter(Node):
    def __init__(self):
        super().__init__('plot_wall_distance')

        self.desired_dist = 0.50

        self.t = []
        self.d = []
        self.t0 = self.get_clock().now()

        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.get_logger().info("plot_wall_distance started. Recording right-wall distance from /scan...")

    @staticmethod
    def _angle_to_index(angle, scan):
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        return int(np.clip(idx, 0, len(scan.ranges) - 1))

    @staticmethod
    def _sanitize_ranges(scan):
        r = np.array(scan.ranges, dtype=float)
        rmax = scan.range_max if scan.range_max > 0 else 10.0
        rmin = scan.range_min if scan.range_min > 0 else 0.05
        r[np.isinf(r)] = rmax
        r[np.isnan(r)] = rmax
        return np.clip(r, rmin, rmax)

    def on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return

        ranges = self._sanitize_ranges(scan)

        # No TF: assume forward is 0 rad, right is -pi/2
        right_angle = -math.pi / 2.0

        # Use a small window around right for robustness (min over ~8 degrees)
        side_window_rad = math.radians(8.0)
        side_window = max(1, int(side_window_rad / max(scan.angle_increment, 1e-6)))

        idx_center = self._angle_to_index(right_angle, scan)
        j0 = max(idx_center - side_window, 0)
        j1 = min(idx_center + side_window + 1, len(ranges))
        dist = float(np.min(ranges[j0:j1]))

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        self.t.append(t)
        self.d.append(dist)

    def finalize(self):
        if len(self.d) < 5:
            print("Not enough samples")
            return

        d = np.array(self.d)
        t = np.array(self.t)

        p2p = d.max() - d.min()
        abs_err = np.abs(d - self.desired_dist)

        print("\n=== Wall Distance Oscillation Stats ===")
        print(f"samples: {len(d)}")
        print(f"peak-to-peak (m): {p2p:.3f}")
        print(f"max |d - desired| (m): {abs_err.max():.3f}")
        print(f"mean |d - desired| (m): {abs_err.mean():.3f}")
        print("======================================")

        plt.figure()
        plt.plot(t, d)
        plt.xlabel("time (s)")
        plt.ylabel("right wall distance (m)")
        plt.grid(True)
        plt.savefig("wall_distance_over_time.png", dpi=200)
        print("Saved wall_distance_over_time.png")


def main():
    rclpy.init()
    node = WallDistancePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
