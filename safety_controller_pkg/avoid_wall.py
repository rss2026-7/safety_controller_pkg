#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools

LOOKAHEAD_SCALE = 0.5
LOOKAHEAD_MIN = 0.5
LOOKAHEAD_MAX = 2.0


def ransac_fit(x, y, n_iters=50, threshold=0.1):
    best_inliers = 0
    best_m, best_b = 0, 0
    for _ in range(n_iters):
        idx = np.random.choice(len(x), 2, replace=False)
        dx = x[idx[1]] - x[idx[0]]
        if abs(dx) < 1e-9:
            continue
        m = (y[idx[1]] - y[idx[0]]) / dx
        b = y[idx[0]] - m * x[idx[0]]
        dists = np.abs(y - m * x - b) / np.sqrt(m**2 + 1)
        inliers = np.sum(dists < threshold)
        if inliers > best_inliers:
            best_inliers = inliers
            best_m, best_b = m, b
    # Refit on inliers only
    dists = np.abs(y - best_m * x - best_b) / np.sqrt(best_m**2 + 1)
    mask = dists < threshold
    if np.sum(mask) >= 2:
        best_m, best_b = np.polyfit(x[mask], y[mask], 1)
    return best_m, best_b


class WallAvoid(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
      

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS!
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.wall_pub = self.create_publisher(Marker, "/wall_marker", 10)
        self.fit_pub = self.create_publisher(Marker, "/fit_marker", 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

    def laser_callback(self, data: LaserScan):
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

        # Select angular window based on SIDE (1 = left, -1 = right)
        # Wide window so RANSAC can see approaching corners
        if self.SIDE == 1:
            mask = (angles > np.radians(10)) & (angles < np.radians(135))
        else:
            mask = (angles > np.radians(-135)) & (angles < np.radians(-10))

        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        # Filter out invalid ranges (inf, NaN, zero)
        valid = np.isfinite(filtered_ranges) & (filtered_ranges > 0.01)
        filtered_ranges = filtered_ranges[valid]
        filtered_angles = filtered_angles[valid]

        if len(filtered_ranges) < 2:
            return

        x = filtered_ranges * np.cos(filtered_angles)
        y = filtered_ranges * np.sin(filtered_angles)

        # Calculate y = mx + b line (RANSAC)
        m, b = ransac_fit(x, y)

        # Visualize raw wall points and RANSAC fit
        VisualizationTools.plot_line(x, y, self.wall_pub, color=(0.0, 1.0, 1.0))

        fit_x = np.linspace(x.min(), x.max(), 50)
        fit_y = m * fit_x + b
        VisualizationTools.plot_line(fit_x, fit_y, self.fit_pub, color=(1.0, 0.0, 1.0))

        # PD wall following
        wall_dist = abs(b) / np.sqrt(1 + m**2)
        error = wall_dist - self.DESIRED_DISTANCE
        wall_angle = np.arctan(m)

        Kp = 2.0
        Kd = 0.8
        steering_angle = self.SIDE * Kp * error + Kd * wall_angle

        # Corner handling: check for walls in a forward cone
        L = np.clip(abs(self.VELOCITY) * LOOKAHEAD_SCALE, LOOKAHEAD_MIN, LOOKAHEAD_MAX)
        fwd_cone = np.abs(angles) < np.radians(25)
        fwd_ranges = ranges[fwd_cone]
        fwd_valid = fwd_ranges[np.isfinite(fwd_ranges) & (fwd_ranges > 0.01)]
        if len(fwd_valid) > 0:
            forward_dist = np.min(fwd_valid)
            if forward_dist < L * 4.0:
                urgency = np.sqrt(1.0 - forward_dist / (L * 4.0))
                corner_steer = -self.SIDE * urgency * 0.34
                # Override PD when corner is urgent — don't let PD fight the turn
                steering_angle = urgency * corner_steer + (1.0 - urgency) * steering_angle

        steering_angle = np.clip(steering_angle, -0.34, 0.34)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = self.VELOCITY
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.drive_pub.publish(msg)

    


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
