#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class WaypointManager(Node):
    """ROS 2 waypoint controller for Crazyflie using velocity commands in m/s."""

    def __init__(self):
        super().__init__("waypoint_manager")

        # --- Publishers ---
        self.takeoff_pub = self.create_publisher(Empty, "/cf/takeoff", 10)
        self.land_pub = self.create_publisher(Empty, "/cf/land", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cf/cmd_vel", 10)

        # --- Waypoints: [vx, vy, vz, duration] --- (gentle speeds)
        self.waypoints = [
            [0.2,  0.0, 0.0, 2.0],  # forward
            [0.0,  0.2, 0.0, 2.0],  # left
            [-0.2, 0.0, 0.0, 2.0],  # back
            [0.0, -0.2, 0.0, 2.0],  # right
        ]

        self.current_index = -1
        self.start_time = None
        self.active_cmd = None

        # --- Begin mission: takeoff then wait more to allow estimator ---
        self.get_logger().info("Publishing takeoff...")
        self.takeoff_pub.publish(Empty())
        # give more time for estimator + motion commander to reach height
        time.sleep(5.0)

        # Run main loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    # --- Timer callback ---
    def loop(self):
        # No active command: move to next waypoint
        if self.active_cmd is None:
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.get_logger().info("All waypoints complete → landing.")
                self.land_pub.publish(Empty())
                self.timer.cancel()
                self.destroy_node()
                return

            vx, vy, vz, dur = self.waypoints[self.current_index]
            self.active_cmd = Twist()
            self.active_cmd.linear.x = vx
            self.active_cmd.linear.y = vy
            self.active_cmd.linear.z = vz
            self.start_time = time.time()
            self.duration = dur

            self.get_logger().info(
                f"Executing waypoint {self.current_index + 1}/{len(self.waypoints)}: "
                f"vx={vx}, vy={vy}, vz={vz}, dur={dur}s"
            )

        # Publish current command
        self.cmd_pub.publish(self.active_cmd)

        # Check if time’s up
        if time.time() - self.start_time >= self.duration:
            # Stop motion briefly
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.active_cmd = None
            time.sleep(1.0)  # short pause between segments


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
