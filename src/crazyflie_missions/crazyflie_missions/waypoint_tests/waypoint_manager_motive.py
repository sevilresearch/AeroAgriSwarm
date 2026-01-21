#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Empty
import tf2_ros


class WaypointManagerMotive(Node):
    """ROS 2 waypoint controller for Crazyflie using Motive mocap pose."""

    def __init__(self):
        super().__init__("waypoint_manager_motive")

        # --- Publishers & Subscribers ---
        self.pose_sub = self.create_subscription(
            PoseStamped, "/Robot_2/pose", self.pose_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cf/cmd_vel", 10)
        self.takeoff_pub = self.create_publisher(Empty, "/cf/takeoff", 10)
        self.land_pub = self.create_publisher(Empty, "/cf/land", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- State ---
        self.current_pose = None
        self.prev_pose = None
        self.filtered_pose = None
        self.alpha = 0.5  # LPF smoothing factor (0.1–0.7)
        self.last_pose_time = None

        # --- Waypoints (+x forward, +y left, +z up) ---
        self.waypoints = [
            [0.0, 0.0, 0.5],
            [0.5, 0.0, 0.5],
            [0.5, -0.5, 0.5],
            [0.0, -0.5, 0.5],
            [0.0, 0.0, 0.5],
            [0.0, 0.0, 0.0],
        ]

        # --- Control parameters ---
        self.kp = 1.0
        self.max_vel = 0.3
        self.tolerance = 0.12
        self.max_jump = 0.5  # reject mocap glitches
        self.pose_timeout = 1.0
        self.rate_hz = 20.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)
        self.executing = False
        self.mission_done = False  # <-- mission complete flag
        self.target_index = 0

        # For simple throttling
        self.last_log_time = {}

        self.get_logger().info("WaypointManagerMotive initialized and waiting for takeoff...")

    # ----------------------------------------------------------------------
    def log_throttle(self, key, level, msg, interval=1.0):
        """Manual throttled logging"""
        now = time.time()
        last = self.last_log_time.get(key, 0.0)
        if now - last > interval:
            self.last_log_time[key] = now
            if level == "info":
                self.get_logger().info(msg)
            elif level == "warn":
                self.get_logger().warn(msg)

    # ----------------------------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        """Reject mocap jumps and update current pose."""
        now = time.time()

        if self.prev_pose is not None:
            dx = msg.pose.position.x - self.prev_pose.pose.position.x
            dy = msg.pose.position.y - self.prev_pose.pose.position.y
            dz = msg.pose.position.z - self.prev_pose.pose.position.z
            jump = math.sqrt(dx*dx + dy*dy + dz*dz)
            if jump > self.max_jump:
                self.log_throttle("mocap_jump", "warn", f"Ignoring mocap jump Δ={jump:.2f} m", 1.0)
                return

        # Apply simple exponential smoothing (LPF)
        if self.filtered_pose is None:
            self.filtered_pose = msg
        else:
            fp = self.filtered_pose.pose.position
            mp = msg.pose.position
            fp.x = fp.x + self.alpha * (mp.x - fp.x)
            fp.y = fp.y + self.alpha * (mp.y - fp.y)
            fp.z = fp.z + self.alpha * (mp.z - fp.z)
            self.filtered_pose.pose.position = fp
            self.filtered_pose.pose.orientation = msg.pose.orientation  # update orientation directly
        
        self.prev_pose = msg
        self.current_pose = self.filtered_pose
        self.last_pose_time = now

    # ----------------------------------------------------------------------
    def broadcast_tf(self):
        """Broadcast mocap pose for visualization (optional)."""
        if not self.current_pose:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "cf"
        t.transform.translation.x = self.current_pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.position.z
        t.transform.rotation = self.current_pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    # ----------------------------------------------------------------------
    def compute_velocity(self, target):
        """Compute simple proportional velocity toward target."""
        dx = target[0] - self.current_pose.pose.position.x
        dy = target[1] - self.current_pose.pose.position.y
        dz = target[2] - self.current_pose.pose.position.z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        speed_scale = max(0.1, min(1.0, dist / 0.3))
        vx, vy, vz = self.kp * dx * speed_scale, self.kp * dy * speed_scale, self.kp * dz * speed_scale

        mag = math.sqrt(vx*vx + vy*vy + vz*vz)
        if mag > self.max_vel:
            s = self.max_vel / mag
            vx, vy, vz = vx * s, vy * s, vz * s

        return vx, vy, vz, dist

    # ----------------------------------------------------------------------
    def start_mission(self):
        """Start mission after pose received."""
        if self.executing or self.mission_done:
            return
        if self.current_pose is None:
            self.get_logger().warn("No mocap pose yet — waiting before starting mission.")
            return

        self.executing = True
        self.get_logger().info("Publishing takeoff...")
        self.takeoff_pub.publish(Empty())
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Starting waypoint mission...")

    # ----------------------------------------------------------------------
    def loop(self):
        """Main control loop (runs at 20 Hz)."""
        self.broadcast_tf()

        if not self.executing:
            # Start once pose is valid
            if self.current_pose is not None and not self.mission_done:
                self.start_mission()
            return

        # Safety: pose timeout check
        if self.current_pose is None or (time.time() - (self.last_pose_time or 0)) > self.pose_timeout:
            self.log_throttle("pose_timeout", "warn", "Pose timeout — holding position", 2.0)
            # Keep last valid filtered pose for a short while
            if self.filtered_pose is not None:
                self.current_pose = self.filtered_pose
            else:
                self.cmd_pub.publish(Twist())
                return

        # Mission complete?
        if self.target_index >= len(self.waypoints):
            if not self.mission_done:
                self.get_logger().info("Mission complete — landing and shutting down.")
                self.land_pub.publish(Empty())
                self.mission_done = True  # prevent restarting
                # stop sending velocity commands
                self.cmd_pub.publish(Twist())
                time.sleep(5.0)  # give the drone time to land safely
                self.timer.cancel()  # stop the timer
                rclpy.shutdown()
            return


        target = self.waypoints[self.target_index]
        vx, vy, vz, dist = self.compute_velocity(target)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        self.cmd_pub.publish(cmd)

        self.log_throttle(
            "waypoint_status",
            "info",
            f"Target {self.target_index+1}/{len(self.waypoints)}: {target} | "
            f"Pos=({self.current_pose.pose.position.x:.2f}, "
            f"{self.current_pose.pose.position.y:.2f}, "
            f"{self.current_pose.pose.position.z:.2f}) | "
            f"Dist={dist:.2f} | Vel=({vx:.2f},{vy:.2f},{vz:.2f})",
            1.0,
        )

        if dist < self.tolerance:
            self.get_logger().info(f"Reached waypoint {self.target_index+1}")
            self.cmd_pub.publish(Twist())
            self.target_index += 1
            time.sleep(0.5)  # pause before next

# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerMotive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt — stopping mission.")
        node.land_pub.publish(Empty())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
