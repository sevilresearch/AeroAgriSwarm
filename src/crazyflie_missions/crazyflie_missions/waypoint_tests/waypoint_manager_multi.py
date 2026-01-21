#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty


class WaypointManagerMulti(Node):
    """
    Waypoint manager for multiple Crazyflies using mocap poses.
    Compatible with high-level API bridge (crazyflie_bridge_multi.py).
    """

    def __init__(self):
        super().__init__("waypoint_manager_multi")

        # Define both drones and their missions
        self.drones = {
            "Robot_1": {
                "pose": None,
                "target": 0,
                "executing": False,
                "mission_done": False,
                "waypoints": [
                    [0.0, 0.0, 0.5],
                    [0.5, 0.0, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.0, 0.5, 0.5],
                    [0.0, 0.0, 0.5],
                    [0.0, 0.0, 0.0],
                ],
            },
            "Robot_3": {
                "pose": None,
                "target": 0,
                "executing": False,
                "mission_done": False,
                "waypoints": [
                    [0.0, -0.3, 0.5],
                    [-0.5, -0.3, 0.5],
                    [-0.5, 1.0, 0.5],
                    [0.0, 1.0, 0.5],
                    [0.0, -0.3, 0.5],
                    [0.0, -0.3, 0.0],
                ],
            },
        }

        # --- PD controller parameters ---
        self.kp = 1.0            # proportional gain
        self.kd_xy = 0.4         # derivative xy movement gain
        self.kd_z = 0.3          # derivative landing gain (smoother landing)
        self.alt_hold_kp = 0.8   # altitude lock strength

        # --- Derivative-Low Pass Filter Alpha Smoothing Factor
        self.d_alpha = 0.25      # Lower --> Smoother

        # --- Separate Velocity Limits ---
        self.max_xy = 0.20
        self.max_z = 0.10

        # Initialize tracking loss parameters ---
        self.tolerance = 0.12    # waypoint tolerance in meters
        self.pose_timeout = 1.0  # lose tracking after 1s

        # --- Initialize Last timing and error parameters ---
        self.last_pose_time = {}
        self.last_err = {name: (0.0, 0.0, 0.0) for name in self.drones}
        self.last_time = {name: time.time() for name in self.drones}

        # --- Initialize Derivative Memory ---
        self.last_ddx = {name: 0.0 for name in self.drones}
        self.last_ddy = {name: 0.0 for name in self.drones}
        self.last_ddz = {name: 0.0 for name in self.drones}

        # Publishers/subscribers per drone
        for name in self.drones:
            self.create_subscription(PoseStamped, f"/{name}/pose",
                                     lambda msg, n=name: self.pose_cb(n, msg), 10)
            self.drones[name]["takeoff_pub"] = self.create_publisher(Empty, f"/{name}/takeoff", 10)
            self.drones[name]["land_pub"] = self.create_publisher(Empty, f"/{name}/land", 10)
            self.drones[name]["cmd_pub"] = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)

        # Run control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("✅ WaypointManagerMulti initialized.")

    # ----------------------------------------------------------------------
    def pose_cb(self, name, msg):
        """Store latest mocap pose for each robot."""
        self.drones[name]["pose"] = msg
        self.last_pose_time[name] = time.time()

    # ----------------------------------------------------------------------
    def compute_velocity(self, name, target):
        """Simple proportional velocity control with altitude hold."""
        pose = self.drones[name]["pose"].pose.position

        dx = target[0] - pose.x
        dy = target[1] - pose.y
        dz = target[2] - pose.z

        # Time delta
        now = time.time()
        dt = now - self.last_time.get(name, now)
        self.last_time[name] = now

        # init PD state
        if name not in self.last_err:
            self.last_err[name] = (dx, dy, dz)

        ldx, ldy, ldz = self.last_err[name]
        self.last_err[name] = (dx, dy, dz)

        # --- Derivative before filtering ---
        raw_ddx = (dx - ldx) / dt if dt > 0 else 0
        raw_ddy = (dy - ldy) / dt if dt > 0 else 0
        raw_ddz = (dz - ldz) / dt if dt > 0 else 0

        alpha = self.d_alpha
        max_xy = self.max_xy
        max_z  = self.max_z

        ddx = alpha * raw_ddx + (1 - alpha) * self.last_ddx[name]
        ddy = alpha * raw_ddy + (1 - alpha) * self.last_ddy[name]
        ddz = alpha * raw_ddz + (1 - alpha) * self.last_ddz[name]

        self.last_ddx[name] = ddx
        self.last_ddy[name] = ddy
        self.last_ddz[name] = ddz

        # ------ PD control ------
        vx = self.kp * dx + self.kd_xy * ddx
        vy = self.kp * dy + self.kd_xy * ddy
        vz = self.kp * dz * self.alt_hold_kp + self.kd_z * ddz

        # XY limit
        mag_xy = math.sqrt(vx*vx + vy*vy)
        if mag_xy > max_xy:
            s = max_xy / mag_xy
            vx *= s
            vy *= s

        # Z limit
        if vz > max_z:
            vz = max_z
        if vz < -max_z:
            vz = -max_z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return vx, vy, vz, dist

    # ----------------------------------------------------------------------
    def loop(self):
        """Main control loop for all drones."""
        now = time.time()

        # --- Shut down after drones complete mission and user does not have to press Ctrl+C ---
        if all(d["mission_done"] for d in self.drones.values()):
            self.get_logger().info("✅ All drone missions completed — shutting down node.")
            rclpy.get_global_executor().call_soon_threadsafe(rclpy.shutdown)
            rclpy.shutdown()

        for name, drone in self.drones.items():
            if drone["mission_done"]:
                continue  # Skip completed drones
            if drone["pose"] is None:
                continue  # Wait for mocap
            if now - self.last_pose_time.get(name, 0) > self.pose_timeout:
                self.get_logger().warn(f"{name}: Pose timeout — holding position.")
                drone["cmd_pub"].publish(Twist())
                continue

            # --- Start mission automatically after first pose ---
            if not drone["executing"]:
                self.get_logger().info(f"{name}: Taking off...")
                drone["takeoff_pub"].publish(Empty())
                drone["executing"] = True
                time.sleep(4.0)
                continue

            # --- Mission complete? ---
            if drone["target"] >= len(drone["waypoints"]):
                self.get_logger().info(f"{name}: Mission complete — landing.")
                drone["land_pub"].publish(Empty())
                drone["mission_done"] = True
                continue

            # --- Active waypoint control ---
            target = drone["waypoints"][drone["target"]]
            vx, vy, vz, dist = self.compute_velocity(name, target)

            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
            drone["cmd_pub"].publish(cmd)

            # Print occasional progress
            if dist < self.tolerance:
                self.get_logger().info(f"{name}: Reached waypoint {drone['target']+1}")
                drone["cmd_pub"].publish(Twist())  # stop motion briefly
                drone["target"] += 1
                time.sleep(0.5)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — stopping all drones.")
        for d in node.drones.values():
            d["land_pub"].publish(Empty())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
