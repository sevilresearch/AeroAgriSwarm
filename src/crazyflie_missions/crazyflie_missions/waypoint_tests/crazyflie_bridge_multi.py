#!/usr/bin/env python3
import time
import logging
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander

logging.basicConfig(level=logging.INFO)


class CrazyflieBridgeMulti(Node):
    """ROS 2 bridge for multiple Crazyflies (Robot_1 and Robot_3)."""

    def __init__(self):
        super().__init__("crazyflie_bridge_multi")

        # Define URIs for all drones you want to control
        self.uris = {
            "Robot_1": "radio://0/80/2M/E7E7E7E701",
            "Robot_3": "radio://0/70/2M/E7E7E7E703",
        }
        
        # Dict to store each connected drone and its state
        self.drones = {}

        # Initialize radio drivers
        cflib.crtp.init_drivers(enable_debug_driver=False)

        # Try connecting each Crazyflie
        for name, uri in self.uris.items():
            self.get_logger().info(f"Connecting to {name} at {uri}...")
            try:
                scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"))
                scf.open_link()

                # Enable high-level commander (allows takeoff/land commands)
                cf = scf.cf
                cf.param.set_value("commander.enHighLevel", "1")

                self.drones[name] = {
                    "scf": scf,
                    "cf": cf,
                    "flying": False,
                    "last_cmd": time.time(),
                }

                self.get_logger().info(f"✅ {name} connected successfully.")

                # ROS topic subscriptions for this drone
                self.create_subscription(Empty, f"/{name}/takeoff",
                                         lambda msg, n=name: self.takeoff(n), 10)
                self.create_subscription(Empty, f"/{name}/land",
                                         lambda msg, n=name: self.land(n), 10)
                self.create_subscription(Twist, f"/{name}/cmd_vel",
                                         lambda msg, n=name: self.cmd_vel(n, msg), 10)

            except Exception as e:
                self.get_logger().error(f"❌ Failed to connect to {name}: {e}")

    # ----------------------------------------------------------------------
    def takeoff(self, name):
        """High-level takeoff command."""
        try:
            drone = self.drones[name]
            if drone["flying"]:
                self.get_logger().warn(f"{name} already flying.")
                return
            cf = drone["cf"]
            self.get_logger().info(f"{name}: Taking off...")
            cf.high_level_commander.takeoff(0.5, 2.0)  # 0.5m in 2s
            time.sleep(2.5)
            drone["flying"] = True
        except Exception as e:
            self.get_logger().error(f"{name}: Takeoff failed — {e}")

    # ----------------------------------------------------------------------
    def land(self, name):
        """High-level land command."""
        try:
            drone = self.drones[name]
            if not drone["flying"]:
                self.get_logger().warn(f"{name} not flying.")
                return
            cf = drone["cf"]
            self.get_logger().info(f"{name}: Landing...")
            cf.high_level_commander.land(0.0, 4.0)
            time.sleep(4.5)
            cf.high_level_commander.stop()
            drone["flying"] = False
        except Exception as e:
            self.get_logger().error(f"{name}: Landing failed — {e}")

    # ----------------------------------------------------------------------
    def cmd_vel(self, name, msg):
        """Send velocity setpoints (in world frame)."""
        try:
            drone = self.drones[name]
            if not drone["flying"]:
                return

            vx, vy, vz = msg.linear.x, msg.linear.y, msg.linear.z
            yaw_rate = msg.angular.z

            # Send a velocity setpoint (world frame)
            cf = drone["cf"]
            cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)
            drone["last_cmd"] = time.time()

        except Exception as e:
            self.get_logger().warn(f"{name}: velocity command failed — {e}")

    # ----------------------------------------------------------------------
    def destroy(self):
        """Gracefully land and close all links on shutdown."""
        self.get_logger().info("Shutting down multi-drone bridge...")
        for name, drone in self.drones.items():
            try:
                if drone["flying"]:
                    drone["cf"].high_level_commander.land(0.0, 4.0)
                    time.sleep(4.5)
                    drone["cf"].high_level_commander.stop()
                drone["scf"].close_link()
                self.get_logger().info(f"{name} disconnected.")
            except Exception:
                pass
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieBridgeMulti()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — landing all drones.")

        # Smooth land every drone
        for name, drone in node.drones.items():
            try:
                node.get_logger().info(f"{name}: Emergency slow landing...")
                drone["cf"].high_level_commander.land(0.0, 3.0)  # smooth 3-second descent
                time.sleep(3.2)
                drone["cf"].high_level_commander.stop()
            except Exception:
                pass

    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
