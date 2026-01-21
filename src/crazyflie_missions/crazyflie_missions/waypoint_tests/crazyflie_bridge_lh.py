#!/usr/bin/env python3
import time
import logging
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from tf_transformations import quaternion_from_euler

logging.basicConfig(level=logging.INFO)


class CrazyflieLighthouseBridgeMulti(Node):
    """
    Combined node:
    - Reads Lighthouse pose from Crazyflie EKF
    - Publishes /Robot_X/pose
    - Subscribes to /cmd_vel, /takeoff, /land
    - Sends motion commands to Crazyflie
    """

    def __init__(self):
        super().__init__("crazyflie_lighthouse_bridge_multi")

        self.uris = {
            "Robot_1": "radio://0/80/2M/E7E7E7E701",
            "Robot_3": "radio://0/70/2M/E7E7E7E703",
        }

        self.drones = {}

        # Init Crazyflie radio
        cflib.crtp.init_drivers(enable_debug_driver=False)

        for name, uri in self.uris.items():
            self.get_logger().info(f"Connecting to {name} at {uri}...")

            scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"))
            scf.open_link()
            cf = scf.cf

            # Enable high-level commander
            cf.param.set_value("commander.enHighLevel", "0")

            # ROS publishers
            pose_pub = self.create_publisher(
                PoseStamped, f"/{name}/pose", 10
            )

            # ROS subscribers
            self.create_subscription(
                Empty, f"/{name}/takeoff",
                lambda msg, n=name: self.takeoff(n), 10
            )
            self.create_subscription(
                Empty, f"/{name}/land",
                lambda msg, n=name: self.land(n), 10
            )
            self.create_subscription(
                Twist, f"/{name}/cmd_vel",
                lambda msg, n=name: self.cmd_vel(n, msg), 10
            )

            # Lighthouse log configuration
            logconf = LogConfig(
                name=f"{name}_lighthouse_pose", period_in_ms=50
            )
            logconf.add_variable("stateEstimate.x", "float")
            logconf.add_variable("stateEstimate.y", "float")
            logconf.add_variable("stateEstimate.z", "float")
            logconf.add_variable("stateEstimate.yaw", "float")

            cf.log.add_config(logconf)
            
            logconf.data_received_cb.add_callback(
                self.make_pose_cb(name)
            )

            logconf.start()

            self.drones[name] = {
                "scf": scf,
                "cf": cf,
                "pose_pub": pose_pub,
                "flying": False,
                "last_cmd": time.time(),
            }

            self.get_logger().info(f"✅ {name} ready")

    # ------------------------------------------------------------------
    def make_pose_cb(self, name):
        def cb(timestamp, data, logconf):
            self.pose_log_cb(name, data)
        return cb

    # ------------------------------------------------------------------
    def pose_log_cb(self, name, data):
        """Publish Lighthouse EKF pose to ROS."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = data["stateEstimate.x"]
        msg.pose.position.y = data["stateEstimate.y"]
        msg.pose.position.z = data["stateEstimate.z"]

        q = quaternion_from_euler(0, 0, data["stateEstimate.yaw"])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.drones[name]["pose_pub"].publish(msg)

    # ------------------------------------------------------------------
    def takeoff(self, name):
        drone = self.drones[name]
        if drone["flying"]:
            return

        self.get_logger().info(f"{name}: Takeoff (HLC)")

        # 1. Stop any low-level setpoints
        drone["cf"].commander.send_stop_setpoint()
        time.sleep(0.1)

        # 2. Enable high-level commander
        drone["cf"].param.set_value("commander.enHighLevel", "1")
        time.sleep(0.1)

        # 3. Takeoff
        drone["cf"].high_level_commander.takeoff(0.5, 2.0)
        time.sleep(2.5)

        # 4. Disable high-level commander (return to velocity mode)
        drone["cf"].high_level_commander.stop()
        drone["cf"].param.set_value("commander.enHighLevel", "0")

        drone["flying"] = True
        drone["last_cmd"] = time.time()


    # ------------------------------------------------------------------
    def land(self, name):
        drone = self.drones[name]
        if not drone["flying"]:
            return

        self.get_logger().info(f"{name}: Land (HLC)")

        # 1. Stop velocity control
        drone["cf"].commander.send_stop_setpoint()
        time.sleep(0.1)

        # 2. Enable high-level commander
        drone["cf"].param.set_value("commander.enHighLevel", "1")
        time.sleep(0.1)

        # 3. Land
        drone["cf"].high_level_commander.land(0.0, 3.0)
        time.sleep(3.5)

        drone["cf"].high_level_commander.stop()
        drone["cf"].param.set_value("commander.enHighLevel", "0")

        drone["flying"] = False


    # ------------------------------------------------------------------
    def cmd_vel(self, name, msg):
        drone = self.drones[name]
        if not drone["flying"]:
            return

        drone["cf"].commander.send_velocity_world_setpoint(
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.z
        )
        drone["last_cmd"] = time.time()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down bridge safely")

        for drone in self.drones.values():
            try:
                drone["cf"].commander.send_stop_setpoint()
                drone["cf"].param.set_value("commander.enHighLevel", "1")
                time.sleep(0.1)

                if drone["flying"]:
                    drone["cf"].high_level_commander.land(0.0, 3.0)
                    time.sleep(3.5)

                drone["cf"].high_level_commander.stop()
                drone["scf"].close_link()

            except Exception:
                pass

        super().destroy_node()



def main():
    rclpy.init()
    node = CrazyflieLighthouseBridgeMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
