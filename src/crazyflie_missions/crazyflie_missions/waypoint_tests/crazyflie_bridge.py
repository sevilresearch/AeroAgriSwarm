#!/usr/bin/env python3
import logging
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

logging.basicConfig(level=logging.INFO)


class CrazyflieBridge(Node):
    """ROS 2 bridge to Crazyflie using topics only (takeoff, land, cmd_vel)."""

    def __init__(self):
        super().__init__("crazyflie_bridge")

        # --- Parameters ---
        self.declare_parameter("uri", "radio://0/80/2M/E7E7E7E703")
        uri = self.get_parameter("uri").get_parameter_value().string_value

        # --- Connect to Crazyflie (robust init) ---
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.open_link()
        self.cf = self.scf.cf

        # --- Enable high-level commander ---
        self.cf.param.set_value("commander.enHighLevel", "1")

        # --- Wait for Flow Deck attach (timeout) ---
        flow_ok = {"val": False}

        def deck_flow_cb(_, value_str):
            try:
                if int(value_str):
                    self.get_logger().info("✅ Flow Deck detected!")
                    flow_ok["val"] = True
                else:
                    self.get_logger().warn("❌ No Flow Deck detected (bcFlow2=0)")
            except Exception:
                pass

        self.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=deck_flow_cb)

        # give the callback some time
        start = time.time()
        timeout = 6.0
        while time.time() - start < timeout and not flow_ok["val"]:
            rclpy.spin_once(self, timeout_sec=0.2)

        if not flow_ok["val"]:
            self.get_logger().warn("Flow deck not confirmed within timeout. Proceeding with caution.")

        # --- Wait for estimator convergence using LogConfig ---
        est_ok = {"val": False}
        logconf = LogConfig(name="KF", period_in_ms=100)
        logconf.add_variable("stateEstimate.z", "float")
        logconf.add_variable("kalman.varPZ", "float")

        def log_cb(ts, data, lc):
            z = data.get("stateEstimate.z", 0.0)
            varz = data.get("kalman.varPZ", 9999.0)
            # If variance small and z is a number, mark ready
            if varz < 0.005 and not (z != z):
                est_ok["val"] = True

        self.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_cb)

        try:
            logconf.start()
            st = time.time()
            while time.time() - st < 6.0 and not est_ok["val"]:
                rclpy.spin_once(self, timeout_sec=0.2)
            logconf.stop()
        except Exception:
            pass
        finally:
            try:
                self.cf.log.remove_config(logconf)
            except Exception:
                pass

        if not est_ok["val"]:
            self.get_logger().warn("Estimator did not converge in time (varPZ threshold).")

        # --- Arm only after the above checks ---
        try:
            self.cf.platform.send_arming_request(True)
            self.get_logger().info("Arming requested.")
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Arming failed: {e}")

        # --- State ---
        self.mc = None
        self.is_flying = False
        self.last_z = 0.0  # updated by logging if enabled

        # --- Subscriptions ---
        self.create_subscription(Empty, "/cf/takeoff", self.takeoff_callback, 10)
        self.create_subscription(Empty, "/cf/land", self.land_callback, 10)
        self.create_subscription(Twist, "/cf/cmd_vel", self.cmd_vel_callback, 10)

        # optional position logging to update last_z
        try:
            pos_log = LogConfig(name="pos", period_in_ms=100)
            pos_log.add_variable("stateEstimate.z", "float")
            def pos_cb(ts, data, lc):
                self.last_z = float(data.get("stateEstimate.z", 0.0))
            self.cf.log.add_config(pos_log)
            pos_log.data_received_cb.add_callback(pos_cb)
            pos_log.start()
        except Exception:
            pass

        self.get_logger().info(f"✅ Connected to Crazyflie at {uri}")
        self.get_logger().info("Listening on /cf/takeoff, /cf/land, /cf/cmd_vel")

    # --- Takeoff and landing ---
    def takeoff_callback(self, _msg):
        if self.is_flying:
            self.get_logger().warn("Already flying.")
            return
        try:
            self.get_logger().info("Taking off...")
            # create MotionCommander (kept until land)
            self.mc = MotionCommander(self.scf, default_height=0.5)
            # take_off(height, duration)
            try:
                self.mc.take_off(0.5, 2.0)
            except TypeError:
                # fallback if signature differs
                pass
            self.is_flying = True
            self.get_logger().info("Takeoff command issued.")
        except Exception as e:
            self.get_logger().error(f"Takeoff failed: {e}")

    def land_callback(self, _msg):
        if not self.is_flying or self.mc is None:
            self.get_logger().warn("Not flying.")
            return
        try:
            self.get_logger().info("Landing...")
            self.mc.land()
            # close commander
            try:
                self.mc = None
            except Exception:
                pass
            self.is_flying = False
            self.get_logger().info("Landed successfully.")
        except Exception as e:
            self.get_logger().error(f"Landing failed: {e}")

    # --- Velocity commands ---
    def cmd_vel_callback(self, msg: Twist):
        # refuse commands if not flying or commander not ready
        if not self.is_flying or self.mc is None:
            return
        # safety: ignore horizontal commands when too low
        if getattr(self, "last_z", 0.0) < 0.12:
            # still ascending / too close to ground
            return
        try:
            vx, vy, vz, yaw_rate = msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z
            # MotionCommander expects speeds in body frame
            self.mc.start_linear_motion(vx, vy, vz, yaw_rate)
        except Exception as e:
            self.get_logger().warn(f"Velocity command error: {e}")

    # --- Cleanup ---
    def destroy(self):
        self.get_logger().info("Shutting down Crazyflie bridge...")
        if self.is_flying and self.mc:
            try:
                self.mc.land()
            except Exception:
                pass
        try:
            self.scf.close_link()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
