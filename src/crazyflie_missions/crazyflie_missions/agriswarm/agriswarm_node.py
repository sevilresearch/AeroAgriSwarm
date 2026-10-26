#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import Bool

import time
import numpy as np
import math

from crazyflie_missions.agriswarm.grid import Grid
from crazyflie_missions.agriswarm.agent import Agent
from crazyflie_missions.agriswarm.state_estimation import StateEstimator
from crazyflie_missions.agriswarm.behavior_planning import LocalPlanner

CALIBRATION_MODE = False

# ----------------------------
# Coordinate transforms
# ----------------------------
GRID_ORIGIN_X = -1.36
GRID_ORIGIN_Y = 1.31
CELL_SIZE = 0.30

def world_to_grid(x, y):
    gx = round((x - GRID_ORIGIN_X) / CELL_SIZE)
    gy = round((GRID_ORIGIN_Y - y) / CELL_SIZE)
    return gx, gy

def grid_to_world(gx, gy):
    x = GRID_ORIGIN_X + gx * CELL_SIZE
    y = GRID_ORIGIN_Y - gy * CELL_SIZE
    return x, y, 0.5

# ----------------------------
class AgriswarmNode(Node):
    def __init__(self):
        super().__init__('agriswarm_node')

        # ----------------------------
        # Publishers
        # ----------------------------
        self.goal_pubs = {
            "Robot_1": self.create_publisher(PoseStamped, "/Robot_1/goal", 10),
            "Robot_3": self.create_publisher(PoseStamped, "/Robot_3/goal", 10),
        }

        self.land_pubs = {
            name: self.create_publisher(Empty, f"/{name}/land", 10)
            for name in self.goal_pubs
        }

        # ----------------------------
        # Subscribers (Bi-directional)
        # ----------------------------
        self.poses = {}

        for name in self.goal_pubs:
            self.create_subscription(
                PoseStamped,
                f"/{name}/pose",
                lambda msg, n=name: self.pose_cb(n, msg),
                10
            )

        # ----------------------------
        # Initialize Agriswarm
        # ----------------------------
        np.random.seed(42)

        self.grid = Grid(size=(5, 10))
        self.global_explored_cells = set()

        self.state_estimator = StateEstimator(self.grid)
        self.behavior_planner = LocalPlanner()

        agent_positions = [(0, 0)]

        self.agents = [
            Agent(self.grid, self.global_explored_cells, 3, pos, [], self.behavior_planner)
            for pos in agent_positions
        ]

        for agent in self.agents:
            agent.agents = self.agents

        # ----------------------------
        # Goal Reached Subscriber
        # ----------------------------

        self.goal_reached_flags = [False] * len(self.agents)

        for i, name in enumerate(self.goal_pubs.keys()):
            self.create_subscription(
                Bool,
                f"/{name}/goal_reached",
                lambda msg, j=i: self.goal_reached_callback(msg, j),
                10,
            )

        # ----------------------------
        # Timing system (like main.py)
        # ----------------------------
        self.simulation_time = 0.0
        self.time_step = 0.5

        self.start_time = time.time()

        # ----------------------------
        # Goal completion tracking
        # ----------------------------
        self.goal_active = [False] * len(self.agents)

        self.goal_tolerance = 0.10  

        # ----------------------------
        # Loop timer (non-blocking)
        # ----------------------------
        self.timer = self.create_timer(self.time_step, self.loop)

        self.get_logger().info("✅ Agriswarm FULL system started")

        # -------------------------
        # Calibration test
        # -------------------------
        self.calibration_cells = [
            (0,0),
            (1,0),
            (2,0),
            (3,0)
        ]

        self.calibration_index = 0
        self.calibration_tolerance = 0.08
        self.calibration_complete = False

    def run_calibration(self):

        if "Robot_1" not in self.poses:
            return

        if self.calibration_index >= len(self.calibration_cells):

            self.get_logger().info("Calibration Finished")
            self.calibration_complete = True
            return

        col,row = self.calibration_cells[self.calibration_index]

        wx,wy,wz = grid_to_world(col,row)

        pose = self.poses["Robot_1"].pose.position

        dx = pose.x-wx
        dy = pose.y-wy

        distance = math.sqrt(dx*dx+dy*dy)

        goal = PoseStamped()

        goal.header.stamp = self.get_clock().now().to_msg()

        goal.header.frame_id="world"

        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.position.z = wz

        self.goal_pubs["Robot_1"].publish(goal)

        self.get_logger().info(
            f"Target Cell ({col},{row})"
            f"  Goal ({wx:.2f},{wy:.2f})"
            f"  Distance {distance:.2f}"
        )

        if distance < self.calibration_tolerance:

            self.get_logger().info(
                f"Reached Cell ({col},{row})"
            )

            self.calibration_index +=1

    def goal_reached_callback(self, msg, idx):

        if not msg.data:
            return
        
        # Update ONLY this agent from the latest OptiTrack pose
        name = list(self.goal_pubs.keys())[idx]

        self.goal_reached_flags[idx] = True
        self.goal_active[idx] = False

        self.get_logger().info(
            f"{name} reached its waypoint."
        )

        if name not in self.poses:
            return

        pose = self.poses[name].pose.position
        gx, gy = world_to_grid(pose.x, pose.y)

        # Clamp to grid
        gx = max(0, min(self.grid.size[1] - 1, gx))
        gy = max(0, min(self.grid.size[0] - 1, gy))

        self.agents[idx].x = gx
        self.agents[idx].y = gy

    # ----------------------------
    def pose_cb(self, name, msg):
        self.poses[name] = msg

    # ----------------------------
    def publish_goal(self, i):
        agent = self.agents[i]
        name = list(self.goal_pubs.keys())[i]

        wx, wy, wz = grid_to_world(agent.x, agent.y)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = wx
        msg.pose.position.y = wy
        msg.pose.position.z = wz

        self.goal_pubs[name].publish(msg)

    # ----------------------------
    def print_stats(self):
        for i, agent in enumerate(self.agents):
            self.get_logger().info(
                f"Agent {i+1}: cells={agent.cells_travelled}, "
                f"revisits={agent.revisit_count}, "
                f"revisit%={agent.get_revisit_percentage():.2f}"
            )

    # ----------------------------
    def stop_all(self):
        self.get_logger().info("🛑 Mission complete. Landing all drones...")

        for pub in self.land_pubs.values():
            pub.publish(Empty())

        total_time = time.time() - self.start_time

        self.get_logger().info(f"⏱ Real Time: {total_time:.2f}s")
        self.get_logger().info(f"⏱ Sim Time: {self.simulation_time:.2f}")

        self.print_stats()

        # Shutdown safely
        self.get_logger().info("Mission complete.")
        
    # ----------------------------
    def loop(self):

        if CALIBRATION_MODE:

            if self.calibration_complete == False:
                self.run_calibration()
                return
            else:
                self.get_logger().info("Calibration complete.")
                rclpy.shutdown()

        for i, agent in enumerate(self.agents):

            # Wait until previous goal reached
            if self.goal_active[i]:

                if not self.goal_reached_flags[i]:
                    continue

                # Goal completed
                self.goal_reached_flags[i] = False
                self.goal_active[i] = False

            # Planner now chooses ONE new action
            action = agent.select_action()

            agent.execute_action(action)

            self.publish_goal(i)

            self.goal_active[i] = True
            
        self.simulation_time += self.time_step

        total_cells = self.grid.size[0] * self.grid.size[1]

        if len(self.global_explored_cells) >= total_cells:

            if all(agent.done or agent.is_frozen for agent in self.agents):

                self.stop_all()


def main(args=None):
    rclpy.init(args=args)
    node = AgriswarmNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()