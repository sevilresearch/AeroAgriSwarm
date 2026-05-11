#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

import time
import numpy as np

from crazyflie_missions.agriswarm.grid import Grid
from crazyflie_missions.agriswarm.agent import Agent
from crazyflie_missions.agriswarm.state_estimation import StateEstimator
from crazyflie_missions.agriswarm.behavior_planning import LocalPlanner


# ----------------------------
# Coordinate transforms
# ----------------------------
def grid_to_world(x, y):
    scale = 0.3
    return x * scale, y * scale, 0.5


def world_to_grid(x, y):
    scale = 0.3
    return int(round(x / scale)), int(round(y / scale))


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

        self.grid = Grid(size=(4, 4))
        self.global_explored_cells = set()

        self.state_estimator = StateEstimator(self.grid)
        self.behavior_planner = LocalPlanner()

        agent_positions = [(0, 0), (1, 0)]

        self.agents = [
            Agent(self.grid, self.global_explored_cells, 3, pos, [], self.behavior_planner)
            for pos in agent_positions
        ]

        for agent in self.agents:
            agent.agents = self.agents

        # ----------------------------
        # Timing system (like main.py)
        # ----------------------------
        self.simulation_time = 0.0
        self.time_step = 0.5

        self.start_time = time.time()

        # ----------------------------
        # Loop timer (non-blocking)
        # ----------------------------
        self.timer = self.create_timer(self.time_step, self.loop)

        self.get_logger().info("✅ Agriswarm FULL system started")

    # ----------------------------
    def pose_cb(self, name, msg):
        self.poses[name] = msg

    # ----------------------------
    def update_agents_from_real_pose(self):
        """Bi-directional: update agent positions from OptiTrack"""
        for i, agent in enumerate(self.agents):
            name = list(self.goal_pubs.keys())[i]

            if name not in self.poses:
                continue

            pose = self.poses[name].pose.position
            gx, gy = world_to_grid(pose.x, pose.y)

            # Clamp to grid
            gx = max(0, min(self.grid.size[1] - 1, gx))
            gy = max(0, min(self.grid.size[0] - 1, gy))

            agent.x = gx
            agent.y = gy

    # ----------------------------
    def publish_goals(self):
        for i, agent in enumerate(self.agents):
            name = list(self.goal_pubs.keys())[i]

            wx, wy, wz = grid_to_world(agent.x, agent.y)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
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
        # --- Bi-directional update ---
        self.update_agents_from_real_pose()

        # --- Decision step ---
        for agent in self.agents:
            action = agent.select_action()
            agent.execute_action(action)

        # --- Publish goals ---
        self.publish_goals()

        # --- Update simulation time ---
        self.simulation_time += self.time_step

        # --- Check completion ---
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