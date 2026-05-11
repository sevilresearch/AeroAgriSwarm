#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os
import time

# ---- AgriSwarm imports (UNCHANGED planner) ----
from crazyflie_missions.agriswarm.agent import Agent
from crazyflie_missions.agriswarm.grid import Grid
from crazyflie_missions.agriswarm.behavior_planning import LocalPlanner


class GridAdapter(Node):
    def __init__(self):
        super().__init__('grid_adapter')

        # ---------------- Load YAML ----------------
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../config/grid.yaml'
        )

        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.grid_cfg = self.cfg['grid']
        self.flight_cfg = self.cfg['flight']
        self.agent_cfg = self.cfg['agents']

        # ---------------- ROS publishers ----------------
        self.publishers = {}
        for name, agent in self.agent_cfg.items():
            self.publishers[name] = self.create_publisher(
                PoseStamped,
                agent['topic'],
                10
            )

        # ---------------- AgriSwarm setup ----------------
        self.grid = Grid(
            width=self.grid_cfg['width'],
            height=self.grid_cfg['height']
        )

        self.agents = {}
        self.planners = {}

        for name, agent in self.agent_cfg.items():
            aid = agent['id']
            self.agents[name] = Agent(aid, self.grid)
            self.planners[name] = LocalPlanner(self.agents[name], self.grid)

        # ---------------- Main loop ----------------
        self.timer = self.create_timer(5.0, self.step)  
        self.get_logger().info("AgriSwarm Grid Adapter started")

    # --------------------------------------------------
    def grid_to_world(self, i, j):
        res = self.grid_cfg['resolution']
        ox, oy = self.grid_cfg['origin']

        x = ox + (i + 0.5) * res
        y = oy + (j + 0.5) * res
        z = self.flight_cfg['altitude']

        return x, y, z

    # --------------------------------------------------
    def step(self):
        for name in self.agents:
            planner = self.planners[name]
            agent = self.agents[name]

            # ---- Planner step (NO MODIFICATION) ----
            planner.step()

            i, j = agent.position  # grid coordinates

            x, y, z = self.grid_to_world(i, j)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z

            # Fixed yaw
            msg.pose.orientation.w = 1.0

            self.publishers[name].publish(msg)

            self.get_logger().debug(
                f"{name}: grid=({i},{j}) -> world=({x:.2f},{y:.2f},{z:.2f})"
            )


def main():
    rclpy.init()
    node = GridAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
