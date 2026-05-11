from crazyflie_missions.agriswarm.grid import Grid
from crazyflie_missions.agriswarm.agent import Agent
from crazyflie_missions.agriswarm.state_estimation import StateEstimator
from crazyflie_missions.agriswarm.behavior_planning import LocalPlanner, PreassignedPlanner
from crazyflie_missions.agriswarm import visualization
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="tkinter")

import numpy as np
import time

def create_seeded_grid(size=(4, 4), seed=42):
    """Create a grid with a fixed seed for reproducibility."""
    np.random.seed(seed)  # Set the seed
    grid = Grid(size=size)  # Initialize grid (this calls initialize_grid with seeded random)
    return grid

def print_stats_main(run, agents):
    # print(f"\n--- run {run + 1} Stats ---")
    for i, agent in enumerate(agents):
        print(f"Agent {i+1} ({agent.color}) stats:")
        print(f"  Total cells travelled: {agent.cells_travelled}")
        # print(f"  visited cells travelled: {agent.agents_actual_visited_cells}")
        # print(f"  visited cells travelled: {agent.visited_cells}")

        print(f"  Total revisits: {agent.revisit_count}")
        print(f"  Percentage of revisited cells: {agent.get_revisit_percentage():.2f}%")

def main():

    runs = 1
    seed = 42 # 42  # Fixed seed for reproducibility

    use_preassigned = False

    no_of_agents = 2  # Change to 1, 2, 3, or 4

    # Predefined, editable start positions
    agent_pos_1 = [(0, 0)]
    agent_pos_2 = [(0, 0), (1, 0)]
    agent_pos_3 = [(0, 0), (1, 0), (2, 0)]
    # agent_pos_3 = [(2, 0), (5, 0), (12, 0)]             # interesting start positions #1
    # agent_pos_3 = [(3, 4), (9, 4), (12, 2)]             # interesting start positions #1
    # agent_pos_4 = [(0, 0), (1, 0), (2, 0), (3, 0)]
    agent_pos_4 = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0)]

    # Assign based on the agent count
    if no_of_agents == 1:
        agent_positions = agent_pos_1
    elif no_of_agents == 2:
        agent_positions = agent_pos_2
    elif no_of_agents == 3:
        agent_positions = agent_pos_3
    elif no_of_agents == 4:
        agent_positions = agent_pos_4

    for run in range(runs):
        base_grid = create_seeded_grid(size=(4, 4), seed=seed + run)  # vary seed per run if needed

        start_time = time.time()
        global_explored_cells = set()

        Agent.used_colors.clear()
        # agent.set_shared_memory(global_explored_cells)
        agents = []

        # Initialize the 10x10 farm environment
        grid = Grid(size=(4, 4))
        grid.grid = base_grid.grid.copy()  # Copy the random grid state
        grid.boundaries = base_grid.boundaries.copy()  # Copy the boundaries
        
        # Initialize state estimation and behavior planning
        state_estimator = StateEstimator(grid)
        if use_preassigned:
            behavior_planner = PreassignedPlanner()
            reroute_threshold = 20
        else:
            behavior_planner = LocalPlanner()
            reroute_threshold = 3


        agents = [Agent(grid, global_explored_cells, reroute_threshold, pos, [], behavior_planner=behavior_planner) for pos in agent_positions]

        # Now, assign the full list to all agents
        for agent in agents:
            agent.agents = agents


        print(f"\n--- run {run+1} ---")

        sim_time = visualization.display_grid(grid, agents, state_estimator, behavior_planner)
        # sim_time = visualization.run_simulation(grid, agents, state_estimator, behavior_planner)
            
        # End timing this run
        end_time = time.time()
        run_time = end_time - start_time
        print_stats_main(runs, agents)
        print(f"Time taken for Run {run + 1}: {run_time:.2f} seconds")
        print(f"Sim time: \033[92m'{sim_time:.2f}\033[0m' units")
        sim_steps_per_sec = sim_time / run_time
        print(f"Sim speed: {sim_steps_per_sec:.4f} steps/sec")
        # print(f"Global visited cells: {global_explored_cells}")
        total_cells = grid.size[0] * grid.size[1]
        explored_percent = (len(global_explored_cells) / total_cells) * 100
        print(f"Explored: {explored_percent:.2f}% of the grid")



        
if __name__ == "__main__":
    main()


