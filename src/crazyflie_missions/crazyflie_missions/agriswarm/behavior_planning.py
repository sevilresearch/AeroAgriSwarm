from crazyflie_missions.agriswarm import visualization

class LocalPlanner:
    def select_movement_action(self, agent, perception_data, agents):
        if not hasattr(agent, 'committed_column'):
            agent.committed_column = None

        if not hasattr(agent, 'column_sweep_direction'):
            agent.column_sweep_direction = {}  # maps x → 'down' or 'up'

        grid_w = agent.grid.size[1]
        grid_h = agent.grid.size[0]
        explored = agent.global_explored_cells

        def get_cells_needing_work(col_x):
            return [
                (col_x, y)
                for y in range(grid_h)
                if (col_x, y) not in explored
            ]

        unexplored_unoccupied_columns = [
            x for x in range(grid_w)
            if len(get_cells_needing_work(x)) > 0
            and not any(a.x == x and a != agent for a in agents)
        ]

        allow_help_anywhere = len(unexplored_unoccupied_columns) == 0

        if not hasattr(agent, 'sweep_direction'):
            if agent.x < grid_w // 2:
                agent.sweep_direction = 'left'
            else:
                agent.sweep_direction = 'right'

        if agent.sweep_direction == 'left':
            ordered_columns = list(range(agent.x, -1, -1)) + list(range(agent.x + 1, grid_w))
        else:
            ordered_columns = list(range(agent.x, grid_w)) + list(range(agent.x - 1, -1, -1))

        for x in ordered_columns:
            occupied = any(a.x == x and a.y not in [0, grid_h-1] and a != agent for a in agents)

            if occupied and allow_help_anywhere:
                occupied_columns = []
                for x2 in range(grid_w):
                    if any(a.x == x2 and a != agent for a in agents) and len(get_cells_needing_work(x2)) > 0:
                        occupied_columns.append(x2)

                if occupied_columns:
                    best_column = max(occupied_columns, key=lambda col: len(get_cells_needing_work(col)))
                    targets = get_cells_needing_work(best_column)

                    # Find which side (top or bottom) is unexplored
                    top_unexplored = (best_column, 0) not in explored
                    bottom_unexplored = (best_column, grid_h - 1) not in explored

                    if top_unexplored and not bottom_unexplored:
                        correct_entry = 0
                        agent.column_sweep_direction[best_column] = 'down'
                    elif bottom_unexplored and not top_unexplored:
                        correct_entry = grid_h - 1
                        agent.column_sweep_direction[best_column] = 'up'
                    else:
                        if agent.y <= grid_h // 2:
                            correct_entry = 0
                            agent.column_sweep_direction[best_column] = 'down'
                        else:
                            correct_entry = grid_h - 1
                            agent.column_sweep_direction[best_column] = 'up'

                    # Step 1: If not at correct side vertically, move up/down
                    if agent.y != correct_entry and agent.x != best_column:
                        if agent.y > correct_entry:
                            return 'up'
                        else:
                            return 'down'

                    # Step 2: If vertically aligned, sweep the column
                    direction = agent.column_sweep_direction[best_column]
                    if direction == 'down':
                        targets_sorted = sorted(targets, key=lambda p: p[1])  # top to bottom
                    else:
                        targets_sorted = sorted(targets, key=lambda p: -p[1])  # bottom to top

                    target = min(targets_sorted, key=lambda p: abs(p[0] - agent.x))
                    return agent._move_towards_target(*target)

                continue

            if occupied and not allow_help_anywhere:
                continue

            targets = get_cells_needing_work(x)
            if targets:
                if x not in agent.column_sweep_direction:
                    if agent.y <= grid_h // 2:
                        agent.column_sweep_direction[x] = 'down'
                    else:
                        agent.column_sweep_direction[x] = 'up'

                direction = agent.column_sweep_direction[x]
                if direction == 'down':
                    targets_sorted = sorted(targets, key=lambda p: p[1])
                else:
                    targets_sorted = sorted(targets, key=lambda p: -p[1])

                target = min(targets_sorted, key=lambda p: abs(p[0] - agent.x))
                return agent._move_towards_target(*target)

        # print("i have finished im just waiting")



class PreassignedPlanner:
    def select_movement_action(self, agent, perception_data, agents):
        if not hasattr(agent, 'assigned_columns'):
            agent_index = agents.index(agent)
            total_agents = len(agents)
            grid_w = agent.grid.size[1]
            agent.assigned_columns = [col for col in range(grid_w) if col % total_agents == agent_index]
            agent.column_sweep_direction = {}

        grid_h = agent.grid.size[0]
        explored = agent.global_explored_cells

        def get_cells_needing_work(col_x):
            return [
                (col_x, y)
                for y in range(grid_h)
                if (col_x, y) not in explored
            ]

        for x in agent.assigned_columns:
            targets = get_cells_needing_work(x)
            if targets:
                if x not in agent.column_sweep_direction:
                    agent.column_sweep_direction[x] = 'down' if agent.y < grid_h // 2 else 'up'

                direction = agent.column_sweep_direction[x]
                sorted_targets = sorted(targets, key=lambda p: p[1]) if direction == 'down' else sorted(targets, key=lambda p: -p[1])

                target = sorted_targets[0]
                return agent._move_towards_target(*target)

        # print("i have finished im just waiting")
        # visualization.simulation_time += visualization.waiting_time_step
        return None


class ColumnBlockPlanner:
    def __init__(self):
        self.initialized = False

    def select_movement_action(self, agent, perception_data, agents):
        if not self.initialized:
            self.initialize_assignments(agents, agent.grid.size[1])
            self.initialized = True

        if not hasattr(agent, 'column_queue'):
            self.setup_agent_column_queue(agent)

        grid_h = agent.grid.size[0]
        explored = agent.global_explored_cells

        def get_cells_needing_work(col_x):
            return [
                (col_x, y)
                for y in range(grid_h)
                if (col_x, y) not in explored
            ]

        for col in agent.column_queue:
            targets = get_cells_needing_work(col)
            if targets:
                direction = 'down' if agent.y < grid_h // 2 else 'up'
                sorted_targets = sorted(targets, key=lambda p: p[1]) if direction == 'down' else sorted(targets, key=lambda p: -p[1])
                return agent._move_towards_target(*sorted_targets[0])

        return None  # idle if no tasks left

    def initialize_assignments(self, agents, grid_w):
        num_agents = len(agents)
        cols_per_agent = grid_w // num_agents
        for i, agent in enumerate(agents):
            start = i * cols_per_agent
            end = (i + 1) * cols_per_agent
            agent.assigned_block = list(range(start, end))

    def setup_agent_column_queue(self, agent):
        start_col = agent.x
        block = agent.assigned_block
        sorted_block = sorted(block)

        # Identify closest direction (left or right) from spawn
        if start_col in block:
            idx = sorted_block.index(start_col)
            # Go outward from start_col: e.g., [3, 4, 5] → start at 4 → [4, 5, 3]
            right = sorted_block[idx+1:]
            left = sorted_block[:idx][::-1]
            agent.column_queue = [start_col] + right + left
        else:
            # fallback to left-to-right
            agent.column_queue = sorted_block


# class BehaviorPlanner:

#     def select_movement_action(self, agent, perception_data, agents):
#         if not hasattr(agent, 'committed_column'):
#             agent.committed_column = None

#         def column_needs_work(col_x):
#             for y in range(agent.grid.size[0]):
#                 if (col_x, y) not in agent.global_explored_cells:
#                     return True
#             return False

#         def is_fully_green(cell):
#             return cell['crop_status'] == 1 and cell['moisture_level'] == 1

#         all_in_last_2 = all(a.x >= agent.grid.size[1] - 2 for a in agents)

#         if all_in_last_2:
#             columns = list(range(agent.grid.size[1] - 2, agent.grid.size[1]))

#             # Check if any unoccupied column still has work
#             unoccupied_with_work = []
#             for x in columns:
#                 column_has_agent = any(a.x == x and a != agent for a in agents)
#                 if not column_has_agent and column_needs_work(x):
#                     unoccupied_with_work.append(x)

#             for x in columns:
#                 column_has_agent = any(a.x == x and a != agent for a in agents)
#                 # Allow entering occupied columns only if no unoccupied ones need work
#                 if column_has_agent and unoccupied_with_work:
#                     continue

#                 if column_needs_work(x):
#                     target_cells = [
#                         (x, y)
#                         for y in range(agent.grid.size[0])
#                         if (cell := agent.grid.get_cell_info(x, y))['moisture_level'] == 0
#                         or cell['crop_status'] == 0
#                         and (x, y) not in agent.global_explored_cells
#                     ]
#                     if target_cells:
#                         target = min(target_cells, key=lambda p: abs(p[0] - agent.x) + abs(p[1] - agent.y))
#                         return agent._move_towards_target(*target)

#         # --- General grid scan (non-final columns)
#         for x in range(agent.grid.size[1]):
#             if not all_in_last_2 and x < agent.grid.size[1] - 1: # -2 earlier
#                 if any(a.x == x and a.y not in [0, agent.grid.size[0] - 1] and a != agent for a in agents):
#                     continue
#             for y in range(agent.grid.size[0]):
#                 if (x, y) in agent.global_explored_cells:
#                     continue
#                 cell = agent.grid.get_cell_info(x, y)
#                 if cell["moisture_level"] == 0 or cell["crop_status"] == 0:
#                     return agent._move_towards_target(x, y)

#         print("[DEBUG] Step 5: Fallback")

#         # --- Fallback: nearby neighbors
#         for direction, cell_info in perception_data.items():
#             if direction != "current" and cell_info is not None:
#                 new_x = agent.x + {'up': 0, 'down': 0, 'left': -1, 'right': 1}[direction]
#                 new_y = agent.y + {'up': -1, 'down': 1, 'left': 0, 'right': 0}[direction]
#                 if (new_x, new_y) in agent.global_explored_cells:
#                     continue
#                 if cell_info["moisture_level"] == 0 or cell_info["crop_status"] == 0:
#                     if not is_fully_green(cell_info) and not any(a.x == new_x and a.y == new_y and a != agent for a in agents):
#                         return direction






# # Logic No. 1:
# # Agents spawned on the left half will sweep right →→→

# # Agents spawned on the right half will sweep ←←← left

# class BehaviorPlanner:
#     def select_movement_action(self, agent, perception_data, agents):
#         if not hasattr(agent, 'committed_column'):
#             agent.committed_column = None

#         grid_w = agent.grid.size[1]
#         grid_h = agent.grid.size[0]
#         explored = agent.global_explored_cells

#         def is_fully_green(cell):
#             return cell['crop_status'] == 1 and cell['moisture_level'] == 1

#         def column_needs_work(col_x):
#             return any((col_x, y) not in explored for y in range(grid_h))

#         def get_cells_needing_work(col_x):
#             return [
#                 (col_x, y)
#                 for y in range(grid_h)
#                 if (col_x, y) not in explored
#                 and (
#                     (cell := agent.grid.get_cell_info(col_x, y))['moisture_level'] == 0
#                     or cell['crop_status'] == 0
#                 )
#             ]

#         # Step 1: Are all columns explored?
#         unexplored_columns = [x for x in range(grid_w) if column_needs_work(x)]
#         allow_help_anywhere = len(unexplored_columns) == 0

#         # Step 2: Decide preferred sweep direction once (based on initial x)
#         if not hasattr(agent, 'sweep_direction'):
#             if agent.x < grid_w // 2:
#                 agent.sweep_direction = 'right'
#             else:
#                 agent.sweep_direction = 'left'

#         # Step 3: Define prioritized column order based on direction
#         if agent.sweep_direction == 'right':
#             ordered_columns = list(range(agent.x, grid_w)) + list(range(agent.x - 1, -1, -1))
#         else:
#             ordered_columns = list(range(agent.x, -1, -1)) + list(range(agent.x + 1, grid_w))

#         # Step 4: Traverse prioritized column order
#         for x in ordered_columns:
#             occupied = any(a.x == x and a.y not in [0, grid_h - 1] and a != agent for a in agents)
#             if occupied and not allow_help_anywhere:
#                 continue

#             if not column_needs_work(x):
#                 continue

#             targets = get_cells_needing_work(x)
#             if targets:
#                 return agent._move_towards_target(*min(targets, key=lambda p: abs(p[0] - agent.x) + abs(p[1] - agent.y)))

#         # Step 5: Fallback — nearby neighbors
#         for direction, cell_info in perception_data.items():
#             if direction == "current" or cell_info is None:
#                 continue
#             new_x = agent.x + {'up': 0, 'down': 0, 'left': -1, 'right': 1}[direction]
#             new_y = agent.y + {'up': -1, 'down': 1, 'left': 0, 'right': 0}[direction]
#             if (new_x, new_y) in explored:
#                 continue
#             if cell_info["moisture_level"] == 0 or cell_info["crop_status"] == 0:
#                 if not is_fully_green(cell_info):
#                     return direction




# Logic no2
# Left-spawned agents work left to right:
# ←←← agent →→→

# Right-spawned agents work right to left:
# ←←← agent →→→

# class BehaviorPlanner:
#     def select_movement_action(self, agent, perception_data, agents):
#         if not hasattr(agent, 'committed_column'):
#             agent.committed_column = None

#         if not hasattr(agent, 'column_sweep_direction'):
#             agent.column_sweep_direction = {}  # maps x → 'down' or 'up'


#         grid_w = agent.grid.size[1]
#         grid_h = agent.grid.size[0]
#         explored = agent.global_explored_cells

#         def is_fully_green(cell):
#             return cell['crop_status'] == 1 and cell['moisture_level'] == 1

#         def column_needs_work(col_x):
#             return any((col_x, y) not in explored for y in range(grid_h))

#         def get_cells_needing_work(col_x):
#             return [
#                 (col_x, y)
#                 for y in range(grid_h)
#                 if (col_x, y) not in explored
#                 and (
#                     (cell := agent.grid.get_cell_info(col_x, y))['moisture_level'] == 0
#                     or cell['crop_status'] == 0
#                 )
#             ]

#         # Step 1: Are all columns explored?
#         unexplored_columns = [x for x in range(grid_w) if column_needs_work(x)]
#         allow_help_anywhere = len(unexplored_columns) == 0

#         # Step 2: Decide preferred sweep direction ONCE (based on initial x)
#         if not hasattr(agent, 'sweep_direction'):
#             if agent.x < grid_w // 2:
#                 agent.sweep_direction = 'left'
#             else:
#                 agent.sweep_direction = 'right'

#         # Step 3: Define prioritized column order based on outward sweep
#         if agent.sweep_direction == 'left':
#             ordered_columns = list(range(agent.x, -1, -1)) + list(range(agent.x + 1, grid_w))
#         else:
#             ordered_columns = list(range(agent.x, grid_w)) + list(range(agent.x - 1, -1, -1))

#         # Step 4: Traverse prioritized column order
#         for x in ordered_columns:
#             occupied = any(a.x == x and a.y not in [0, grid_h - 1] and a != agent for a in agents)
#             if occupied and not allow_help_anywhere:
#                 continue

#             if not column_needs_work(x):
#                 continue

#             targets = get_cells_needing_work(x)
#             # if targets:
#             #     return agent._move_towards_target(*min(targets, key=lambda p: abs(p[0] - agent.x) + abs(p[1] - agent.y)))
#             if targets:
#                 # Decide sweep direction for this column (if not already set)
#                 if x not in agent.column_sweep_direction:
#                     if agent.y <= grid_h // 2:
#                         agent.column_sweep_direction[x] = 'down'
#                     else:
#                         agent.column_sweep_direction[x] = 'up'

#                 direction = agent.column_sweep_direction[x]

#                 if direction == 'down':
#                     targets_sorted = sorted(targets, key=lambda p: p[1])  # top to bottom
#                 else:
#                     targets_sorted = sorted(targets, key=lambda p: -p[1])  # bottom to top

#                 target = min(
#                     targets_sorted,
#                     key=lambda p: abs(p[0] - agent.x)  # prioritize horizontally closer column first
#                 )
#                 return agent._move_towards_target(*target)


#         # Step 5: Fallback — nearby neighbors
#         for direction, cell_info in perception_data.items():
#             if direction == "current" or cell_info is None:
#                 continue
#             new_x = agent.x + {'up': 0, 'down': 0, 'left': -1, 'right': 1}[direction]
#             new_y = agent.y + {'up': -1, 'down': 1, 'left': 0, 'right': 0}[direction]
#             if (new_x, new_y) in explored:
#                 continue
#             if cell_info["moisture_level"] == 0 or cell_info["crop_status"] == 0:
#                 if not is_fully_green(cell_info):
#                     return direction


# import visualization 

# class LocalPlanner:
#     def select_movement_action(self, agent, perception_data, agents):
#         if not hasattr(agent, 'committed_column'):
#             agent.committed_column = None

#         if not hasattr(agent, 'column_sweep_direction'):
#             agent.column_sweep_direction = {}  # maps x → 'down' or 'up'


#         grid_w = agent.grid.size[1]
#         grid_h = agent.grid.size[0]
#         explored = agent.global_explored_cells

#         def is_fully_green(cell):
#             return cell['crop_status'] == 1 and cell['moisture_level'] == 1

#         def get_cells_needing_work(col_x):
#             return [
#                 (col_x, y)
#                 for y in range(grid_h)
#                 if (col_x, y) not in explored
#             ]

#         # Step 1: Are all columns explored?
#         unexplored_unoccupied_columns = [
#             x for x in range(grid_w)
#             if len(get_cells_needing_work(x)) > 0
#             and not any(a.x == x and a != agent for a in agents)
#         ]

#         allow_help_anywhere = len(unexplored_unoccupied_columns) == 0



#         # Step 2: Decide preferred sweep direction ONCE (based on initial x)
#         if not hasattr(agent, 'sweep_direction'):
#             if agent.x < grid_w // 2:
#                 agent.sweep_direction = 'left'
#             else:
#                 agent.sweep_direction = 'right'

#         # Step 3: Define prioritized column order based on outward sweep
#         if agent.sweep_direction == 'left':
#             ordered_columns = list(range(agent.x, -1, -1)) + list(range(agent.x + 1, grid_w))
#         else:
#             ordered_columns = list(range(agent.x, grid_w)) + list(range(agent.x - 1, -1, -1))


#         # Step 4: Traverse prioritized column order
#         for x in ordered_columns:
#             occupied = any(a.x == x and a.y not in [0, grid_h - 1] and a != agent for a in agents)
#             if occupied and not allow_help_anywhere:
#                 continue

#             # if len(get_cells_needing_work(x)) == 0:
#             #     continue


#             targets = get_cells_needing_work(x)
#             if targets:
#                 # Decide sweep direction for this column (if not already set)
#                 if x not in agent.column_sweep_direction:
#                     if agent.y <= grid_h // 2:
#                         agent.column_sweep_direction[x] = 'down'
#                     else:
#                         agent.column_sweep_direction[x] = 'up'

#                 direction = agent.column_sweep_direction[x]

#                 if direction == 'down':
#                     targets_sorted = sorted(targets, key=lambda p: p[1])  # top to bottom
#                 else:
#                     targets_sorted = sorted(targets, key=lambda p: -p[1])  # bottom to top

#                 target = min(
#                     targets_sorted,
#                     key=lambda p: abs(p[0] - agent.x)  # prioritize horizontally closer column first
#                 )
#                 return agent._move_towards_target(*target)

#         print("i have finished im just waiting")
#         # visualization.simulation_time += visualization.waiting_time_step
