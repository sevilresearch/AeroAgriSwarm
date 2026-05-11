import numpy as np
from crazyflie_missions.agriswarm.state_estimation import StateEstimator
import time
import heapq
from crazyflie_missions.agriswarm import visualization

class Agent:
    used_colors = set()

    def __init__(self, grid, global_explored_cells, reroute_threshold, start_pos=None, agents=None, behavior_planner=None):
        self.grid = grid
        self.agents = agents if agents is not None else []  # List of all agents
        self.x, self.y = start_pos
        self.reroute_threshold = reroute_threshold  # Configurable
        self.done = False  # True when agent has nothing left to do
        self.path_queue = []
        self.wait_until = 0  # Agent can act immediately
        self.cells_travelled = 0  # Start at cell 1
        self.visited_cells = set()  # Track visited cells
        self.agents_actual_visited_cells = []  # Track visited cells
        self.revisit_count = 0  # Count total revisits
        # self.visited_cells.add((self.x, self.y))  # Mark initial position as visited
        self.global_explored_cells = global_explored_cells
        self.color = self.assign_color()  # Assign a unique color
        self.behavior_planner = behavior_planner  # <- Injected planner       
        self.is_frozen = False  # Used to freeze an agent mid-simulation
        self.blocked_cell_attempts = {}  # (x, y): count of failed checks

        self.update_position(self.x, self.y)  # Mark the initial cell properly

    def assign_color(self):
        available_colors = ["blue", "yellow", "purple", "orange", "pink", "cyan"]
        for color in available_colors:
            if color not in Agent.used_colors:
                Agent.used_colors.add(color)
                return color
        return "white"  # Default if all colors are used


    def astar_to_next_unexplored_column(self):
        import heapq

        grid_w, grid_h = self.grid.size
        explored = self.global_explored_cells
        start = (self.x, self.y)

        def get_cells_needing_work(col_x):
            return [
                (col_x, y)
                for y in range(grid_w)
                if (col_x, y) not in explored
            ]

        unexplored_unoccupied_columns = [
            x for x in range(grid_h)
            if len(get_cells_needing_work(x)) > 0
            and not any(a.x == x and a != self for a in self.agents)
        ]

        unexplored_targets = [
            (x, y)
            for x in unexplored_unoccupied_columns
            for y in range(grid_w)
            if 0 <= x < grid_h and 0 <= y < grid_w
            and (x, y) not in explored
            and not self.is_cell_occupied(x, y)
        ]


        # if not unexplored_targets:
        #     return None
        if not unexplored_targets:
            # fallback to explore occupied columns with unfinished work
            explored = self.global_explored_cells
            fallback_targets = [
                (x, y)
                for x in range(grid_h)
                for y in range(grid_w)
                if (x, y) not in explored and not self.is_cell_occupied(x, y)
            ]
            unexplored_targets = fallback_targets

        def neighbors(x, y):
            for dx, dy, direction in [(-1, 0, 'left'), (1, 0, 'right'), (0, -1, 'up'), (0, 1, 'down')]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid_h and 0 <= ny < grid_w:
                    if not self.is_cell_occupied(nx, ny) and not self.grid.is_boundary(x, y, direction):
                        yield nx, ny

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        goal = None

        while frontier:
            _, current = heapq.heappop(frontier)
            if current in unexplored_targets:
                goal = current
                break

            for nx, ny in neighbors(*current):
                new_cost = cost_so_far[current] + 1
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + abs(nx - self.x) + abs(ny - self.y)
                    heapq.heappush(frontier, (priority, (nx, ny)))
                    came_from[(nx, ny)] = current

        # Safety checks
        if goal is None or not (0 <= goal[0] < grid_h and 0 <= goal[1] < grid_w):
            return None

        # Final path reconstruction
        path = []
        while goal != start:
            path.append(goal)
            goal = came_from[goal]
        path.reverse()

        # Final validation
        if not path or any(p[0] < 0 or p[0] >= grid_h or p[1] < 0 or p[1] >= grid_w for p in path):
            return None

        return path


    def update_position(self, new_x, new_y):
        is_first_update = len(self.agents_actual_visited_cells) == 0
        # visualization.simulation_time += visualization.movement_time_step

        if is_first_update or (new_x, new_y) != (self.x, self.y):
            self.cells_travelled += 1
            self.agents_actual_visited_cells.append((new_x, new_y))

            if (new_x, new_y) in self.visited_cells:
                self.revisit_count += 1
            else:
                self.visited_cells.add((new_x, new_y))

            self.x, self.y = new_x, new_y

            # Mark globally as explored
            self.global_explored_cells.add((new_x, new_y))


    def is_cell_occupied(self, x, y):
        return any(agent.x == x and agent.y == y and agent != self for agent in self.agents)

    def reroute_around(self, blocked_cell):
        path = self.astar_to_next_unexplored_column()
        if path:
            target = path[-1]
            grid_w, grid_h = self.grid.size
            if 0 <= target[0] < grid_h and 0 <= target[1] < grid_w:
                # print(f"[{self.color}] Rerouting to {target}")
                self.path_queue = path
            # else:
                # print(f"[{self.color}] Rerouting target {target} is out of bounds! Ignoring reroute.")
        else:
            # print(f"[{self.color}] Couldn't find any valid reroute from {blocked_cell}")
            self.path_queue = []  # ← Add this to discard stale reroute steps




    def move(self, direction):
        """Move the agent in the given direction or use A* to find a path if blocked by a boundary."""
        if time.time() < self.wait_until:
            # print(f"Agent {self.color} is waiting at ({self.x}, {self.y})")
            return

        moves = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}

        def heuristic(x, y, goal_x, goal_y):
            """Manhattan distance heuristic."""
            return abs(x - goal_x) + abs(y - goal_y)

        def a_star(start, direction):
            """A* to find a path around the boundary."""
            moves = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}
            dx, dy = moves[direction]
            goal_x, goal_y = start[0] + dx, start[1] + dy
            # print(f"A* started from {start}, goal: ({goal_x}, {goal_y}), direction: {direction}")

            boundary_extent = find_boundary_extent(start[0], start[1], direction)
            if not boundary_extent:
                return None

            bypass_points = [(start[0], boundary_extent[0] - 1), (start[0], boundary_extent[1] + 1)]

            best_path = None
            best_path_length = float('inf')

            for bypass_x, bypass_y in bypass_points:
                if 0 <= bypass_y < self.grid.size[0]:
                    path = find_path_to_point(start, (bypass_x, bypass_y))
                    if path:
                        total_path = path + [(bypass_x, bypass_y)]
                        if len(total_path) < best_path_length:
                            best_path = total_path
                            best_path_length = len(total_path)

            if best_path:
                # print(f"A* found best path: {best_path}")
                return best_path
            else:
                # print("A* failed to find path")
                return None

        def find_boundary_extent(start_x, start_y, direction):
            # [Existing implementation unchanged]
            moves = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}
            dx, dy = moves[direction]

            min_y = start_y
            max_y = start_y

            while self.grid.is_boundary(start_x, min_y, direction) and min_y >= 0:
                min_y -= 1
            min_y += 1

            while self.grid.is_boundary(start_x, max_y, direction) and max_y < self.grid.size[0]:
                max_y += 1
            max_y -= 1

            if min_y <= max_y:
                return (min_y, max_y)
            else:
                return None

        def find_path_to_point(start, end):
            # [Existing implementation unchanged]
            moves = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}

            def heuristic(x, y, goal_x, goal_y):
                return abs(x - goal_x) + abs(y - goal_y)

            open_set = []
            heapq.heappush(open_set, (0, start[0], start[1], []))
            g_costs = {start: 0}
            visited = set()

            while open_set:
                f, x, y, path = heapq.heappop(open_set)
                if (x, y) == end:
                    return path
                visited.add((x, y))

                for dir, (dx, dy) in moves.items():
                    new_x, new_y = x + dx, y + dy
                    if not (0 <= new_x < self.grid.size[1] and 0 <= new_y < self.grid.size[0]) or (new_x, new_y) in visited:
                        continue
                    if self.grid.is_boundary(x, y, dir):
                        continue
                    new_g = g_costs[(x, y)] + 1
                    if (new_x, new_y) not in g_costs or new_g < g_costs[(new_x, new_y)]:
                        g_costs[(new_x, new_y)] = new_g
                        f_score = new_g + heuristic(new_x, new_y, end[0], end[1])
                        heapq.heappush(open_set, (f_score, new_x, new_y, path + [(x, y)]))
            return None
        
        # If there's a path in the queue, follow it
        if self.path_queue:
            step_x, step_y = self.path_queue[0]
            key = (step_x, step_y)
            
            if self.is_cell_occupied(step_x, step_y):
                # Track blocked attempts even during reroute path
                self.blocked_cell_attempts[key] = self.blocked_cell_attempts.get(key, 0) + 1
                # print(f"[{self.color}] Waiting on reroute step {key} - attempt {self.blocked_cell_attempts[key]}")
                # visualization.simulation_time += visualization.waiting_time_step

                if self.blocked_cell_attempts[key] >= self.reroute_threshold:
                    # print(f"[{self.color}] Rerouting from blocked reroute step {key}")
                    self.reroute_around(key)
                return visualization.waiting_time_step

            else:
                # print(f"[{self.color}] Executing reroute step {key}")
                self.blocked_cell_attempts.pop(key, None)
                self.path_queue.pop(0)
                self.update_position(step_x, step_y)
                return visualization.movement_time_step

        if direction in moves:
            new_x, new_y = self.x + moves[direction][0], self.y + moves[direction][1]

            if not (0 <= new_x < self.grid.size[1] and 0 <= new_y < self.grid.size[0]):
                # print(f"Move out of bounds")
                return

            if self.is_cell_occupied(new_x, new_y):
                key = (new_x, new_y)
                self.blocked_cell_attempts[key] = self.blocked_cell_attempts.get(key, 0) + 1

                if self.blocked_cell_attempts[key] >= self.reroute_threshold:  # Threshold to reroute
                    # print(f"[{self.color}] Waiting on cell {key} - attempt {self.blocked_cell_attempts[key]}")
                    # print(f"[{self.color}] Rerouting from blocked cell {key}")
                    self.reroute_around(key)
                # else:
                    # print(f"[{self.color}] Waiting on cell {key} - attempt {self.blocked_cell_attempts[key]}")
                return visualization.waiting_time_step


            # Check for boundary and run A* if needed
            if self.grid.is_boundary(self.x, self.y, direction):
                # print(f"Boundary detected, running A*")
                path = a_star((self.x, self.y), direction)

                if path is not None:
                    self.path_queue = path[1:]  # Store all steps except the current position
                    self.update_position(*path[0])
                    return visualization.movement_time_step
                else:
                    # print("A* returned None, agent stays still")
                    return visualization.waiting_time_step

            if 0 <= new_x < self.grid.size[1] and 0 <= new_y < self.grid.size[0]:
                self.update_position(new_x, new_y)
            if (new_x, new_y) != (self.x, self.y):
                self.blocked_cell_attempts.pop((new_x, new_y), None)  # Clear retry count
            
            return visualization.movement_time_step


    def plant(self):
        """Plant a seed if the plot is empty."""
        cell_info = self.grid.get_cell_info(self.x, self.y)
        if cell_info['crop_status'] == 0:
            self.wait_until = time.time() + 0.1  # Set personal wait timer
            # visualization.simulation_time += visualization.planting_time_step
            self.grid.update_cell(self.x, self.y, [cell_info['soil_type'], cell_info['moisture_level'], 1])

    def water(self):
        """Water the plot if it's dry."""
        cell_info = self.grid.get_cell_info(self.x, self.y)
        if cell_info['moisture_level'] == 0:
            self.wait_until = time.time() + 0.025  # Set personal wait timer
            # visualization.simulation_time += visualization.watering_time_step
            self.grid.update_cell(self.x, self.y, [cell_info['soil_type'], 1, cell_info['crop_status']])

    def execute_action(self, action):
        """Perform the selected action."""


        if self.is_frozen or action is None:
            self.done = True
            return  # Do nothing if frozen or no action

        self.done = False  # Reset, assuming we'll take a valid action

        if action in ['up', 'down', 'left', 'right']:
            move_time = self.move(action)
            return move_time 

        elif action == 'plant':
            self.plant()
            return visualization.planting_time_step
        elif action == 'water':
            self.water()
            return visualization.watering_time_step
            # Ensure watered cell is updated
            self.grid.update_cell(self.x, self.y, [self.grid.get_cell_info(self.x, self.y)['soil_type'],
                                                   1,  
                                                   self.grid.get_cell_info(self.x, self.y)['crop_status']])

        # After trying everything, recheck:
        if not self.path_queue and action is None:
            self.done = True


    def select_action(self):
        perception = StateEstimator(self.grid).perceive_environment(self)

        if perception['current']['moisture_level'] == 0 and perception['current']['crop_status'] == 1:
            return 'water'
        elif perception['current']['crop_status'] == 0:
            return 'plant'

        # if self.color == "purple" and (self.x, self.y) == (3, 6):
        #     self.is_frozen = True
        #     return 0.0
        # if self.is_frozen:
        #     return 0.0


        # If no local task, delegate to the injected behavior planner
        if hasattr(self, 'behavior_planner'):
            return self.behavior_planner.select_movement_action(self, perception, self.agents)


    def _move_towards_target(self, target_x, target_y):
        """Move towards the target cell with updated logic for last 2 columns."""
        if target_x < self.x:
            return 'left'
        elif target_x > self.x:
            return 'right'
        elif target_y < self.y:
            return 'up'
        elif target_y > self.y:
            return 'down'


    def get_revisit_percentage(self):
        """Calculate and return revisit percentage."""
        if self.cells_travelled == 0:
            return 0
        return (self.revisit_count / self.cells_travelled) * 100

    def print_stats(self, agents):
        """Print key statistics."""

        for i, agent in enumerate(agents):
            print(f"Agent {i+1} ({agent.color}) stats:")
            print(f"  Total cells travelled: {agent.cells_travelled}")
            print(f"  Total revisits: {agent.revisit_count}")
            print(f"  Percentage of revisited cells: {agent.get_revisit_percentage():.2f}%")
