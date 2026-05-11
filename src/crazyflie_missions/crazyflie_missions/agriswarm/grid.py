
# 10 x 10 grid
# import numpy as np

# class Grid:
#     def __init__(self, size=15):
#         self.size = size
#         self.grid = np.zeros((size, size), dtype=[('soil_type', 'i4'), ('moisture_level', 'i4'), ('crop_status', 'i4')])
#         self.initialize_grid()
    
#     def initialize_grid(self):
#         """Initialize the grid with random values."""
#         self.grid['soil_type'] = np.random.choice([0, 1], size=(self.size, self.size))  # 0 = poor, 1 = fertile
#         self.grid['moisture_level'] = np.random.choice([0, 1], size=(self.size, self.size))  # 0 = dry, 1 = wet
#         self.grid['crop_status'] = np.random.choice([0, 1], size=(self.size, self.size))  # 0 = empty, 1 = planted
    
#     def get_cell_info(self, x, y):
#         """Return the state of a specific cell."""
#         if 0 <= x < self.size and 0 <= y < self.size:
#             return self.grid[x, y]
#         else:
#             raise ValueError("Coordinates out of bounds")
    
#     def update_cell(self, x, y, new_values):
#         """Update the properties of a cell."""
#         if 0 <= x < self.size and 0 <= y < self.size:
#             self.grid[x, y] = tuple(new_values)
#         else:
#             raise ValueError("Coordinates out of bounds")


# 8x15 grid
import numpy as np

class Grid:
    def __init__(self, size=(7, 15)):
        self.size = size
        self.grid = np.zeros((size[0], size[1]), dtype=[('soil_type', 'i4'), ('moisture_level', 'i4'), ('crop_status', 'i4')])
        self.boundaries = np.zeros((size[0], size[1]-1), dtype=bool)  # Boundary exists between columns
        self.initialize_grid()

    # def initialize_grid(self):
    #     """Initialize the grid with random values and obstacles."""
    #     # Initialize soil, moisture, and crop status
    #     # self.grid['soil_type'] = np.random.choice([0, 1], size=self.size)  # 0 = poor, 1 = fertile
    #     # self.grid['moisture_level'] = np.random.choice([0, 1], size=self.size)  # 0 = dry, 1 = wet
    #     # self.grid['crop_status'] = np.random.choice([0, 1], size=self.size)  # 0 = empty, 1 = planted
        
    #     # self.grid['soil_type'] = np.random.choice([0, 1], size=self.size)  # 0 = poor, 1 = fertile
    #     self.grid['moisture_level'] = np.random.choice([0, 0], size=self.size)  # 0 = dry, 1 = wet
    #     self.grid['crop_status'] = np.random.choice([1, 1], size=self.size)  # 0 = empty, 1 = planted



    #     # # Set obstacles in the last row except for (8, 0), (8, 1), and (8, 2)
    #     # for x in range(self.size[1]):
    #     #     if x not in [0, 1, 2]:
    #     #         self.grid[7, x] = (2,1,0)  # Mark these as obstacles (soil type: fertile, moisture: wet, crop status: planted)
        
    #     for y in range(self.size[0]):  # Iterate through rows
    #         for x in range(1, self.size[1]):  # Exclude first column for boundaries
    #             if y != 0 and y != self.size[0] - 1:  # Not the first or last row
    #                 self.boundaries[y, x - 1] = True  # Mark boundary between columns (excluding first and last rows)


    def initialize_grid(self):
        """Initialize the grid with specific dry columns and random values elsewhere."""
        for y in range(self.size[0]):
            for x in range(self.size[1]):
                if x % 3 == 0:  # Every 3rd column (0-based indexing): col 0, 3, 6, 9...
                    self.grid[y, x] = (1, 0, 0)  # fertile soil, dry, unplanted
                else:
                    # Randomize, but avoid moisture=0 and crop_status=0 simultaneously
                    moisture = np.random.choice([0, 1])
                    crop_status = np.random.choice([0, 1])
                    if moisture == 0 and crop_status == 0:
                        moisture = 1  # Force at least some water or crop
                    soil = np.random.choice([0, 1])  # poor or fertile
                    self.grid[y, x] = (soil, moisture, crop_status)

        # Set vertical boundaries between columns (exclude first & last rows)
        for y in range(self.size[0]):
            for x in range(1, self.size[1]):
                if y != 0 and y != self.size[0] - 1:
                    self.boundaries[y, x - 1] = True



    
    def is_boundary(self, x, y, direction):
        """Check if there's a boundary in the direction the agent wants to move."""
        if direction == 'right' and x < self.size[1] - 1:
            return self.boundaries[y, x]  # Check right, use x instead of x + 1
        elif direction == 'left' and x > 0:
            return self.boundaries[y, x - 1]  # Check left
        return False  # No boundary for up/down movement
        
    def get_cell_info(self, x, y):
        """Return the state of a specific cell."""
        if 0 <= x < self.size[1] and 0 <= y < self.size[0]:
            return self.grid[y, x]
        else:
            raise ValueError("Coordinates out of bounds")
            print(f"[ERROR] Invalid cell access: ({x}, {y})")
            raise ValueError("Coordinates out of bounds")
            return self.grid[y][x]
    
    def update_cell(self, x, y, new_values):
        """Update the properties of a cell."""
        if 0 <= x < self.size[1] and 0 <= y < self.size[0]:
            self.grid[y, x] = tuple(new_values)
        else:
            raise ValueError("Coordinates out of bounds")


    def is_cell_occupied(self, x, y, agents, current_agent):
        for agent in agents:
            if agent != current_agent and agent.x == x and agent.y == y:
                return True
        return False
