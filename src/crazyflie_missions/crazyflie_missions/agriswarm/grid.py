import numpy as np
import random

class Grid:
    def __init__(self, size=(52, 50)):
        self.size = size
        self.grid = np.zeros((size[0], size[1]), dtype=[('soil_type', 'i4'), ('moisture_level', 'i4'), ('crop_status', 'i4')])
        self.boundaries = np.zeros((size[0], size[1]-1), dtype=bool)  # Boundary exists between columns
        self.boundary_density = round(1.00, 2)
        self.initialize_grid()

    def initialize_grid(self):
        """Initialize the grid with specific dry columns and random values elsewhere."""
        for y in range(self.size[0]):
            for x in range(self.size[1]):
                # Case 1-6:
                soil  = np.random.choice([1,0], p=[0.5,0.5])
                # Case 7: Random Distribution
                # fertile_probability = np.random.uniform(0.0,1.0)
                # soil = np.random.choice([1,0], p=[fertile_probability,1-fertile_probability])
                # Case 8: Column-Based Distribution
                # if x % 2 == 0:
                #   soil = 1
                # else:
                #   soil = 0
                # Case 9: Row-Based Distribution
                # if y % 2 == 0:
                #   soil = 1
                # else:
                #   soil = 0
                # Case 10: Highly Heterogenous
                # patch_size = 5
                # patch_row = y // patch_size
                # patch_col = x // patch_size
                # np.random.seed(patch_row * 100 + patch_col)
                # soil = np.random.choice([1,0], p=[0.5,0.5])
                moisture = np.random.choice([0, 1], p=[0.5, 0.5])
                crop = np.random.choice([0, 1], p=[0.5, 0.5])
                self.grid[y, x] = (soil, moisture, crop)

        # Set vertical boundaries between columns (exclude first & last rows)
        for y in range(self.size[0]):
            for x in range(1, self.size[1]):
                    if y != 0 and y != self.size[0] - 1:
                        if random.random() < self.boundary_density:
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
