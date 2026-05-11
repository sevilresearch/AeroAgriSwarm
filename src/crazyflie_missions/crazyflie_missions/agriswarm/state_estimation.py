import numpy as np

class StateEstimator:
    def __init__(self, grid):
        self.grid = grid
    
    def perceive_environment(self, agent):
        """Perceive the current cell and its surroundings."""
        x, y = agent.x, agent.y
        perception = {'current': self.grid.get_cell_info(x, y)}
        return perception
    
    # def is_plot_dry(self, x, y):
    #     """Check if a plot is dry."""
    #     return self.grid.get_cell_info(x, y)['moisture_level'] == 0
    
    # def is_plot_empty(self, x, y):
    #     """Check if a plot is empty (ready for planting)."""
    #     return self.grid.get_cell_info(x, y)['crop_status'] == 0


