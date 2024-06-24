import numpy as np
import matplotlib.pyplot as plt

# Hyperparameters for grid mapping
grid_size = 10  # the size of each grid in cm
map_size = 200  # the size of the map in cm
robot_pos = 0   # the initial position of the robot
robot_orientation = 1 # the initial orientation of the robot
prior = 0.5 # the prior probability of the grid

grid_num = int(map_size / grid_size) # the number of grids
grid_range = [0, map_size]  # the range of the grid map


# Define inverse sensor model
def inverse_sensor_model(grid_pos, measurement):
    # find the grid to which the measurement corresponds
    measurement_grid = measurement // grid_size

    if grid_pos < measurement_grid:
        return 0.3
    elif grid_pos <= measurement_grid + 2:
        return 0.6
    else:
        return 0.5


# Define a 1D grid mapping class
class GridMap1D():
    def __init__(self, grid_size, grid_num, grid_range, prior):
        """Initialize the grid map
        Args:
            grid_size: the size of each grid
            grid_num: the number of grids
            grid_range: the range of the grid map
        """
        self.prior = prior
        self.grid_size = grid_size
        self.grid_num = grid_num
        self.grid_range = grid_range
        self.grid_map = np.ones(grid_num) * prior

    def plot(self):
        """Plot the grid map
        """
        plt.bar(np.arange(self.grid_num), self.grid_map, width=1.0, color='b')
        plt.show()

    def update(self, measurement):
        """Update the grid map
        Args:
            pos: the position of the object
        """
        
        for i in range(grid_num):
            p_m_z = inverse_sensor_model(i, measurement)
            z_term = (1-p_m_z) / p_m_z
            recursive_term = (1-self.grid_map[i]) / self.grid_map[i]
            prior_term = 0 #self.prior / (1 - self.prior)

            self.grid_map[i] = 1 / (1 + z_term + prior_term + recursive_term)




            # self.grid_map[i] = 1 / (1 + (1 - p_m_z)/p_m_z + prior/(1-prior) 
                                    # + (1-self.grid_map[i])/self.grid_map[i])
            
# Create a grid map
grid_map = GridMap1D(grid_size, grid_num, grid_range, prior)

# measurements = [101, 82, 91, 112, 99, 151, 96, 85, 99, 105]  # the measurements of the object
measurements = [10]  # the measurements of the object


# Update the grid map
for measurement in measurements:
    grid_map.update(measurement)

# Plot the grid map
grid_map.plot()
