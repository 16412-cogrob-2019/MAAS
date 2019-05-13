import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import scipy.ndimage


class Map:
    """docstring for Map"""

    def __init__(self, map_msg, add_blur=True):
        """
        :type map_msg: OccupancyGrid
        """
        self.array = self.update_grid(map_msg.data)
        self.height = map_msg.info.height  # Ngridpoints
        self.width = map_msg.info.width  # Ngridpoints
        self.res = map_msg.info.resolution  # m/cell
        self.origin = map_msg.info.origin
        self.pos = [self.origin.position.x, self.origin.position.y]
        self.grid = np.asarray(self.array, dtype=np.int8).reshape(self.height, self.width)

    def update_grid(self, grid):
        grid = list(grid)
        for i, value in enumerate(grid):
            if value < 0:
                grid[i] = 1
            else:
                grid[i] = value / 100.
        return grid

    def get_cell_pos(self, p):
        x_cell = int((p[0] - self.pos[0]) / self.res)
        y_cell = int((p[1] - self.pos[1]) / self.res)
        return x_cell, y_cell

    def get_cell_val(self, x, y):
        cell_pos_x, cell_pos_y = self.get_cell_pos([x, y])
        print(type(self.grid))
        print(self.grid)

        val = self.grid[cell_pos_x][cell_pos_y]
        return val


class MapMsg():
    def __init__(self, info, data):
        self.info = info
        self.data = data


class Info():
    def __init__(self, height, width, res, origin):
        self.height = height
        self.width = width
        self.resolution = res
        self.origin = origin


class Origin():
    def __init__(self, position):
        self.position = position


class Position():
    def __init__(self, x, y):
        self.x = x
        self.y = y
