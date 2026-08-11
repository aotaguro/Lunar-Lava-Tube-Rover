from occupancy_grid import OccupancyGrid


class VisualOccupancyGrid(OccupancyGrid):

    def __init__(self, width, height, cellSize):

        super().__init__(
            width,
            height,
            cellSize
        )
