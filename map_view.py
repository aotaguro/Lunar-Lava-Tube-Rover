from panda3d.core import CardMaker
from panda3d.core import NodePath


#draws a top-down occupancy grid
class MapView:

    def __init__(self, render, grid):

        #reference to the occupancy grid
        self.grid = grid

        #parent node for every cell
        self.mapNode = render.attachNewNode("OccupancyMap")

        #size of one square
        self.cellSize = 0.20

        #move map beside the cave
        self.mapNode.setPos(35, 0, 0)

        #rotate so we're looking down on it
        self.mapNode.setHpr(0, -90, 0)


    #draw map
    def draw(self):

        #erase previous frame
        self.mapNode.getChildren().detach()

        grid = self.grid.getGrid()

        height = len(grid)
        width = len(grid[0])

        for y in range(height):

            for x in range(width):

                #ignore empty cells
                if grid[y][x] == 0:
                    continue

                card = CardMaker("cell")

                card.setFrame(
                    0,
                    self.cellSize,
                    0,
                    self.cellSize
                )

                square = self.mapNode.attachNewNode(
                    card.generate()
                )

                square.setPos(
                    x * self.cellSize,
                    y * self.cellSize,
                    0
                )

                #black occupied cells
                square.setColor(0, 0, 0, 1)