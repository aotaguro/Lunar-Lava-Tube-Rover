#stores one frontier location
class Frontier:

    def __init__(self, x, y):

        self.x = x
        self.y = y



#finds exploration frontiers
class FrontierDetector:

    def __init__(self):

        pass



    #search occupancy grid
    def detect(self, grid):

        frontiers = []

        rows = len(grid)
        cols = len(grid[0])

        for y in range(1, rows - 1):

            for x in range(1, cols - 1):

                #skip occupied cells
                if grid[y][x] == 1:
                    continue

                #look at neighboring cells
                neighbors = [

                    grid[y-1][x],
                    grid[y+1][x],
                    grid[y][x-1],
                    grid[y][x+1]

                ]

                #frontier if any neighbor is occupied
                if 1 in neighbors:

                    frontiers.append(
                        Frontier(x, y)
                    )

        return frontiers