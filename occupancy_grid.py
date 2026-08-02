#stores a simple occupancy grid map
class OccupancyGrid:


    def __init__(self, width, height, cellSize):


        #number of cells in x direction
        self.width = width


        #number of cells in y direction
        self.height = height


        #size of each cell in world units
        self.cellSize = cellSize


        #center of the map
        self.originX = width // 2
        self.originY = height // 2



        #create unknown map
        # -1 = unknown
        #  0 = free space
        #  1 = obstacle

        self.grid = [

            [-1 for x in range(width)]

            for y in range(height)

        ]



    #clear the map back to unknown
    def clear(self):


        for y in range(self.height):

            for x in range(self.width):

                self.grid[y][x] = -1





    #convert lidar points into occupied cells
    def update(self, points):


        for point in points:


            worldX = point.x
            worldY = point.y



            gridX = int(worldX / self.cellSize) + self.originX
            gridY = int(worldY / self.cellSize) + self.originY



            #make sure point is inside map

            if (

                0 <= gridX < self.width and
                0 <= gridY < self.height

            ):


                #this location contains a wall

                self.grid[gridY][gridX] = 1





    #convert world position to grid position
    def worldToGrid(self, x, y):


        gridX = int(x / self.cellSize) + self.originX
        gridY = int(y / self.cellSize) + self.originY


        return gridX, gridY





    #get current map
    def getGrid(self):

        return self.grid