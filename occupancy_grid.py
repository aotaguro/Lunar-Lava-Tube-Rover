import math



#creates a 2D occupancy grid map
class OccupancyGrid:


    def __init__(self, width, height, resolution):


        #number of cells horizontally
        self.width = width

        #number of cells vertically
        self.height = height


        #size of each grid cell in meters
        self.resolution = resolution



        #create empty map

        #0 = unknown/free
        #1 = occupied

        self.grid = [

            [0 for x in range(width)]

            for y in range(height)

        ]



    #convert world coordinates into grid coordinates
    def worldToGrid(self, x, y):


        gridX = int(
            x / self.resolution
            +
            self.width / 2
        )


        gridY = int(
            y / self.resolution
            +
            self.height / 2
        )


        return gridX, gridY




    #add lidar points to map
    def update(self, points):


        for point in points:


            gridX, gridY = self.worldToGrid(
                point.x,
                point.y
            )



            #make sure point is inside map

            if (

                gridX >= 0 and

                gridX < self.width and

                gridY >= 0 and

                gridY < self.height

            ):


                #mark obstacle

                self.grid[gridY][gridX] = 1




    #return current map
    def getGrid(self):

        return self.grid