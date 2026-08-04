import math


class FrontierPlanner:


    def __init__(self):

        # previously explored frontier locations
        self.visited = []



    def chooseFrontier(self, rover, frontiers, gridMap):


        if len(frontiers) == 0:

            return None



        roverModel = rover.getModel()


        roverX = roverModel.getX()
        roverY = roverModel.getY()



        # convert rover world position to grid position

        roverGX, roverGY = gridMap.worldToGrid(
            roverX,
            roverY
        )



        best = None

        bestScore = float("inf")



        for frontier in frontiers:



            # ignore already explored areas

            alreadyVisited = False



            for old in self.visited:


                distance = math.hypot(

                    frontier.x - old[0],

                    frontier.y - old[1]

                )


                if distance < 10:

                    alreadyVisited = True
                    break



            if alreadyVisited:

                continue



            # compare grid coordinates

            distance = math.hypot(

                frontier.x - roverGX,

                frontier.y - roverGY

            )



            if distance < bestScore:


                bestScore = distance

                best = frontier



        return best





    def addVisited(self, frontier):


        if frontier is None:

            return



        self.visited.append(

            (
                frontier.x,
                frontier.y
            )

        )