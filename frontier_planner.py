import math


#chooses which frontier the rover should explore
class FrontierPlanner:


    def __init__(self):

        pass


    #returns the closest frontier
    def chooseFrontier(
        self,
        frontiers,
        roverX,
        roverY
    ):

        #no frontiers found
        if len(frontiers) == 0:

            return None


        closest = None

        shortestDistance = float("inf")


        #check every frontier
        for frontier in frontiers:


            dx = frontier.x - roverX
            dy = frontier.y - roverY

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )


            if distance < shortestDistance:

                shortestDistance = distance

                closest = frontier


        return closest