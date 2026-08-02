import math


# chooses which frontier the rover should explore
class FrontierPlanner:


    def __init__(self):

        # stores current exploration target
        self.target = None



    # chooses closest frontier
    def chooseFrontier(self, rover, frontiers):


        # no frontiers found
        if len(frontiers) == 0:

            return None



        # rover position
        roverX = rover.getModel().getX()
        roverY = rover.getModel().getY()



        closest = None

        closestDistance = float("inf")



        # check every frontier
        for frontier in frontiers:



            # calculate distance to frontier
            distance = math.sqrt(

                (frontier.x - roverX) ** 2 +

                (frontier.y - roverY) ** 2

            )



            # if this is closest
            if distance < closestDistance:


                closestDistance = distance

                closest = frontier



        # store selected target
        self.target = closest



        return closest