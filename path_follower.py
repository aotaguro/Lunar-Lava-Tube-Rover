import math


class PathFollower:


    def __init__(self):

        # current path from A*
        self.path = []

        # which point the rover is moving toward
        self.currentIndex = 0

        # movement speed
        self.speed = 2



    # give the follower a new path
    def setPath(self, path):

        self.path = path

        self.currentIndex = 0



    # move rover along path
    def update(self, rover, dt):


        # no path to follow
        if len(self.path) == 0:

            return



        # finished path
        if self.currentIndex >= len(self.path):

            return



        # current target grid location

        targetX = self.path[self.currentIndex][0]

        targetY = self.path[self.currentIndex][1]



        # get rover position

        model = rover.getModel()


        currentX = model.getX()

        currentY = model.getY()



        # find direction to target

        dx = targetX - currentX

        dy = targetY - currentY



        distance = math.sqrt(

            dx * dx +
            dy * dy

        )



        # reached waypoint

        if distance < 0.2:


            self.currentIndex += 1

            return



        # normalize direction

        dx /= distance

        dy /= distance



        # move rover

        model.setX(

            model.getX() +
            dx *
            self.speed *
            dt

        )


        model.setY(

            model.getY() +
            dy *
            self.speed *
            dt

        )