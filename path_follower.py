import math


class PathFollower:

    def __init__(self):

        self.path = []
        self.currentIndex = 0
        self.speed = 2
        self.blocked = False
        self.blockedFrames = 0

    # give follower a new path
    def setPath(self, path):

        self.path = path
        self.currentIndex = 1 if len(path) > 1 else len(path)
        self.blocked = False
        self.blockedFrames = 0

    # clear current path
    def clearPath(self):

        self.path = []
        self.currentIndex = 0
        self.blocked = False
        self.blockedFrames = 0

    # move rover along path
    def update(self, rover, gridMap, world, dt):

        if self.finished():
            return

        gridX, gridY = self.path[self.currentIndex]

        if not gridMap.isSafeCell(
            gridX,
            gridY,
            clearance=1
        ):
            self.setBlocked()
            return

        targetX, targetY = gridMap.gridToWorld(
            gridX,
            gridY
        )

        model = rover.getModel()

        currentX = model.getX()
        currentY = model.getY()

        directionX = targetX - currentX
        directionY = targetY - currentY

        distance = math.hypot(
            directionX,
            directionY
        )

        if distance < 0.3:
            self.currentIndex += 1
            self.blockedFrames = 0
            return

        directionX /= distance
        directionY /= distance

        movement = min(
            self.speed * dt,
            distance
        )

        newX = currentX + directionX * movement
        newY = currentY + directionY * movement

        newGX, newGY = gridMap.worldToGrid(
            newX,
            newY
        )

        if not gridMap.isSafeCell(
            newGX,
            newGY,
            clearance=1
        ):
            self.setBlocked()
            return

        lidarHeight = model.getZ() + 0.8

        if not world.isRoverMoveClear(
            currentX,
            currentY,
            newX,
            newY,
            lidarHeight,
            radius=1.1
        ):
            self.setBlocked()
            return

        heading = math.degrees(
            math.atan2(
                directionY,
                directionX
            )
        ) - 90

        model.setH(heading)
        model.setX(newX)
        model.setY(newY)

        self.blocked = False
        self.blockedFrames = 0

    def setBlocked(self):

        self.blockedFrames += 1

        if self.blockedFrames >= 3:
            self.blocked = True

    def isBlocked(self):

        return self.blocked

    def finished(self):

        return (
            not self.path or
            self.currentIndex >= len(self.path)
        )
