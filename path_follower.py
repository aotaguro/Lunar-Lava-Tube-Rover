import math


class PathFollower:

    def __init__(self):

        self.path = []
        self.currentIndex = 0
        self.speed = 2
        self.blocked = False

    # give follower a new path
    def setPath(self, path):

        self.path = path
        self.currentIndex = 1 if len(path) > 1 else len(path)
        self.blocked = False

    # clear current path
    def clearPath(self):

        self.path = []
        self.currentIndex = 0

    # move rover along path
    def update(self, rover, gridMap, world, dt):

        if self.finished():
            return

        gridX, gridY = self.path[self.currentIndex]

        if not gridMap.isSafeCell(
            gridX,
            gridY,
            clearance=0
        ):
            self.blocked = True
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

        if distance < 0.25:
            self.currentIndex += 1
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
            clearance=0
        ):
            self.blocked = True
            return

        lidarHeight = model.getZ() + 0.8

        if not world.isPathClear(
            currentX,
            currentY,
            newX,
            newY,
            lidarHeight
        ):
            self.blocked = True
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

    def isBlocked(self):

        return self.blocked

    def finished(self):

        return (
            not self.path or
            self.currentIndex >= len(self.path)
        )
