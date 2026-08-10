import math


class FrontierPlanner:

    def __init__(self):

        self.visited = []
        self.failed = []

        # only block frontiers very close to old completed targets
        self.visitedRadius = 1.5
        self.failedRadius = 2

        # starting location is saved the first time a target is chosen
        self.startX = None
        self.startY = None

        # once the rover moves away, keep it from choosing the entrance again
        self.startAvoidDistance = 7
        self.startBlockRadius = 5

    # choose the best frontier that has not been completed
    def chooseFrontier(self, rover, frontiers, gridMap):

        if not frontiers:
            return None

        roverModel = rover.getModel()

        roverGX, roverGY = gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

        # save the rover starting grid position
        if self.startX is None:
            self.startX = roverGX
            self.startY = roverGY

        roverStartDistance = math.hypot(
            roverGX - self.startX,
            roverGY - self.startY
        )

        best = None
        bestScore = float("inf")

        for frontier in frontiers:

            if self.wasVisited(frontier):
                continue

            if not gridMap.isSafeCell(
                frontier.x,
                frontier.y,
                clearance=1
            ):
                continue

            frontierStartDistance = math.hypot(
                frontier.x - self.startX,
                frontier.y - self.startY
            )

            # after leaving the start, never choose the entrance area again
            if (
                roverStartDistance > self.startAvoidDistance
                and
                frontierStartDistance < self.startBlockRadius
            ):
                continue

            distance = math.hypot(
                frontier.x - roverGX,
                frontier.y - roverGY
            )

            failedPenalty = 0

            if self.wasFailed(frontier):
                failedPenalty = 20

            # prefer nearby large frontiers while still encouraging deeper exploration
            score = (
                distance
                - frontier.size * 0.75
                - frontierStartDistance * 0.3
                + failedPenalty
            )

            if score < bestScore:
                bestScore = score
                best = frontier

        return best

    def wasVisited(self, frontier):

        for oldX, oldY in self.visited:

            if math.hypot(
                frontier.x - oldX,
                frontier.y - oldY
            ) < self.visitedRadius:
                return True

        return False

    def wasFailed(self, frontier):

        for oldX, oldY in self.failed:

            if math.hypot(
                frontier.x - oldX,
                frontier.y - oldY
            ) < self.failedRadius:
                return True

        return False

    # remember a completed frontier
    def addVisited(self, frontier):

        if frontier is None:
            return

        self.visited.append(
            (
                frontier.x,
                frontier.y
            )
        )

    # remember a target that could not be reached
    def addFailed(self, frontier):

        if frontier is None:
            return

        self.failed.append(
            (
                frontier.x,
                frontier.y
            )
        )

        if len(self.failed) > 20:
            self.failed.pop(0)

    # return distance from the starting point for debugging
    def getDistanceFromStart(self, rover, gridMap):

        if self.startX is None:
            return 0

        roverModel = rover.getModel()

        roverGX, roverGY = gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

        return math.hypot(
            roverGX - self.startX,
            roverGY - self.startY
        )
