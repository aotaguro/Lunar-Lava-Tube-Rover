import math


class FrontierPlanner:

    def __init__(self):

        self.visited = []
        self.failed = []
        self.visitedRadius = 3
        self.failedRadius = 2

    # choose the best frontier that has not been completed
    def chooseFrontier(self, rover, frontiers, gridMap):

        if not frontiers:
            return None

        roverModel = rover.getModel()

        roverGX, roverGY = gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
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

            distance = math.hypot(
                frontier.x - roverGX,
                frontier.y - roverGY
            )

            failedPenalty = 0

            if self.wasFailed(frontier):
                failedPenalty = 25

            # prefer larger frontiers and avoid recently blocked targets
            score = (
                distance -
                frontier.size * 0.75 +
                failedPenalty
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
