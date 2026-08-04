import math


class FrontierPlanner:

    def __init__(self):

        self.visited = []
        self.visitedRadius = 6

    # choose a useful frontier that has not been visited
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

            # larger frontier groups are more useful
            score = distance - frontier.size * 0.75

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

    def addVisited(self, frontier):

        if frontier is None:
            return

        self.visited.append(
            (
                frontier.x,
                frontier.y
            )
        )
