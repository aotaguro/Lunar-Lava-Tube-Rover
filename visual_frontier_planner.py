import math

from frontier_planner import FrontierPlanner


class VisualFrontierPlanner(FrontierPlanner):

    # camera mapping works best when it moves toward the edge of visible space
    def chooseFrontier(self, rover, frontiers, gridMap):

        if not frontiers:
            return None

        roverModel = rover.getModel()

        roverGX, roverGY = gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

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

            failedPenalty = 20 if self.wasFailed(frontier) else 0

            # prefer the outer edge of the camera's current mapped area
            score = (
                distance * 0.2
                - frontierStartDistance * 1.0
                - frontier.size * 0.5
                + failedPenalty
            )

            if score < bestScore:
                bestScore = score
                best = frontier

        return best
