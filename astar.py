import heapq
import math


class AStar:

    def __init__(self, clearance=1):

        self.clearance = clearance

    # estimate remaining path distance
    def heuristic(self, x1, y1, x2, y2):

        dx = abs(x1 - x2)
        dy = abs(y1 - y2)

        return (
            max(dx, dy) +
            (math.sqrt(2) - 1) * min(dx, dy)
        )

    # find shortest safe path through known free space
    def findPath(self, grid, startX, startY, goalX, goalY):

        rows = len(grid)
        cols = len(grid[0])

        if not self.isInside(startX, startY, cols, rows):
            return []

        if not self.isInside(goalX, goalY, cols, rows):
            return []

        if grid[startY][startX] != 0:
            return []

        if grid[goalY][goalX] != 0:
            return []

        openList = []
        cameFrom = {}
        bestCost = {(startX, startY): 0.0}

        startEstimate = self.heuristic(
            startX,
            startY,
            goalX,
            goalY
        )

        heapq.heappush(
            openList,
            (
                startEstimate,
                0.0,
                startX,
                startY
            )
        )

        while openList:

            _, currentCost, currentX, currentY = heapq.heappop(openList)

            if currentCost > bestCost.get((currentX, currentY), float("inf")):
                continue

            if currentX == goalX and currentY == goalY:
                return self.buildPath(
                    cameFrom,
                    startX,
                    startY,
                    goalX,
                    goalY
                )

            for offsetX, offsetY in [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1)
            ]:

                nextX = currentX + offsetX
                nextY = currentY + offsetY

                if not self.isSafe(
                    grid,
                    nextX,
                    nextY,
                    cols,
                    rows
                ):
                    continue

                if offsetX != 0 and offsetY != 0:

                    if not self.isSafe(
                        grid,
                        currentX + offsetX,
                        currentY,
                        cols,
                        rows
                    ):
                        continue

                    if not self.isSafe(
                        grid,
                        currentX,
                        currentY + offsetY,
                        cols,
                        rows
                    ):
                        continue

                    movementCost = math.sqrt(2)
                else:
                    movementCost = 1.0

                newCost = currentCost + movementCost
                nextPosition = (nextX, nextY)

                if newCost >= bestCost.get(nextPosition, float("inf")):
                    continue

                bestCost[nextPosition] = newCost
                cameFrom[nextPosition] = (currentX, currentY)

                estimate = newCost + self.heuristic(
                    nextX,
                    nextY,
                    goalX,
                    goalY
                )

                heapq.heappush(
                    openList,
                    (
                        estimate,
                        newCost,
                        nextX,
                        nextY
                    )
                )

        return []

    def isInside(self, x, y, cols, rows):

        return (
            0 <= x < cols and
            0 <= y < rows
        )

    # keep planned path away from known walls
    def isSafe(self, grid, x, y, cols, rows):

        if not self.isInside(x, y, cols, rows):
            return False

        if grid[y][x] != 0:
            return False

        for offsetY in range(-self.clearance, self.clearance + 1):

            for offsetX in range(-self.clearance, self.clearance + 1):

                checkX = x + offsetX
                checkY = y + offsetY

                if not self.isInside(checkX, checkY, cols, rows):
                    return False

                if grid[checkY][checkX] == 1:
                    return False

        return True

    def buildPath(self, cameFrom, startX, startY, goalX, goalY):

        current = (goalX, goalY)
        path = [current]

        while current != (startX, startY):

            current = cameFrom.get(current)

            if current is None:
                return []

            path.append(current)

        path.reverse()

        return path
