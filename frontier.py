from collections import deque


class Frontier:

    def __init__(self, x, y, size):

        self.x = x
        self.y = y
        self.size = size


class FrontierDetector:

    def __init__(self, minimumSize=2):

        self.minimumSize = minimumSize

    # find groups of free cells beside unknown space
    def detect(self, grid):

        rows = len(grid)
        cols = len(grid[0])

        visited = set()
        frontiers = []

        for y in range(rows):

            for x in range(cols):

                if (x, y) in visited:
                    continue

                if not self.isFrontier(grid, x, y):
                    continue

                cluster = self.buildCluster(
                    grid,
                    x,
                    y,
                    visited
                )

                if len(cluster) < self.minimumSize:
                    continue

                target = self.chooseClusterTarget(
                    grid,
                    cluster
                )

                if target is None:
                    continue

                frontiers.append(
                    Frontier(
                        target[0],
                        target[1],
                        len(cluster)
                    )
                )

        return frontiers

    # collect connected frontier cells
    def buildCluster(self, grid, startX, startY, visited):

        queue = deque([(startX, startY)])
        visited.add((startX, startY))

        cluster = []

        while queue:

            currentX, currentY = queue.popleft()
            cluster.append((currentX, currentY))

            for nextX, nextY in [
                (currentX + 1, currentY),
                (currentX - 1, currentY),
                (currentX, currentY + 1),
                (currentX, currentY - 1)
            ]:

                if (nextX, nextY) in visited:
                    continue

                if not self.isFrontier(grid, nextX, nextY):
                    continue

                visited.add((nextX, nextY))
                queue.append((nextX, nextY))

        return cluster

    # choose a real free cell closest to cluster center
    def chooseClusterTarget(self, grid, cluster):

        centerX = sum(point[0] for point in cluster) / len(cluster)
        centerY = sum(point[1] for point in cluster) / len(cluster)

        candidates = [
            point for point in cluster
            if self.isSafe(grid, point[0], point[1])
        ]

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda point: (
                (point[0] - centerX) ** 2 +
                (point[1] - centerY) ** 2
            )
        )

    def isFrontier(self, grid, x, y):

        rows = len(grid)
        cols = len(grid[0])

        if not (
            0 <= x < cols and
            0 <= y < rows
        ):
            return False

        if grid[y][x] != 0:
            return False

        for neighborX, neighborY in [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1)
        ]:

            if not (
                0 <= neighborX < cols and
                0 <= neighborY < rows
            ):
                continue

            if grid[neighborY][neighborX] == -1:
                return True

        return False

    # keep targets away from known wall cells
    def isSafe(self, grid, x, y):

        rows = len(grid)
        cols = len(grid[0])

        for offsetY in range(-1, 2):

            for offsetX in range(-1, 2):

                checkX = x + offsetX
                checkY = y + offsetY

                if not (
                    0 <= checkX < cols and
                    0 <= checkY < rows
                ):
                    return False

                if grid[checkY][checkX] == 1:
                    return False

        return True
