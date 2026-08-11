class OccupancyGrid:

    def __init__(self, width, height, cellSize):

        self.width = width
        self.height = height
        self.cellSize = cellSize

        # keep rover centered left and right
        self.originX = width // 2

        # start rover near bottom so the map extends deeper into cave
        self.originY = 30

        # -1 = unknown
        # 0 = free
        # 1 = obstacle
        self.grid = [
            [-1 for _ in range(width)]
            for _ in range(height)
        ]

    # convert world position into grid position
    def worldToGrid(self, x, y):

        gridX = round(x / self.cellSize) + self.originX
        gridY = round(y / self.cellSize) + self.originY

        return gridX, gridY

    # convert grid position into world position
    def gridToWorld(self, x, y):

        worldX = (x - self.originX) * self.cellSize
        worldY = (y - self.originY) * self.cellSize

        return worldX, worldY

    # update map using lidar rays
    def update(self, lidarHits):

        for hit in lidarHits:

            startGX, startGY = self.worldToGrid(
                hit.startX,
                hit.startY
            )

            endGX, endGY = self.worldToGrid(
                hit.endX,
                hit.endY
            )

            cells = self.getLine(
                startGX,
                startGY,
                endGX,
                endGY
            )

            freeCells = cells[:-1] if hit.hit else cells

            for x, y in freeCells:

                if self.isInside(x, y) and self.grid[y][x] != 1:
                    self.grid[y][x] = 0

            if hit.hit and self.isInside(endGX, endGY):
                self.grid[endGY][endGX] = 1

        # rover cell is always known free space
        if lidarHits:

            roverGX, roverGY = self.worldToGrid(
                lidarHits[0].startX,
                lidarHits[0].startY
            )

            if self.isInside(roverGX, roverGY):
                self.grid[roverGY][roverGX] = 0

    # create cells between two grid positions
    def getLine(self, x0, y0, x1, y1):

        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        error = dx - dy

        while True:

            cells.append((x0, y0))

            if x0 == x1 and y0 == y1:
                break

            error2 = 2 * error

            if error2 > -dy:
                error -= dy
                x0 += sx

            if error2 < dx:
                error += dx
                y0 += sy

        return cells

    def isInside(self, x, y):

        return (
            0 <= x < self.width and
            0 <= y < self.height
        )

    def isFree(self, x, y):

        return self.isInside(x, y) and self.grid[y][x] == 0

    def isOccupied(self, x, y):

        if not self.isInside(x, y):
            return True

        return self.grid[y][x] == 1

    # check if rover center can safely enter a cell
    def isSafeCell(self, x, y, clearance=0):

        if not self.isFree(x, y):
            return False

        for offsetY in range(-clearance, clearance + 1):

            for offsetX in range(-clearance, clearance + 1):

                checkX = x + offsetX
                checkY = y + offsetY

                if not self.isInside(checkX, checkY):
                    return False

                if self.grid[checkY][checkX] == 1:
                    return False

        return True

    def getGrid(self):

        return self.grid
