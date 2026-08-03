import heapq


# stores one search node
class AStarNode:

    def __init__(self, x, y):

        # grid position
        self.x = x
        self.y = y

        # A* values
        # distance traveled
        self.g = 0

        # estimated distance to goal
        self.h = 0

        # total cost
        self.f = 0

        # previous node in path
        self.parent = None


    # allows heapq to compare nodes
    def __lt__(self, other):

        return self.f < other.f



# A* pathfinding algorithm
class AStar:

    def __init__(self):

        pass



    # estimates distance to goal
    def heuristic(self, x1, y1, x2, y2):

        return abs(x1 - x2) + abs(y1 - y2)



    # finds shortest path through grid
    def findPath(self, grid, startX, startY, goalX, goalY):

        # nodes waiting to be checked
        openList = []

        # nodes already checked
        closed = set()


        start = AStarNode(
            startX,
            startY
        )


        heapq.heappush(
            openList,
            (start.f, start)
        )


        while openList:

            # get lowest cost node
            current = heapq.heappop(openList)[1]


            # skip nodes already checked
            if (current.x, current.y) in closed:
                continue


            closed.add(
                (current.x, current.y)
            )


            # goal reached
            if current.x == goalX and current.y == goalY:

                path = []

                while current:

                    path.append(
                        (current.x, current.y)
                    )

                    current = current.parent


                path.reverse()

                return path



            # possible movement directions
            neighbors = [
                (1,0),
                (-1,0),
                (0,1),
                (0,-1)
            ]


            for dx, dy in neighbors:

                nx = current.x + dx
                ny = current.y + dy


                # ignore positions outside map
                if (
                    nx < 0 or
                    ny < 0 or
                    ny >= len(grid) or
                    nx >= len(grid[0])
                ):
                    continue


                # ignore walls
                if grid[ny][nx] == 1:
                    continue


                # ignore checked positions
                if (nx, ny) in closed:
                    continue


                node = AStarNode(
                    nx,
                    ny
                )


                # calculate movement cost
                node.g = current.g + 1


                # calculate distance to goal
                node.h = self.heuristic(
                    nx,
                    ny,
                    goalX,
                    goalY
                )


                # total cost
                node.f = node.g + node.h


                # remember previous node
                node.parent = current


                heapq.heappush(
                    openList,
                    (node.f, node)
                )


        # no possible path
        return []