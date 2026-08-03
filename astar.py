import heapq


class AStarNode:


    def __init__(self, x, y):

        self.x = x
        self.y = y

        self.g = 0
        self.h = 0
        self.f = 0

        self.parent = None



    def __lt__(self, other):

        return self.f < other.f




class AStar:


    def heuristic(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        return abs(x1-x2) + abs(y1-y2)




    def findPath(
        self,
        grid,
        startX,
        startY,
        goalX,
        goalY
    ):


        openList = []

        closed = set()


        start = AStarNode(
            startX,
            startY
        )


        heapq.heappush(
            openList,
            (
                start.f,
                start
            )
        )



        while openList:


            current = heapq.heappop(openList)[1]



            if (
                current.x,
                current.y
            ) in closed:

                continue



            closed.add(
                (
                    current.x,
                    current.y
                )
            )



            if (
                current.x == goalX and
                current.y == goalY
            ):


                path = []


                while current:

                    path.append(
                        (
                            current.x,
                            current.y
                        )
                    )

                    current = current.parent



                path.reverse()

                return path





            neighbors = [

                (1,0),
                (-1,0),
                (0,1),
                (0,-1)

            ]



            for dx,dy in neighbors:


                nx = current.x + dx
                ny = current.y + dy



                if (

                    nx < 0 or
                    ny < 0 or
                    ny >= len(grid) or
                    nx >= len(grid[0])

                ):

                    continue



                if grid[ny][nx] == 1:

                    continue



                if (
                    nx,
                    ny
                ) in closed:

                    continue



                node = AStarNode(
                    nx,
                    ny
                )


                node.g = current.g + 1


                node.h = self.heuristic(
                    nx,
                    ny,
                    goalX,
                    goalY
                )


                node.f = node.g + node.h


                node.parent = current



                heapq.heappush(
                    openList,
                    (
                        node.f,
                        node
                    )
                )



        return []