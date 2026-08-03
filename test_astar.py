from astar import AStar  # import the A* pathfinding implementation


astar = AStar()  # create an instance of the A* solver


# testGrid: 0 = walkable, 1 = obstacle
# This is a small 5x5 map with a barrier in the middle
testGrid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0]
]


# Find a path from top-left (0,0) to bottom-right (4,4)
path = astar.findPath(testGrid, 0, 0, 4, 4)


print("A* TEST RESULT")
print(path)  # prints the list of coordinates in the computed path


if len(path) > 0:
    print("A* SUCCESS")  # path found
else:
    print("A* FAILED")  # no path found