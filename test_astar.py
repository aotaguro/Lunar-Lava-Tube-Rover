from astar import AStar



astar = AStar()



testGrid = [

    [0,0,0,0,0],

    [0,1,1,1,0],

    [0,0,0,1,0],

    [0,1,0,0,0],

    [0,0,0,0,0]

]



path = astar.findPath(

    testGrid,

    0,
    0,

    4,
    4

)



print("A* TEST RESULT")

print(path)



if len(path) > 0:

    print("A* SUCCESS")

else:

    print("A* FAILED")