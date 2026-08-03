from direct.showbase.ShowBase import ShowBase
from direct.task import Task

from panda3d.core import (
    AmbientLight,
    DirectionalLight,
    Vec4,
    LineSegs,
    NodePath
)

from world import World
from rover import Rover
from lidar import LidarSensor
from pointcloud import PointCloud
from occupancy_grid import OccupancyGrid
from frontier import FrontierDetector
from planner import FrontierPlanner
from astar import AStar



class LavaTubeSim(ShowBase):


    def __init__(self):

        ShowBase.__init__(self)


        # -------------------------
        # WORLD
        # -------------------------

        self.world = World(
            self.render,
            self.loader
        )


        self.lavaTube = self.world.getTube()

        self.lavaTube.reparentTo(
            self.render
        )


        self.lavaTube.setPos(
            0,
            0,
            0
        )

        self.lavaTube.setScale(
            2
        )

        self.lavaTube.setHpr(
            0,
            90,
            0
        )



        # -------------------------
        # CAMERA
        # -------------------------

        self.disableMouse()

        self.camera.setPos(
            0,
            -30,
            5
        )

        self.camera.lookAt(
            0,
            0,
            0
        )



        # -------------------------
        # LIGHTING
        # -------------------------

        ambient = AmbientLight(
            "ambient"
        )

        ambient.setColor(
            Vec4(
                0.6,
                0.6,
                0.6,
                1
            )
        )


        ambientNP = self.render.attachNewNode(
            ambient
        )

        self.render.setLight(
            ambientNP
        )



        sun = DirectionalLight(
            "sun"
        )

        sun.setColor(
            Vec4(
                1,
                1,
                1,
                1
            )
        )


        sunNP = self.render.attachNewNode(
            sun
        )

        sunNP.setHpr(
            -45,
            -45,
            0
        )

        self.render.setLight(
            sunNP
        )



        # -------------------------
        # ROBOT SYSTEMS
        # -------------------------

        self.rover = Rover(
            self.render,
            self.loader
        )


        self.lidar = LidarSensor(
            self.rover,
            self.world
        )


        self.lidarLines = self.render.attachNewNode(
            "LiDAR"
        )


        self.pointCloud = PointCloud()



        self.map = OccupancyGrid(
            100,
            100,
            1
        )



        # exploration systems

        self.frontier = FrontierDetector()

        self.planner = FrontierPlanner()


        # path planner

        self.astar = AStar()



        # stores current path

        self.currentPath = []



        self.taskMgr.add(
            self.update,
            "Update"
        )





    # -------------------------
    # DRAW LIDAR
    # -------------------------

    def drawLidar(self):


        self.lidarLines.removeNode()


        self.lidarLines = self.render.attachNewNode(
            "LiDAR"
        )


        lines = LineSegs()


        lines.setThickness(
            2
        )


        lines.setColor(
            0,
            1,
            0,
            1
        )


        for hit in self.lidar.scanPoints:


            lines.moveTo(
                hit.startX,
                hit.startY,
                2
            )


            lines.drawTo(
                hit.endX,
                hit.endY,
                2
            )



        node = lines.create()


        NodePath(node).reparentTo(
            self.lidarLines
        )





    # -------------------------
    # UPDATE LOOP
    # -------------------------

    def update(self, task):


        dt = globalClock.getDt()



        # physics

        self.world.update(
            dt
        )



        # rover movement

        self.rover.update(
            dt
        )



        # lidar scan

        scan = self.lidar.scan()



        # point cloud

        points = self.pointCloud.createPointCloud(
            scan
        )



        # update map

        self.map.update(
            points
        )


        grid = self.map.getGrid()



        # -------------------------
        # FRONTIER DETECTION
        # -------------------------

        frontiers = self.frontier.detect(
            grid
        )


        print(
            "Frontiers:",
            len(frontiers)
        )



        # choose exploration target

        target = self.planner.chooseFrontier(
            self.rover,
            frontiers
        )



        if target:


            print(
                "Target frontier:",
                target.x,
                target.y
            )



            # rover position in grid

            rover = self.rover.getModel()


            roverGX, roverGY = self.map.worldToGrid(
                rover.getX(),
                rover.getY()
            )



            # run A*

            self.currentPath = self.astar.findPath(

                grid,

                roverGX,
                roverGY,

                target.x,
                target.y

            )



            print(
                "A* Path length:",
                len(self.currentPath)
            )



            if self.currentPath:

                print(
                    "Next step:",
                    self.currentPath[1]
                    if len(self.currentPath) > 1
                    else self.currentPath[0]
                )





        # -------------------------
        # DRAW
        # -------------------------

        self.drawLidar()



        # camera follow

        self.camera.setPos(

            self.rover.getModel().getX(),

            self.rover.getModel().getY()-15,

            self.rover.getModel().getZ()+6

        )


        self.camera.lookAt(
            self.rover.getModel()
        )



        return Task.cont





app = LavaTubeSim()

app.run()