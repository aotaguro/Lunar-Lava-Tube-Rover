from direct.showbase.ShowBase import ShowBase
from direct.task import Task
import math
from panda3d.core import AmbientLight, DirectionalLight, LineSegs, NodePath, Vec4

from astar import AStar
from frontier import FrontierDetector
from frontier_planner import FrontierPlanner
from lidar import LidarSensor
from map_view import MapView
from occupancy_grid import OccupancyGrid
from path_follower import PathFollower
from pointcloud import PointCloud
from rover import Rover
from world import World


class LavaTubeSim(ShowBase):

    def __init__(self):

        ShowBase.__init__(self)

        # create world
        self.world = World(
            self.render,
            self.loader
        )

        self.lavaTube = self.world.getTube()

        # camera position
        self.disableMouse()
        self.camera.setPos(0, -30, 5)
        self.camera.lookAt(0, 0, 0)

        # lighting
        ambient = AmbientLight("ambient")
        ambient.setColor(Vec4(0.6, 0.6, 0.6, 1))

        ambientNP = self.render.attachNewNode(ambient)
        self.render.setLight(ambientNP)

        sun = DirectionalLight("sun")
        sun.setColor(Vec4(1, 1, 1, 1))

        sunNP = self.render.attachNewNode(sun)
        sunNP.setHpr(-45, -45, 0)
        self.render.setLight(sunNP)

        # rover systems
        self.rover = Rover(
            self.render,
            self.loader
        )

        self.lidar = LidarSensor(
            self.rover,
            self.world
        )

        self.lidarLines = self.render.attachNewNode("LiDAR")
        self.pointCloud = PointCloud()

        # create map
        self.map = OccupancyGrid(
            100,
            100,
            1
        )

        # exploration systems
        self.frontier = FrontierDetector()
        self.planner = FrontierPlanner()
        self.astar = AStar(clearance=1)
        self.pathFollower = PathFollower()

        self.currentPath = []
        self.currentTarget = None
        self.lastStatusTime = -1
        self.cameraDistance = 8
        self.cameraHeight = 3.5

        # create visual slam map
        self.mapView = MapView(
            self.a2dTopRight,
            self.map
        )

        self.taskMgr.add(
            self.update,
            "Update"
        )

    # draw lidar beams
    def drawLidar(self):

        self.lidarLines.removeNode()
        self.lidarLines = self.render.attachNewNode("LiDAR")

        lines = LineSegs()
        lines.setThickness(2)
        lines.setColor(0, 1, 0, 1)

        for hit in self.lidar.scanPoints:

            lines.moveTo(
                hit.startX,
                hit.startY,
                hit.startZ
            )

            lines.drawTo(
                hit.endX,
                hit.endY,
                hit.endZ
            )

        NodePath(lines.create()).reparentTo(
            self.lidarLines
        )

    # choose target and create path
    def createPath(self, frontiers, grid):

        self.currentTarget = self.planner.chooseFrontier(
            self.rover,
            frontiers,
            self.map
        )

        if self.currentTarget is None:
            return

        roverModel = self.rover.getModel()

        roverGX, roverGY = self.map.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

        self.currentPath = self.astar.findPath(
            grid,
            roverGX,
            roverGY,
            self.currentTarget.x,
            self.currentTarget.y
        )

        if not self.currentPath:

            self.planner.addVisited(
                self.currentTarget
            )

            self.currentTarget = None
            return

        self.pathFollower.setPath(
            self.currentPath
        )

        print(
            "NEW TARGET:",
            self.currentTarget.x,
            self.currentTarget.y,
            "PATH LENGTH:",
            len(self.currentPath)
        )

    # print map status twice per second
    def printStatus(self, task, scan, frontiers, grid):

        currentTime = int(task.time * 2)

        if currentTime == self.lastStatusTime:
            return

        self.lastStatusTime = currentTime

        hits = sum(1 for ray in scan if ray.hit)
        free = sum(cell == 0 for row in grid for cell in row)
        walls = sum(cell == 1 for row in grid for cell in row)
        unknown = sum(cell == -1 for row in grid for cell in row)

        print(
            "Hits:",
            hits,
            "Free:",
            free,
            "Walls:",
            walls,
            "Unknown:",
            unknown,
            "Frontiers:",
            len(frontiers)
        )

    # update simulation
    def update(self, task):

        dt = globalClock.getDt()

        # update physics
        self.world.update(dt)

        # lidar scan
        scan = self.lidar.scan()

        # create point cloud
        self.pointCloud.createPointCloud(scan)

        # update map
        self.map.update(scan)
        grid = self.map.getGrid()

        # find frontiers
        frontiers = self.frontier.detect(grid)

        # replan if path becomes blocked
        if self.pathFollower.isBlocked():

            if self.currentTarget is not None:
                self.planner.addFailed(self.currentTarget)

            self.pathFollower.clearPath()
            self.currentPath = []
            self.currentTarget = None

        # finish current target
        if self.currentTarget is not None and self.pathFollower.finished():

            self.planner.addVisited(
                self.currentTarget
            )

            print(
                "FRONTIER COMPLETE:",
                self.currentTarget.x,
                self.currentTarget.y
            )

            self.currentPath = []
            self.currentTarget = None

        # choose next target
        if self.currentTarget is None and frontiers:
            self.createPath(frontiers, grid)

        # move rover
        self.pathFollower.update(
            self.rover,
            self.map,
            self.world,
            dt
        )

        # draw lidar
        self.drawLidar()

        # update visual slam map
        self.mapView.draw(
            task.time,
            self.rover,
            self.currentPath,
            self.currentTarget
        )

        # camera follows behind rover
        roverModel = self.rover.getModel()
        heading = math.radians(roverModel.getH())

        forwardX = -math.sin(heading)
        forwardY = math.cos(heading)

        self.camera.setPos(
            roverModel.getX() - forwardX * self.cameraDistance,
            roverModel.getY() - forwardY * self.cameraDistance,
            roverModel.getZ() + self.cameraHeight
        )

        self.camera.lookAt(
            roverModel.getX() + forwardX * 2,
            roverModel.getY() + forwardY * 2,
            roverModel.getZ() + 0.5
        )

        self.printStatus(
            task,
            scan,
            frontiers,
            grid
        )

        return Task.cont


app = LavaTubeSim()
app.run()
