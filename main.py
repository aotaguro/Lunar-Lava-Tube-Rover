from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
import math

from panda3d.core import (
    AmbientLight,
    DirectionalLight,
    LineSegs,
    NodePath,
    TextNode,
    Vec4
)

from astar import AStar
from camera_sensor import CameraSensor
from frontier import FrontierDetector
from frontier_planner import FrontierPlanner
from lidar import LidarSensor
from map_view import MapView
from occupancy_grid import OccupancyGrid
from path_follower import PathFollower
from pointcloud import PointCloud
from rover import Rover
from visual_frontier_planner import VisualFrontierPlanner
from visual_map_view import VisualMapView
from visual_occupancy_grid import VisualOccupancyGrid
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

        self.visualCamera = CameraSensor(
            self.rover,
            self.world
        )

        self.lidarLines = self.render.attachNewNode("LiDAR")
        self.cameraLines = self.render.attachNewNode("VisualCamera")

        self.pointCloud = PointCloud()

        # lidar map
        self.map = OccupancyGrid(
            160,
            160,
            1
        )

        # separate visual map
        self.visualMap = VisualOccupancyGrid(
            160,
            160,
            1
        )

        # lidar exploration systems
        self.frontier = FrontierDetector()
        self.planner = FrontierPlanner()

        # visual exploration systems
        self.visualFrontier = FrontierDetector()
        self.visualPlanner = VisualFrontierPlanner()

        self.astar = AStar(clearance=1)
        self.pathFollower = PathFollower()

        self.currentPath = []
        self.currentTarget = None
        self.lastStatusTime = -1

        self.navigationMode = "lidar"

        self.cameraDistance = 8
        self.cameraHeight = 3.5

        # lidar slam map
        self.mapView = MapView(
            self.a2dTopRight,
            self.map
        )

        # visual slam map
        self.visualMapView = VisualMapView(
            self.a2dTopLeft,
            self.visualMap
        )

        self.modeText = OnscreenText(
            text="NAVIGATION: LIDAR",
            pos=(0, 0.9),
            scale=0.05,
            align=TextNode.ACenter,
            fg=(1, 1, 1, 1),
            mayChange=True
        )

        self.controlText = OnscreenText(
            text="L = LiDAR navigation   V = Visual navigation",
            pos=(0, 0.83),
            scale=0.035,
            align=TextNode.ACenter,
            fg=(1, 1, 1, 1),
            mayChange=False
        )

        self.accept(
            "l",
            self.setNavigationMode,
            ["lidar"]
        )

        self.accept(
            "v",
            self.setNavigationMode,
            ["visual"]
        )

        self.taskMgr.add(
            self.update,
            "Update"
        )

    # switch which map controls navigation
    def setNavigationMode(self, mode):

        if mode == self.navigationMode:
            return

        self.navigationMode = mode

        self.pathFollower.clearPath()
        self.currentPath = []
        self.currentTarget = None

        if mode == "visual":
            self.modeText.setText("NAVIGATION: VISUAL CAMERA")
        else:
            self.modeText.setText("NAVIGATION: LIDAR")

        print(
            "NAVIGATION MODE:",
            mode.upper()
        )

    # return the systems used by the selected navigation mode
    def getActiveNavigation(self):

        if self.navigationMode == "visual":

            return (
                self.visualMap,
                self.visualFrontier,
                self.visualPlanner
            )

        return (
            self.map,
            self.frontier,
            self.planner
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

    # draw camera field of view rays
    def drawCameraSensor(self):

        self.cameraLines.removeNode()
        self.cameraLines = self.render.attachNewNode("VisualCamera")

        if self.navigationMode != "visual":
            return

        lines = LineSegs()
        lines.setThickness(2)
        lines.setColor(1, 0.75, 0, 1)

        # draw every second ray so the display stays clear
        for hit in self.visualCamera.scanPoints[::2]:

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
            self.cameraLines
        )

    # choose target and create path on selected map
    def createPath(self, frontiers, gridMap, planner):

        self.currentTarget = planner.chooseFrontier(
            self.rover,
            frontiers,
            gridMap
        )

        if self.currentTarget is None:
            return

        roverModel = self.rover.getModel()

        roverGX, roverGY = gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

        self.currentPath = self.astar.findPath(
            gridMap.getGrid(),
            roverGX,
            roverGY,
            self.currentTarget.x,
            self.currentTarget.y
        )

        if not self.currentPath:

            planner.addFailed(
                self.currentTarget
            )

            self.currentTarget = None
            return

        self.pathFollower.setPath(
            self.currentPath
        )

        print(
            "NEW",
            self.navigationMode.upper(),
            "TARGET:",
            self.currentTarget.x,
            self.currentTarget.y,
            "PATH LENGTH:",
            len(self.currentPath)
        )

    # print sensor and map status twice per second
    def printStatus(
        self,
        task,
        lidarScan,
        visualScan,
        lidarFrontiers,
        visualFrontiers
    ):

        currentTime = int(task.time * 2)

        if currentTime == self.lastStatusTime:
            return

        self.lastStatusTime = currentTime

        lidarHits = sum(
            1 for ray in lidarScan
            if ray.hit
        )

        visualHits = sum(
            1 for ray in visualScan
            if ray.hit
        )

        print(
            "Mode:",
            self.navigationMode,
            "LiDAR hits:",
            lidarHits,
            "LiDAR frontiers:",
            len(lidarFrontiers),
            "Camera hits:",
            visualHits,
            "Visual frontiers:",
            len(visualFrontiers)
        )

    # update simulation
    def update(self, task):

        dt = globalClock.getDt()

        # update physics
        self.world.update(dt)

        # lidar mapping stays independent
        lidarScan = self.lidar.scan()
        self.pointCloud.createPointCloud(lidarScan)
        self.map.update(lidarScan)

        # camera mapping does not use lidar data
        visualScan = self.visualCamera.scan()
        self.visualMap.update(visualScan)

        lidarFrontiers = self.frontier.detect(
            self.map.getGrid()
        )

        visualFrontiers = self.visualFrontier.detect(
            self.visualMap.getGrid()
        )

        activeMap, activeFrontier, activePlanner = self.getActiveNavigation()

        if self.navigationMode == "visual":
            activeFrontiers = visualFrontiers
        else:
            activeFrontiers = lidarFrontiers

        # replan if current movement becomes blocked
        if self.pathFollower.isBlocked():

            if self.currentTarget is not None:
                activePlanner.addFailed(self.currentTarget)

            self.pathFollower.clearPath()
            self.currentPath = []
            self.currentTarget = None

        # finish current target
        if (
            self.currentTarget is not None
            and
            self.pathFollower.finished()
        ):

            activePlanner.addVisited(
                self.currentTarget
            )

            print(
                "FRONTIER COMPLETE:",
                self.currentTarget.x,
                self.currentTarget.y
            )

            self.currentPath = []
            self.currentTarget = None

        # choose the next target from only the active map
        if self.currentTarget is None and activeFrontiers:

            self.createPath(
                activeFrontiers,
                activeMap,
                activePlanner
            )

        # move using only the active navigation map
        self.pathFollower.update(
            self.rover,
            activeMap,
            self.world,
            dt
        )

        self.drawLidar()
        self.drawCameraSensor()

        # show both maps at the same time
        self.mapView.draw(
            task.time,
            self.rover,
            self.currentPath if self.navigationMode == "lidar" else [],
            self.currentTarget if self.navigationMode == "lidar" else None
        )

        self.visualMapView.draw(
            task.time,
            self.rover,
            self.currentPath if self.navigationMode == "visual" else [],
            self.currentTarget if self.navigationMode == "visual" else None
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
            lidarScan,
            visualScan,
            lidarFrontiers,
            visualFrontiers
        )

        return Task.cont


app = LavaTubeSim()
app.run()
