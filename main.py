from direct.showbase.ShowBase import ShowBase
from direct.task import Task

from panda3d.core import AmbientLight, DirectionalLight, Vec4
from panda3d.core import LineSegs, NodePath

from world import World
from rover import Rover
from lidar import LidarSensor
from pointcloud import PointCloud
from occupancy_grid import OccupancyGrid


class LavaTubeSim(ShowBase):

    # constructor and initializing panda3D
    def __init__(self):

        ShowBase.__init__(self)


        # create world
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

        self.lavaTube.setScale(2)

        self.lavaTube.setHpr(
            0,
            90,
            0
        )



        # camera
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



        # lighting
        ambient = AmbientLight("ambient")

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



        # directional lighting
        sun = DirectionalLight("sun")

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



        # create rover
        self.rover = Rover(
            self.render,
            self.loader
        )



        # create lidar with access to physics world
        self.lidar = LidarSensor(
            self.rover,
            self.world
        )



        # node used to draw lidar beams
        self.lidarLines = self.render.attachNewNode(
            "LiDAR"
        )



        # point cloud generator
        self.pointCloud = PointCloud()



        # occupancy grid
        # width, height, meters per cell
        self.map = OccupancyGrid(
            100,
            100,
            1
        )



        # update every frame
        self.taskMgr.add(
            self.update,
            "Update"
        )




    # draw lidar rays
    def drawLidar(self):

        self.lidarLines.removeNode()

        self.lidarLines = self.render.attachNewNode(
            "LiDAR"
        )


        lines = LineSegs()

        lines.setThickness(2)

        lines.setColor(
            0,
            1,
            0,
            1
        )



        # draw lidar beams
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





    # update simulation
    def update(self, task):

        dt = globalClock.getDt()



        # update physics
        self.world.update(dt)



        # update rover movement
        self.rover.update(dt)



        # perform lidar scan
        scan = self.lidar.scan()



        # convert lidar data into point cloud
        points = self.pointCloud.createPointCloud(
            scan
        )



        # update occupancy map
        self.map.update(
            points
        )



        # draw lidar
        self.drawLidar()



        # camera follows rover
        self.camera.setPos(
            self.rover.getModel().getX(),
            self.rover.getModel().getY() - 15,
            self.rover.getModel().getZ() + 6
        )

        self.camera.lookAt(
            self.rover.getModel()
        )


        return Task.cont




app = LavaTubeSim()

app.run()