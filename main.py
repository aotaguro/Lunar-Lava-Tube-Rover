from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4
from panda3d.core import LineSegs, NodePath

from rover import Rover
from lidar import LidarSensor


class LavaTubeSim(ShowBase):

   #constructer and ininitalizing panda3D
    def __init__(self):
        ShowBase.__init__(self)

        #load lava tube
        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)

        self.lavaTube.setPos(0,0,0)
        self.lavaTube.setScale(2)
        self.lavaTube.setHpr(0,90,0)

        #camera
        self.disableMouse()

        self.camera.setPos(0,-30,5)
        self.camera.lookAt(0,0,0)

        #lighting
        ambient = AmbientLight("ambient")
        ambient.setColor(Vec4(0.6,0.6,0.6,1))
        ambientNP = self.render.attachNewNode(ambient)
        self.render.setLight(ambientNP)

        #more lighting
        sun = DirectionalLight("sun")
        sun.setColor(Vec4(1,1,1,1))
        sunNP = self.render.attachNewNode(sun)
        sunNP.setHpr(-45,-45,0)
        self.render.setLight(sunNP)

        #create rover
        self.rover = Rover(self.render, self.loader)

        #create lidar
        self.lidar = LidarSensor(self.rover)

        #node used to draw lidar beams
        self.lidarLines = self.render.attachNewNode("LiDAR")

        #update each frame
        self.taskMgr.add(self.update,"Update")


    #draw lidar rays
    def drawLidar(self):

        self.lidarLines.removeNode()
        self.lidarLines = self.render.attachNewNode("LiDAR")

        lines = LineSegs()

        lines.setThickness(2)
        lines.setColor(0,1,0,1)

        #draws points
        for hit in self.lidar.scanPoints:
            lines.moveTo(hit.startX, hit.startY, 2)
            lines.drawTo(hit.endX, hit.endY, 2)

        node = lines.create()
        NodePath(node).reparentTo(self.lidarLines)


    #camera follows rover
    def update(self, task):

        dt = globalClock.getDt()

        #move rover
        self.rover.update(dt)

        #lidar scan
        self.lidar.scan()

        #draw lidar
        self.drawLidar()

        #camera follows rover
        self.camera.setPos(
            self.rover.getModel().getX(),
            self.rover.getModel().getY() - 15,
            self.rover.getModel().getZ() + 6
        )

        self.camera.lookAt(self.rover.getModel())

        return Task.cont


app = LavaTubeSim()
app.run()