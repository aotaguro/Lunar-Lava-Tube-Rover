from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4
from panda3d.core import LineSegs, NodePath
import math


class LidarSensor:

    def __init__(self, rover): #rovers positions 
        self.rover = rover
        self.numRays = 72          # 5 degree spacing for 360 total
        self.maxDistance = 25    
        self.scanPoints = []       #array of data points it picks up

    def scan(self):
        
        self.scanPoints.clear() #keeps new scan information open

        angleStep = 360 / self.numRays #angle for rays

        for i in range(self.numRays): #for each lidar beam

            angle = math.radians(i * angleStep) #degrees to radians for pythons stupid brain

            #Beam ending 
            endX = self.rover.getX() + math.cos(angle) * self.maxDistance 
            endY = self.rover.getY() + math.sin(angle) * self.maxDistance
            
            #save beam data
            self.scanPoints.append((
                self.rover.getX(),
                self.rover.getY(),
                endX,
                endY
            ))


class LavaTubeSim(ShowBase):

    # constructor and initializing panda3D
    def __init__(self):

        ShowBase.__init__(self)

        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)

        self.lavaTube.setPos(0, 0, 0)
        self.lavaTube.setScale(1)
        self.lavaTube.setHpr(0, 90, 0)

        self.disableMouse()

        self.camera.setPos(0, -30, 5)
        self.camera.lookAt(0, 0, 0)

        # lighting
        ambient = AmbientLight("ambient")
        ambient.setColor(Vec4(0.6, 0.6, 0.6, 1))
        ambientNP = self.render.attachNewNode(ambient)
        self.render.setLight(ambientNP)

        # more lighting
        sun = DirectionalLight("sun")
        sun.setColor(Vec4(1, 1, 1, 1))
        sunNP = self.render.attachNewNode(sun)
        sunNP.setHpr(-45, -45, 0)
        self.render.setLight(sunNP)

        # Rover
        self.rover = self.loader.loadModel("models/box")
        self.rover.reparentTo(self.render)

        self.rover.setScale(1.5, 2.5, 0.8)
        self.rover.setPos(0, -20, 2)

        self.speed = 5

        # Lidar
        self.lidar = LidarSensor(self.rover)

        self.lidarLines = self.render.attachNewNode("LiDAR")

        # Update loop
        self.taskMgr.add(self.update, "Update")

    def drawLidar(self):

        self.lidarLines.removeNode()
        self.lidarLines = self.render.attachNewNode("LiDAR")

        lines = LineSegs()

        lines.setThickness(2)
        lines.setColor(0, 1, 0, 1)

        for ray in self.lidar.scanPoints:

            lines.moveTo(ray[0], ray[1], 2)
            lines.drawTo(ray[2], ray[3], 2)

        node = lines.create()
        NodePath(node).reparentTo(self.lidarLines)

    # Camera follows rover
    def update(self, task):

        dt = globalClock.getDt()

        # Move rover
        self.rover.setY(self.rover, self.speed * dt)
        
        self.lidar.scan()
        self.drawLidar()

        # Camera follows rover
        self.camera.setPos(
            self.rover.getX(),
            self.rover.getY() - 15,
            self.rover.getZ() + 6
        )

        self.camera.lookAt(self.rover)

        return Task.cont


app = LavaTubeSim()
app.run()