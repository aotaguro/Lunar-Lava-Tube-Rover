from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4

class LavaTubeSim(ShowBase):
   #constructer and ininitalizing panda3D 
    def __init__(self): 
        ShowBase.__init__(self)
        
        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)

        self.lavaTube.setPos(0,0,0)
        self.lavaTube.setScale(1)
        self.disableMouse()
        self.camera.setPos(0, -30, 5)
        self.camera.lookAt(0,0,0)
        self.lavaTube.setHpr(0, 90, 0)


       #lighting
        ambient = AmbientLight("ambient")
        ambient.setColor(Vec4(0.6, 0.6, 0.6, 1))
        ambientNP = self.render.attachNewNode(ambient)
        self.render.setLight(ambientNP)

        #more lighting
        sun = DirectionalLight("sun")
        sun.setColor(Vec4(1, 1, 1, 1))
        sunNP = self.render.attachNewNode(sun)
        sunNP.setHpr(-45, -45, 0)
        self.render.setLight(sunNP)

        self.rover = self.loader.loadModel("models/box")
        self.rover.reparentTo(self.render)
        self.rover.setScale(1.5, 2.5, 0.8)
        self.rover.setPos(0, -20, 2)
        self.speed = 5

        #  Update each frame
        self.taskMgr.add(self.update, "Update")


#Camera follows moving rectnagle finally
    def update(self, task):

        dt = globalClock.getDt()

        # Move rover forward
        self.rover.setY(self.rover, self.speed * dt)

        # Camera follows behind
        self.camera.setPos(
            self.rover.getX(),
            self.rover.getY() - 15,
            self.rover.getZ() + 6
        )

        self.camera.lookAt(self.rover)

        return Task.cont
    
app = LavaTubeSim()
app.run()

