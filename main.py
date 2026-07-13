from direct.showbase.ShowBase import ShowBase 
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


app = LavaTubeSim()
app.run()

