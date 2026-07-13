from direct.showbase.ShowBase import ShowBase 
class LavaTubeSim(ShowBase):
   #constructer and ininitalizing panda3D 
    def __init__(self): 
        ShowBase.__init__(self)

        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)
        self.lavaTube.setPos(0,0,0)
        self.lavaTube.setScale(1)
        self.disableMouse()
        self.camera.setPos(0,-25, 8)
        self.camera.lookAt(0,0,0)

app = LavaTubeSim()
app.run()

