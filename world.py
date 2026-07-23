from panda3d.bullet import BulletWorld
from panda3d.core import Vec3


class World:

    #simulation
    def __init__(self, render, loader):

        self.render = render
        self.loader = loader

        #create bullet physics world
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -1.62))

        #load lava tube
        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)


        self.lavaTube.setPos(0,0,0)
        self.lavaTube.setScale(2)
        self.lavaTube.setHpr(0,90,0)


    #updates physics every frame
    def update(self, dt):

        self.world.doPhysics(dt)


    #returns bullet world
    def getWorld(self):

        return self.world

    #returns cave model
    def getTube(self):

        return self.lavaTube