from panda3d.bullet import (
    BulletWorld,
    BulletTriangleMesh,
    BulletTriangleMeshShape,
    BulletRigidBodyNode
)

from panda3d.core import Vec3, Point3


class World:

    def __init__(self, render, loader):

        self.render = render
        self.loader = loader

        # Physics of world
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -1.62)) #simulating moon gravity thats -1.62m/s^2 


        # Load cave
        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)

        self.lavaTube.setPos(0, 0, 0)
        self.lavaTube.setScale(2)
        self.lavaTube.setHpr(0, 90, 0)


        # Create collision
        self.createCollisionMesh()


        # Test ray
        self.testRay()


#
    def createCollisionMesh(self):

        # Make a copy for physics
        collisionModel = self.lavaTube.copyTo(self.render)

        # flattens copy to create one collision mesh 
        collisionModel.flattenStrong()


        mesh = BulletTriangleMesh()

        geomCount = 0


        for geomNodePath in collisionModel.findAllMatches("**/+GeomNode"):

            geomNode = geomNodePath.node()

            for i in range(geomNode.getNumGeoms()):

                geom = geomNode.getGeom(i)

                mesh.addGeom(geom) #adds geometry for the lidar to detect 

                geomCount += 1


        print("Added", geomCount, "geom to Bullet mesh")


        shape = BulletTriangleMeshShape( 
            mesh,
            dynamic=False
        )


        body = BulletRigidBodyNode("LavaTube") #actually creates bullet object

        body.addShape(shape)    #give bullet a shape


        bodyPath = self.render.attachNewNode(body)


        self.world.attachRigidBody(body) #body added to the physics world


        collisionModel.removeNode()


        print("Collision mesh created")


#uses raycasting to detect if the lidar is hitting the cave walls
    def testRay(self):

        start = Point3(0, 0, 100)
        end = Point3(0, 0, -100)


        result = self.world.rayTestClosest(
            start,
            end
        )


        if result.hasHit():

            print("Hit!")
            print("Position:", result.getHitPos())
            print("Normal:", result.getHitNormal())


        else:

            print("No hit")



    def update(self, dt):

        self.world.doPhysics(dt)



    def getWorld(self):

        return self.world



    def getTube(self):

        return self.lavaTube