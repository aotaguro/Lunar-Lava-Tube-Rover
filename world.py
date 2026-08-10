from panda3d.bullet import (
    BulletRigidBodyNode,
    BulletTriangleMesh,
    BulletTriangleMeshShape,
    BulletWorld
)
from panda3d.core import Point3, Vec3


class World:

    def __init__(self, render, loader):

        self.render = render
        self.loader = loader

        # create physics world
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -1.62))

        # load cave
        self.lavaTube = self.loader.loadModel("assets/lunarTube.glb")
        self.lavaTube.reparentTo(self.render)
        self.lavaTube.setPos(0, 0, 0)
        self.lavaTube.setScale(3)
        self.lavaTube.setHpr(0, 90, 0)

        # create cave collision
        self.createCollisionMesh()

        # test collision mesh
        self.testRay()

    # create collision from the transformed cave model
    def createCollisionMesh(self):

        collisionModel = self.lavaTube.copyTo(self.render)
        collisionModel.flattenStrong()

        mesh = BulletTriangleMesh()
        geomCount = 0

        for geomNodePath in collisionModel.findAllMatches("**/+GeomNode"):

            geomNode = geomNodePath.node()

            for i in range(geomNode.getNumGeoms()):

                mesh.addGeom(geomNode.getGeom(i))
                geomCount += 1

        if geomCount == 0:
            collisionModel.removeNode()
            raise RuntimeError("No cave geometry was found for Bullet collision")

        shape = BulletTriangleMeshShape(
            mesh,
            dynamic=False
        )

        body = BulletRigidBodyNode("LavaTube")
        body.addShape(shape)

        self.collisionPath = self.render.attachNewNode(body)
        self.world.attachRigidBody(body)

        collisionModel.removeNode()

        print("Added", geomCount, "geom to Bullet mesh")
        print("Collision mesh created")

    # test that the cave collision can be detected
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

    # check movement using several points around the rover body
    def isRoverMoveClear(
        self,
        startX,
        startY,
        endX,
        endY,
        z,
        radius=1.1
    ):

        offsets = [
            (0, 0),
            (radius, 0),
            (-radius, 0),
            (0, radius),
            (0, -radius),
            (radius * 0.7, radius * 0.7),
            (radius * 0.7, -radius * 0.7),
            (-radius * 0.7, radius * 0.7),
            (-radius * 0.7, -radius * 0.7)
        ]

        for offsetX, offsetY in offsets:

            start = Point3(
                startX + offsetX,
                startY + offsetY,
                z
            )

            end = Point3(
                endX + offsetX,
                endY + offsetY,
                z
            )

            result = self.world.rayTestClosest(
                start,
                end
            )

            if result.hasHit():
                return False

        return True

    def update(self, dt):

        self.world.doPhysics(
            dt,
            4,
            1.0 / 120.0
        )

    def getWorld(self):

        return self.world

    def getTube(self):

        return self.lavaTube