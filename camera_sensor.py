import math

from panda3d.core import Point3


class CameraHit:

    def __init__(
        self,
        angle,
        startX,
        startY,
        startZ,
        endX,
        endY,
        endZ,
        distance,
        hit
    ):

        self.angle = angle
        self.startX = startX
        self.startY = startY
        self.startZ = startZ
        self.endX = endX
        self.endY = endY
        self.endZ = endZ
        self.distance = distance
        self.hit = hit


class CameraSensor:

    def __init__(self, rover, world):

        self.rover = rover
        self.world = world

        # camera field of view
        self.fieldOfView = 80

        # samples across the camera image
        self.numRays = 45

        # camera can see this far
        self.maxDistance = 18

        # camera height above rover center
        self.height = 0.8

        self.scanPoints = []

    # take one camera depth scan
    def scan(self):

        self.scanPoints.clear()

        rover = self.rover.getModel()

        startX = rover.getX()
        startY = rover.getY()
        startZ = rover.getZ() + self.height

        roverHeading = rover.getH()

        if self.numRays == 1:
            angleStep = 0
        else:
            angleStep = self.fieldOfView / (self.numRays - 1)

        firstAngle = -self.fieldOfView / 2

        for i in range(self.numRays):

            localAngle = firstAngle + i * angleStep
            worldAngle = math.radians(roverHeading + localAngle)

            # Panda3D heading 0 points toward positive Y
            directionX = -math.sin(worldAngle)
            directionY = math.cos(worldAngle)

            endX = startX + directionX * self.maxDistance
            endY = startY + directionY * self.maxDistance
            endZ = startZ

            start = Point3(
                startX,
                startY,
                startZ
            )

            end = Point3(
                endX,
                endY,
                endZ
            )

            result = self.world.getWorld().rayTestClosest(
                start,
                end
            )

            if result.hasHit():

                hitPosition = result.getHitPos()

                endX = hitPosition.x
                endY = hitPosition.y
                endZ = hitPosition.z

                distance = (
                    hitPosition - start
                ).length()

                hit = True

            else:

                distance = self.maxDistance
                hit = False

            self.scanPoints.append(
                CameraHit(
                    localAngle,
                    startX,
                    startY,
                    startZ,
                    endX,
                    endY,
                    endZ,
                    distance,
                    hit
                )
            )

        return self.scanPoints
