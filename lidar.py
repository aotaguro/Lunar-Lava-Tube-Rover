import math

from panda3d.core import Point3


#stores information about one lidar beam
class LidarHit:

    def __init__(self, angle, startX, startY, endX, endY, distance, hit):

        self.angle = angle

        self.startX = startX
        self.startY = startY

        self.endX = endX
        self.endY = endY

        #distance from rover to hit point
        self.distance = distance

        #true if beam hit something
        self.hit = hit


#simulated lidar sensor
class LidarSensor:

    def __init__(self, rover, world):

        #reference to rover
        self.rover = rover

        #reference to bullet world
        self.world = world

        #5 degree spacing
        self.numRays = 72

        #maximum scan distance
        self.maxDistance = 25

        #stores current scan
        self.scanPoints = []


    #perform one scan
    def scan(self):

        #clear previous scan
        self.scanPoints.clear()

        angleStep = 360 / self.numRays

        #current rover position
        startX = self.rover.getModel().getX()
        startY = self.rover.getModel().getY()
        startZ = self.rover.getModel().getZ()

        #shoot every lidar beam
        for i in range(self.numRays):

            angle = math.radians(i * angleStep)

            #direction of beam
            dirX = math.cos(angle)
            dirY = math.sin(angle)

            #maximum beam length
            endX = startX + dirX * self.maxDistance
            endY = startY + dirY * self.maxDistance
            endZ = startZ

            #start and end points for bullet
            startPoint = Point3(startX, startY, startZ)
            endPoint = Point3(endX, endY, endZ)

            #shoot ray into physics world
            result = self.world.getWorld().rayTestClosest(startPoint, endPoint)

            #if something was hit
            if result.hasHit():

                hitPos = result.getHitPos()

                endX = hitPos.x
                endY = hitPos.y
                endZ = hitPos.z

                distance = (hitPos - startPoint).length()

                hit = True

            #nothing was hit
            else:

                distance = self.maxDistance

                hit = False

            #save beam
            self.scanPoints.append(

                LidarHit(

                    angle,
                    startX,
                    startY,
                    endX,
                    endY,
                    distance,
                    hit

                )

            )

        return self.scanPoints