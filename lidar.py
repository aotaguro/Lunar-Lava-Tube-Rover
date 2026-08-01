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
        roverModel = self.rover.getModel()

        startX = roverModel.getX()
        startY = roverModel.getY()
        startZ = roverModel.getZ()


        #NEW
        #get rover rotation
        #allows lidar to rotate with rover
        roverHeading = roverModel.getH()



        #shoot every lidar beam
        for i in range(self.numRays):


            #beam angle relative to lidar
            localAngle = i * angleStep


            #convert to radians
            angle = math.radians(localAngle)


            #NEW
            #convert lidar angle into world angle
            #by adding rover rotation
            worldAngle = angle + math.radians(roverHeading)



            #direction of beam
            dirX = math.cos(worldAngle)
            dirY = math.sin(worldAngle)



            #maximum beam length
            endX = startX + dirX * self.maxDistance
            endY = startY + dirY * self.maxDistance
            endZ = startZ



            #start and end points for bullet
            startPoint = Point3(
                startX,
                startY,
                startZ
            )


            endPoint = Point3(
                endX,
                endY,
                endZ
            )



            #shoot ray into physics world
            result = self.world.getWorld().rayTestClosest(
                startPoint,
                endPoint
            )



            #if something was hit
            if result.hasHit():


                #get actual collision location
                hitPos = result.getHitPos()


                #replace fake endpoint
                #with actual wall location
                endX = hitPos.x
                endY = hitPos.y


                distance = (
                    hitPos - startPoint
                ).length()


                hit = True



            #nothing was hit
            else:

                distance = self.maxDistance

                hit = False



            #save beam
            self.scanPoints.append(

                LidarHit(

                    localAngle,
                    startX,
                    startY,
                    endX,
                    endY,
                    distance,
                    hit

                )

            )


        return self.scanPoints