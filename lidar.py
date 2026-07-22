import math


#stores information about one lidar beam
class LidarHit:

    def __init__(self, angle, startX, startY, endX, endY):

        self.angle = angle

        self.startX = startX
        self.startY = startY

        self.endX = endX
        self.endY = endY


#simulated lidar sensor thats moved over from previous github push 
class LidarSensor:

    def __init__(self, rover):

        #reference to rover
        self.rover = rover

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

        for i in range(self.numRays):

            angle = math.radians(i * angleStep)

            startX = self.rover.getModel().getX()
            startY = self.rover.getModel().getY()

            endX = startX + math.cos(angle) * self.maxDistance
            endY = startY + math.sin(angle) * self.maxDistance

            hit = LidarHit(
                angle,
                startX,
                startY,
                endX,
                endY
            )

            self.scanPoints.append(hit)

        return self.scanPoints