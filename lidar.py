import math

from panda3d.core import Point3



class LidarHit:


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


        self.angle=angle


        self.startX=startX
        self.startY=startY
        self.startZ=startZ


        self.endX=endX
        self.endY=endY
        self.endZ=endZ


        self.distance=distance


        self.hit=hit





class LidarSensor:



    def __init__(self,rover,world):


        self.rover=rover

        self.world=world


        self.numRays=72


        self.maxDistance=25


        self.scanPoints=[]






    def scan(self):


        self.scanPoints.clear()



        step=360/self.numRays



        rover=self.rover.getModel()



        startX=rover.getX()

        startY=rover.getY()

        startZ=rover.getZ()+0.8



        heading=math.radians(
            rover.getH()
        )



        for i in range(self.numRays):


            localAngle=i*step


            angle=math.radians(
                localAngle
            )



            worldAngle=angle+heading



            dx=math.cos(worldAngle)

            dy=math.sin(worldAngle)



            endX=startX+dx*self.maxDistance

            endY=startY+dy*self.maxDistance

            endZ=startZ




            start=Point3(
                startX,
                startY,
                startZ
            )


            end=Point3(
                endX,
                endY,
                endZ
            )



            result=self.world.getWorld().rayTestClosest(
                start,
                end
            )



            if result.hasHit():


                hit=result.getHitPos()


                endX=hit.x

                endY=hit.y

                endZ=hit.z



                distance=(hit-start).length()


                hitDetected=True



            else:


                distance=self.maxDistance

                hitDetected=False




            self.scanPoints.append(

                LidarHit(

                    localAngle,

                    startX,
                    startY,
                    startZ,

                    endX,
                    endY,
                    endZ,

                    distance,

                    hitDetected

                )

            )



        return self.scanPoints