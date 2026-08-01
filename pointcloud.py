import math

from panda3d.core import Point3



#stores a single lidar point
class PointCloudPoint:


    def __init__(self, x, y, z):

        self.x = x
        self.y = y
        self.z = z




#converts lidar scans into 3D points
class PointCloud:


    def __init__(self):

        #stores all detected points
        self.points = []



    #convert lidar measurements into coordinates
    def createPointCloud(self, scan):


        #remove previous scan
        self.points.clear()



        #go through every lidar beam
        for hit in scan:



            #ignore beams that hit nothing
            if hit.hit == False:

                continue



            #convert angle to radians
            angle = math.radians(hit.angle)



            #calculate point location

            x = (
                hit.startX
                +
                math.cos(angle)
                *
                hit.distance
            )


            y = (
                hit.startY
                +
                math.sin(angle)
                *
                hit.distance
            )


            #currently using rover height
            #later this becomes real 3D lidar
            z = 0



            #save point

            self.points.append(

                PointCloudPoint(
                    x,
                    y,
                    z
                )

            )



        return self.points