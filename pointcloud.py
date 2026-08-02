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


    #convert lidar measurements into world coordinates
    def createPointCloud(self, scan):

        #remove previous scan
        self.points.clear()

        #go through every lidar beam
        for hit in scan:

            #ignore beams that hit nothing
            if not hit.hit:
                continue

            #Bullet already returned the exact collision point,
            #so we simply store it in the point cloud.
            self.points.append(

                PointCloudPoint(

                    hit.endX,
                    hit.endY,
                    0

                )

            )

        return self.points