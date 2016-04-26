import numpy as np


class SensorObjManual(object):

    def __init__(self, FOV=90.0, numRays=21, rayLength=20):
        self.numRays = numRays
        self.rayLength = rayLength

        FOVrad = FOV * np.pi/180.0
        self.angleMin = -FOVrad/2
        self.angleMax = FOVrad/2

        self.angleGrid = np.linspace(self.angleMin, self.angleMax, self.numRays)

        self.rays = np.zeros((3,self.numRays))
        self.rays[0,:] = np.cos(self.angleGrid)
        self.rays[1,:] = -np.sin(self.angleGrid)


    def setLineSegmentWorld(self, initialRaycastLocations):
        print "I received as my initialRaycastLocations", initialRaycastLocations
        print np.shape(initialRaycastLocations)
        self.LineSegmentEndpoints = np.zeros((np.shape(initialRaycastLocations)[0]+2,3))
        print np.shape(self.LineSegmentEndpoints)

        firstX = -0.4
        firstY = 0.0
        firstZ = 1.0

        self.LineSegmentEndpoints[0,:] = [firstX, firstY, firstZ]
        
        for i in range(1,np.shape(initialRaycastLocations)[0]+1):
             print "initialRaycastLocations", i
             self.LineSegmentEndpoints[i,:] = initialRaycastLocations[i-1,:]
             self.LineSegmentEndpoints[i,2] = 1.0


        self.LineSegmentEndpoints[-1,:] = [firstX, firstY, firstZ]

        self.numLineSegmentEndpoints = np.shape(self.LineSegmentEndpoints)[0]
        print self.LineSegmentEndpoints

    def CastPointsToDouble(self, x1, y1, x2, y2, x3, y3, x4, y4):
        return float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4)

    def LineIntersection(self, x1, y1, x2, y2, x3, y3, x4, y4):
        if ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)) == 0:
            print "These lines are parallel"
            return
        x1, y1, x2, y2, x3, y3, x4, y4 = self.CastPointsToDouble(x1, y1, x2, y2, x3, y3, x4, y4)
        Px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4))
        Py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4))
        return [Px, Py]

    def IntersectionIsOnSegments(self, x1, y1, x2, y2, x3, y3, x4, y4, Px, Py):
        # Check if Px is in the right range for Line 1
        if ((x1 > Px) and (x2 > Px)) or ((x1 < Px) and (x2 < Px)):
            return False
        
        # Check if Px is in the right range for Line 2
        if ((x3 > Px) and (x4 > Px)) or ((x3 < Px) and (x4 < Px)):
            return False
        
        # Check if Py is in the right range for Line 1
        if ((y1 > Py) and (y2 > Py)) or ((y1 < Py) and (y2 < Py)):
            return False
        
        # Check if Py is in the right range for Line 2
        if ((y3 > Py) and (y4 > Py)) or ((y3 < Py) and (y4 < Py)):
            return False
        
        return True

    def LineSegmentIntersection(self, x1, y1, x2, y2, x3, y3, x4, y4):
        Point = self.LineIntersection(x1, y1, x2, y2, x3, y3, x4, y4)
        if Point is None:
            print "parallel lines, no intersection"
            return
            
        if not self.IntersectionIsOnSegments(x1, y1, x2, y2, x3, y3, x4, y4, Point[0], Point[1]):
            print "NOT on the line segments"
            return

        return Point


    def IntersectionDistance(self, x1, y1, x2, y2, x3, y3, x4, y4):
        
        # Line 1, the laser, is (x1, y1) , (x2, y2)

        # (x1, y1) is state of vehicle
        # (x2, y2) is a ray projected forward to the maximum range


        # Line 2, the line segment in the world, is (x3, y3), (x4, y4)

        # (x3, y3) is left side of line segment
        # (x4, y4) is right side of line segment

        Point = self.LineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4)

        if Point is None:
            print "NO INTERSECTION"
            return

        # return length of intersecting laser

        Length = ((Point[0] - x1)**2 + (Point[1] - y1)**2)**(0.5)

        return Length


    def raycastAllManual(self,frame):
        distances = np.zeros(self.numRays)

        origin = np.array(frame.transform.GetPosition())
        
        # iterate through each laser

        for i in range(self.numRays):

            # transform the ray and find the max range location

            ray = self.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            maxRangeLocation = origin + rayTransformed*self.rayLength


            # initialize a vector of all of the possible intersections

            laserWithEachWorldSegment = np.zeros((self.numLineSegmentEndpoints-1))
            

            # find intersections with this laser with each line segment in the world 

            for j in range(self.numLineSegmentEndpoints-1):

                 length = self.IntersectionDistance(origin[0], origin[1], maxRangeLocation[0], maxRangeLocation[1], self.LineSegmentEndpoints[j,0], self.LineSegmentEndpoints[j,1], self.LineSegmentEndpoints[j+1,0], self.LineSegmentEndpoints[j+1,1])
                 if length is None:
                    length = self.rayLength
                 laserWithEachWorldSegment[j] = length


            # choose the smallest distance

            distances[i] = np.min(laserWithEachWorldSegment)


        return distances




