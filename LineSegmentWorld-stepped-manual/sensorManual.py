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

        print self.LineSegmentEndpoints
            
        #     firstX = firstRaycastLocations[i,0]
        #     firstY = firstRaycastLocations[i,1]

        #     secondX = firstRaycastLocations[i+1,0]
        #     secondY = firstRaycastLocations[i+1,1]
            
        #     firstEndpt = (firstX,firstY,0.0)
        #     secondEndpt = (secondX,secondY,0.0)

        #     d.addLine(firstEndpt, secondEndpt, radius=0.1)

        # firstX = firstRaycastLocations[len(firstRaycastLocations)-1,0]
        # firstY = firstRaycastLocations[len(firstRaycastLocations)-1,1]

        # secondX = -0.4
        # secondY = 0.0
            
        # firstEndpt = (firstX,firstY,0.0)
        # secondEndpt = (secondX,secondY,0.0)

        # d.addLine(firstEndpt, secondEndpt, radius=0.1)

    def raycastAllManual(self,frame):
        distances = np.zeros(self.numRays)
        return distances