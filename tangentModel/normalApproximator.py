import numpy as np
import scipy.optimize as opt
import math
from director.debugVis import DebugData
import director.visualization as vis

class NormalApproximatorObj(object):

    def __init__(self, numRays, circleRadius):
        self.N = 1
        self.numRays = numRays
        self.circleRadius = circleRadius

    def initializeThetaVector(self,thetaVector):
        self.thetaVector = thetaVector
        self.leftThetaVector = self.thetaVector - 0.001
        self.rightThetaVector = self.thetaVector + 0.001

        self.leftRays = np.zeros((3,self.numRays))
        self.leftRays[0,:] = np.cos(self.leftThetaVector)
        self.leftRays[1,:] = -np.sin(self.leftThetaVector)

        self.rightRays = np.zeros((3,self.numRays))
        self.rightRays[0,:] = np.cos(self.rightThetaVector)
        self.rightRays[1,:] = -np.sin(self.rightThetaVector)

    def findNormalEstimationPoints(self,frame):
        leftPoints = np.zeros((3,self.numRays))
        rightPoints = np.zeros((3,self.numRays))
        origin = np.array(frame.transform.GetPosition())
        for i in range(0,self.numRays):
            # find left intersection
            ray = self.leftRays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            leftPoints[i] = self.raycast(self.locator, origin, origin + rayTransformed*self.rayLength)
            if intersection is None:
                break
            
            # find right intersection
            ray = self.rightRays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            rightPoints[i] = self.raycast(self.locator, origin, origin + rayTransformed*self.rayLength)
            if intersection is None:
                break
            
        return distances


    def raycastLeftOrRight(self,frame,rays):
        distances = np.zeros(self.numRays)
        origin = np.array(frame.transform.GetPosition())
        for i in range(0,self.numRays):
            ray = rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            intersection = self.raycast(self.locator, origin, origin + rayTransformed*self.rayLength)
            if intersection is None:
                distances[i] = self.rayLength
            else:
                distances[i] = np.linalg.norm(intersection - origin)
        return distances


    def getNormals(self, frame):
        leftDistances = self.raycastLeftOrRight(frame, self.leftRays)
        rightDistances = self.raycastLeftOrRight(frame, self.rightRays)

        normals = np.zeros((self.numRays,1))[:,0]
        return normals

    def updateDrawNormals(self, frame):
        leftDistances = self.raycastLeftOrRight(frame, self.leftRays)
        rightDistances = self.raycastLeftOrRight(frame, self.rightRays)

        d = DebugData()
        
        x = self.SensorApproximator.approxThetaVector
        y = x * 0.0
        for index,val in enumerate(y):
            y[index] = self.horner(x[index],polyCoefficients)
        
        origin = np.array(frame.transform.GetPosition())
        origin[2] = -0.001

        for i in xrange(self.numRays):
            
            #ray = self.SensorApproximator.approxRays[:,i]
            #rayTransformed = np.array(frame.transform.TransformNormal(ray))
            #intersection = origin + rayTransformed * y[i]
            #intersection[2] = -0.001
            p1 = leftDistances[i]
            p2 = rightDistances[i]
            d.addLine(p1, p2, color=[0,0.1,1])

        vis.updatePolyData(d.getPolyData(), 'polyApprox', colorByName='RGB255')

        #print "draw normals callback is working"


        


    
    