import numpy as np
import scipy.integrate as integrate
import director.objectmodel as om
import math


class ControllerObj(object):

    def __init__(self, sensor, sensor_approximator, u_max=4, epsilonRand=0.4):
        self.Sensor = sensor
        self.SensorApproximator = sensor_approximator
        self.SensorApproximator.initializeThetaVector(self.Sensor.angleGrid)
        self.SensorApproximator.initializeApproxThetaVector(self.Sensor.angleMin, self.Sensor.angleMax)
        self.numRays = self.Sensor.numRays
        self.actionSet = np.array([u_max,0,-u_max])
        self.epsilonRand = epsilonRand
        self.actionSetIdx = np.arange(0,np.size(self.actionSet))
        self.u_max = u_max

        self.slackParam = 0.1
        self.k = 5
        self.kTurn = 50000000

    def initializeVelocity(self,velocity):
        self.velocity = velocity
        
    def computeControlInput(self, state, t, frame, raycastDistance=None, randomize=False):
        # test cases
        # u = 0
        # u = np.sin(t)
        if raycastDistance is None:
            self.distances = self.Sensor.raycastAll(frame)
        else:
            self.distances = raycastDistance

        # #Barry 12 controller
        

        #u = self.countStuffController()
        #u, actionIdx = self.countInverseDistancesController()
        #u, actionIdx = self.supervisedDPController()
        u, actionIdx = self.polyController()
        #u, actionIdx = self.threeController()

        if randomize:
            if np.random.uniform(0,1,1)[0] < self.epsilonRand:
                # otherActionIdx = np.setdiff1d(self.actionSetIdx, np.array([actionIdx]))
                # randActionIdx = np.random.choice(otherActionIdx)
                actionIdx = np.random.choice(self.actionSetIdx)
                u = self.actionSet[actionIdx]

        return u, actionIdx


    def threeController(self):
        mid_index = (len(self.distances)+1)/2
        d_0 = np.min(self.distances[(mid_index-3):(mid_index+3)])
        d_neg1 = np.min(self.distances[0:(mid_index-3)])
        d_pos1 = np.min(self.distances[(mid_index+4):])

        if np.min(self.distances) > 5:
            u = 0
        elif (d_0 > d_pos1) and (d_0 > d_neg1):
            u = 0
        else:
            c_0 = d_0

            if (d_neg1 > d_pos1):
                c_1 = (d_0 - d_neg1) / 0.25
            else:
                c_1 = (d_pos1 - d_0) / 0.25
            
            if c_1 == 0:
                u = 0
            else:
                u = 10 * (self.velocity + self.slackParam) / (c_0 * c_1)
        
        if u > self.u_max:
            u = self.u_max
        if u < -self.u_max:
            u = -self.u_max

        return -u, 0

    def doubleLPController(self):
        polyCoefficientsLeft = self.SensorApproximator.polyFitConstrainedLP(self.distances)

        if np.min(self.distances) > 5:
            u = 0
        elif (d_0 > d_pos1) and (d_0 > d_neg1):
            u = 0
        else:
            c_0 = d_0

            if (d_neg1 > d_pos1):
                c_1 = (d_0 - d_neg1) / 0.25
            else:
                c_1 = (d_pos1 - d_0) / 0.25
            
            if c_1 == 0:
                u = 0
            else:
                u = 10 * (self.velocity + self.slackParam) / (c_0 * c_1)
        
        if u > self.u_max:
            u = self.u_max
        if u < -self.u_max:
            u = -self.u_max

        return -u, 0


    # this is from derivation with John's help
    def polyController(self):
        polyCoefficients = self.SensorApproximator.polyFitConstrainedLP(self.distances)

        if polyCoefficients == None:
            u = 0
        elif polyCoefficients[0] > 12:
            u = 0
        elif polyCoefficients[1] == 0:
            u = 0
        else:
            u = self.k * (self.velocity + self.slackParam) / (polyCoefficients[0] * polyCoefficients[1])

        if u > self.u_max:
            u = self.u_max
        if u < -self.u_max:
            u = -self.u_max

        #print polyCoefficients[0], polyCoefficients[1], u
        return -u, 0


    # this was from a confused derivation
    def polyControllerTangent(self):
        polyCoefficients = self.SensorApproximator.polyFitConstrainedLP(self.distances)

        if polyCoefficients[0] > 19:
            u = 0
        elif polyCoefficients[1] == 0:
            u = 0
        else:
            u = (1/math.tan(polyCoefficients[1])) *  ( self.kTurn * self.velocity / polyCoefficients[0] + self.slackParam)

        if u > self.u_max:
            u = self.u_max
        if u < -self.u_max:
            u = -self.u_max

        #print polyCoefficients[0], polyCoefficients[1], u
        return u, 0


    def countStuffController(self):
        firstHalf = self.distances[0:self.numRays/2]
        secondHalf = self.distances[self.numRays/2:]
        tol = 1e-3;

        numLeft = np.size(np.where(firstHalf < self.Sensor.rayLength - tol))
        numRight = np.size(np.where(secondHalf < self.Sensor.rayLength - tol))

        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def countInverseDistancesController(self):
        midpoint = np.floor(self.numRays/2.0)
        leftHalf = np.array((self.distances[0:midpoint]))
        rightHalf = np.array((self.distances[midpoint:]))
        tol = 1e-3;

        inverseLeftHalf = (1.0/leftHalf)**2
        inverseRightHalf = (1.0/rightHalf)**2

        numLeft = np.sum(inverseLeftHalf)
        numRight = np.sum(inverseRightHalf)


        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0


        # print "leftHalf ", leftHalf
        # print "rightHalf", rightHalf
        # print "inverseLeftHalf", inverseLeftHalf
        # print "inverserRightHalf", inverseRightHalf
        # print "numLeft", numLeft
        # print "numRight", numRight

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def supervisedDPController(self):

        #9 sensors
        #w = np.array([-0.03364086, -0.06146491, -0.11796771, -0.1699006,  -0.00097573,  0.17137526, 0.11952639,  0.06076998,  0.03276566])

        # 20 sensors slow
        #w = [-0.02109653, -0.01746332, -0.02388135, -0.0314405,  -0.04294771, -0.05559809, -0.07757404, -0.08611176, -0.07874338, -0.04490507,  0.04384566,  0.08218653, 0.08214135,  0.08184778,  0.05594081,  0.04173576,  0.03131204,  0.02372157, 0.01681253,  0.02070505]
        

        w = np.array([-0.00300497, -0.00130277, -0.00148445, -0.00313336, -0.01317847, -0.02037713, -0.04797057, -0.09098885, -0.13847444, -0.11547472,  0.11733177,  0.13888244, 0.08363806,  0.04846861,  0.02326903,  0.01233246,  0.00382634,  0.00258145, 0.00284502,  0.00306195])
        w = w[::-1]

        u = np.dot(self.distances, w)

        #if u > 4: u = 4
        #if u < -4: u = -4

        return u, 0


    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame)

