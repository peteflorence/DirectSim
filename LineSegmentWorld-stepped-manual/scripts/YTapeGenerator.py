import numpy as np

class YTapeGenerator(object):

    def __init__(self, FOV=90.0, numRays=21, rayLength=20):
        self.numSteps = 40
        self.initial_state = [0.0, 0.0, 0,0]  #x, y, theta

        self.numRays = numRays
        self.rayLength = rayLength

        FOVrad = FOV * np.pi/180.0
        self.angleMin = -FOVrad/2
        self.angleMax = FOVrad/2

        self.angleGrid = np.linspace(self.angleMin, self.angleMax, self.numRays)

        self.rays = np.zeros((3,self.numRays))
        self.rays[0,:] = np.cos(self.angleGrid)
        self.rays[1,:] = -np.sin(self.angleGrid)

    def GenerateYTape(self, initial_distances):
        
        # convert initial_distances into initial_locations
        initial_locations = self.invertRaycastsToLocations(initial_distances)

        # set line segment world
        self.setLineSegmentWorld(initial_locations)

        laser_distances = initial_distances
        state = initial_state

        # loop:
        for step in range(self.numSteps):

            # compute control input from laser_distances (first time, they are the initial values)
            control_input = self.ComputeControlInput(laser_distances)

            # simulate one step of dynamics to get next state
            state = self.simulateOneStep(start_state, control_input):

            # set laser_distances by raycasting lasers from new state against the line segment world
            laser_distances = raycastAllManual(self,state)


        pass


    def invertRaycastsToLocations(self, raycasts):


        locations = np.zeros((self.numRays,3))

        origin = np.array([self.initial_state[0], self.initial_state[1], 0.0]) # x, y, z.  notice z is not part of state

        for i in range(0,self.numRays):
            ray = self.rays[:,i]
            xnew, ynew = self.rotateByTheta(ray)
            rayTransformed = np.array([xnew, ynew, 0.0])
            intersection = origin + (rayTransformed * raycasts[i])
            locations[i] = intersection

        return locations

    def rotateByTheta(self,ray):
        xold = ray[0]
        yold = ray[1]

        xnew = xold*np.cos(self.state[2]) - yold*np.sin(self.state[2])
        ynew = xold*np.sin(self.state[2]) + yold*np.cos(self.state[2])

        return xnew, ynew


    def setLineSegmentWorld(self, initialRaycastLocations):
        self.LineSegmentEndpoints = np.zeros((np.shape(initialRaycastLocations)[0]+2,3))

        firstX = -0.4
        firstY = 0.0
        firstZ = 0.0

        self.LineSegmentEndpoints[0,:] = [firstX, firstY, firstZ]
        
        for i in range(1,np.shape(initialRaycastLocations)[0]+1):
             self.LineSegmentEndpoints[i,:] = initialRaycastLocations[i-1,:]
             self.LineSegmentEndpoints[i,2] = 0.0


        self.LineSegmentEndpoints[-1,:] = [firstX, firstY, firstZ]

        self.numLineSegmentEndpoints = np.shape(self.LineSegmentEndpoints)[0]
        print "####################"
        print "LineSegmentEndpoints"
        print self.LineSegmentEndpoints
        print "####################"


    def ComputeControlInput(self, laser_distances):
        return self.countInverseDistancesController(laser_distances)
        

    def countInverseDistancesController(self, laser_distances):
        midpoint = np.floor(self.numRays/2.0)
        leftHalf = np.array((laser_distances[0:midpoint]))
        rightHalf = np.array((laser_distances[midpoint:]))

        inverseLeftHalf = (1.0/leftHalf)**2
        inverseRightHalf = (1.0/rightHalf)**2

        numLeft = np.sum(inverseLeftHalf)
        numRight = np.sum(inverseRightHalf)

        if numLeft == numRight:
            u = 0
        elif numLeft > numRight:
            u = -self.u_max
        else:
            u = self.u_max

        return u

    def simulateOneStep(self, start_state, controlInput, startTime=0.0, dt=0.05):
        t = np.linspace(startTime, startTime+dt, 2)
        newState = integrate.odeint(self.dynamics, start_state, t, args=(controlInput,))
        return newState[-1,:]

    def dynamics(self, state, t, controlInput):

        dqdt = np.zeros_like(state)

        u = controlInput
        
        dqdt[0] = self.v*np.cos(state[2])
        dqdt[1] = self.v*np.sin(state[2])
        dqdt[2] = u # we are directly controlling yaw rate
    
        return dqdt

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


    def raycastAllManual(self, state):
        distances = np.zeros(self.numRays)

        origin = np.array([state[0], state[1], 0.0)
        
        # iterate through each laser

        for i in range(self.numRays):

            # transform the ray and find the max range location

            ray = self.rays[:,i]
            xnew, ynew = self.rotateByTheta(ray)
            rayTransformed = np.array([xnew, ynew, 0.0])

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





my_generator = YTapeGenerator()


initial_distances = np.ones((21)) * 20.0
initial_distances[5] = 13.0
initial_distances[6] = 8.0
initial_distances[17] = 15.0
print "Using for my initial_distances, the one input into this function:"
print initial_distances

my_generator.GenerateYTape(initial_distances)
