import director.vtkAll as vtk
from director import ioUtils
from director import filterUtils
import director.visualization as vis
from director.debugVis import DebugData

import numpy as np

from PythonQt import QtCore, QtGui

class World(object):

    def __init__(self, worldType='simple'):
        if worldType == 'simple':
            print "initializing world object"

    @staticmethod
    def buildSimpleWorld():
        d = DebugData()
        d.addLine((2,-1,0), (2,1,0), radius=0.1)
        d.addLine((2,-1,0), (1,-2,0), radius=0.1)
        obj = vis.showPolyData(d.getPolyData(), 'world')
        return obj

    @staticmethod
    def buildBoundaries(d, scale=1.0, boundaryType="Warehouse"):
        
        if boundaryType == "Warehouse":
            worldXmin = -20
            worldXmax = 100
            worldYmin = -10
            worldYmax = 10

        if boundaryType == "Square":
            worldXmin = -50*scale
            worldXmax = 50*scale
            worldYmin = -50*scale
            worldYmax = 10*scale

        # draw boundaries for the world
        NW = (worldXmax, worldYmax, 0)
        NE = (worldXmax, worldYmin, 0)
        SE = (worldXmin, worldYmin, 0)
        SW = (worldXmin, worldYmax, 0)
        NW = (worldXmax, worldYmax, 0)

        listOfCorners = [NW, NE, SE, SW, NW]
        for idx, value in enumerate(listOfCorners[:-1]):
            firstEndpt = value
            secondEndpt = listOfCorners[idx+1]
            d.addLine(firstEndpt, secondEndpt, radius=1.0)

        return worldXmin, worldXmax, worldYmin, worldYmax

    
    @staticmethod
    def buildLineSegmentWorld(firstRaycastLocations):
        print "building line segment world"


        print "positions were ", firstRaycastLocations

        d = DebugData()

        firstX = -0.1
        firstY = 0.0

        secondX = firstRaycastLocations[0,0]
        secondY = firstRaycastLocations[0,1]
            
        firstEndpt = (firstX,firstY,0.0)
        secondEndpt = (secondX,secondY,0.0)

        d.addLine(firstEndpt, secondEndpt, radius=0.05)
        
        for i in range(0,len(firstRaycastLocations[:,0])-1):
            print "firstRaycastLocations", i
            
            firstX = firstRaycastLocations[i,0]
            firstY = firstRaycastLocations[i,1]

            secondX = firstRaycastLocations[i+1,0]
            secondY = firstRaycastLocations[i+1,1]
            
            firstEndpt = (firstX,firstY,0.0)
            secondEndpt = (secondX,secondY,0.0)

            d.addLine(firstEndpt, secondEndpt, radius=0.05)

        firstX = firstRaycastLocations[len(firstRaycastLocations)-1,0]
        firstY = firstRaycastLocations[len(firstRaycastLocations)-1,1]

        secondX = -0.1
        secondY = 0.0
            
        firstEndpt = (firstX,firstY,0.0)
        secondEndpt = (secondX,secondY,0.0)

        d.addLine(firstEndpt, secondEndpt, radius=0.05)


        obj = vis.updatePolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        
        return world


    @staticmethod
    def buildLineSegmentTestWorld(percentObsDensity, nonRandom=False, circleRadius=3, scale=1.0, randomSeed=5,
                         obstaclesInnerFraction=1.0):
        #print "building circle world"

        if nonRandom:
            np.random.seed(randomSeed)

        d = DebugData()
        worldXmin = 3
        worldXmax = 15
        worldYmin = -10
        worldYmax = 10
        #print "boundaries done"

        worldArea = (worldXmax-worldXmin)*(worldYmax-worldYmin)
        #print worldArea
        obsScalingFactor = 5.0/12.0
        maxNumObstacles = obsScalingFactor * worldArea
        
        numObstacles = int(obstaclesInnerFraction**2 * percentObsDensity/100.0 * maxNumObstacles)
        print numObstacles, " is num obstacles"

        # draw random stick obstacles
        obsLength = 2.0

        obsXmin = worldXmin + (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsXmax = worldXmax - (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsYmin = worldYmin + (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)
        obsYmax = worldYmax - (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)

        for i in xrange(numObstacles):
            firstX = obsXmin + np.random.rand()*(obsXmax-obsXmin)
            firstY = obsYmin + np.random.rand()*(obsYmax-obsYmin)
            
            secondX = obsXmin + np.random.rand()*(obsXmax-obsXmin)
            secondY = obsYmin + np.random.rand()*(obsYmax-obsYmin)

            firstEndpt = (firstX,firstY,0.2)
            secondEndpt = (firstX,firstY,-0.2)

            #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
            d.addLine(firstEndpt, secondEndpt, radius=0.5)


        obj = vis.updatePolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world


    @staticmethod
    def buildGlobalGoal():
        #print "building circle world"


        d = DebugData()
        worldXmin = 25
        worldXmax = 50
        worldYmin = -20
        worldYmax = 20

        firstX = worldXmin + np.random.rand()*(worldXmax-worldXmin)
        firstY = worldYmin + np.random.rand()*(worldYmax-worldYmin)
       
        firstEndpt = (firstX,firstY,0.2)
        secondEndpt = (firstX,firstY,-0.2)

        #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
        d.addLine(firstEndpt, secondEndpt, radius=0.3, color=[0.5,1,0])

        obj = vis.updatePolyData(d.getPolyData(), 'global_goal', colorByName='RGB255')


        world = World()
        world.visObj = obj
        world.global_goal_x = firstX
        world.global_goal_y = firstY

        return world

    @staticmethod
    def placeLocalGoal(location):
        #print "building circle world"

        d = DebugData()

        firstX = location[0]
        firstY = location[1]
       
        firstEndpt = (firstX,firstY,0.2)
        secondEndpt = (firstX,firstY,-0.2)

        #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
        d.addLine(firstEndpt, secondEndpt, radius=0.3, color=[0.5,1,0])

        obj = vis.updatePolyData(d.getPolyData(), 'local_goal', colorByName='RGB255')

        world = World()
        world.visObj = obj
        world.local_goal_x = firstX
        world.local_goal_y = firstY

        return world


    @staticmethod
    def buildStickWorld(percentObsDensity):
        print "building stick world"

        d = DebugData()
        worldXmin, worldXmax, worldYmin, worldYmax = World.buildBoundaries(d)
        print "boundaries done"

        worldArea = (worldXmax-worldXmin)*(worldYmax-worldYmin)
        print worldArea
        obsScalingFactor = 1.0/12.0
        maxNumObstacles = obsScalingFactor * worldArea
        
        numObstacles = int(percentObsDensity/100.0 * maxNumObstacles)
        print numObstacles

        # draw random stick obstacles
        obsLength = 2.0


        for i in xrange(numObstacles):
            firstX = worldXmin + np.random.rand()*(worldXmax-worldXmin)
            firstY = worldYmin + np.random.rand()*(worldYmax-worldYmin)
            firstEndpt = (firstX,firstY,0)
            
            randTheta = np.random.rand() * 2.0*np.pi
            secondEndpt = (firstX+obsLength*np.cos(randTheta), firstY+obsLength*np.sin(randTheta), 0)

            d.addLine(firstEndpt, secondEndpt, radius=0.2)


        obj = vis.showPolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world


    @staticmethod
    def buildCircleWorld(percentObsDensity, nonRandom=False, circleRadius=3, scale=None, randomSeed=5,
                         obstaclesInnerFraction=1.0):
        #print "building circle world"

        if nonRandom:
            np.random.seed(randomSeed)

        d = DebugData()
        worldXmin, worldXmax, worldYmin, worldYmax = World.buildBoundaries(d, scale=scale, boundaryType="Square")
        #print "boundaries done"

        worldArea = (worldXmax-worldXmin)*(worldYmax-worldYmin)
        #print worldArea
        obsScalingFactor = 1.0/12.0
        maxNumObstacles = obsScalingFactor * worldArea
        
        numObstacles = int(obstaclesInnerFraction**2 * percentObsDensity/100.0 * maxNumObstacles)
        #print numObstacles

        # draw random stick obstacles
        obsLength = 2.0

        obsXmin = worldXmin + (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsXmax = worldXmax - (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsYmin = worldYmin + (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)
        obsYmax = worldYmax - (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)

        for i in xrange(numObstacles):
            firstX = obsXmin + np.random.rand()*(obsXmax-obsXmin)
            firstY = obsYmin + np.random.rand()*(obsYmax-obsYmin)
            firstEndpt = (firstX,firstY,+0.2)
            secondEndpt = (firstX,firstY,-0.2)

            #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
            d.addLine(firstEndpt, secondEndpt, radius=circleRadius)


        obj = vis.showPolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world

    @staticmethod
    def buildWarehouseWorld(percentObsDensity, nonRandom=False, circleRadius=0.1, scale=None, randomSeed=5,
                         obstaclesInnerFraction=1.0):

        if nonRandom:
            np.random.seed(randomSeed)

        d = DebugData()
        worldXmin, worldXmax, worldYmin, worldYmax = World.buildBoundaries(d, scale=scale, boundaryType="Warehouse")

        numObstacles = 8
 
        obsLength = 2.0

        worldLength = worldXmax - worldXmin

        print worldXmin
        print worldXmax

        obstacleZone = [worldXmin + 0.2 * worldLength, worldXmax - 0.2 * worldLength ]

        print obstacleZone

        obstacleLength = obstacleZone[1] - obstacleZone[0]

        incrementSize = obstacleLength * 1.0 / numObstacles

        print incrementSize

        leftOrRight = -1.0
        for i in xrange(numObstacles):
            
            firstX = obstacleZone[0] + incrementSize * i
            leftOrRight = leftOrRight * -1.0

            firstEndpt = (firstX, leftOrRight * worldYmax,0.0)
            secondEndpt = (firstX, 0.0, 0.0)

            #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
            d.addLine(firstEndpt, secondEndpt, radius=circleRadius)

        obj = vis.showPolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world

    @staticmethod
    def buildCircleWarehouseWorld(percentObsDensity, nonRandom=False, circleRadius=3, scale=None, randomSeed=5,
                         obstaclesInnerFraction=1.0):

        if nonRandom:
            np.random.seed(randomSeed)

        d = DebugData()
        worldXmin, worldXmax, worldYmin, worldYmax = World.buildBoundaries(d, scale=scale, boundaryType="Warehouse")

        worldArea = (worldXmax-worldXmin)*(worldYmax-worldYmin)
        obsScalingFactor = 1.0/12.0
        maxNumObstacles = obsScalingFactor * worldArea
        
        numObstacles = int(obstaclesInnerFraction**2 * percentObsDensity/100.0 * maxNumObstacles)
        #print numObstacles

        # draw random stick obstacles
        obsLength = 2.0

        obsXmin = worldXmin + (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsXmax = worldXmax - (1-obstaclesInnerFraction)/2.0*(worldXmax - worldXmin)
        obsYmin = worldYmin + (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)
        obsYmax = worldYmax - (1-obstaclesInnerFraction)/2.0*(worldYmax - worldYmin)

        for i in xrange(numObstacles):
            firstX = obsXmin + np.random.rand()*(obsXmax-obsXmin)
            firstY = obsYmin + np.random.rand()*(obsYmax-obsYmin)
            firstEndpt = (firstX,firstY,+0.2)
            secondEndpt = (firstX,firstY,-0.2)

            #d.addLine(firstEndpt, secondEndpt, radius=2*np.random.randn())
            d.addLine(firstEndpt, secondEndpt, radius=circleRadius)


        obj = vis.showPolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world

    @staticmethod
    def buildFixedTriangleWorld(percentObsDensity):
        print "building fixed triangle world"

        d = DebugData()
        worldXmin, worldXmax, worldYmin, worldYmax = World.buildBoundaries(d)
        print "boundaries done"

        worldArea = (worldXmax-worldXmin)*(worldYmax-worldYmin)
        print worldArea
        obsScalingFactor = 1.0/12.0
        maxNumObstacles = obsScalingFactor * worldArea
        
        numObstacles = int(percentObsDensity/100.0 * maxNumObstacles)
        print numObstacles

        # draw random stick obstacles
        obsLength = 2.0


        for i in xrange(numObstacles):
            firstX = worldXmin + np.random.rand()*(worldXmax-worldXmin)
            firstY = worldYmin + np.random.rand()*(worldYmax-worldYmin)
            firstEndpt = (firstX,firstY,0)
            
            randTheta = np.random.rand() * 2.0*np.pi
            secondEndpt = (firstX+obsLength*np.cos(randTheta), firstY+obsLength*np.sin(randTheta), 0)

            d.addLine(firstEndpt, secondEndpt, radius=0.1)


        obj = vis.showPolyData(d.getPolyData(), 'world')

        world = World()
        world.visObj = obj
        world.Xmax = worldXmax
        world.Xmin = worldXmin
        world.Ymax = worldYmax
        world.Ymin = worldYmin
        world.numObstacles = numObstacles
        world.percentObsDensity = percentObsDensity

        return world

    @staticmethod
    def buildRobot(x=0,y=0):
        #print "building robot"
        polyData = ioUtils.readPolyData('crazyflie.obj')
        
        scale = 0.005
        t = vtk.vtkTransform()
        t.RotateX(90)
        t.Scale(scale, scale, scale)
        polyData = filterUtils.transformPolyData(polyData, t)

        #d = DebugData()
        #d.addCone((x,y,0), (1,0,0), height=0.2, radius=0.1)
        #polyData = d.getPolyData()

        obj = vis.showPolyData(polyData, 'robot')
        robotFrame = vis.addChildFrame(obj)
        return obj, robotFrame

    @staticmethod
    def buildCellLocator(polyData):
        #print "buidling cell locator"

        loc = vtk.vtkCellLocator()
        loc.SetDataSet(polyData)
        loc.BuildLocator()
        return loc