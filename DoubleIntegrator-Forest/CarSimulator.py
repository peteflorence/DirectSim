import director.vtkAll as vtk
import director.visualization as vis
import director.objectmodel as om
from director.debugVis import DebugData
from director.consoleapp import ConsoleApp
from director.timercallback import TimerCallback
from director import applogic
from director import screengrabberpanel
from director import cameracontrolpanel

from director import transformUtils
import numpy as np
import time
import scipy.integrate as integrate
import argparse
import matplotlib.pyplot as plt
import shelve

from PythonQt import QtCore, QtGui

from world import World
from car import CarPlant
from controller import ControllerObj
from actionSet import ActionSetObj



class Simulator(object):


    def __init__(self, percentObsDensity=20, endTime=40, nonRandomWorld=False,
                 circleRadius=0.7, worldScale=1.0, autoInitialize=True, verbose=True):
        self.verbose = verbose
        self.startSimTime = time.time()
        self.collisionThreshold = 0.4
        self.randomSeed = 5

        self.percentObsDensity = percentObsDensity
        self.defaultControllerTime = 1000
        self.nonRandomWorld = nonRandomWorld
        self.circleRadius = circleRadius
        self.worldScale = worldScale

        # create the visualizer object
        self.app = ConsoleApp()
        self.view = self.app.createView(useGrid=False)

        self.initializeOptions()
        self.initializeColorMap()
        
        if autoInitialize:
            self.initialize()

        self.XVelocity_drawing = 0.0
        self.YVelocity_drawing = 0.0

    def initializeOptions(self):
        self.options = dict()

        self.options['World'] = dict()
        self.options['World']['obstaclesInnerFraction'] = 0.98
        self.options['World']['randomSeed'] = 40
        self.options['World']['percentObsDensity'] = 0.0
        self.options['World']['nonRandomWorld'] = True
        self.options['World']['circleRadius'] = 1.0
        self.options['World']['scale'] = 1


        self.options['Car'] = dict()
        self.options['Car']['velocity'] = 4.0

        self.options['dt'] = 0.05

        self.options['runTime'] = dict()
        self.options['runTime']['defaultControllerTime'] = 100


    def setDefaultOptions(self):

        defaultOptions = dict()


        defaultOptions['World'] = dict()
        defaultOptions['World']['obstaclesInnerFraction'] = 0.98
        defaultOptions['World']['randomSeed'] = 40
        defaultOptions['World']['percentObsDensity'] = 30
        defaultOptions['World']['nonRandomWorld'] = True
        defaultOptions['World']['circleRadius'] = 1.75
        defaultOptions['World']['scale'] = 2.5


        defaultOptions['Car'] = dict()
        defaultOptions['Car']['velocity'] = 20

        defaultOptions['dt'] = 0.05


        defaultOptions['runTime'] = dict()
        defaultOptions['runTime']['defaultControllerTime'] = 100


        for k in defaultOptions:
            self.options.setdefault(k, defaultOptions[k])


        for k in defaultOptions:
            if not isinstance(defaultOptions[k], dict):
                continue

            for j in defaultOptions[k]:
                self.options[k].setdefault(j, defaultOptions[k][j])


    def initializeColorMap(self):
        self.colorMap = dict()
        self.colorMap['default'] = [0,1,0]

    def initialize(self):

        self.dt = self.options['dt']
        self.controllerTypeOrder = ['default']

        self.setDefaultOptions()


        self.Controller = ControllerObj()

        self.Car = CarPlant(controller=self.Controller,
                            velocity=self.options['Car']['velocity'])

        self.Controller.initializeVelocity(self.Car.v)

        self.ActionSet = ActionSetObj()



        # create the things needed for simulation
        om.removeFromObjectModel(om.findObjectByName('world'))
        self.world = World.buildLineSegmentTestWorld(percentObsDensity=self.options['World']['percentObsDensity'],
                                            circleRadius=self.options['World']['circleRadius'],
                                            nonRandom=self.options['World']['nonRandomWorld'],
                                            scale=self.options['World']['scale'],
                                            randomSeed=self.options['World']['randomSeed'],
                                            obstaclesInnerFraction=self.options['World']['obstaclesInnerFraction'])

        om.removeFromObjectModel(om.findObjectByName('robot'))
        self.robot, self.frame = World.buildRobot()
        

        self.frame = self.robot.getChildFrame()
        self.frame.setProperty('Scale', 3)
        #self.frame.setProperty('Visible', False)
        #self.frame.setProperty('Edit', True)
        self.frame.widget.HandleRotationEnabledOff()
        rep = self.frame.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)

        self.defaultControllerTime = self.options['runTime']['defaultControllerTime']

        self.Car.setFrame(self.frame)
        print "Finished initialization"


    def runSingleSimulation(self, controllerType='default', simulationCutoff=None):


        #self.setRandomCollisionFreeInitialState()
        self.setInitialStateAtZero()

        currentCarState = np.copy(self.Car.state)
        nextCarState = np.copy(self.Car.state)
        self.setRobotFrameState(currentCarState[0], currentCarState[1], currentCarState[2])
        
        # DELETED FIRST RAYCAST AND RAYCAST LOCATIONS



        # record the reward data
        runData = dict()
        startIdx = self.counter

        thisRunIndex = 0
        NMaxSteps = 100

        while (self.counter < self.numTimesteps - 1):
            thisRunIndex += 1
            idx = self.counter
            currentTime = self.t[idx]
            self.stateOverTime[idx,:] = currentCarState
            x = self.stateOverTime[idx,0]
            y = self.stateOverTime[idx,1]
            
            self.setRobotFrameState(x,y,0.0)
            # self.setRobotState(currentCarState[0], currentCarState[1], currentCarState[2])



            if controllerType in ["default"]:
                controlInput, controlInputIdx = self.Controller.computeControlInput(currentCarState,
                                                                            currentTime, self.frame,
                                                                            randomize=False)

            self.controlInputData[idx] = controlInput

            nextCarState = self.Car.simulateOneStep(controlInput=controlInput, dt=self.dt)
            print "NEXTCARSTATE is ", nextCarState

        
            x = nextCarState[0]
            y = nextCarState[1]
            
            self.setRobotFrameState(x,y,0.0)


            if controllerType in ["default", "defaultRandom"]:
                nextControlInput, nextControlInputIdx = self.Controller.computeControlInput(nextCarState,
                                                                            currentTime, self.frame,
                                                                            randomize=False)


            #bookkeeping
            self.counter+=1

            # break if we are in collision


            if self.counter >= simulationCutoff:
                break


        # fill in the last state by hand
        self.stateOverTime[self.counter,:] = currentCarState


        # this just makes sure we don't get stuck in an infinite loop.
        if startIdx == self.counter:
            self.counter += 1

        return runData

    def setNumpyRandomSeed(self, seed=1):
        np.random.seed(seed)

    def runBatchSimulation(self, endTime=None, dt=0.05):

        
        self.controllerTypeOrder = ['default']
        self.counter = 0
        self.simulationData = []
    
        self.initializeStatusBar()

        self.idxDict = dict()
        numRunsCounter = 0


        self.idxDict['default'] = self.counter
        loopStartIdx = self.counter
        simCutoff = min(loopStartIdx + self.defaultControllerTime/dt, self.numTimesteps)
        
        while ((self.counter - loopStartIdx < self.defaultControllerTime/dt) and self.counter < self.numTimesteps-1):
            self.printStatusBar()
            startIdx = self.counter
            runData = self.runSingleSimulation(controllerType='default',
                                               simulationCutoff=simCutoff)
            runData['startIdx'] = startIdx
            runData['controllerType'] = "default"
            runData['duration'] = self.counter - runData['startIdx']
            runData['endIdx'] = self.counter
            runData['runNumber'] = numRunsCounter
            numRunsCounter+=1
            self.simulationData.append(runData)

        # BOOKKEEPING
        # truncate stateOverTime, raycastData, controlInputs to be the correct size
        self.numTimesteps = self.counter + 1
        self.stateOverTime = self.stateOverTime[0:self.counter+1, :]
        self.controlInputData = self.controlInputData[0:self.counter+1]
        self.endTime = 1.0*self.counter/self.numTimesteps*self.endTime




    def initializeStatusBar(self):
        self.numTicks = 10
        self.nextTickComplete = 1.0 / float(self.numTicks)
        self.nextTickIdx = 1
        print "Simulation percentage complete: (", self.numTicks, " # is complete)"
    
    def printStatusBar(self):
        fractionDone = float(self.counter) / float(self.numTimesteps)
        if fractionDone > self.nextTickComplete:

            self.nextTickIdx += 1
            self.nextTickComplete += 1.0 / self.numTicks

            timeSoFar = time.time() - self.startSimTime 
            estimatedTimeLeft_sec = (1 - fractionDone) * timeSoFar / fractionDone
            estimatedTimeLeft_minutes = estimatedTimeLeft_sec / 60.0

            print "#" * self.nextTickIdx, "-" * (self.numTicks - self.nextTickIdx), "estimated", estimatedTimeLeft_minutes, "minutes left"


    def setCollisionFreeInitialState(self):
        tol = 5

        while True:
            
            x = 0.0
            y =   -5.0
            theta = 0 #+ np.random.uniform(0,2*np.pi,1)[0] * 0.01
            
            self.Car.setCarState(x,y,0.0,0.0)
            self.setRobotFrameState(x,y,theta)

            print "In loop"

            if not self.checkInCollision():
                break
                
        return x,y,theta


    
    def setInitialStateAtZero(self):
        
        x = 0.0
        y = 0.0
        theta = 0.0
        
        self.Car.setCarState(x,y,0.0,0.0)
        self.setRobotFrameState(x,y,theta)

        return x,y,theta



    def setRandomCollisionFreeInitialState(self):
        tol = 5

        while True:
            
            x = np.random.uniform(self.world.Xmin+tol, self.world.Xmax-tol, 1)[0]
            y = np.random.uniform(self.world.Ymin+tol, self.world.Ymax-tol, 1)[0]
            theta = np.random.uniform(0,2*np.pi,1)[0]
            
            self.Car.setCarState(x,y,theta)
            self.setRobotFrameState(x,y,theta)

            if not self.checkInCollision():
                break

        return x,y,theta

    def setupPlayback(self):

        self.timer = TimerCallback(targetFps=30)
        self.timer.callback = self.tick

        playButtonFps = 1.0/self.dt
        print "playButtonFPS", playButtonFps
        self.playTimer = TimerCallback(targetFps=playButtonFps)
        self.playTimer.callback = self.playTimerCallback
        self.sliderMovedByPlayTimer = False

        panel = QtGui.QWidget()
        l = QtGui.QHBoxLayout(panel)

        self.max_velocity = 20.0

        sliderXVelocity = QtGui.QSlider(QtCore.Qt.Horizontal)
        sliderXVelocity.connect('valueChanged(int)', self.onXVelocityChanged)
        sliderXVelocity.setMaximum(self.max_velocity)
        sliderXVelocity.setMinimum(-self.max_velocity)

        l.addWidget(sliderXVelocity)

        sliderYVelocity = QtGui.QSlider(QtCore.Qt.Horizontal)
        sliderYVelocity.connect('valueChanged(int)', self.onYVelocityChanged)
        sliderYVelocity.setMaximum(self.max_velocity)
        sliderYVelocity.setMinimum(-self.max_velocity)
        l.addWidget(sliderYVelocity)


        randomGlobalGoalButton = QtGui.QPushButton('Initialize Random Global Goal')
        randomGlobalGoalButton.connect('clicked()', self.onRandomGlobalGoalButton)
        l.addWidget(randomGlobalGoalButton)

        randomObstaclesButton = QtGui.QPushButton('Initialize Random Obstacles')
        randomObstaclesButton.connect('clicked()', self.onRandomObstaclesButton)
        l.addWidget(randomObstaclesButton)

        drawActionSetButton = QtGui.QPushButton('Draw Action Set')
        drawActionSetButton.connect('clicked()', self.onDrawActionSetButton)
        l.addWidget(drawActionSetButton)

        runSimButton = QtGui.QPushButton('Simulate')
        runSimButton.connect('clicked()', self.onRunSimButton)
        l.addWidget(runSimButton)

        playButton = QtGui.QPushButton('Play/Pause')
        playButton.connect('clicked()', self.onPlayButton)

        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.connect('valueChanged(int)', self.onSliderChanged)
        self.sliderMax = self.numTimesteps
        slider.setMaximum(self.sliderMax)
        self.slider = slider

        

        # slider4 = QtGui.QSlider(QtCore.Qt.Horizontal)
        # slider4.setMaximum(self.sliderMax)
        # l.addWidget(slider4)

        # slider5 = QtGui.QSlider(QtCore.Qt.Horizontal)
        # slider5.setMaximum(self.sliderMax)
        # l.addWidget(slider5)

        # slider5 = QtGui.QSlider(QtCore.Qt.Horizontal)
        # slider5.setMaximum(self.sliderMax)
        # l.addWidget(slider5)

        # slider6 = QtGui.QSlider(QtCore.Qt.Horizontal)
        # slider6.setMaximum(self.sliderMax)
        # l.addWidget(slider6)

        # slider7 = QtGui.QSlider(QtCore.Qt.Horizontal)
        # slider7.setMaximum(self.sliderMax)
        # l.addWidget(slider7)



        l.addWidget(playButton)
        l.addWidget(slider)

        w = QtGui.QWidget()
        l = QtGui.QVBoxLayout(w)
        l.addWidget(self.view)
        self.view.orientationMarkerWidget().Off()
        l.addWidget(panel)
        w.showMaximized()

        
        # We need to connectFrameModified with updateDrawActionSet
        # self.frame.connectFrameModified(self.updateDrawIntersection)
        # self.updateDrawIntersection(self.frame)
        
        applogic.resetCamera(viewDirection=[0.2,0,-1])
        self.view.showMaximized()
        self.view.raise_()
        panel = screengrabberpanel.ScreenGrabberPanel(self.view)
        panel.widget.show()

        cameracontrolpanel.CameraControlPanel(self.view).widget.show()

        elapsed = time.time() - self.startSimTime
        simRate = self.counter/elapsed
        print "Total run time", elapsed
        print "Ticks (Hz)", simRate
        print "Number of steps taken", self.counter
        self.onRandomObstaclesButton()
        self.app.start()
        
       


    def getControllerTypeFromCounter(self, counter):
        name = self.controllerTypeOrder[0]

        for controllerType in self.controllerTypeOrder[1:]:
            if counter >= self.idxDict[controllerType]:
                name = controllerType

        return name


    def setRobotFrameState(self, x, y, theta):
        t = vtk.vtkTransform()
        t.Translate(x,y,0.0)
        t.RotateZ(np.degrees(theta))
        self.robot.getChildFrame().copyFrame(t)

    # returns true if we are in collision
    def checkInCollision(self, raycastDistance=None):
        
        # We need to re-evaluate this
        return False

        # if raycastDistance[(len(raycastDistance)+1)/2] < self.collisionThreshold:
        #     return True
        # else:
        #     return False

    def tick(self):
        #print timer.elapsed
        #simulate(t.elapsed)
        x = np.sin(time.time())
        y = np.cos(time.time())
        self.setRobotFrameState(x,y,0.0)
        if (time.time() - self.playTime) > self.endTime:
            self.playTimer.stop()

    def tick2(self):
        newtime = time.time() - self.playTime
        print time.time() - self.playTime
        x = np.sin(newtime)
        y = np.cos(newtime)
        self.setRobotFrameState(x,y,0.0)

    # just increment the slider, stop the timer if we get to the end
    def playTimerCallback(self):
        self.sliderMovedByPlayTimer = True
        currentIdx = self.slider.value
        nextIdx = currentIdx + 1
        self.slider.setSliderPosition(nextIdx)
        if currentIdx >= self.sliderMax:
            print "reached end of tape, stopping playTime"
            self.playTimer.stop()

    def onSliderChanged(self, value):
        if not self.sliderMovedByPlayTimer:
            self.playTimer.stop()
        numSteps = len(self.stateOverTime)
        idx = int(np.floor(numSteps*(1.0*value/self.sliderMax)))
        idx = min(idx, numSteps-1)
        x,y, xdot, ydot = self.stateOverTime[idx]
        self.setRobotFrameState(x,y,0)
        self.sliderMovedByPlayTimer = False

    def onXVelocityChanged(self, value):

        self.XVelocity_drawing = value
        self.onDrawActionSetButton()
        print "x velocity changed to ", value

    def onYVelocityChanged(self, value):
        print value

        self.YVelocity_drawing = -value
        self.onDrawActionSetButton()
        print "y velocity changed to ", -value


    def onRandomObstaclesButton(self):
        print "random obstacles button pressed"
        self.setInitialStateAtZero()
        self.world = World.buildLineSegmentTestWorld(percentObsDensity=8.0,
                                            circleRadius=self.options['World']['circleRadius'],
                                            nonRandom=False,
                                            scale=self.options['World']['scale'],
                                            randomSeed=self.options['World']['randomSeed'],
                                            obstaclesInnerFraction=self.options['World']['obstaclesInnerFraction'])
        
        self.locator = World.buildCellLocator(self.world.visObj.polyData)

    def onRandomGlobalGoalButton(self):
        print "random global goal button pressed"
        self.globalGoal = World.buildGlobalGoal()
   

    def onDrawActionSetButton(self):
        print "drawing action set"
        #self.ActionSet.computeFinalPositions(self.XVelocity_drawing,self.YVelocity_drawing)
        self.ActionSet.computeAllPositions(self.XVelocity_drawing,self.YVelocity_drawing)
        #self.ActionSet.drawActionSetFinal()
        self.ActionSet.drawActionSetFull()

    def onRunSimButton(self):
        self.runBatchSimulation()
        self.saveToFile("latest")
        
    def onPlayButton(self):

        if self.playTimer.isActive():
            self.onPauseButton()
            return

        print 'play'
        self.playTimer.start()
        self.playTime = time.time()

    def onPauseButton(self):
        print 'pause'
        self.playTimer.stop()

    def saveToFile(self, filename):

        # should also save the run data if it is available, i.e. stateOverTime, rewardOverTime

        filename = 'data/' + filename + ".out"
        my_shelf = shelve.open(filename,'n')

        my_shelf['options'] = self.options

        my_shelf['simulationData'] = self.simulationData
        my_shelf['stateOverTime'] = self.stateOverTime
        my_shelf['controlInputData'] = self.controlInputData
        my_shelf['numTimesteps'] = self.numTimesteps
        my_shelf['idxDict'] = self.idxDict
        my_shelf['counter'] = self.counter
        my_shelf.close()

    def run(self, launchApp=True):
        self.counter = 1
        
        # for use in playback
        self.dt = self.options['dt']
        self.endTime = self.defaultControllerTime # used to be the sum of the other times as well

        self.t = np.arange(0.0, self.endTime, self.dt)
        maxNumTimesteps = np.size(self.t)

        self.stateOverTime = np.zeros((maxNumTimesteps+1, 4))
        self.controlInputData = np.zeros((maxNumTimesteps+1,2))
        self.numTimesteps = maxNumTimesteps

        self.runBatchSimulation()


        if launchApp:
            self.setupPlayback() 


    @staticmethod
    def loadFromFile(filename):
        filename = 'data/' + filename + ".out"
        sim = Simulator(autoInitialize=False, verbose=False)

        my_shelf = shelve.open(filename)
        sim.options = my_shelf['options']
        sim.initialize()

        sim.simulationData = my_shelf['simulationData']
        sim.stateOverTime = np.array(my_shelf['stateOverTime'])
        sim.raycastData = np.array( my_shelf['raycastData'])
        sim.controlInputData = np.array(my_shelf['controlInputData'])
        sim.numTimesteps = my_shelf['numTimesteps']
        sim.idxDict = my_shelf['idxDict']
        sim.counter = my_shelf['counter']

        my_shelf.close()

        return sim




if __name__ == "__main__":
    # main(sys.argv[1:])
    parser = argparse.ArgumentParser(description='interpret simulation parameters')
    parser.add_argument('--percentObsDensity', type=float, nargs=1, default=[10])
    parser.add_argument('--endTime', type=int, nargs=1, default=[40])
    parser.add_argument('--nonRandomWorld', action='store_true', default=False)
    parser.add_argument('--circleRadius', type=float, nargs=1, default=0.7)
    parser.add_argument('--worldScale', type=float, nargs=1, default=1.0)
    
    argNamespace = parser.parse_args()
    percentObsDensity = argNamespace.percentObsDensity[0]
    endTime = argNamespace.endTime[0]

    nonRandomWorld = argNamespace.nonRandomWorld
    circleRadius = argNamespace.circleRadius[0]
    worldScale = argNamespace.worldScale[0]
    
    sim = Simulator(percentObsDensity=percentObsDensity, endTime=endTime, randomizeControl=randomizeControl,
                    nonRandomWorld=nonRandomWorld, circleRadius=circleRadius, worldScale=worldScale,
                    supervisedTrainingTime=supervisedTrainingTime)
    sim.run()


