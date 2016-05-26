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
from sensor import SensorObj
from controller import ControllerObj
from actionSet import ActionSetObj


class Simulator(object):


    def __init__(self, percentObsDensity=20, endTime=40, nonRandomWorld=False,
                 circleRadius=0.7, worldScale=1.0, autoInitialize=True, verbose=True):
        self.verbose = verbose
        self.startSimTime = time.time()
        self.collisionThreshold = 1.0
        self.randomSeed = 5
        self.Sensor_rayLength = 8

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

        self.initial_XVelocity = 0.0
        self.initial_YVelocity = 0.0

        self.running_sim = True

        self.currentIdx = 0;
        
        if autoInitialize:
            self.initialize()

    def initializeOptions(self):
        self.options = dict()

        self.options['World'] = dict()
        self.options['World']['obstaclesInnerFraction'] = 0.98
        self.options['World']['randomSeed'] = 40
        self.options['World']['percentObsDensity'] = 7.0
        self.options['World']['nonRandomWorld'] = True
        self.options['World']['circleRadius'] = 0.6
        self.circleRadius = self.options['World']['circleRadius']
        self.options['World']['scale'] = 2.0

        self.options['Sensor'] = dict()
        self.options['Sensor']['rayLength'] = 10
        self.options['Sensor']['numRays'] = 40


        self.options['Car'] = dict()
        self.options['Car']['velocity'] = 20

        self.options['dt'] = 0.05

        self.options['runTime'] = dict()
        self.options['runTime']['defaultControllerTime'] = 100


    def initializeColorMap(self):
        self.colorMap = dict()
        self.colorMap['default'] = [0,1,0]

    def initialize(self):

        self.dt = self.options['dt']
        self.controllerTypeOrder = ['default']

        self.Controller = ControllerObj()

        self.Sensor = SensorObj(rayLength=self.options['Sensor']['rayLength'],
                                numRays=self.options['Sensor']['numRays'])

        self.Car = CarPlant(controller=self.Controller,
                            velocity=self.options['Car']['velocity'])

        self.Controller.initializeVelocity(self.Car.v)

        self.ActionSet = ActionSetObj()

        # create the things needed for simulation
        om.removeFromObjectModel(om.findObjectByName('world'))
        self.world = World.buildCircleWorld(percentObsDensity=self.options['World']['percentObsDensity'],
                                            circleRadius=self.options['World']['circleRadius'],
                                            nonRandom=self.options['World']['nonRandomWorld'],
                                            scale=self.options['World']['scale'],
                                            randomSeed=self.options['World']['randomSeed'],
                                            obstaclesInnerFraction=self.options['World']['obstaclesInnerFraction'])

        om.removeFromObjectModel(om.findObjectByName('robot'))
        self.robot, self.frame = World.buildRobot()
        self.locator = World.buildCellLocator(self.world.visObj.polyData)
        
        #will need to set locator for purple trajectories
        #self.Sensor.setLocator(self.locator)
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


    def terminalEuclideanCostForTrajectories(self):
        # access the trajectories
        x_traj = self.ActionSet.p_x_trajectories
        y_traj = self.ActionSet.p_y_trajectories


        final_x_positions = x_traj[:,-1]
        final_y_positions = y_traj[:,-1]

        #access the global goal
        global_goal_x = self.globalGoal.global_goal_x
        global_goal_y = self.globalGoal.global_goal_y

        euclideans_vector = []

        # def getKey(item):
        #     return item[1]

        for i in xrange(self.ActionSet.num_x_bins):
            for j in xrange(self.ActionSet.num_y_bins):
                distance = np.sqrt((final_x_positions[i] - global_goal_x)**2 + (final_y_positions[j] - global_goal_y)**2)
                euclideans_vector.append(distance)


        #sorted_ranks_with_indices = sorted(ranks_with_indices,key=getKey)

        return np.array(euclideans_vector)

    def CheckIfCollisionFreeTrajectoryIndex(self, traj_index):
        # accessing the right trajectory
        x_trajectory_to_check = self.ActionSet.p_x_trajectories[traj_index[0]]
        y_trajectory_to_check = self.ActionSet.p_y_trajectories[traj_index[1]]


        #check if anything is too close to the center of the circles
        #print self.world.list_of_circles
        for i in range(len(x_trajectory_to_check)):
            point = ( x_trajectory_to_check[i], y_trajectory_to_check[i]  )
            if not self.CheckIfCollisionFreePoint(point):
                #print "trajectory wasn't free"
                return False 


        return True

    def CheckIfCollisionFreePoint(self, point):

        for circle_center in self.world.list_of_circles:
            #print "circle centers first"
            #print circle_center[0], circle_center[1]
            #print self.circleRadius
            #print point[0], point[1]
            #print (circle_center[0] - point[0])**2 + (circle_center[1]-point[1])**2
            if  ( (circle_center[0] - point[0])**2 + (circle_center[1]-point[1])**2 < self.circleRadius**2):
                #print "I found a point in a circle"
                return False

        if point[0] > self.world.Xmax:
            #print "I would have gone past top wall"
            return False
        if point[0] < self.world.Xmin:
            #print "I would have gone below bottom wall"
            return False
        if point[1] > self.world.Ymax:
            #print "I would have gone to the right of the side wall"
            return False    
        if point[1] < self.world.Ymin:
            #print "I would have gone to the left of the side wall"
            return False

        return True

    def computeProbabilitiesOfCollisionAllTrajectories(self, currentRaycastIntersectionLocations):
        probability_vector = []
        indices_vector = []
        for x_index in xrange(self.ActionSet.num_x_bins):
            for y_index in xrange(self.ActionSet.num_y_bins):
                probability_of_collision = self.computeProbabilityOfCollisionOneTrajectory(x_index, y_index, currentRaycastIntersectionLocations)
                probability_vector.append(probability_of_collision)
                indices_vector.append([x_index, y_index])

        return np.array(probability_vector), indices_vector
        

    def computeProbabilityOfCollisionOneTrajectory(self, x_index, y_index, currentRaycastIntersectionLocations):
        probability_no_collision = 1
        for time_step_index, time_step_value in enumerate(self.ActionSet.t_vector):
            for obstacle_center in currentRaycastIntersectionLocations:

                probability_of_collision_step_k_obstacle_j = self.computeProbabilityOfCollisionOneStepOneObstacle(x_index, y_index, time_step_index, time_step_value, obstacle_center)

                probability_no_collision_step_k_obstacle_j = 1 - probability_of_collision_step_k_obstacle_j

                probability_no_collision = probability_no_collision * probability_no_collision_step_k_obstacle_j

        return float(1.0 - probability_no_collision)

    def computeProbabilityOfCollisionOneStepOneObstacle(self, x_index, y_index, time_step_index, time_step_value, obstacle_center):
        volume = 5.0
        Sigma_sensor = np.zeros((3,3))
        np.fill_diagonal(Sigma_sensor, self.sigma_sensor)

        variance_x = (2.5 + abs(self.current_initial_velocity_x*0.1))*time_step_value+0.01
        variance_y = (2.5 + abs(self.current_initial_velocity_y*0.1))*time_step_value+0.01
        variance_z = (1.5)*time_step_value+0.01

        Sigma_robot = np.zeros((3,3))
        np.fill_diagonal(Sigma_sensor, [variance_x, variance_y, variance_z])


        Sigma_C = Sigma_robot + Sigma_sensor

        denominator = np.sqrt(np.linalg.det(2*np.pi*Sigma_C))

        x = self.ActionSet.p_x_trajectories[x_index, time_step_index]
        y = self.ActionSet.p_y_trajectories[y_index, time_step_index]
        z = 0

        x_robot = np.array([x, y, z])
        obstacle_center = np.array(obstacle_center)

        exponent = - 0.5 * np.matrix((x_robot - obstacle_center)) * np.matrix(np.linalg.inv(Sigma_C)) * np.matrix((x_robot - obstacle_center)).T

        return volume / denominator * np.exp(exponent)

    def computeJerkVector(self,controlInput):
        jerk_vector = []
        for x_index in xrange(self.ActionSet.num_x_bins):
            for y_index in xrange(self.ActionSet.num_y_bins):
                last_x_accel = controlInput[0]
                last_y_accel = controlInput[1]

                this_x_accel = self.ActionSet.a_x[x_index]
                this_y_accel = self.ActionSet.a_y[y_index]

                jerk = np.sqrt( (last_x_accel - this_x_accel)**2 + (last_y_accel - this_y_accel)**2 )
                jerk_vector.append(jerk)

        return np.array(jerk_vector) 


    def runSingleSimulation(self, controllerType='default', simulationCutoff=None):


        self.Car.setCarState(0.0,0.0,self.initial_XVelocity,self.initial_YVelocity)
        self.setRobotFrameState(0.0,0.0,0.0)


        currentCarState = np.copy(self.Car.state)
        nextCarState = np.copy(self.Car.state)
        self.setRobotFrameState(currentCarState[0], currentCarState[1], currentCarState[2])

        self.Sensor.setLocator(self.locator)

        firstRaycast = self.Sensor.raycastAll(self.frame)
        firstRaycastLocations = self.Sensor.raycastAllLocations(self.frame)

        nextRaycast = np.zeros(self.Sensor.numRays)

        # record the reward data
        runData = dict()
        startIdx = self.counter

        controlInput = [0,0]


        while (self.counter < self.numTimesteps - 1):
            idx = self.counter
            currentTime = self.t[idx]
            self.stateOverTime[idx,:] = currentCarState
            x = self.stateOverTime[idx,0]
            y = self.stateOverTime[idx,1]
            self.setRobotFrameState(x,y,0.0)
            # self.setRobotState(currentCarState[0], currentCarState[1], currentCarState[2])

            currentRaycastIntersectionLocations = self.Sensor.raycastLocationsOnlyOfIntersections(self.frame)
            


            if controllerType not in self.colorMap.keys():
                print
                raise ValueError("controller of type " + controllerType + " not supported")

            # compute all trajectories from action set
            self.ActionSet.computeAllPositions(currentCarState[0], currentCarState[1], currentCarState[2],currentCarState[3])
            self.current_initial_velocity_x = currentCarState[2]
            self.current_initial_velocity_y = currentCarState[3]

            probability_vector, indices_list = self.computeProbabilitiesOfCollisionAllTrajectories(currentRaycastIntersectionLocations)

            euclideans_vector = self.terminalEuclideanCostForTrajectories()

            jerk_vector = self.computeJerkVector(controlInput)

            # k_collision_cost = 10
            # k_terminal_cost = 1
            # k_jerk = 0.1
            # sum_vector = probability_vector*k_collision_cost + euclideans_vector/np.max(euclideans_vector)*k_terminal_cost + k_jerk*jerk_vector/np.max(jerk_vector)
            # min_action_index_in_vector = np.argmin(sum_vector)
            # x_index_to_use = indices_list[min_action_index_in_vector][0]
            # y_index_to_use = indices_list[min_action_index_in_vector][1]

            

            # convert to probability no collision
            probability_vector = np.ones(len(probability_vector)) - probability_vector
            
            # convert distances to amount of progress
            current_distance = np.sqrt((currentCarState[0] - self.globalGoal.global_goal_x)**2 + (currentCarState[1] - self.globalGoal.global_goal_y)**2)
            #print "euclideans vector", euclideans_vector

            euclidean_progress_vector = np.ones(len(euclideans_vector))*current_distance - euclideans_vector
            #print "euclidean progress vector", euclidean_progress_vector
            reward_vector = euclidean_progress_vector# - 0.1*jerk_vector
            #print "reward_vector", reward_vector
            
            expected_reward = np.multiply(probability_vector, reward_vector)
            max_action_index_in_vector = np.argmax(expected_reward)
            x_index_to_use = indices_list[max_action_index_in_vector][0]
            y_index_to_use = indices_list[max_action_index_in_vector][1]

        
            self.actionIndicesOverTime[idx,:] = [x_index_to_use, y_index_to_use] 

            x_accel = self.ActionSet.a_x[x_index_to_use]
            y_accel = self.ActionSet.a_y[y_index_to_use]

            controlInput = [x_accel, y_accel]

            self.controlInputData[idx] = controlInput

            nextCarState = self.Car.simulateOneStep(controlInput=controlInput, dt=self.dt)

        
            x = nextCarState[0]
            y = nextCarState[1]
            theta = nextCarState[2]
            self.setRobotFrameState(x,y,theta)
            

            #bookkeeping
            currentCarState = nextCarState
            
            self.counter+=1

            # break if we are in collision

            if self.counter >= simulationCutoff:
                break


        # fill in the last state by hand
        self.stateOverTime[self.counter,:] = currentCarState
        self.actionIndicesOverTime[self.counter,:] = [x_index_to_use, y_index_to_use]


        # this just makes sure we don't get stuck in an infinite loop.
        if startIdx == self.counter:
            self.counter += 1

        return runData

    def setNumpyRandomSeed(self, seed=1):
        np.random.seed(seed)

    def runBatchSimulation(self, endTime=None, dt=0.05):

        # for use in playback
        self.dt = self.options['dt']

        self.endTime = self.defaultControllerTime # used to be the sum of the other times as well

        self.t = np.arange(0.0, self.endTime, dt)
        maxNumTimesteps = np.size(self.t)
        self.stateOverTime = np.zeros((maxNumTimesteps+1, 4))
        self.actionIndicesOverTime = np.zeros((maxNumTimesteps+1, 2))
        self.controlInputData = np.zeros((maxNumTimesteps+1,2))
        self.numTimesteps = maxNumTimesteps

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
        self.actionIndicesOverTime = self.actionIndicesOverTime[0:self.counter+1, :]
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
            
            self.Car.setCarState(x,y,theta)
            self.setRobotFrameState(x,y,theta)

            print "In loop"

            if not self.checkInCollision():
                break
                
        return x,y,theta


    def setRandomCollisionFreeInitialState(self):
        tol = 5

        while True:
            
            x = np.random.uniform(self.world.Xmin+tol, self.world.Xmax-tol, 1)[0]
            y = np.random.uniform(self.world.Ymin+tol, self.world.Ymax-tol, 1)[0]
            #theta = np.random.uniform(0,2*np.pi,1)[0]
            theta = 0 #always forward

            self.Car.setCarState(x,y,self.initial_XVelocity,self.initial_YVelocity)
            self.setRobotFrameState(x,y,theta)

            if not self.checkInCollision():
                break

        self.firstRandomCollisionFreeInitialState_x = x
        self.firstRandomCollisionFreeInitialState_y = y

        return x,y,0,0

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

        l.addWidget(playButton)
        l.addWidget(slider)

        toggleSensorNoiseButton = QtGui.QPushButton('Toggle Sensor Noise')
        toggleSensorNoiseButton.connect('clicked()', self.onToggleSensorNoise)
        l.addWidget(toggleSensorNoiseButton)

        showObstaclesButton = QtGui.QPushButton('Show Obsatacles')
        showObstaclesButton.connect('clicked()', self.onShowObstacles)
        l.addWidget(showObstaclesButton)

        w = QtGui.QWidget()
        l = QtGui.QVBoxLayout(w)
        l.addWidget(self.view)
        self.view.orientationMarkerWidget().Off()
        l.addWidget(panel)
        w.showMaximized()

        # need to connect frames with DrawActionSet
        self.frame.connectFrameModified(self.updateDrawActionSet)
        self.updateDrawActionSet(self.frame)
        self.frame.connectFrameModified(self.updateDrawIntersection)
        self.updateDrawIntersection(self.frame)
        
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
        self.app.start()

    def run(self, launchApp=True):
        self.sphere_toggle = False
        self.sigma_sensor = 0.75


        self.running_sim = True
        self.onRandomGlobalGoalButton()
        self.counter = 1
        self.runBatchSimulation()
        self.running_sim = False

        if launchApp:
            self.setupPlayback()

    def onToggleSensorNoise(self):
        self.sphere_toggle = not self.sphere_toggle

    def onShowObstacles(self):
        print "time to show obstacles"
        A = World.buildCircleWorld(percentObsDensity=self.options['World']['percentObsDensity'],
                                            circleRadius=self.options['World']['circleRadius'],
                                            nonRandom=self.options['World']['nonRandomWorld'],
                                            scale=self.options['World']['scale'],
                                            randomSeed=self.options['World']['randomSeed'],
                                            obstaclesInnerFraction=self.options['World']['obstaclesInnerFraction'], alpha=1.0)


    def drawFirstIntersections(self, frame, firstRaycast):
        origin = np.array(frame.transform.GetPosition())
        d = DebugData()

        firstRaycastLocations = self.Sensor.invertRaycastsToLocations(self.frame, firstRaycast)

        for i in xrange(self.Sensor.numRays):
            endpoint = firstRaycastLocations[i,:]

            if firstRaycast[i] >= self.Sensor.rayLength - 0.1:
                d.addLine(origin, endpoint, color=[0,1,0])
            else:
                d.addLine(origin, endpoint, color=[1,0,0])

        vis.updatePolyData(d.getPolyData(), 'rays', colorByName='RGB255')

        self.LineSegmentWorld = World.buildLineSegmentWorld(firstRaycastLocations)
        self.LineSegmentLocator = World.buildCellLocator(self.LineSegmentWorld.visObj.polyData)
        self.locator = self.LineSegmentLocator
        self.Sensor.setLocator(self.LineSegmentLocator)


    def updateDrawIntersection(self, frame, locator="None"):
        if locator=="None":
            locator = self.locator

        if self.sphere_toggle:
            sphere_radius=self.sigma_sensor
        else:
            sphere_radius=0.0


        origin = np.array(frame.transform.GetPosition())
        #print "origin is now at", origin
        d = DebugData()
        d_sphere=DebugData()

        sliderIdx = self.slider.value

        controllerType = self.getControllerTypeFromCounter(sliderIdx)
        colorMaxRange = self.colorMap[controllerType]

        for i in xrange(self.Sensor.numRays):
            ray = self.Sensor.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            #print "rayTransformed is", rayTransformed
            intersection = self.Sensor.raycast(locator, origin, origin + rayTransformed*self.Sensor.rayLength)

            if intersection is not None:
                d.addLine(origin, intersection, color=[1,0,0])
                d_sphere.addSphere(intersection, radius=sphere_radius, color=[1,0,0])

            else:
                d.addLine(origin, origin+rayTransformed*self.Sensor.rayLength, color=colorMaxRange)
                d_sphere.addSphere(origin, radius=0.0, color=[0,1,0])

        vis.updatePolyData(d.getPolyData(), 'rays', colorByName='RGB255')
        vis.updatePolyData(d_sphere.getPolyData(), 'spheres', colorByName='RGB255', alpha=0.3)



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
    def checkInCollision(self):

        return False

    def updateDrawActionSet(self, frame):

        origin = np.array(frame.transform.GetPosition())

        if not self.running_sim:
            self.ActionSet.computeAllPositions(self.stateOverTime[self.currentIdx,0], self.stateOverTime[self.currentIdx,1], self.stateOverTime[self.currentIdx,2],self.stateOverTime[self.currentIdx,3])
            #self.ActionSet.drawActionSetFinal()
            self.ActionSet.drawActionSetFull()
            self.drawChosenFunnel()

    def drawChosenFunnel(self):
        velocity_initial_x = self.stateOverTime[self.currentIdx,2]
        velocity_initial_y = self.stateOverTime[self.currentIdx,3]

        variance_x = 2.5 + abs(velocity_initial_x*0.1)
        variance_y = 2.5 + abs(velocity_initial_y*0.1)
        variance_z = 1.5

        x_index = self.actionIndicesOverTime[self.currentIdx,0]
        y_index = self.actionIndicesOverTime[self.currentIdx,1]

        for i in xrange(0,10):
            time = 0.5/10.0*i
            x_center = self.ActionSet.p_x_trajectories[x_index, i]
            y_center = self.ActionSet.p_y_trajectories[y_index, i]
            z_center = 0.0
            World.buildEllipse(i, [x_center,y_center,z_center], variance_x*time, variance_y*time, variance_z*time, alpha=0.3)


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

    def onXVelocityChanged(self, value):
        self.initial_XVelocity = value
        print "initial x velocity changed to ", value

    def onYVelocityChanged(self, value):
        self.initial_YVelocity = -value
        print "initial y velocity changed to ", -value

    def onSliderChanged(self, value):
        if not self.sliderMovedByPlayTimer:
            self.playTimer.stop()
        numSteps = len(self.stateOverTime)
        idx = int(np.floor(numSteps*(1.0*value/self.sliderMax)))
        idx = min(idx, numSteps-1)
        self.currentIdx = idx
        x,y, xdot, ydot = self.stateOverTime[idx]
        self.setRobotFrameState(x,y,0)
        self.sliderMovedByPlayTimer = False

    def onRandomGlobalGoalButton(self):
        print "random global goal button pressed"
        self.globalGoal = World.buildGlobalGoal(scale=self.options['World']['scale'])

    def onDrawActionSetButton(self):
        print "drawing action set"
        #self.ActionSet.computeFinalPositions(self.XVelocity_drawing,self.YVelocity_drawing)
        self.ActionSet.computeAllPositions(self.Car.state[0], self.Car.state[1], self.initial_XVelocity, self.initial_YVelocity)
        #self.ActionSet.drawActionSetFinal()
        self.ActionSet.drawActionSetFull()

    def onRunSimButton(self):
        self.running_sim = True
        self.runBatchSimulation()
        self.running_sim = False
        #self.saveToFile("latest")

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


    @staticmethod
    def loadFromFile(filename):
        filename = 'data/' + filename + ".out"
        sim = Simulator(autoInitialize=False, verbose=False)

        my_shelf = shelve.open(filename)
        sim.options = my_shelf['options']
        sim.initialize()

        sim.simulationData = my_shelf['simulationData']
        sim.stateOverTime = np.array(my_shelf['stateOverTime'])
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


