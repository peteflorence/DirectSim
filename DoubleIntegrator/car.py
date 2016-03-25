import numpy as np
import scipy.integrate as integrate

class CarPlant(object):

    def __init__(self, controller=None, velocity=12):
        # if dt is None:
        #     raise ValueError("must specify timestep dt when constructing CarPlant")
        # initial state
        self.x = 0.0
        self.y = 0.0
        self.xdot = 0.0
        self.ydot = 0.0
      

        self.state = np.array([self.x, self.y, self.xdot, self.ydot])

        # constant velocity
        self.v = velocity

        self.Controller = controller


    def dynamics(self, state, t, controlInput=None):

        dqdt = np.zeros_like(state)

        if controlInput is not None:
            u = controlInput
        else:
            u = self.Controller.computeControlInput(state, t, self.frame)

        dqdt[0] = state[2]
        dqdt[1] = state[3]
        dqdt[2] = u[0]
        dqdt[3] = u[1]
    
        return dqdt

    def setFrame(self, frame):
        self.frame = frame

    def setStateFromFrame(self, frame):
         # get roll, pitch, yaw from the frame, set the state to that . . .
         pass

    def setCarState(self, x, y, xdot, ydot):
        self.state = np.array([x, y, xdot, ydot])

    def simulate(self, dt=0.05):
        t = np.arange(0.0, 10, dt)
        newState = integrate.odeint(self.dynamics, self.state, t)
        print "Finished simulation:", newState
        print "Shape is", np.shape(newState)
        return newState

    def simulateOneStep(self, startTime=0.0, dt=0.05, controlInput=None):
        t = np.linspace(startTime, startTime+dt, 2)
        newState = integrate.odeint(self.dynamics, self.state, t, args=(controlInput,))
        self.state = newState[-1,:]
        return self.state