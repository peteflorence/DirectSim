import numpy as np
import scipy.integrate as integrate
import director.objectmodel as om
import math


class ControllerObj(object):

    def __init__(self, u_max=4, epsilonRand=0.4):
        self.u_max = u_max

    def initializeVelocity(self,velocity):
        self.velocity = velocity
        
    def computeControlInput(self, state, t, frame, randomize=False):
        # test cases

        u, actionIdx = self.ActionSetController()

        return u, actionIdx

    def ActionSetController(self):

        u_x = 25.0

        u_y = 0

        u = [u_x, -u_y]
        return u, 0



    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame)

