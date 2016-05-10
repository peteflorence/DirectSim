import director.vtkAll as vtk
import director.visualization as vis

import numpy as np
import director.objectmodel as om

from director.debugVis import DebugData


class ActionSetObj(object):

    def __init__(self):
        self.a_max = 10

        self.num_x_bins = 5
        self.a_x = np.linspace(-self.a_max, self.a_max, self.num_x_bins)

        self.num_y_bins = 5
        self.a_y = np.linspace(-self.a_max, self.a_max, self.num_y_bins)

        self.t_f = 0.500 # 500 ms simulate forward time


    def computeFinalPositions(self, v_x_initial, v_y_initial):
        self.p_x_final = 1.0/2.0 * self.a_x * self.t_f**2 + np.ones(self.num_x_bins) * v_x_initial *self.t_f
        self.p_y_final = 1.0/2.0 * self.a_y * self.t_f**2 + np.ones(self.num_y_bins) * v_y_initial *self.t_f


    def drawActionSet(self):
        print "I am drawing the action set"

        d = DebugData()

        for x_index, x_value in enumerate(self.p_x_final):
            for y_index, y_value in enumerate(self.p_y_final):

                print x_value, y_value
        
                firstEndpt = (0.0,0.0,0.0)
                secondEndpt = (x_value,y_value,0.0)

                d.addLine(firstEndpt, secondEndpt, radius=0.02, color=[0.8,0,0.8])


        obj = vis.updatePolyData(d.getPolyData(), 'action_set', colorByName='RGB255')
        
    