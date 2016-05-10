import director.vtkAll as vtk
import director.visualization as vis

import numpy as np
import director.objectmodel as om

from director.debugVis import DebugData


class ActionSetObj(object):

    def __init__(self):
        self.a_max = 5

        self.num_x_bins = 5
        self.a_x = np.linspace(-self.a_max, self.a_max, self.num_x_bins)

        self.num_y_bins = 5
        self.a_y = np.linspace(-self.a_max, self.a_max, self.num_y_bins)


    def drawActionSet(self):
        print "I am drawing the action set"

        d = DebugData()

        for x_index, x_value in enumerate(self.a_x):
            for y_index, y_value in enumerate(self.a_y):

                print x_value, y_value
        
                firstEndpt = (0.0,0.0,0.0)
                secondEndpt = (x_value,y_value,0.0)

                d.addLine(firstEndpt, secondEndpt, radius=0.02, color=[0.8,0,0.8])


        obj = vis.updatePolyData(d.getPolyData(), 'action_set', colorByName='RGB255')
        
    