import director.vtkAll as vtk
import director.visualization as vis
import math

import numpy as np
import director.objectmodel as om

from director.debugVis import DebugData


class ActionSetObj(object):

    def __init__(self):
        self.a_max = 9.8*2 # this is approximately for a 70 degree pitch angle

        self.num_accel_thrust_bins = 3
        self.accel_thrust = np.linspace(0.3*self.a_max, self.a_max, self.num_accel_thrust_bins)
        
        self.num_roll_bins = 5
        self.roll = np.linspace(-np.pi*70.0/90.0, np.pi*70.0/90.0, self.num_roll_bins)

        self.num_pitch_bins = 5
        self.pitch = np.linspace(-np.pi*70.0/90.0, np.pi*70.0/90.0, self.num_pitch_bins)

        self.action_vector = np.zeros((self.num_accel_thrust_bins*self.num_roll_bins*self.num_pitch_bins+1, 3))
        self.action_vector[0,:] = [0,0,0]
        index = 1
        for i in self.accel_thrust:
            for j in self.roll:
                for k in self.pitch:
                    self.action_vector[index,:] = [i,j,k]
                    index=index+1

        print "ACTION VECTOR", self.action_vector

        self.a_vector = self.action_vector *0.0

        for index, value in enumerate(self.action_vector):
            accel_thrust = value[0]
            roll = value[1]
            pitch = value[2]

            a_x = accel_thrust*math.sin(pitch)
            a_y = accel_thrust*math.cos(pitch)*math.sin(roll)
            a_z = accel_thrust*math.cos(pitch)*math.cos(roll)
            self.a_vector[index,:] = [a_x, a_y, a_z]

        print "A VECTOR", self.a_vector


        self.num_x_bins = 5
        self.a_x = np.linspace(-self.a_max, self.a_max, self.num_x_bins)

        self.num_y_bins = 5
        self.a_y = np.linspace(-self.a_max, self.a_max, self.num_y_bins)

        self.t_f = 0.500 # 500 ms simulate forward time

        self.numPointsToDraw = 10
        self.t_vector = np.linspace(0,self.t_f,self.numPointsToDraw)
        self.t_vector_squared = 1.0*self.t_vector
        for index, value in enumerate(self.t_vector_squared):
            self.t_vector_squared[index] = value**2


    def computeFinalPositions_old(self, v_x_initial, v_y_initial):
        self.p_x_final = 1.0/2.0 * self.a_x * self.t_f**2 + np.ones(self.num_x_bins) * v_x_initial *self.t_f
        self.p_y_final = 1.0/2.0 * self.a_y * self.t_f**2 + np.ones(self.num_y_bins) * v_y_initial *self.t_f


    def computeAllPositions_old(self, v_x_initial, v_y_initial):
        self.p_x_trajectories = 1.0/2.0 * np.outer(self.a_x, self.t_vector_squared) + np.outer(np.ones(self.num_x_bins) * v_x_initial, self.t_vector)
        self.p_y_trajectories = 1.0/2.0 * np.outer(self.a_y, self.t_vector_squared) + np.outer(np.ones(self.num_y_bins) * v_y_initial, self.t_vector)



    def computeAllPositions(self, v_x_initial, v_y_initial, v_z_initial):
        v_initial = [v_x_initial, v_y_initial, v_z_initial]
        self.pos_trajectories = np.zeros(( np.size(self.a_vector,0), np.size(self.a_vector,1), np.size(self.t_vector,0) ))
        for index, value in enumerate(self.pos_trajectories):
            self.pos_trajectories[index,:,:] =  1.0/2.0 * np.outer(self.a_vector[index,:], self.t_vector_squared) + np.outer( v_initial , self.t_vector )


    def drawActionSetFinal(self):
        #print "I am drawing the action set"

        d = DebugData()

        for x_index, x_value in enumerate(self.p_x_final):
            for y_index, y_value in enumerate(self.p_y_final):
        
                firstEndpt = (0.0,0.0,0.0)
                secondEndpt = (x_value,y_value,0.0)

                d.addLine(firstEndpt, secondEndpt, radius=0.02, color=[0.8,0,0.8])


        obj = vis.updatePolyData(d.getPolyData(), 'action_set', colorByName='RGB255')

    def drawActionSetFull(self):
        #print "I am drawing the action set"

        d = DebugData()

        for index, value in enumerate(self.pos_trajectories):
                for time_step_index in xrange(self.numPointsToDraw-1):
        
                    firstX = value[0,time_step_index]
                    firstY = value[1,time_step_index]
                    firstZ = value[2,time_step_index]

                    secondX = value[0,time_step_index+1]
                    secondY = value[1,time_step_index+1]
                    secondZ = value[2,time_step_index+1]

                    firstEndpt = (firstX,firstY,firstZ)
                    secondEndpt = (secondX,secondY,secondZ)

                    d.addLine(firstEndpt, secondEndpt, radius=0.02, color=[0.8,0,0.8])


        obj = vis.updatePolyData(d.getPolyData(), 'action_set', colorByName='RGB255')

    def drawActionSetFull_old(self):
        #print "I am drawing the action set"

        d = DebugData()

        for x_index, x_value in enumerate(self.a_x):
            for y_index, y_value in enumerate(self.a_y):

                for time_step_index in xrange(self.numPointsToDraw-1):
        
                    firstX = self.p_x_trajectories[x_index, time_step_index]
                    firstY = self.p_y_trajectories[y_index, time_step_index]

                    secondX = self.p_x_trajectories[x_index, time_step_index+1]
                    secondY = self.p_y_trajectories[y_index, time_step_index+1]

                    firstEndpt = (firstX,firstY,0.0)
                    secondEndpt = (secondX,secondY,0.0)

                    d.addLine(firstEndpt, secondEndpt, radius=0.02, color=[0.8,0,0.8])


        obj = vis.updatePolyData(d.getPolyData(), 'action_set', colorByName='RGB255')
        
    