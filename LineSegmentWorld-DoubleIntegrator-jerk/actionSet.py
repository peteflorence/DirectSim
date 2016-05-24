import director.vtkAll as vtk
import director.visualization as vis
import math

import numpy as np
import director.objectmodel as om

from director.debugVis import DebugData


class ActionSetObj(object):

    def __init__(self):
        self.a_max = 9.8*2.4 # this is approximately for a 70 degree pitch angle

        self.a_max_horizontal = math.sqrt(self.a_max**2 - 9.8**2)

        self.a_vector = np.zeros((57, 3))

        # first one is actually zeros
        self.a_vector[0,:] = [0,0,0]

        # near horizontal max
        for i in xrange(1,9):
            theta = (i-1)*2*np.pi/8
            self.a_vector[i,:] = [np.cos(theta)*self.a_max_horizontal, np.sin(theta)*self.a_max_horizontal, 0] 

        #0.6*near horizontal max
        for i in xrange(9, 17):
            max_up = math.sqrt(self.a_max**2 - 0.6*self.a_max**2)-9.8

            theta = (i-9)*2*np.pi/8
            self.a_vector[i,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, 0] 
            self.a_vector[i+8,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, max_up] 
            self.a_vector[i+16,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, -max_up] 

        #0.3*near horizontal max
        for i in xrange(33, 41):
            max_up = math.sqrt(self.a_max**2 - 0.3*self.a_max**2)-9.8

            theta = (i-33)*2*np.pi/8
            self.a_vector[i,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, 0] 
            self.a_vector[i+8,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, max_up] 
            self.a_vector[i+16,:] = [np.cos(theta)*self.a_max_horizontal*0.6, np.sin(theta)*self.a_max_horizontal*0.6, -max_up] 


        self.t_f_jerk = 0.0
        self.t_f = 0.500 # 500 ms simulate forward time

        self.numPointsToDraw = 10 # each, for jerk portion and accel portion
        self.computeTimeVectors()
        
        
    def setTFinalJerk(self, t_f_jerk):
        self.t_f_jerk = t_f_jerk
        self.computeTimeVectors()

    def computeTimeVectors(self):
        self.t_vector_jerk = np.linspace(0,self.t_f_jerk,self.numPointsToDraw)
        self.t_vector_jerk_squared = 1.0*self.t_vector_jerk
        for index, value in enumerate(self.t_vector_jerk_squared):
            self.t_vector_jerk_squared[index] = value**2
        self.t_vector_jerk_cubed = 1.0*self.t_vector_jerk
        for index, value in enumerate(self.t_vector_jerk_cubed):
            self.t_vector_jerk_cubed[index] = value**3

        self.t_vector_accel = np.linspace(0,self.t_f-self.t_f_jerk,self.numPointsToDraw)
        self.t_vector_accel_squared = 1.0*self.t_vector_accel
        for index, value in enumerate(self.t_vector_accel_squared):
            self.t_vector_accel_squared[index] = value**2

        self.overall_t_vector = np.hstack((self.t_vector_jerk, self.t_vector_accel+np.ones(10)*self.t_f_jerk))


    def computeFinalPositions_old(self, v_x_initial, v_y_initial):
        self.p_x_final = 1.0/2.0 * self.a_x * self.t_f**2 + np.ones(self.num_x_bins) * v_x_initial *self.t_f
        self.p_y_final = 1.0/2.0 * self.a_y * self.t_f**2 + np.ones(self.num_y_bins) * v_y_initial *self.t_f


    def computeAllPositions_old(self, v_x_initial, v_y_initial):
        self.p_x_trajectories = 1.0/2.0 * np.outer(self.a_x, self.t_vector_squared) + np.outer(np.ones(self.num_x_bins) * v_x_initial, self.t_vector)
        self.p_y_trajectories = 1.0/2.0 * np.outer(self.a_y, self.t_vector_squared) + np.outer(np.ones(self.num_y_bins) * v_y_initial, self.t_vector)



    def computeAllPositions(self, v_x_initial, v_y_initial, v_z_initial, a_x_initial=0.0, a_y_initial=0.0, a_z_initial=0.0):
        v_initial = np.array([v_x_initial, v_y_initial, v_z_initial])
        a_initial = np.array([a_x_initial, a_y_initial, a_z_initial])
        self.pos_trajectories = np.zeros(( np.size(self.a_vector,0), np.size(self.a_vector,1), np.size(self.t_vector_jerk,0) + np.size(self.t_vector_accel,0) ))

        if self.t_f_jerk ==0:
            for index, value in enumerate(self.pos_trajectories):
                self.pos_trajectories[index,:,self.numPointsToDraw:] =  1.0/2.0 * np.outer(self.a_vector[index,:], self.t_vector_accel_squared) + np.outer( v_initial, self.t_vector_accel )

        else:
            for index, value in enumerate(self.pos_trajectories):
                
                # jerk portion
                jerk = (self.a_vector[index,:] - a_initial) / self.t_f_jerk
                self.pos_trajectories[index,:,0:self.numPointsToDraw] =  1.0/6.0 * np.outer(jerk, self.t_vector_jerk_cubed)  + 1.0/2.0 * np.outer(a_initial, self.t_vector_jerk_squared) + np.outer( v_initial , self.t_vector_jerk )
                velocity_end_of_jerk = 1/2*jerk*self.t_f_jerk**2 + a_initial*self.t_f_jerk + v_initial
                position_end_of_jerk = self.pos_trajectories[index,:,self.numPointsToDraw-1]

                # constant accel portion
                self.pos_trajectories[index,:,self.numPointsToDraw:] =  1.0/2.0 * np.outer(self.a_vector[index,:], self.t_vector_accel_squared) + np.outer( velocity_end_of_jerk, self.t_vector_accel ) + np.outer( position_end_of_jerk, np.ones(self.numPointsToDraw))


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
                
                for time_step_index in xrange(2*self.numPointsToDraw-1):
        
                    firstX = value[0,time_step_index]
                    firstY = value[1,time_step_index]
                    firstZ = value[2,time_step_index]

                    secondX = value[0,time_step_index+1]
                    secondY = value[1,time_step_index+1]
                    secondZ = value[2,time_step_index+1]

                    firstEndpt = (firstX,firstY,firstZ)
                    secondEndpt = (secondX,secondY,secondZ)

                    if time_step_index >= 10:
                        color=[0.8,0,0.8]
                    else:
                        color=[0.1,0.1,1.0]

                    d.addLine(firstEndpt, secondEndpt, radius=0.02, color=color)


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
        
    