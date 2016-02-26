import numpy as np
import scipy.optimize as opt
from linear_regression import LinearRegression
import cvxopt
import math
from ddapp.debugVis import DebugData
import ddapp.visualization as vis

class SensorApproximatorObj(object):

    def __init__(self, numRays, circleRadius):
        self.N = 4
        self.numRays = numRays
        self.circleRadius = circleRadius

    def initializeThetaVector(self,thetaVector):
        self.thetaVector = thetaVector

    def initializeApproxThetaVector(self, angleMin, angleMax):
        self.numApproxPoints = 200
        self.approxThetaVector = np.linspace(angleMin, angleMax, self.numApproxPoints)
        self.approxRays = np.zeros((3,self.numApproxPoints))
        self.approxRays[0,:] = np.cos(self.approxThetaVector)
        self.approxRays[1,:] = -np.sin(self.approxThetaVector)
        

    def polyFitConstrainedLP(self, distances):
        self.laserDepths = np.array(distances) - np.ones((np.shape(distances)))*self.circleRadius # decrease each sensor by the circle radius (i.e., inflate all obstacles)
        self.laserDepths[0] = 0.1
        self.laserDepths[-1] = 0.1
        self.setUpOptimization()
        self.constrainedLP()
        return self.polyCoefficientsLP


    def setUpOptimization(self):
        
        lr = LinearRegression(self.thetaVector,self.laserDepths,self.N)
        A_pete = lr.phi
        
        #W = weighting matrix
        weights = self.laserDepths * 1.0
        for index, value in enumerate(weights):
            weights[index] = (1/value)**4
        W_pete = np.diag(weights)
        
        #b = vector of sensor measurements
        b_pete = self.laserDepths

        A_pete = np.matrix(A_pete)
        W_pete = np.matrix(W_pete)
        b_pete = np.matrix(b_pete)

        # P = A^T W A
        self.P = cvxopt.matrix(A_pete.T * W_pete * A_pete)

        # q^T = -b^T A
        # q = -A^T b
        self.q = cvxopt.matrix(- A_pete.T * W_pete *b_pete.T)

        # G = A
        
        # no restrictions on coefficients
        #self.G = cvxopt.matrix(A_pete)

        # restrict c_1 to be in nice tangent domain
        # G_pete_add = np.zeros((2,self.N+1))
        # G_pete_add[0,1] = 1
        # G_pete_add[1,1] = -1
        # G_pete_ineq = np.vstack((A_pete, G_pete_add))
        # self.G = cvxopt.matrix(G_pete_ineq)

        # restrict c_0 to be positive
        G_pete_add = np.zeros((1,self.N+1))
        G_pete_add[0,0] = -1
        G_pete_ineq = np.vstack((A_pete, G_pete_add))
        self.G = cvxopt.matrix(G_pete_ineq)


        # h = b_pete

        # no restrictions on coefficents
        #self.h = cvxopt.matrix(b_pete.T)

        # restrict c_1 to be in nice tangent doman
        # h_add = np.zeros((1,2))
        # h_add[0,0] = math.pi/2
        # h_add[0,1] = math.pi/2
        # h_pete_ineq = np.hstack((b_pete, h_add))
        # self.h = cvxopt.matrix(h_pete_ineq.T)

        #restrict c_0 to be positive
        h_add = np.zeros((1,1))
        h_add[0,0] = 0.1
        h_pete_ineq = np.hstack((b_pete, h_add))
        self.h = cvxopt.matrix(h_pete_ineq.T)

        # c for LP
        c = np.zeros((self.N+1,1))
        for index, row in enumerate(c):
            total = 0
            for i in range(self.numRays):
                total = total + self.thetaVector[i]**index
            c[index] = -total

        self.c = cvxopt.matrix(c)
        

    def constrainedQP(self):
        cvxopt.solvers.options['show_progress'] = False
        solution = cvxopt.solvers.qp(self.P, self.q, self.G, self.h)
        self.polyCoefficientsQP = np.array(solution['x'])

    def constrainedLP(self):
        cvxopt.solvers.options['show_progress'] = False
        solution = cvxopt.solvers.lp(self.c, self.G, self.h)
        self.polyCoefficientsLP = solution['x']
        


# def plotHorner(w):
#     x = np.linspace(-math.pi/4,math.pi/4,1000)
#     y = x * 0.0
#     for index,val in enumerate(y):
#         y[index] = horner(x[index],w)
#     plt.plot(x,y)
#     plt.axis([-3.14, 3.14, 0, 20])
#     plt.show()
    
# plotHorner(alpha)


# def plotConstrained(w):
#     x = np.linspace(-math.pi/4,math.pi/4,1000)
#     y = x * 0.0
#     for index,val in enumerate(y):
#         y[index] = horner(x[index],w)
#     plt.plot(x,y, color='r')
#     plt.scatter(laseAngles2, laserDepths, color='b', marker='o',facecolors='none')
#     plt.xlabel('x')
#     plt.ylabel('y')
#     plt.show()
    
# plotConstrained(alpha) 


#plotHorner(alphaLP)
 
#plotConstrained(alphaLP)  


    
    