import numpy as np
import matplotlib.pyplot as plt
import shelve



filename = "../data/latest.out"

my_shelf = shelve.open(filename)
options = my_shelf['options']

simulationData = my_shelf['simulationData']
stateOverTime = np.array(my_shelf['stateOverTime'])
raycastData = np.array( my_shelf['raycastData'])
controlInputData = np.array(my_shelf['controlInputData'])
numTimesteps = my_shelf['numTimesteps']
idxDict = my_shelf['idxDict']
counter = my_shelf['counter']

my_shelf.close()


print np.shape(raycastData)

numSteps   = np.shape(raycastData)[0]
numSensors = np.shape(raycastData)[1]

stepsVector = np.linspace(1,numSteps,numSteps)


for i in range(numSensors):
    plt.plot(stepsVector,raycastData[:,i])

plt.show()
