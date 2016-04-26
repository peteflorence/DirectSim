import numpy as np
import matplotlib.pyplot as plt
import shelve



filename = "../data/YTape.out"
my_shelf = shelve.open(filename)
raycastData = np.array( my_shelf['raycastData'])
my_shelf.close()


print np.shape(raycastData)

numSteps   = np.shape(raycastData)[0]
numSensors = np.shape(raycastData)[1]

stepsVector = np.linspace(1,numSteps,numSteps)

plt.figure(1)
for i in range(numSensors):
    plt.plot(stepsVector,raycastData[:,i])


plt.figure(2)
plt.plot(stepsVector,raycastData[:,numSensors/2])
plt.show()
