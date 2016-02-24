from ddapp import consoleapp
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import vtkAll as vtk
from ddapp import applogic
from ddapp import vtkNumpy as vnp

import numpy as np


app = consoleapp.ConsoleApp()
view = app.createView(useGrid=False)

cellSize = 0.5
numberOfCells = 2

grid = vtk.vtkGridSource()
grid.SetScale(cellSize)
grid.SetGridSize(numberOfCells)
grid.SetSurfaceEnabled(True)
grid.Update()
grid = grid.GetOutput()


pts = vnp.getNumpyFromVtk(grid, 'Points')
print "pts ",  np.shape(pts)
print "pts is this ", pts
print "grid is this ", grid

randomHeatMap = np.random.randn(len(pts))
print "heatMap ", np.shape(randomHeatMap)
vnp.addNumpyToVtk(grid, randomHeatMap, 'heat_map')


gridObj = vis.showPolyData(grid, 'heat map', colorByName='heat_map', parent='scene')
gridObj.setProperty('Surface Mode', 'Surface with edges')

applogic.resetCamera()

view.show()
app.start()