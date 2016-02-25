from ddapp import consoleapp
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import vtkAll as vtk
from ddapp import applogic
from ddapp import vtkNumpy as vnp
from ddapp.shallowCopy import shallowCopy

import numpy as np


app = consoleapp.ConsoleApp()
view = app.createView(useGrid=False)

cellSize = 0.5
numberOfCells = 10

grid = vtk.vtkGridSource()
grid.SetScale(cellSize)
grid.SetGridSize(numberOfCells)
grid.SetSurfaceEnabled(True)
grid.Update()
grid = grid.GetOutput()


pts = vnp.getNumpyFromVtk(grid, 'Points')
print "pts ",  np.shape(pts)
print "pts is this ", pts
#print "grid is this ", grid


def evalXYpoly(x,y):
  return x**2 + 3*y**2 + x - y


# # # This distorts the map in z
# for row in pts:
#   print "row before, ", row
#   row[2] = evalXYpoly(row[0],row[1])
#   print "row after,  ", row

polyHeatMap = np.zeros((len(pts)))
print np.shape(polyHeatMap)
for i in range(len(polyHeatMap)):
  polyHeatMap[i] = evalXYpoly(pts[i][0],pts[i][1])

vnp.addNumpyToVtk(grid, polyHeatMap, 'heat_map')


# ## This adds random heat map
# randomHeatMap = np.random.randn(len(pts))
# print "heatMap ", np.shape(randomHeatMap)
# vnp.addNumpyToVtk(grid, randomHeatMap, 'heat_map')


def clipByPlane(polyData, planeOrigin, planeNormal):
    f = vtk.vtkClipPolyData()
    f.SetInput(polyData)
    p = vtk.vtkPlane()
    p.SetOrigin(planeOrigin)
    p.SetNormal(planeNormal)
    f.SetClipFunction(p)
    f.Update()
    return shallowCopy(f.GetOutput())

# grid = clipByPlane(grid, (0,0,10), (0,0,1))
# grid = clipByPlane(grid, (0,0,30), (0,0,-1))


gridObj = vis.showPolyData(grid, 'heat map', colorByName='heat_map', parent='scene')
gridObj.setProperty('Surface Mode', 'Surface with edges')
prop = gridObj.actor.GetProperty()
prop.LightingOff()
#prop.LightingOn()

applogic.resetCamera()

view.show()
app.start()
