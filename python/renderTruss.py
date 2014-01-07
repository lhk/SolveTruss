import vtk
import solveTruss
import math
import time
import json
import zlib

#Truss = generateTrussSimple.generateTruss()

with open('pool/truss.z', 'rb') as file: 
    Truss = json.loads(zlib.decompress(file.read()))

start = time.time()
Truss = solveTruss.solveTruss(Truss)
print "Solve Time:", time.time()-start

print "Truss['BeamOrientationTest'] =", Truss['BeamOrientationTest']
print "Truss['BeamStressLimitTest'] =", Truss['BeamStressLimitTest']
print "Truss['MaterialCost'] =", Truss['MaterialCost']
print "Truss['EfficiencyScore'] =", Truss['EfficiencyScore']


# window
window = vtk.vtkRenderWindow()
window.SetSize(1800,1000)
window.SetPosition(80,0)

# renderer
renderer = vtk.vtkRenderer()
renderer.RemoveAllLights()
window.AddRenderer(renderer)
light = renderer.MakeLight()
#light.SetAmbientColor(255,255,255)
light.SetLightTypeToCameraLight()
renderer.AddLight(light)
renderer.GetActiveCamera().SetViewAngle(80)
renderer.GetActiveCamera().SetClippingRange(0.001, 100000000)
 
# interactor
interactor = vtk.vtkRenderWindowInteractor()
interactor.SetRenderWindow(window)




for beam in Truss['Beams']:
    npts      = 2
    vtkPoints = vtk.vtkPoints()
    vtkPoints.SetNumberOfPoints(npts)
    
    vtkPoints.SetPoint(0, Truss['Joints'][beam['JointIndex_i']]['DisplacedPosition'])
    vtkPoints.SetPoint(1, Truss['Joints'][beam['JointIndex_j']]['DisplacedPosition'])
    
    vtkCellArray = vtk.vtkCellArray()
    vtkCellArray.InsertNextCell(npts)
    for i in range(npts):
      vtkCellArray.InsertCellPoint(i)
      
    value = 1-beam['TensionRelativeMagnitude']
    vtkFloatArray = vtk.vtkFloatArray()
    vtkFloatArray.SetNumberOfValues(npts)
    vtkFloatArray.SetValue(0, value)  
    vtkFloatArray.SetValue(1, value) 
    
    vtkPolyData = vtk.vtkPolyData()
    vtkPolyData.SetPoints(vtkPoints)
    vtkPolyData.SetLines(vtkCellArray)
    vtkPolyData.GetPointData().SetScalars(vtkFloatArray)
    
    vtkSplineFilter = vtk.vtkSplineFilter()
    vtkSplineFilter.SetInput(vtkPolyData)
    vtkSplineFilter.SetNumberOfSubdivisions(5*npts)
    vtkSplineFilter.Update()
    
    vtkTubeFilter = vtk.vtkTubeFilter()
    vtkTubeFilter.SetInputConnection(vtkSplineFilter.GetOutputPort())
    vtkTubeFilter.SetRadius(math.sqrt(beam['CrossSectionArea']/3.1415))
    vtkTubeFilter.SetNumberOfSides(20)
    vtkTubeFilter.CappingOn()
    
    vtkPolyDataMapper = vtk.vtkPolyDataMapper()
    vtkPolyDataMapper.SetInputConnection(vtkTubeFilter.GetOutputPort())
    
    vtkActor = vtk.vtkActor()
    vtkActor.SetMapper(vtkPolyDataMapper)
    
    renderer.AddActor(vtkActor)




def Render(obj, ev):

    renderer =  obj.GetRenderWindow().GetRenderers().GetFirstRenderer()
    camera = renderer.GetActiveCamera()
    camera.SetViewUp(0,1,0)

interactor.AddObserver('RenderEvent', Render, -1.0)

interactor.Initialize()
window.Render()
interactor.Start()

window.Finalize()
interactor.TerminateApp()
del window, interactor
exit()