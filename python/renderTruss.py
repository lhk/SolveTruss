import vtk
#import json
import generateTruss
import solveTruss


Truss = generateTruss.generateTurss(17)
Truss = solveTruss.solveTruss(Truss)

#with open('truss.json','rb') as file: Truss = json.load(file)

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
 
# interactor
interactor = vtk.vtkRenderWindowInteractor()
interactor.SetRenderWindow(window)
 
# sphere
sphereSource = vtk.vtkSphereSource()
sphereSource.SetCenter(0,0,0)
sphereSource.SetRadius(0.05)
sphereSource.SetThetaResolution(20)
sphereSource.SetPhiResolution(20)


for joint in Truss['Joints']:
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(sphereSource.GetOutput())
    else:
        mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetPosition(joint['Position'][0], joint['Position'][1], joint['Position'][2])
    renderer.AddActor(actor)






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
    vtkTubeFilter.SetRadius(0.015)
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
    #camera.SetViewAngle(90)

interactor.AddObserver('RenderEvent', Render, -1.0)
# enable user interface interactor

interactor.Initialize()
window.Render()
interactor.Start()

window.Finalize()
interactor.TerminateApp()
del window, interactor
exit()