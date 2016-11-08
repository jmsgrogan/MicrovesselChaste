import vtk
from IPython.display import Image

import angiogenesis.geometry.glyphs
import angiogenesis.mesh.glyphs
import angiogenesis.population.cell.glyphs
import angiogenesis.population.vessel.glyphs
import angiogenesis.interfaces.vtk_tools.vtk_tools
import angiogenesis.utility.readwrite

class Scene():
    def __init__(self):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(1.0, 1.0, 1.0)
        self.filename = None
        
    def add_axes(self, polydata):
        axes = vtk.vtkCubeAxesActor2D()
        normals = vtk.vtkPolyDataNormals()
        normals.SetInput(polydata)
        axes.SetInput(normals.GetOutput())
        axes.SetCamera(self.renderer.GetActiveCamera())
        axes.SetLabelFormat("%6.4g")
        axes.SetFlyModeToOuterEdges()
        axes.SetFontFactor(2.0)
        tprop = vtk.vtkTextProperty()
        tprop.SetColor(1, 1, 1)
        axes.SetAxisTitleTextProperty(tprop)
        axes.SetAxisLabelTextProperty(tprop)
        self.renderer.AddViewProp(axes)
        
    def add_image(self, image):
        threshold = 200.0
        volume_mapper = vtk.vtkVolumeRayCastMapper()
        volume_mapper.SetInput(image.GetOutput() )
        composite_function = vtk.vtkVolumeRayCastCompositeFunction()
        volume_mapper.SetVolumeRayCastFunction( composite_function )
        
        color_transfer_func = vtk.vtkColorTransferFunction()
        color_transfer_func.AddRGBPoint( 0, 0.0, 1.0, 0.0 )
        color_transfer_func.AddRGBPoint( threshold-1, 0.0, 1.0, 0.0 )
        color_transfer_func.AddRGBPoint( threshold, 1.0, 0.0, 0.0 )
        color_transfer_func.AddRGBPoint( 255.0, 1.0, 0.0, 0.0 )
        
        opacity_transfer_func = vtk.vtkPiecewiseFunction()
        opacity_transfer_func.AddPoint( 0, 0.0 )
        opacity_transfer_func.AddPoint( threshold-1.0, 0.0 )
        opacity_transfer_func.AddPoint( threshold, 1.0 )
        opacity_transfer_func.AddPoint( 255.0, 1.0 )
        
        volume_properties = vtk.vtkVolumeProperty()
        volume_properties.SetColor( color_transfer_func )
        volume_properties.SetScalarOpacity( opacity_transfer_func )
        
        volume = vtk.vtkVolume()
        volume.SetMapper( volume_mapper )
        volume.SetProperty( volume_properties )
        self.renderer.AddVolume( volume )
        
    def add_part(self, part):
        glyph = angiogenesis.geometry.glyphs.PartGlyph3d(part)
        self.renderer.AddActor(glyph.actor) 
        
    def add_points(self, points):
        converter = angiogenesis.interfaces.vtk_tools.vtk_tools.LocationsToVtkPoints()
        converter.input = points
        polydata = converter.update()
        
        # Each point is a sphere
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(1.0)
        sphere.SetPhiResolution(16)
        sphere.SetThetaResolution(16)
         
        # make the glyphs
        glyph = vtk.vtkGlyph3D()
        glyph.SetInput(polydata)
        glyph.SetSource(sphere.GetOutput())
        glyph.ClampingOff()
        glyph.SetScaleModeToScaleByScalar()
        glyph.SetScaleFactor(1.0) 
         
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInput(glyph.GetOutput())
        mapper.ScalarVisibilityOn()
        
        actor = vtk.vtkActor()
        actor.GetProperty().SetColor(0,0,0) # (R,G,B)
        actor.SetMapper(mapper)
        
        self.renderer.AddActor(actor)
        
    def add_cells(self, cell_population):
        glyphs = angiogenesis.population.cell.glyphs.CellGlyph3d()
        self.renderer.AddActor(glyphs.actor)
        
    def add_network(self, network):
        glyph = angiogenesis.population.vessel.glyphs.VesselGlyph3d(network)
        self.renderer.AddActor(glyph.actors[0])
        self.renderer.AddActor(glyph.actors[1])
                    
    def add_mesh(self, mesh):
        
        glyph = angiogenesis.mesh.glyphs.MeshGlyph3d(mesh)
        
        self.renderer.AddActor(glyph.actors[0]) 
        self.renderer.AddActor(glyph.actors[1])  
    
    def add(self, component):
        if "list" in component.__class__.__name__:
            component_type = type(component[0]).__name__
        else:
            component_type = type(component).__name__
        
        # Render the component, depending on its type
            if "Part" in component_type:
                self.add_part(component, False)
            
            if "Vertex" in component_type:
                self.add_points(component)
                
            if "SimpleCellPopulation" in component_type:
                self.sadd_cells(component)
            
            if "Mesh" in component_type:
                self.add_mesh(component)
    
    def get_renderer(self):
        return self.renderer
    
    def show(self, interactive = False, width=400, height=300):
        
        if interactive:
            self.renderer.ResetCamera()
            renWin = vtk.vtkRenderWindow()
            renWin.AddRenderer(self.renderer)
            renWin.SetSize(width, height)
            iren = vtk.vtkRenderWindowInteractor()
            iren.SetRenderWindow(renWin)
            iren.Initialize()
            iren.Start()
        else:
            renderWindow = vtk.vtkRenderWindow()
            renderWindow.SetOffScreenRendering(1)
            renderWindow.AddRenderer(self.renderer)
            renderWindow.SetSize(width, height)
            renderWindow.Render()
             
            windowToImageFilter = vtk.vtkWindowToImageFilter()
            windowToImageFilter.SetInput(renderWindow)
            windowToImageFilter.Update()
             
            writer = vtk.vtkPNGWriter()
            writer.SetWriteToMemory(1)
            writer.SetInputConnection(windowToImageFilter.GetOutputPort())
            if self.filename is not None:
                writer.SetWriteToMemory(0)
                angiogenesis.utility.readwrite.dir_maker(self.filename)
                writer.SetFileName(self.filename)
                writer.Write()
            else:
                writer.Write()
                data = str(buffer(writer.GetResult()))
                return Image(data)            