import vtk
import chaste_project_Angiogenesis.population.vessel
import chaste_project_Angiogenesis.utility.bases as bases

class NetworkToPlanarBoundaries(bases.SimpleIOBase):
    
    """
    Take in a vessel network and return a VTK polydata formed by the intersection with a cutting plane, assuming cylindrical vessels 
    The cutting plane can be described by extents (x, y), origin (x,y,z) and normal (x,y,z).
    This class uses FreeCAD to do the intersection with the plane
    @param self.input a vessel network in Chaste format
    @param self.plane_extents the extents of the cutting plane, for general use should be a large number
    @param self.plane_origin the origin of the cutting plane, for general use should be a large negative number in x,y, and 0 in z
    @param self.plane_normal the normal of the cutting plane
    @return self.output VtkPolydata with element edges as lines    
    """
    
    def __init__(self):
        super(NetworkToPlanarBoundaries, self).__init__()
        self.BaseModule = __import__("FreeCAD") # FreeCAD import happens here to reduce loading overhead and dependencies
        self.PartModule = __import__("Part")
        self.plane_extents = [1.e6,  1.e6]
        self.plane_origin = [-5.e-5, -5.e-5, 0.0]
        self.plane_normal = [0.0, 0.0, 1.0]
        
    def set_plane_extents(self, extents):
        self.plane_extents = extents
        
    def set_plane_normal(self, normal):
        self.plane_normal = normal
        
    def set_plane_origin(self, origin):
        self.plane_origin = origin
        
    def update(self):
        
        vessels = self.input.GetVessels()
        for eachVessel in vessels:
            start = eachVessel.GetStartNode().GetLocation()
            end = eachVessel.GetEndNode().GetLocation()
            cylinder = self.PartModule.makeCylinder(eachVessel.GetStartNode().GetRadius(), eachVessel.GetLength(), 
                                         self.BaseModule.Vector(tuple(start)), self.BaseModule.Vector(tuple(end - start)))
            cylinder = self.PartModule.makeShell(cylinder.Faces)
            
        clipping_plane = self.PartModule.makePlane(self.plane_extents[0], self.plane_extents[1], 
                                             self.BaseModule.Vector(self.plane_origin[0], self.plane_origin[1], self.plane_origin[2]), 
                                             self.BaseModule.Vector(self.plane_normal[0], self.plane_normal[1], self.plane_normal[2]))
        section = cylinder.section(clipping_plane)
        
        polydata= vtk.vtkPolyData() 
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        for eachEdge in section.Edges:
            line = vtk.vtkLine()
            pointId1 = points.InsertNextPoint(eachEdge.Vertexes[0].Point)
            pointId2 = points.InsertNextPoint(eachEdge.Vertexes[1].Point)
            line.GetPointIds().InsertId(0, pointId1)
            line.GetPointIds().InsertId(1, pointId2)
            lines.InsertNextCell(line)
             
        polydata.SetPoints(points)
        polydata.SetLines(lines)   
        clean = vtk.vtkCleanPolyData()
        clean.SetInput(polydata)
        clean.Update()
        self.output = clean.GetOutput()
        return self.output
    
class NetworkTo3dCad(bases.SimpleIOBase):
    
    """
    Take in a vessel network and return a 3D Cad model
    @param self.input a vessel network in Chaste format
    @return self.output a 3D Cad geometry in FreeCAD format
    """
    
    def __init__(self):
        super(NetworkTo3dCad, self).__init__()
        self.BaseModule = __import__("FreeCAD") # FreeCAD import happens here to reduce loading overhead and dependencies
        self.PartModule = __import__("Part")
        
    def update(self):
        vessels = self.input.GetVessels()
        for eachVessel in vessels:
            start = eachVessel.GetStartNode().GetLocation()
            end = eachVessel.GetEndNode().GetLocation()
            cylinder = self.PartModule.makeCylinder(eachVessel.GetStartNode().GetRadius(), eachVessel.GetLength(), 
                                         self.BaseModule.Vector(tuple(start)), self.BaseModule.Vector(tuple(end - start)))
            cylinder = self.PartModule.makeShell(cylinder.Faces)
            
        self.output = cylinder
        return self.output
    
class NetworkToVtkLines(bases.SimpleIOBase):
    
    def __init__(self):
        super(NetworkToVtkLines, self).__init__()
        
    def update(self):
        self.output = vtk.vtkPolyData() 
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        info = vtk.vtkFloatArray()
        info.SetNumberOfComponents(1)
        info.SetName("Radius")
        
        for eachVessel in self.input.GetVessels():
            line = vtk.vtkLine()
            avRadius = 0
            for idx, eachSegment in enumerate(eachVessel.GetSegments()):
                pointId = points.InsertNextPoint(eachSegment.GetNodes().first.GetLocation())
                line.GetPointIds().InsertId(idx, pointId)
                avRadius = avRadius + eachSegment.GetRadius()
                if idx == len(eachVessel.GetSegments()) - 1:
                    pointId = points.InsertNextPoint(eachSegment.GetNodes().second.GetLocation())
                    line.GetPointIds().InsertId(idx + 1, pointId)
            avRadius = avRadius / len(eachVessel.GetSegments())
            lines.InsertNextCell(line)
            info.InsertNextTupleValue([avRadius])   
                
        self.output.SetPoints(points)
        self.output.SetLines(lines)
        self.output.GetCellData().SetScalars(info)
        return self.output
    
class VtkLinesToNetwork(bases.SimpleIOBase):
    
    def __init__(self, polydata):
        super(VtkLinesToNetwork, self).__init__()
        
    def update(self):
        vtk_numPoints = self.input.GetNumberOfPoints()    
        vtk_points = self.input.GetPoints()  
        
        nodes = [] 
        for i in range(vtk_numPoints):
            nodes.append(chaste_project_Angiogenesis.population.vessel.VascularNode(vtk_points.GetPoint(i)))
        
        numCells = self.polydata.GetNumberOfLines()  
        cellArray = self.polydata.GetLines()
        cellArray.InitTraversal()
        segList = vtk.vtkIdList()
        self.output = chaste_project_Angiogenesis.population.vessel.VascularNetwork()
        for i in range(numCells): 
            cellArray.GetNextCell(segList)
            point_indices = []
            for j in range(0, segList.GetNumberOfIds()):
                seg_id = segList.GetId(j)
                point_indices.append(int(seg_id))
            vessel = chaste_project_Angiogenesis.population.vessel.Vessel([nodes[point_indices[0]], nodes[point_indices[1]]])
            self.output.addVessel(vessel)   
        self.output.UpdateAll(False)
        return self.output