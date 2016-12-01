import numpy as np
import vtk
import bases

class LocationsToVtkPoints(bases.SimpleIOBase):
    
    """
    Conversion from a list of 3D point locations to vtk points
    @param self.input a list of point tuples in 3D
    @return self.output VtkPolydata with locations as points   
    """
    
    def __init__(self):
        super(LocationsToVtkPoints, self).__init__()  
    
    def update(self, points):
        self.output = vtk.vtkPolyData() 
        vtk_points = vtk.vtkPoints()
        for eachPoint in self.input:
            vtk_points.InsertNextPoint(eachPoint.get_location())    
        self.output.SetPoints(vtk_points)
        return self.output

def vtk_polygon_to_lines(surface):
    
    """
    Take a vtk polygon in the form of Polydata and return polylines of the boundaries
    @param surface a list of point tuples in 3D
    @return VtkPolydata with polylines on the boundaries
    """
    
    feature_edges = vtk.vtkFeatureEdges()
    feature_edges.SetInput(surface)  
    feature_edges.Update()   
        
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(feature_edges.GetOutputPort())
    clean.Update()
    
    triangle2 = vtk.vtkTriangleFilter()
    triangle2.SetInputConnection(clean.GetOutputPort())
    triangle2.Update()

    return triangle2.GetOutput()


def vtk_lines_to_polylines_region(boundary):
    """
    Take vtk lines in the form of polydata of a structure with distinct regions and merge lines on a common region
    :param vtkpolydata a structure with distinct regions describes as a colleciton of lines
    :returns vtkpolydata with polylines for each region
    """  
    numPoints = boundary.GetNumberOfPoints()    
    vtkpoints = boundary.GetPoints()  
    connectivity = []
    
    points = [] 
    for i in range(numPoints):
        points.append(vtkpoints.GetPoint(i))
        connectivity.append([])
        
    numCells = boundary.GetNumberOfLines()  
    cellArray = boundary.GetLines()
    cellArray.InitTraversal()
    segList = vtk.vtkIdList()
        
    edges = []
    for i in range(numCells): 
        cellArray.GetNextCell(segList)
        point_indices = []
        for j in range(0, segList.GetNumberOfIds()):
            seg_id = segList.GetId(j)
            point_indices.append(int(seg_id))
        edges.append((point_indices[0], point_indices[1]))
        connectivity[point_indices[0]].append(i)
        connectivity[point_indices[1]].append(i)
        
    regions = []
    point_visited = np.zeros(len(points))
    
    for idx, eachPoint in enumerate(points):
        if point_visited[idx] == 0:
            point_visited[idx] = 1
            region_points = [idx]
            current_point_id = idx
            previous_point_id = idx
            found_visited = False
            while not found_visited:
                edge_1 = edges[connectivity[current_point_id][0]]
                edge_2 = edges[connectivity[current_point_id][1]]
                if edge_1[0] == current_point_id:
                    opp1 = edge_1[1]
                else:
                    opp1 = edge_1[0]
                if opp1 != previous_point_id:
                    next_edge = edge_1
                else:
                    next_edge = edge_2
                          
                if next_edge[0]== current_point_id:
                    next_point_id = next_edge[1]  
                else:
                    next_point_id = next_edge[0]     
                          
                if point_visited[next_point_id] == 1:
                    region_points.append(next_point_id)
                    break
                  
                point_visited[next_point_id] = 1
                region_points.append(next_point_id)
                previous_point_id = current_point_id
                current_point_id = next_point_id
                      
            if len(region_points) > 1:
                regions.append(region_points)
                    
    new_points = vtk.vtkPoints()
    new_points.SetNumberOfPoints(len(points))
    for idx, eachPoint in enumerate(points):
        new_points.SetPoint(idx, points[idx][0], points[idx][1], 0.0)
        
    lines = vtk.vtkCellArray()
    for eachRegion in regions:
        lines.InsertNextCell(len(eachRegion))
        for eachPoint in eachRegion:
            lines.InsertCellPoint(eachPoint)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(new_points)
    polygon.SetLines(lines)
    
    return polygon

def vtk_lines_to_polylines_branch(boundary):
    """
    Take vtk lines in the form of polydata of a branching structure and merge lines between branch points
    :param vtkpolydata a branched structure described as a collection of lines
    :returns vtkpolydata with polylines between branch points
    """  
    numPoints = boundary.GetNumberOfPoints()    
    vtkpoints = boundary.GetPoints()  
    connectivity = []
    
    points = [] 
    for i in range(numPoints):
        points.append(vtkpoints.GetPoint(i))
        connectivity.append([])
        
    numCells = boundary.GetNumberOfLines()  
    cellArray = boundary.GetLines()
    cellArray.InitTraversal()
    segList = vtk.vtkIdList()
        
    edges = []
    for i in range(numCells): 
        cellArray.GetNextCell(segList)
        point_indices = []
        for j in range(0, segList.GetNumberOfIds()):
            seg_id = segList.GetId(j)
            point_indices.append(int(seg_id))
        edges.append((point_indices[0], point_indices[1]))
        connectivity[point_indices[0]].append(i)
        connectivity[point_indices[1]].append(i)
        
    branches = []
    point_visited = np.zeros(len(points))

    for idx, eachPoint in enumerate(points):
        num_segs = len(connectivity[idx])
        if num_segs ==3 or num_segs==1:
            point_visited[idx] = 1
            for eachEdgeId in connectivity[idx]:
                branch_points = [idx]
                current_point_id = idx
                previous_point_id = idx
                found_branch = False
                while not found_branch:
                      
                    # get the next node   
                    current_num_segs = len(connectivity[current_point_id])
                    if current_num_segs == 2:
                        edge_1 = edges[connectivity[current_point_id][0]]
                        edge_2 = edges[connectivity[current_point_id][1]]
                        if edge_1[0] == current_point_id:
                            opp1 = edge_1[1]
                        else:
                            opp1 = edge_1[0]
                        if opp1 != previous_point_id:
                            next_edge = edge_1
                        else:
                            next_edge = edge_2
                    else:
                        next_edge = edges[eachEdgeId]
                              
                    if next_edge[0]== current_point_id:
                        next_point_id = next_edge[1]  
                    else:
                        next_point_id = next_edge[0]     
                              
                    if point_visited[next_point_id] == 1 and len(connectivity[next_point_id]) == 2:
                        break
                      
                    point_visited[next_point_id] = 1
                    branch_points.append(next_point_id)
                    
                    if len(connectivity[next_point_id]) == 2:
                        previous_point_id = current_point_id
                        current_point_id = next_point_id
                    else:
                        found_branch = True
                        break
                          
                if len(branch_points) > 1:
                    branches.append(branch_points)
                    
    new_points = vtk.vtkPoints()
    new_points.SetNumberOfPoints(len(points))
    for idx, eachPoint in enumerate(points):
        new_points.SetPoint(idx, points[idx][0], points[idx][1], 0.0)
        
    lines = vtk.vtkCellArray()
    for eachBranch in branches:
        lines.InsertNextCell(len(eachBranch))
        for eachPoint in eachBranch:
            lines.InsertCellPoint(eachPoint)
            
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(new_points)
    polygon.SetLines(lines)
    
    return polygon