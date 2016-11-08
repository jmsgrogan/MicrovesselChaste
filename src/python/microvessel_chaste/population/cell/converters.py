"""
Single vessel network
"""

import vtk
import chaste_project_Angiogenesis.utility.standalone_bases as bases

class SimpleCellPopulationToVtkPoints(bases.SimpleIOBase):
    
    """
    Conversion from a cell population to collection of vtk points
    @param self.input a SimpleCellPopulation
    @return self.output VtkPolydata with cell centres as points   
    """
    
    def __init__(self):
        super(SimpleCellPopulationToVtkPoints, self).__init__()
    
    def update(self):
        self.output = vtk.vtkPolyData() 
        points = vtk.vtkPoints()
        for eachCell in self.input.GetCells():
            points.InsertNextPoint(eachCell.GetLocation())  
        self.output.SetPoints(points)
        return self.output
    

    
