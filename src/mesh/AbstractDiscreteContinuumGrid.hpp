/*

Copyright (c) 2005-2017, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ABSTRACTDISCRETECONTINUUMGRID_HPP_
#define ABSTRACTDISCRETECONTINUUMGRID_HPP_

#include <string>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include "Vertex.hpp"
#include "UnitCollection.hpp"
#include "Part.hpp"

/**
 * Forward declare VTK members
 */
class vtkDataSet;
class vtkCellLocator;
class vtkPoints;
class vtkDoubleArray;
class vtkUnsignedIntArray;
class vtkArrayData;
class vtkPolyData;
class vtkUnstructuredGrid;
class vtkPointData;
class vtkCellData;

/**
 * Grids are similar to vtkDataSet objects, but have extra (VTK based) convenience functions for
 * locating points and cells and managing parallel data transfer.
 * They are composed of geometric (point) and topological (cell) structures which can be assigned
 * attributes (boundary labels, region markers) and data (PDE solutions). The two current concrete
 * types are RegularGrids and DiscreteContinuumMeshs. UnstructuredGrids closely follow the vtkUnstructuredGrid.
 * RegularGrids are similar to vtkImageData, but cells are explicitly represented as vtkVoxels, which
 * can be truncated on edges and corners. Grids are parallel structures, most operations and storage
 * are LOCAL to the process unless otherwise specified.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class AbstractDiscreteContinuumGrid
{

protected:

    /**
     * Cell volumes
     */
    std::vector<double> mCellVolumes;

    /**
     * The reference length scale for the grid
     */
    QLength mReferenceLength;

    /**
     * A global vtk representation
     */
    vtkSmartPointer<vtkDataSet> mpGlobalVtkGrid;

    /**
     * A vtk representation local to the current process
     */
    vtkSmartPointer<vtkDataSet> mpVtkGrid;

    /**
     * The point locations as VTK points.
     */
    vtkSmartPointer<vtkPoints> mpPointLocations;

    /**
     * The cell locations as VTK points. For unstructured grids this is the centroid.
     */
    vtkSmartPointer<vtkPoints> mpCellLocations;

    /**
     * A local vtk cell locator
     */
    vtkSmartPointer<vtkCellLocator> mpVtkCellLocator;

    /**
     * Is the vtk representation up to date
     */
    bool mVtkRepresentationUpToDate;

    /**
     * Local to global index map
     */
    std::vector<unsigned> mLocalGlobalMap;

    /**
     * Map of attribute labels for mesh attribute numbers
     */
    std::map<unsigned, std::string> mAttributeKeys;

public:

    /**
     * Constructor
     */
    AbstractDiscreteContinuumGrid();

    /**
     * Destructor
     */
    virtual ~AbstractDiscreteContinuumGrid();

    /**
     * Add point attributes
     * @param rAttributes the point attributes
     * @param rName the data name
     */
    void AddPointAttributes(const std::vector<unsigned>& rAttributes, const std::string& rName = "Default Attribute");

    /**
     * Add cell attributes
     * @param rAttributes the cell attributes
     * @param rName the data name
     */
    void AddCellAttributes(const std::vector<unsigned>& rAttributes, const std::string& rName = "Default Attribute");

    /**
     * Set point or nodal data
     * @param rPointValues the point values
     * @param rName the data name
     */
    virtual void AddPointData(const std::vector<double>& rPointValues, const std::string& rName = "Default Data");

    /**
     * Set cell data
     * @param rCellValues the cell values
     * @param rName the data name
     */
    virtual void AddCellData(const std::vector<double>& rCellValues, const std::string& rName = "Default Data");

    /**
     * Return the attribute keys for the mesh
     * @return the attribute keys for the mesh
     */
    std::map<unsigned, std::string> GetAttributesKeys();

    /**
     * Return the bounding geometry on this processor
     * @return the bounding geometry on this processor
     */
    virtual vtkSmartPointer<vtkPolyData> GetBoundingGeometry();

    /**
     * Return the global grid index for the supplied local index
     * @param localIndex the local index
     * @return the global index
     */
    virtual unsigned GetGlobalIndex(unsigned localIndex);

    /**
     * Return the local cell index for the supplied global index. Return -1 if not on this proc.
     * This versions just spits the global index back, it should be over-ridden if child
     * classes need to work in parallel.
     * @param globalIndex the global index
     * @return the local grid index for the supplied global index. Return -1 if not on this proc
     */
    virtual int GetLocalIndex(unsigned globalIndex);

    /**
     * Return the LOCAL point data
     * @return the LOCAL point data
     */
    vtkSmartPointer<vtkPointData> GetPointData();

    /**
     * Return the LOCAL cell data
     * @return the LOCAL cell data
     */
    vtkSmartPointer<vtkCellData> GetCellData();

    vtkSmartPointer<vtkDataSet> CalculateDistanceMap(std::shared_ptr<Part<SPACE_DIM> > pSamplePart);

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    virtual Vertex<SPACE_DIM> GetGlobalPoint(unsigned index);

    /**
     * Return the location of the supplied LOCAL index
     * @return the location of the supplied LOCAL index
     */
    virtual Vertex<SPACE_DIM> GetPoint(unsigned index);

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    virtual Vertex<SPACE_DIM> GetGlobalCellLocation(unsigned index)=0;

    /**
     * Return the location of the supplied LOCAL index
     * @return the location of the supplied LOCAL index
     */
    virtual Vertex<SPACE_DIM> GetCellLocation(unsigned index);

    /**
     * Return the LOCAL point or element dimensionless volumes
     * @return the LOCAL point or element dimensionless volumes
     */
    virtual const std::vector<double>& rGetCellVolumes(bool update=false, bool jiggle=false)=0;

    /**
     * Return the GLOBAL grid in vtk form
     * @return the GLOBAL grid in vtk form
     */
    virtual vtkSmartPointer<vtkDataSet> GetGlobalVtkGrid();

    /**
     * Return the LOCAL grid in vtk form
     * @return the LOCAL grid in vtk form
     */
    virtual vtkSmartPointer<vtkDataSet> GetVtkGrid();

    /**
     * Do MPI GATHER for the named cell data. Assumes data arrays are ordered over ranks.
     * @param rName the point data name
     */
    virtual void GatherPointData(const std::string& rName);

    /**
     * Do MPI GATHER for all cell data. Assumes data arrays are ordered over ranks.
     */
    virtual void GatherAllPointData();

    /**
     * Do simple MPI ALL GATHER for the named point data. Assumes data arrays are ordered over ranks.
     * @param rName the point data name
     */
    virtual void AllGatherPointData(const std::string& rName);

    /**
     * Do MPI ALL GATHER for all point data. Assumes data arrays are ordered over ranks.
     */
    virtual void AllGatherAllPointData();

    /**
     * Do MPI GATHER for the named point data. Assumes data arrays are ordered over ranks.
     * @param rName the point data name
     */
    virtual void GatherCellData(const std::string& rName);

    /**
     * Do MPI GATHER for all cell data. Assumes data arrays are ordered over ranks.
     */
    virtual void GatherAllCellData();

    /**
     * Do simple MPI ALL GATHER for the named cell data. Assumes data arrays are ordered over ranks.
     * @param rName the point data name
     */
    virtual void AllGatherCellData(const std::string& rName);

    /**
     * Do MPI ALL GATHER for all cell data. Assumes data arrays are ordered over ranks.
     */
    virtual void AllGatherAllCellData();

    /**
     * Get the nearest LOCAL location index to the supplied point. An exception is
     * thrown if the input location does not lie inside the grid.
     * @param rLocation the point to get the nearest index to
     * @return the 1-d index of the nearest grid point
     */
    unsigned GetNearestCellIndex(const Vertex<SPACE_DIM>& rLocation);

    /**
     * Return the LOCAL number of grid points
     * @return the LOCAL number of grid points
     */
    unsigned GetNumberOfPoints();

    /**
     * Return the LOCAL number of grid cells
     * @return the LOCAL number of grid cells
     */
    unsigned GetNumberOfCells();

    /**
     * Return the LOCAL grid locations
     * @return the locations of grid points
     */
    virtual vtkSmartPointer<vtkPoints> GetPoints();

    /**
     * Return the LOCAL grid locations
     * @return the locations of grid cells
     */
    vtkSmartPointer<vtkPoints> GetCellLocations();

    /**
     * Return the reference length scale
     * @return the reference length scale
     */
    QLength GetReferenceLengthScale();

    /**
     * Return a vtk cell locator for quickly finding elements
     * @return a vtk cell locator for quickly finding elements
     */
    virtual vtkSmartPointer<vtkCellLocator> GetVtkCellLocator();

    /**
     * Set the attribute keys
     * @param attributeKeys attribute keys for the mesh
     */
    void SetAttributesKeys(std::map<unsigned, std::string> attributeKeys);

    /**
     * Set the internal vtk representation of the grid
     */
    virtual void SetUpVtkGrid()=0;

    /**
     * Set up the cell locator
     */
    virtual void SetUpVtkCellLocator();

};

#endif /* ABSTRACTDISCRETECONTINUUMGRID_HPP_*/
