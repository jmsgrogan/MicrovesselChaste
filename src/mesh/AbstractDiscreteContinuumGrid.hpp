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
#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkCellLocator.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkArrayData.h>
#include <vtkPolyData.h>
#include "PetscTools.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"

/**
 * A grid is a made up of point locations and containing (topological) cells. Grids can
 * be regular, where points are evenly spaced and cells correspond to pixels or voxels or
 * unstructured where cells correspond to triangles or tetrahedra. Behind the scenes grids
 * are represented as VTK structures, ImageData for regular grids and UnstructureGrids for
 * unstructured grids. VTK data arrays can be associated with either points or topological cells.
 * For the regular grid points and cell storage is equivalent, there is one point for every cell.
 * Parallelism is only partially supported at the moment. Unless otherwise specified quantities are
 * LOCAL to the process.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class AbstractDiscreteContinuumGrid
{

protected:

    /**
     * Point attributes
     */
    vtkSmartPointer<vtkArrayData> mPointAttributes;

    /**
     * Cell attributes
     */
    vtkSmartPointer<vtkArrayData> mCellAttributes;

    /**
     * Cell volumes
     */
    std::vector<double> mCellVolumes;

    /**
     * The reference length scale for the grid
     */
    units::quantity<unit::length> mReferenceLength;

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
     * The cell locations as VTK points. For unstructured grids this is the centroid
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
     * Storage for point data
     */
    vtkSmartPointer<vtkArrayData> mPointData;

    /**
     * Storage for cell data
     */
    vtkSmartPointer<vtkArrayData> mCellData;

    /**
     * Local to global index map
     */
    std::vector<unsigned> mLocalGlobalMap;

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
    void AddPointAttributes(const std::vector<unsigned>& rAttributes,
            const std::string& rName = "Default Attribute");

    /**
     * Add cell attributes
     * @param rAttributes the cell attributes
     * @param rName the data name
     */
    void AddCellAttributes(const std::vector<unsigned>& rAttributes,
            const std::string& rName = "Default Attribute");

    /**
     * Set point or nodal data
     * @param rPointValues the point or nodal values
     * @param rName the data name
     */
    virtual void AddPointData(const std::vector<double>& rPointValues,
            const std::string& rName = "Default Location Data");

    /**
     * Set cell data
     * @param rCellValues the point or nodal values
     * @param rName the data name
     */
    virtual void AddCellData(const std::vector<double>& rCellValues,
            const std::string& rName = "Default Location Data");

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
    virtual unsigned GetGlobalCellIndex(unsigned localIndex);

    /**
     * Return the local grid index for the supplied global index. Return -1 if not on this proc.
     * This versions just spits the global index back, it should be over-ridden if child
     * classes need to work in parallel.
     * @param globalIndex the global index
     * @return the local grid index for the supplied global index. Return -1 if not on this proc
     */
    virtual int GetLocalCellIndex(unsigned globalIndex);

    /**
     * Return the LOCAL point attributes
     * @return the LOCAL point attributes
     */
    vtkSmartPointer<vtkArrayData> GetPointAttributes();

    /**
     * Return the LOCAL cell attributes
     * @return the LOCAL cell attributes
     */
    vtkSmartPointer<vtkArrayData> GetCellAttributes();

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetGlobalPoint(unsigned index)=0;

    /**
     * Return the location of the supplied LOCAL index
     * @return the location of the supplied LOCAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetPoint(unsigned index);

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetGlobalCell(unsigned index)=0;

    /**
     * Return the location of the supplied LOCAL index
     * @return the location of the supplied LOCAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetCell(unsigned index);

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
    virtual void GatherPointData(const std::string& rName);

    /**
     * Do MPI GATHER for all point data. Assumes data arrays are ordered over ranks.
     */
    virtual void GatherAllPointData();

    /**
     * Get the nearest LOCAL location index to the supplied point. An exception is
     * thrown if the input location does not lie inside the grid.
     * @param rLocation the point to get the nearest index to
     * @return the 1-d index of the nearest grid point
     */
    unsigned GetNearestCellIndex(const DimensionalChastePoint<SPACE_DIM>& rLocation);

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
    vtkSmartPointer<vtkPoints> GetPoints();

    /**
     * Return the LOCAL grid locations
     * @return the locations of grid cells
     */
    vtkSmartPointer<vtkPoints> GetCells();

    /**
     * Return the reference length scale
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale();

    /**
     * Return a vtk cell locator for quickly finding elements
     * @return a vtk cell locator for quickly finding elements
     */
    virtual vtkSmartPointer<vtkCellLocator> GetVtkCellLocator();

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
