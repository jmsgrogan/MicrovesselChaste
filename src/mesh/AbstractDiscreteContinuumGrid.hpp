/*

Copyright (c) 2005-2016, University of Oxford.
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
#include <vtkPolyData.h>
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"

/**
 * Common interface for using regular (structured) grids and Chaste (unstructured) meshes with Discrete Continuum PDEs.
 * It is based on VTK structures. Unless otherwise specified quantities are LOCAL to the process.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class AbstractDiscreteContinuumGrid
{

protected:

    /**
     * Point or element attributes
     */
    std::vector<vtkSmartPointer<vtkUnsignedIntArray> > mAttributes;

    /**
     * Point or element volumes
     */
    std::vector<double> mVolumes;

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
     * The grid locations as VTK points
     */
    vtkSmartPointer<vtkPoints> mpGridLocations;

    /**
     * A local vtk cell locator
     */
    vtkSmartPointer<vtkCellLocator> mpVtkCellLocator;

    /**
     * Is the vtk representation up to date
     */
    bool mVtkRepresentationUpToDate;

    /**
     * Simple storage for point or node data
     */
    std::vector<vtkSmartPointer<vtkDoubleArray> > mPointData;

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
     * Set point or element attributes
     * @param rAttributes the point or element attributes
     * @param clearExisting clear existing point data
     * @param rName the data name
     */
    void AddAttributes(const std::vector<unsigned>& rAttributes, bool clearExisting = false,
            const std::string& rName = "Default Attribute");

    /**
     * Set point or nodal data
     * @param rPointValues the point or nodal values
     * @param rName the data name
     */
    virtual void AddPointData(const std::vector<double>& rPointValues,
            const std::string& rName = "Default Location Data");

    /**
     * Return the bounding geometry on this processor
     * @return the bounding geometry on this processor
     */
    virtual vtkSmartPointer<vtkPolyData> GetBoundingGeometry();

    /**
     * Return the LOCAL point or element-wise attributes
     * @return the LOCAL point or element-wise attributes
     */
    const std::vector<vtkSmartPointer<vtkUnsignedIntArray> >& rGetLocationAttributes();

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetLocationOfGlobalIndex(unsigned index)=0;

    /**
     * Return the location of the supplied LOCAL index
     * @return the location of the supplied LOCAL index
     */
    virtual DimensionalChastePoint<SPACE_DIM> GetLocation(unsigned index);

    /**
     * Return the LOCAL point or element dimensionless volumes
     * @return the LOCAL point or element dimensionless volumes
     */
    virtual const std::vector<double>& rGetLocationVolumes(bool update=false, bool jiggle=false)=0;

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
     * Get the nearest LOCAL location index to the supplied point. An exception is
     * thrown if the input location does not lie inside the grid.
     * @param rLocation the point to get the nearest index to
     * @return the 1-d index of the nearest grid point
     */
    unsigned GetNearestLocationIndex(const DimensionalChastePoint<SPACE_DIM>& rLocation);

    /**
     * Return the LOCAL number of grid points (structured) or cells (unstructured)
     * @return the LOCAL number of grid points (structured) or cells (unstructured)
     */
    unsigned GetNumberOfLocations();

    /**
     * Return the LOCAL grid locations
     * @return the number of grid points (structured) or cells (unstructured)
     */
    vtkSmartPointer<vtkPoints> GetLocations();

    /**
     * Return the reference length scale
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale();

    /**
     * Return a vtk cell locator for quickly finding points or elements
     * @return a vtk cell locator for quickly finding points or elements
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
