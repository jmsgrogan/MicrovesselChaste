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

#ifndef REGULARGRID_HPP_
#define REGULARGRID_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkDataSet.h>
#include <vtkSmartPointer.h>
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"
#include "DistributedVectorFactory.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"

/**
 * A 'rectangular' grid with equal spacing in all directions. It can be used for
 * Finite Difference solvers or lattice based angiogenesis models.
 */
template<unsigned DIM>
class RegularGrid : public AbstractDiscreteContinuumGrid<DIM, DIM>
{

public:

    using AbstractDiscreteContinuumGrid<DIM, DIM>::GetPoint;

protected:

    /**
     * The spacing between grid points
     */
    units::quantity<unit::length> mSpacing;

    /**
     * The number of grid points in each direction
     */
    c_vector<unsigned, 3> mDimensions;

    /**
     * The extents (grid indices) on this processor
     */
    c_vector<unsigned, 6> mExtents;

    /**
     * The origin of the grid in x,y,z. Corresponds to location of front, bottom, left corner.
     */
    DimensionalChastePoint<DIM> mOrigin;

    /**
     * A vector of GLOBAL neighbour indices
     */
    std::vector<std::vector<unsigned> > mNeighbourData;

    /**
     * Factory for producing vectors over the number of nodes in the grid.
     */
    boost::shared_ptr<DistributedVectorFactory> mpDistributedVectorFactory;

public:

    /**
     * Constructor
     */
    RegularGrid();

    /**
     * Factory constructor method
     * @return a shared pointer to a new grid
     */
    static boost::shared_ptr<RegularGrid<DIM> > Create();

    /**
     * Desctructor
     */
    ~RegularGrid();

    /**
     * Calculate von Neumann neighbour indices for each grid point
     */
    void CalculateNeighbourData();

    /**
     * Calculate Moore neighbour indices for each grid point
     */
    void CalculateMooreNeighbourData();

    /**
     * Generate a grid based on the bounding box of the supplied part
     * Important: The grid spacing is maintained, so the final geometry may differ from the part.
     * @param pPart the part from which to get the bounding box
     * @param gridSize the grid spacing
     */
    void GenerateFromPart(boost::shared_ptr<Part<DIM> > pPart, units::quantity<unit::length> gridSize);

    /**
     * Return the distributed vector factory
     * @return the distributed vector factory
     */
    boost::shared_ptr<DistributedVectorFactory> GetDistributedVectorFactory();

    /**
     * Get the GLOBAL grid index for given x,y,z indices
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the grid 1-d index
     */
    unsigned GetGlobalGridIndex(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Get the LOCAL grid index for given x,y,z indices local on this process
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the grid 1-d index
     */
    unsigned GetGridIndex(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Return the local grid index for the supplied global index. Return -1 if not on this proc.
     * @param globalIndex the global index
     * @return the local grid index for the supplied global index. Return -1 if not on this proc
     */
    int GetLocalIndex(unsigned globalIndex);

    /**
     * Return the location of the supplied GLOBAL index
     * @return the location of the supplied GLOBAL index
     */
    DimensionalChastePoint<DIM> GetGlobalCellLocation(unsigned index);

    /**
     * Calculate GLOBAL von Neumann neighbour indices for each LOCAL grid point
     * @return a vector of neighbour indices for each grid point
     */
    const std::vector<std::vector<unsigned> >& rGetNeighbourData();

    /**
     * Calculate GLOBAL Moore neighbour indices for each LOCAL grid point
     * @return a vector of neighbour indices for each grid point
     */
    const std::vector<std::vector<unsigned> >& rGetMooreNeighbourData();

    /**
     * Return the GLOBAL grid dimensions in x, y, z. Always dimension 3.
     * @return the GLOBAL grid dimensions
     */
    c_vector<unsigned, 3> GetDimensions();

    /**
     * Return the LOCAL grid extents (i.e. the indices on this processor)
     * @return the LOCAL grid extents
     */
    c_vector<unsigned, 6> GetExtents();

    /**
     * Get the location of a point on the grid for given x, y ,z indices
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the location of the point
     */
    DimensionalChastePoint<DIM> GetPoint(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Return the origin in x, y, z
     * @return the grid origin
     */
    DimensionalChastePoint<DIM> GetOrigin();

    /**
     * Return the grid spacing
     *
     * @return the grid spacing
     */
    units::quantity<unit::length> GetSpacing();

    /**
     * The bounding box for the grid point. In 2D the z bounds are +1 and -1 to
     * all the use of VTK filters. The scale is the grid's reference length scale.
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @param jiggle increase the box size by a small amount to catch features half way between grid points
     * @return the bounding box for this point
     */
    c_vector<double,6> GetPointBoundingBox(unsigned xIndex, unsigned yIndex, unsigned zIndex, bool jiggle=false);

    /**
     * The bounding box for the LOCAL grid point. In 2D the z bounds are +1 and -1 to
     * all the use of VTK filters. The scale is the grid's reference length scale.
     * @param gridIndex the grid index
     * @param jiggle increase the box size by a small amount to catch features half way between grid points
     * @return the bounding box for this point
     */
    c_vector<double,6> GetPointBoundingBox(unsigned gridIndex, bool jiggle=false);

    /**
     * Return the DIMENSIONLESS volume for each point in the grid, accounts for smaller
     * corner and edge volumes.
     * @param update recalculate from scatch
     * @param jiggle increase the box size by a small amount to catch features half way between grid points
     * @return the DIMENSIONLESS volume for each point in the grid
     */
    const std::vector<double>& rGetCellVolumes(bool update=false, bool jiggle=false);

    /**
     * Is the LOCAL index on the outer boundary of the domain
     * @param gridIndex the 1d grid index gridIndex
     * @return is the point on the outer boundary of the domain
     */
    bool IsOnBoundary(unsigned gridIndex);

    /**
     * Is the point on the outer boundary of the domain
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return is the point on the outer boundary of the domain
     */
    bool IsOnBoundary(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Set the grid dimensions in x, y, z
     * @param dimensions the grid dimensions
     * @param updateVtk update the vtk representation at this point
     */
    void SetDimensions(c_vector<unsigned, 3> dimensions);

    /**
     * Set the origin in x, y, z
     * @param origin the grid origin
     * @param updateVtk update the vtk representation at this point
     */
    void SetOrigin(DimensionalChastePoint<DIM> origin);

    /**
     * Set the grid spacing
     * @param spacing the grid spacing
     * @param updateVtk update the vtk representation at this point
     */
    void SetSpacing(units::quantity<unit::length> spacing);

    /**
     * Set the internal vtk representation of the grid
     */
    void SetUpVtkGrid();

    /**
     * Over-ridden method to set up the cell locator
     */
    void SetUpVtkCellLocator();

    /**
     * Update the local extents
     */
    void UpdateExtents();

    /**
     * Write the grid and any field to file as a VTI file
     * @param pFileHandler a file handler for the write location
     */
    void Write(boost::shared_ptr<OutputFileHandler> pFileHandler);
};

#endif /* REGULARGRID_HPP_*/
