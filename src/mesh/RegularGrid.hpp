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

#ifndef REGULARGRID_HPP_
#define REGULARGRID_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"
#include "DistributedVectorFactory.hpp"

/**
 * A class for describing regular grids, VTK and PETSc structures are used underneath
 */
template<unsigned DIM>
class RegularGrid
{
    /**
     * The spacing between grid points
     */
    units::quantity<unit::length> mSpacing;

    /**
     * The number of grid points in each direction
     */
    c_vector<unsigned, 3> mDimensions;

    /**
     * The extents of on this processor
     */
    c_vector<unsigned, 6> mExtents;

    /**
     * The origin of the grid in x,y,z. Corresponds to location of front, bottom, left corner.
     */
    DimensionalChastePoint<DIM> mOrigin;

    /**
     * The global grid in the form of vtk image data
     */
    vtkSmartPointer<vtkImageData> mpGlobalVtkGrid;

    /**
     * The local grid in the form of vtk image data
     */
    vtkSmartPointer<vtkImageData> mpLocalVtkGrid;

    /**
     * Is the VTK grid set up
     */
    bool mVtkGridIsSetUp;

    /**
     * A vector of neighbour indices
     */
    std::vector<std::vector<unsigned> > mNeighbourData;

    /**
     * The dimensionless volume of the bounding region for each point.
     * This is not dimensionalised because we might be dealing with area or
     * volume-like quantities, depending on dimension.
     */
    std::vector<double> mPointVolumes;

    /**
     * Storage for a single attribute value at grid points
     */
    std::vector<double> mPointValues;

    /**
     * The reference length scale, default in microns.
     */
    units::quantity<unit::length> mReferenceLength;

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
     * Calculate neighbour indices for each grid point
     */
    void CalculateNeighbourData();

    /**
     * Calculate neighbour indices for each grid point
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
     * Get the 1-D grid index for given x,y,z indices
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the grid 1-d index
     */
    unsigned GetGlobal1dGridIndex(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Get the 1-D grid index for given x,y,z indices local on this process
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the grid 1-d index
     */
    unsigned GetLocal1dGridIndex(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Get the 1-D grid index for given x,y,z indices
     * @param rLocation the point to get the nearest index to
     * @return the 1-d index of the nearest grid point
     */
    unsigned GetNearestGlobalGridIndex(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Calculate local neighbour indices for each grid point
     * @return a vector of neighbour indices for each grid point
     */
    const std::vector<std::vector<unsigned> >& GetNeighbourData();

    /**
     * Calculate local neighbour indices for each grid point
     * @return a vector of neighbour indices for each grid point
     */
    const std::vector<std::vector<unsigned> >& GetMooreNeighbourData();

    /**
     * Return the grid dimensions in x, y, z. Always dimension 3.
     * @return the grid dimensions
     */
    c_vector<unsigned, 3> GetDimensions();

    /**
     * Return the grid extents on this processor
     * @return the grid extents on this processor
     */
    c_vector<unsigned, 6> GetExtents();

    /**
     * Get the location of a point on the grid for given x, y ,z indices
     * @param xIndex the grid x index
     * @param yIndex the grid y index
     * @param zIndex the grid z index
     * @return the location of the point
     */
    DimensionalChastePoint<DIM> GetLocation(unsigned xIndex, unsigned yIndex, unsigned zIndex);

    /**
     * Get the location of a point on the grid for given 1-d grid index
     * @param gridIndex the 1d grid index
     * @return the location of the point
     */
    DimensionalChastePoint<DIM> GetLocationOfLocal1dIndex(unsigned gridIndex);

    /**
     * Get the location of a point on the grid for given 1-d grid index
     * @param gridIndex the 1d grid index
     * @return the location of the point
     */
    DimensionalChastePoint<DIM> GetLocationOfGlobal1dIndex(unsigned gridIndex);

    /**
     * Get all of the grid locations
     * @return a vector containing all grid locations in grid order
     */
    std::vector<DimensionalChastePoint<DIM> > GetGlobalLocations();

    /**
     * Get all of the grid locations
     * @return a vector containing all grid locations in grid order
     */
    std::vector<DimensionalChastePoint<DIM> > GetLocalLocations();

    /**
     * Return the number of points in the grid
     * @return the number of points in the grid
     */
    unsigned GetNumberOfGlobalPoints();

    /**
     * Return the number of points on this process
     * @return the number of points on this process
     */
    unsigned GetNumberOfLocalPoints();

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
     * Return the reference length scale
     *
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale();

    /**
     * Return the grid in vtk format
     *
     * @return the grid in vtk format
     */
    vtkSmartPointer<vtkImageData> GetGlobalVtkGrid();

    /**
     * Return the grid in vtk format
     *
     * @return the grid in vtk format
     */
    vtkSmartPointer<vtkImageData> GetLocalVtkGrid();

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
     * The bounding box for the local grid point. In 2D the z bounds are +1 and -1 to
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
    std::vector<double> GetPointVolumes(bool update=false, bool jiggle=false);

    /**
     * Sample a function specified on the grid at the specified locations
     * @param locations the sample locations
     * @param values the known values at the grid points
     * @param useVtk use VTK to do the sampling, faster but algorithm not clearly documented
     * @return the grid spacing
     */
    std::vector<double> InterpolateGridValues(std::vector<DimensionalChastePoint<DIM> > locations,
                                              std::vector<double> values, bool useVtk = false);

    /**
     * Is the input location in the bounding box of the grid point
     * @param point the location of interest
     * @param gridIndex the grid point of interest
     * @return is the input location in the bounding box of the grid point
     */
    bool IsLocationInPointVolume(DimensionalChastePoint<DIM> point, unsigned gridIndex);

    /**
     * Is the point on the outer boundary of the domain
     * @param gridIndex the 1d grid index gridIndex
     * @return is the point on the outer boundary of the domain
     */
    bool IsGlobalIndexOnBoundary(unsigned gridIndex);

    /**
     * Is the point on the outer boundary of the domain
     * @param gridIndex the 1d grid index gridIndex
     * @return is the point on the outer boundary of the domain
     */
    bool IsLocalIndexOnBoundary(unsigned gridIndex);

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
     */
    void SetDimensions(c_vector<unsigned, 3> dimensions);

    /**
     * Set the origin in x, y, z
     * @param origin the grid origin
     */
    void SetOrigin(DimensionalChastePoint<DIM> origin);

    /**
     * Set the values of a field at all points on the grid
     * @param pointSolution the value of the field
     */
    void SetPointValues(std::vector<double> pointSolution);

    /**
     * Set the grid spacing
     * @param spacing the grid spacing
     */
    void SetSpacing(units::quantity<unit::length> spacing);

    /**
     * Set the internal vtk representation of the grid
     */
    void SetUpVtkGrid();

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
