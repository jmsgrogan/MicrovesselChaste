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

#ifndef REGULARGRIDCALCULATOR_HPP_
#define REGULARGRIDCALCULATOR_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "AbstractCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"
#include "RegularGrid.hpp"

/**
 * A class for calculating point and line to grid point relationships and cell and vessel to grid point maps.
 */
template<unsigned DIM>
class RegularGridCalculator
{
    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The cell population. This memory pointed to is not managed in this class.
     */
    AbstractCellPopulation<DIM>* mpCellPopulation;

    /**
     * The reference length scale for the cellpopulation.
     */
    units::quantity<unit::length> mCellPopulationReferenceLength;

    /**
     * A map of cells corresponding to a point on the grid
     */
    std::vector<std::vector<CellPtr> > mPointCellMap;

    /**
     * A map of vessel nodes corresponding to a point on the grid
     */
    std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > > mPointNodeMap;

    /**
     * A map of vessel segments corresponding to a point on the grid
     */
    std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > mPointSegmentMap;

    /**
     * Has a cell population
     */
    bool mHasCellPopulation;

    /**
     * The reference length scale, default in microns.
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * The reference length scale, default in microns.
     */
    boost::shared_ptr<RegularGrid<DIM> > mpRegularGrid;

public:

    /**
     * Constructor
     */
    RegularGridCalculator();

    /**
     * Factory constructor method
     * @return a shared pointer to a new grid
     */
    static boost::shared_ptr<RegularGridCalculator<DIM> > Create();

    /**
     * Desctructor
     */
    ~RegularGridCalculator();

    /**
     * Return a vector of input point indices which in the bounding boxes of each grid point
     * @param inputPoints a vector of point locations
     * @return the indices of input points in the bounding box of each grid point
     */
    std::vector<std::vector<unsigned> > GetPointPointMap(std::vector<DimensionalChastePoint<DIM> > inputPoints);

    /**
     * Return the point cell map
     * @param update update the map
     * @return the point cell map
     */
    const std::vector<std::vector<CellPtr> >& GetPointCellMap(bool update = true);

    /**
     * Return the point node map
     * @param update update the map
     * @return the point node map
     */
    const std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >& GetPointNodeMap(bool update = true);

    /**
     * Return the point segments map
     * @param update update the map
     * @param useVesselSurface use the vessel surface for distance calculations
     * @return the point segment map
     */
    std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > GetPointSegmentMap(bool update = true, bool useVesselSurface = false);

    /**
     * Return the grid itself
     * @return the grid itself
     */
    boost::shared_ptr<RegularGrid<DIM> > GetGrid();

    /**
     * Return true if the segment is at a lattice site
     * @param index the lattice index
     * @param update update the segment-grid map
     * @return true if the segment is at a lattice site
     */
    bool IsSegmentAtLatticeSite(unsigned index, bool update);

    /**
     * Set the cell population
     * @param rCellPopulation a reference to the cell population
     * @param cellPopulationLengthScale the length scale for the cell population
     */
    void SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation, units::quantity<unit::length> cellPopulationLengthScale);

    /**
     * Set the values of a field at all points on the grid
     * @param pointSolution the value of the field
     */
    void SetPointValues(std::vector<double> pointSolution);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the regular grid
     * @param pGrid the regular grid
     */
    void SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid);
};

#endif /* REGULARGRIDCALCULATOR_HPP_*/
