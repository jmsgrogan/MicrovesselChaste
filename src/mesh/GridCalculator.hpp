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

#ifndef GRIDCALCULATOR_HPP_
#define GRIDCALCULATOR_HPP_

#include <memory>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include "UblasIncludes.hpp"
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "AbstractCellPopulation.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "Vertex.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"

/**
 * Forward declare VTK members
 */
class vtkPoints;

/**
 * A class for working with structured and unstructured grids and discrete entities.
 * For structured grids storage is on grid points. For unstructured grids it is on
 * 'cells' or (finite) elements.
 */
template<unsigned DIM>
class GridCalculator
{
    /**
     * The vessel network
     */
    VesselNetworkPtr<DIM> mpNetwork;

    /**
     * The cell population.
     */
    AbstractCellPopulation<DIM>* mpCellPopulation;

    /**
     * The reference length scale for the cellpopulation.
     */
    QLength mCellPopulationReferenceLength;

    /**
     * The reference concentration scale for the cellpopulation.
     */
    QConcentration mCellPopulationReferenceConcentration;

    /**
     * A map of cells corresponding to points or elements
     */
    std::vector<std::vector<CellPtr> > mCellMap;

    /**
     * A map of vessel nodes corresponding to points or elements
     */
    std::vector<std::vector<VesselNodePtr<DIM> > > mVesselNodeMap;

    /**
     * A map of vessel segments corresponding to a points or elements
     */
    std::vector<std::vector<VesselSegmentPtr<DIM> > > mSegmentMap;

    /**
     * The grid
     */
    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > mpGrid;

    /**
     * Use regular grid
     */
    bool mHasRegularGrid;

    /**
     * Use mesh
     */
    bool mHasUnstructuredGrid;

public:

    /**
     * Constructor
     */
    GridCalculator();

    /**
     * Factory constructor method
     * @return a shared pointer to a new grid calculator
     */
    static std::shared_ptr<GridCalculator<DIM> > Create();

    /**
     * Desctructor
     */
    ~GridCalculator();

    /**
     * Has a cell population been set?
     * @return whether a cell population been set.
     */
    bool CellPopulationIsSet();

    /**
     * Return a vector of input point indices which in the bounding boxes of each grid location
     * @param rInputPoints a vector of point locations
     * @return the indices of input points in the bounding box of each grid location
     */
    std::vector<std::vector<unsigned> > GetPointMap(const std::vector<Vertex<DIM> >& rInputPoints);

    /**
     * Return a vector of input point indices which in the bounding boxes of each grid location
     * @param pInputPoints input points in VTK form
     * @return the indices of input points in the bounding box of each grid location
     */
    std::vector<std::vector<unsigned> > GetPointMap(vtkSmartPointer<vtkPoints> pInputPoints);

    /**
     * Return the cell map
     * @param update update the map
     * @return the cell map
     */
    const std::vector<std::vector<CellPtr> > & rGetCellMap(bool update = true);

    /**
     * Return the vessel network
     * @return the vessel network
     */
    VesselNetworkPtr<DIM> GetVesselNetwork();

    /**
     * Return the vessel node map
     * @param update update the vessel node map
     * @return the vessel node map
     */
    const std::vector<std::vector<VesselNodePtr<DIM> > >& rGetVesselNodeMap(bool update = true);

    /**
     * Return the segments map
     * @param update update the map
     * @param useVesselSurface use the vessel surface for distance calculations
     * @return the segment map
     */
    const std::vector<std::vector<VesselSegmentPtr<DIM> > >& rGetSegmentMap(bool update = true,
            bool useVesselSurface = false);

    /**
     * Return the grid itself
     * @return the grid itself
     */
    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > GetGrid();

    /**
     * Return true if the solver uses a regular grid to store solutions
     * @return true if the solver uses a regular grid to store solutions
     */
    bool HasStructuredGrid();

    /**
     * Return true if the solver uses a unstructured grid to store solutions
     * @return true if the solver uses a unstructured grid to store solutions
     */
    bool HasUnstructuredGrid();

    /**
     * Return true if the segment is at the grid location
     * @param index the location index
     * @param update update the segment-grid map
     * @return true if the segment is at a lattice site
     */
    bool IsSegmentAtLocation(unsigned index, bool update);

    /**
     * Set the cell population
     * @param rCellPopulation a reference to the cell population
     * @param cellPopulationReferenceLength the length scale for the cell population
     * @param cellPopulationReferenceConcentration the concentration scale for the cell population
     */
    void SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation,
                           QLength cellPopulationReferenceLength,
                           QConcentration cellPopulationReferenceConcentration);

    /**
     * Set the values of a field at all locations on the grid
     * @param pointSolution the value of the field
     */
    //void SetLocationValues(const std::vector<double>& rPointSolution);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(VesselNetworkPtr<DIM> pNetwork);

    /**
     * Set the grid
     * @param pGrid the grid
     */
    void SetGrid(std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid);

};

#endif /* GRIDCALCULATOR_HPP_*/
