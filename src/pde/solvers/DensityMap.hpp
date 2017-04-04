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

#ifndef DENSITYMAPA_HPP_
#define DENSITYMAPA_HPP_

#include <vtkUnstructuredGrid.h>
#include "SmartPointers.hpp"
#include "AbstractCellMutationState.hpp"
#include "VesselNetwork.hpp"
#include "RegularGrid.hpp"
#include "GridCalculator.hpp"
#include "AbstractCellPopulation.hpp"

/**
 * Calculate the density of vessel network features (nodes, branches, segments) or cells on a
 * grid. The density map can then be used in the solution of PDEs with discrete sinks or
 * sources.
 */
template<unsigned DIM>
class DensityMap
{
    /**
     * The vessel network.
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The cell population.
     */
    AbstractCellPopulation<DIM>* mpCellPopulation;

    /**
     * Dimensionless vessel surface area density
     */
    std::vector<double> mVesselSurfaceAreaDensity;

    /**
     * Dimensionless vessel line density
     */
    std::vector<double> mVesselLineDensity;

    /**
     * Dimensionless perfused vessel surface area density
     */
    std::vector<double> mPerfusedVesselSurfaceAreaDensity;

    /**
     * Dimensionless perfused vessel line area density
     */
    std::vector<double> mPerfusedVesselLineDensity;

    /**
     * Dimensionless vessel tip density
     */
    std::vector<double> mVesselTipDensity;

    /**
     * Dimensionless vessel branch density
     */
    std::vector<double> mVesselBranchDensity;

    /**
     * Dimensionless vessel quantity density
     */
    std::vector<double> mVesselQuantityDensity;

    /**
     * Dimensionless cell density
     */
    std::vector<double> mDimensionlessCellDensity;

    /**
     * Dimensionless cell density by mutation type
     */
    std::vector<double> mDimensionlessCellDensityByMutationType;

    /**
     * A grid calculator
     */
    boost::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

public:

    /**
     * Constructor
     */
    DensityMap();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<DensityMap<DIM> > Create();

    /**
     * Destructor
     */
    ~DensityMap();

    double LengthOfLineInCell(vtkSmartPointer<vtkUnstructuredGrid> pSamplingGrid, c_vector<double, DIM> loc1,
            c_vector<double, DIM> loc2, unsigned index, bool loc1InCell, bool loc2InCell);

    bool IsPointInCell(vtkSmartPointer<vtkCellLocator> pCellLocator, c_vector<double, DIM> loc, unsigned index);

    /**
     * Return the grid in a form suitable for length of line in box computations
     * @param pGrid the input grid
     * @return the processed grid
     */
    vtkSmartPointer<vtkUnstructuredGrid> GetSamplingGrid(vtkSmartPointer<vtkUnstructuredGrid> pGrid);

    /**
     * Return the grid in a form suitable for length of line in box computations
     * @param pGrid the input grid
     * @return the processed grid
     */
    vtkSmartPointer<vtkUnstructuredGrid> GetSamplingGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
     * Get the vessel surface area density
     * @param update update the stored quantity
     * @return the vessel surface area density
     */
    const std::vector<double>& rGetVesselSurfaceAreaDensity(bool update=true);

    /**
     * Get the vessel line density
     * @param update update the stored quantity
     * @return the vessel line density
     */
    std::vector<double> rGetVesselLineDensity(bool update=true);

    /**
     * Get the perfused vessel surface area density
     * @param update update the stored quantity
     * @return the perfused vessel surface area density
     */
    const std::vector<double>& rGetPerfusedVesselSurfaceAreaDensity(bool update=true);

    /**
     * Get the perfused vessel line density
     * @param update update the stored quantity
     * @return the perfused vessel line  density
     */
    const std::vector<double>& rGetPerfusedVesselLineDensity(bool update=true);

    /**
     * Get the vessel tip density
     * @param update update the stored quantity
     * @return the vessel tip density
     */
    const std::vector<double>& rGetVesselTipDensity(bool update=true);

    /**
     * Get the vessel branch density
     * @param update update the stored quantity
     * @return the vessel branch density
     */
    const std::vector<double>& rGetVesselBranchDensity(bool update=true);

    /**
     * Get the vessel quantity density
     * @param rQuantity the quantity to update
     * @param update update the stored quantity
     * @return the vessel quantity density
     */
    const std::vector<double>& rGetVesselQuantityDensity(const std::string& rQuantity, bool update=true);

    /**
     * Get the cell density
     * @param update update the stored quantity
     * @return the cell density
     */
    const std::vector<double>& rGetCellDensity(bool update=true);

    /**
     * Get the cell by mutation state
     * @param pMutationState the cell mutation state to check
     * @param update update the stored quantity
     * @return the cell density by mutation state
     */
    const std::vector<double>& rGetCellDensity(boost::shared_ptr<AbstractCellMutationState> pMutationState, bool update=true);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the cell population
     * @param rCellPopulation a reference to the cell population
     * @param cellPopulationReferenceLength the length scale for the cell population
     * @param cellPopulationReferenceConcentration the concentration scale for the cell population
     */
    void SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation);

    /**
     * Set the regular grid
     * @param pGrid the regular grid
     */
    void SetGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
     * Set the mesh
     * @param pGrid the mesh
     */
    void SetGrid(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pGrid);

};

#endif /* DENSITYMAPA_HPP_ */
