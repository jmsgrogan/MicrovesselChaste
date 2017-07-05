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

#ifndef DENSITYMAP_HPP_
#define DENSITYMAP_HPP_

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include "AbstractCellMutationState.hpp"
#include "VesselNetwork.hpp"
#include "RegularGrid.hpp"
#include "GridCalculator.hpp"
#include "AbstractCellPopulation.hpp"

/**
 * Forward declare VTK members
 */
class vtkUnstructuredGrid;

/**
 * This class is central to the discrete continuum solvers. It calculates the density of discrete entities (vessels, biological cells)
 * at each grid location (lattice point or grid cell centroid). The resulting density maps can the be used to generate sink or
 * source terms in continuum PDEs.
 */
template<unsigned DIM>
class DensityMap
{
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
    std::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

public:

    /**
     * Constructor
     */
    DensityMap();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static std::shared_ptr<DensityMap<DIM> > Create();

    /**
     * Destructor
     */
    ~DensityMap();

    /**
     * Return the vessel network
     * @return the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GetVesselNetwork();

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
    vtkSmartPointer<vtkUnstructuredGrid> GetSamplingGrid(std::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
     * Return the grid calculator
     * @return the grid calculator
     */
    std::shared_ptr<GridCalculator<DIM> > GetGridCalculator();

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
     * Return true if the specified point is in the cell with the provided index. A VTK cell locator containing
     * the grid corresponding to the input index is also required.
     * @param pCellLocator a VTK cell locator with the indexed grid
     * @param loc the location to be checked
     * @param index the index of the cell to be checked
     * @return true if the specified point is in the cell
     */
    bool IsPointInCell(vtkSmartPointer<vtkCellLocator> pCellLocator, c_vector<double, DIM> loc, unsigned index);

    /**
     * Return the length of the provided line segment in the indexed cell. Provide the VTK grid corresponding
     * to the index and whether or not each end of the line segment is inside the cell.
     * @param pSamplingGrid the grid corresponding to the supplied index
     * @param loc1 the start point of the line
     * @param loc2 the end point of the line
     * @param index the cell index
     * @param loc1InCell is the first point in the cell
     * @param loc2InCell is the second point in the cell
     * @return the dimensionless length of the line in the cell
     */
    double LengthOfLineInCell(vtkSmartPointer<vtkUnstructuredGrid> pSamplingGrid, c_vector<double, DIM> loc1,
            c_vector<double, DIM> loc2, unsigned index, bool loc1InCell, bool loc2InCell);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

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
     * Set the grid
     * @param pGrid the grid
     */
    void SetGrid(std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid);

    /**
     * Set the grid calculator directly
     * @param pGridCalculator the grid calculator
     */
    void SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGridCalculator);

};

#endif /* DENSITYMAP_HPP_ */
