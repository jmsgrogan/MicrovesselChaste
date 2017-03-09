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



#ifndef OWEN11CELLPOPULATIONGENERATOR_HPP_
#define OWEN11CELLPOPULATIONGENERATOR_HPP_

#include <vector>
#include "Part.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "GridCalculator.hpp"
#include "UnitCollection.hpp"
#include "AbstractCellMutationState.hpp"
#include "CaBasedCellPopulation.hpp"
#include "PottsMeshGenerator.hpp"

/**
 * This class generates a CaBasedCellPopulation on a regular grid filled with 'normal' cells and a tumour spheroid of specified radius
 * in the centre. It also sets up a suitable cell cycle model, mutation states and population writers.
 */
template<unsigned DIM>
class Owen11CellPopulationGenerator
{
    /**
     * A vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * A grid for the cells
     */
    boost::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * A potts mesh generator. We store this because we can't let it go out of scope in the
     * lifetime of the Owen11CellPopulationGenerator.
     */
    boost::shared_ptr<PottsMeshGenerator<DIM> > mpPottsMeshGenerator;

    /**
     * Mutation state for cancer cells
     */
    boost::shared_ptr<AbstractCellMutationState> mpCancerCellMutationState;

    /**
     * Mutation state for normal cells
     */
    boost::shared_ptr<AbstractCellMutationState> mpNormalCellMutationState;

    /**
     * Mutation state for vessel stalk endothelial cells
     */
    boost::shared_ptr<AbstractCellMutationState> mpStalkCellMutationState;

    /**
     * The reference length scale for the population, default in microns. This is needed as units can't be combined
     * with c_vectors, which hold the cell's location.
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * The tumour radius
     */
    units::quantity<unit::length> mTumourRadius;

    /**
     * The reference length scale for the cellpopulation.
     */
    units::quantity<unit::length> mCellPopulationReferenceLength;


public:

    /**
     * Constructor
     */
    Owen11CellPopulationGenerator();

    /**
     * Factory constructor method
     *
     * @return a shared pointer to a new population
     */
    static boost::shared_ptr<Owen11CellPopulationGenerator<DIM> > Create();

    /**
     * Desctructor
     */
    ~Owen11CellPopulationGenerator();

    /**
     * Set the vessel network
     *
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the regular grid
     *
     * @param pGrid the regular grid
     */
    void SetGridCalculator(boost::shared_ptr<GridCalculator<DIM> > pGrid);

    /**
     * Set the reference length
     *
     * @param lengthScale the reference length
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lengthScale);

    /**
     * Set the tumour radius
     *
     * @param tumourRadius the tumour radius
     */
    void SetTumourRadius(units::quantity<unit::length> tumourRadius);

    /**
     * Generate the cell population
     *
     * @return a pointer to the cell population
     */
    boost::shared_ptr<CaBasedCellPopulation<DIM> > Update();
};

#endif /* Owen11CellPopulationGenerator_HPP_*/
