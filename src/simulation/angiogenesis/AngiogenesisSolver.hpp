/*

 Copyright (c) 2005-2015, University of Oxford.
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

#ifndef ANGIOGENESISSOLVER_HPP_
#define ANGIOGENESISSOLVER_HPP_

#include <vector>
#include <string>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "Part.hpp"
#include "GridCalculator.hpp"
#include "AbstractSproutingRule.hpp"
#include "AbstractMigrationRule.hpp"
#include "AbstractCellPopulation.hpp"

/**
 * This class is for simulating modifications to the vessel network due to sprouting angiogenesis.
 */
template<unsigned DIM>
class AngiogenesisSolver
{
    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The radius in which anastamosis is allowed in angiogenesis simulations
     */
    units::quantity<unit::length> mNodeAnastamosisRadius;

    /**
     * The migration rule for tip cells
     */
    boost::shared_ptr<AbstractMigrationRule<DIM> > mpMigrationRule;

    /**
     * The sprouting rule for angiogenesis
     */
    boost::shared_ptr<AbstractSproutingRule<DIM> > mpSproutingRule;

    /**
     * The bounding domain for the vessel network
     */
    boost::shared_ptr<Part<DIM> > mpBoundingDomain;

    /**
     * File handler containing output directory information
     */
    boost::shared_ptr<OutputFileHandler> mpFileHandler;

    /**
     * The grid calculator for lattice based angiogenesis simulations
     */
    boost::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * The cell population for discrete cell angiogenesis models
     */
    boost::shared_ptr<AbstractCellPopulation<DIM> > mpCellPopulation;

    /**
     * The reference length scale for the cellpopulation.
     */
    units::quantity<unit::length> mCellPopulationReferenceLength;

    /**
     * Tip cell collection for discrete cell angiogenesis models
     */
    std::vector<boost::shared_ptr<Cell> > mTipCells;

    /**
     * Cell node map for discrete cell angiogenesis models
     */
    std::map<boost::shared_ptr<Cell> , boost::shared_ptr<VesselNode<DIM> > > mCellNodeMap;

public:

    /**
     * Constructor.
     */
    AngiogenesisSolver();

    /**
     * Destructor.
     */
    virtual ~AngiogenesisSolver();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<AngiogenesisSolver<DIM> > Create();

    /**
     * Increment the solver one step in time
     */
    virtual void Increment();

    /**
     * Has a sprouting rule been set
     * @return bool true if a sprouting rule has been set
     */
    bool IsSproutingRuleSet();

    /**
     * Run until the specified end time
     * @param writeOutput whether to write output
     */
    void Run(bool writeOutput = false);

    /**
     * Set the radius within which anastamosis of vessels is allowed
     * @param radius the radius within which anastamosis of vessels is allowed
     */
    void SetAnastamosisRadius(units::quantity<unit::length> radius);

    /**
     * A domain which vessels a not permitted to leave
     * @param pDomain the domain which vessels a not permitted to leave
     */
    void SetBoundingDomain(boost::shared_ptr<Part<DIM> > pDomain);

    /**
     * Set a cell population for discrete cell solves
     * @param pCellPopulation the cell population for discrete cell solves
     * @param cellPopulationReferenceLength the cell population reference length
     */
    void SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation, units::quantity<unit::length> cellPopulationReferenceLength);

    /**
     * Add a migration rule for tip cells
     * @param pMigrationRule a migration rule for tip cells
     */
    void SetMigrationRule(boost::shared_ptr<AbstractMigrationRule<DIM> > pMigrationRule);

    /**
     * Set the output file handler
     * @param pHandler the output file handler
     */
    void SetOutputFileHandler(boost::shared_ptr<OutputFileHandler> pHandler);

    /**
     * Set the rule for managing sprouting
     * @param pSproutingRule the rule for vessel sprouting
     */
    void SetSproutingRule(boost::shared_ptr<AbstractSproutingRule<DIM> > pSproutingRule);

    /**
     * Set a vessel grid, this means that on-lattice rules will be used
     * @param pVesselGrid the grid for the vessel network
     */
    void SetVesselGridCalculator(boost::shared_ptr<GridCalculator<DIM> >pVesselGrid);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

protected:

    /**
     * Identify and grow sprouts
     */
    virtual void DoSprouting();

    /**
     * Do the anastamosis step
     */
    virtual void DoAnastamosis();

    /**
     * Update the position of all nodes
     * @param sprouting whether to do sprouting during this call
     */
    virtual void UpdateNodalPositions(bool sprouting = false);
};

#endif /* ANGIOGENESISSOLVER_HPP_ */
