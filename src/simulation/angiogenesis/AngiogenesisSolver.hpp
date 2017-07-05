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
     * The reference length scale
     */
    QLength mReferenceLength;

    /**
     * The vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The radius in which anastamosis is allowed in angiogenesis simulations
     */
    QLength mNodeAnastamosisRadius;

    /**
     * The migration rule for tip cells
     */
    std::shared_ptr<AbstractMigrationRule<DIM> > mpMigrationRule;

    /**
     * The sprouting rule for angiogenesis
     */
    std::shared_ptr<AbstractSproutingRule<DIM> > mpSproutingRule;

    /**
     * The bounding domain for the vessel network
     */
    std::shared_ptr<Part<DIM> > mpBoundingDomain;

    /**
     * File handler containing output directory information
     */
    std::shared_ptr<OutputFileHandler> mpFileHandler;

    /**
     * The grid calculator for lattice based angiogenesis simulations
     */
    std::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * The cell population for discrete cell angiogenesis models
     */
    std::shared_ptr<AbstractCellPopulation<DIM> > mpCellPopulation;

    /**
     * The reference length scale for the cellpopulation.
     */
    QLength mCellPopulationReferenceLength;

    /**
     * Tip cell collection for discrete cell angiogenesis models
     */
    std::vector<std::shared_ptr<Cell> > mTipCells;

    /**
     * Cell node map for discrete cell angiogenesis models
     */
    std::map<std::shared_ptr<Cell> , std::shared_ptr<VesselNode<DIM> > > mCellNodeMap;

    /**
     * Whether to do anastamosis
     */
    bool mDoAnastamosis;

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
    static std::shared_ptr<AngiogenesisSolver<DIM> > Create();

    /**
     * Increment the solver one step in time
     */
    virtual void Increment();

    /**
     * Has a sprouting rule been set
     * @return bool true if a sprouting rule has been set
     */
    bool IsSproutingRuleSet();

    void SetDoAnastomosis(bool doAnastomosis);

    /**
     * Run until the specified end time
     * @param writeOutput whether to write output
     */
    void Run(bool writeOutput = false);

    /**
     * Set the radius within which anastamosis of vessels is allowed
     * @param radius the radius within which anastamosis of vessels is allowed
     */
    void SetAnastamosisRadius(QLength radius);

    /**
     * A domain which vessels a not permitted to leave
     * @param pDomain the domain which vessels a not permitted to leave
     */
    void SetBoundingDomain(std::shared_ptr<Part<DIM> > pDomain);

    /**
     * Set a cell population for discrete cell solves
     * @param pCellPopulation the cell population for discrete cell solves
     * @param cellPopulationReferenceLength the cell population reference length
     */
    void SetCellPopulation(std::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation, QLength cellPopulationReferenceLength);

    /**
     * Add a migration rule for tip cells
     * @param pMigrationRule a migration rule for tip cells
     */
    void SetMigrationRule(std::shared_ptr<AbstractMigrationRule<DIM> > pMigrationRule);

    /**
     * Set the output file handler
     * @param pHandler the output file handler
     */
    void SetOutputFileHandler(std::shared_ptr<OutputFileHandler> pHandler);

    /**
     * Set the rule for managing sprouting
     * @param pSproutingRule the rule for vessel sprouting
     */
    void SetSproutingRule(std::shared_ptr<AbstractSproutingRule<DIM> > pSproutingRule);

    /**
     * Set a vessel grid, this means that on-lattice rules will be used
     * @param pVesselGrid the grid for the vessel network
     */
    void SetVesselGridCalculator(std::shared_ptr<GridCalculator<DIM> >pVesselGrid);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

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
