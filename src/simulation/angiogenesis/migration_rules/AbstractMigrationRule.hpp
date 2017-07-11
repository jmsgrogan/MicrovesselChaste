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



#ifndef ABSTRACTMIGRATIONRULE_HPP_
#define ABSTRACTMIGRATIONRULE_HPP_

#include <vector>
#include <string>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractCellPopulation.hpp"
#include "GridCalculator.hpp"
#include "Part.hpp"
#include "Vertex.hpp"

/**
 * Abstract class for implementing a vessel tip cell migration rule. On and off-lattice specializations
 * are implemented in subclasses.
 */
template<unsigned DIM>
class AbstractMigrationRule
{

protected:

    /**
     * A DiscreteContinuum solver containing a solution field of interest
     */
    std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > mpSolver;

    /**
     * The vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * Distinguish between sprouting and migrating events
     */
    bool mIsSprouting;

    /**
     * The cell population, only used in certain child classes
     */
    std::shared_ptr<AbstractCellPopulation<DIM> > mpCellPopulation;

    /**
     * A regular grid, used in some lattice based simulations
     */
    std::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * The bounding domain, optional
     */
    PartPtr<DIM> mpBoundingDomain;

    /**
     * Use Moore neighbourhood
     */
    bool mUseMooreNeighbourhood;

public:

    /**
     * Constructor.
     */
    AbstractMigrationRule();

    /**
     * Destructor.
     */
    virtual ~AbstractMigrationRule();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to the class instance
     */
    static std::shared_ptr<AbstractMigrationRule<DIM> > Create();

    /**
     * Get the sprout directions
     * @param rNodes candidate sprout nodes
     * @return the sprout directions
     */
    virtual std::vector<Vertex<DIM> > GetDirections(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Get the sprout indices
     * @param rNodes candidate sprout nodes
     * @return the sprout indices
     */
    virtual std::vector<int> GetIndices(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set whether this is a sprouting event
     * @param isSprouting is this a sprouting event
     */
     void SetIsSprouting(bool isSprouting = true);

    /**
     * Set the DiscreteContinuum solver containing the stimulus field
     * @param pSolver the DiscreteContinuum solver containing the stimulus field
     */
    void SetDiscreteContinuumSolver(std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the bounding domain
     * @param pPart the bounding domain
     */
    void SetBoundingDomain(PartPtr<DIM> pPart);

    /**
     * Set the lattice/grid for the vessel network
     * @param pGrid the grid for the vessel network
     */
    void SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGrid);

    /**
     * Set the cell population
     * @param pCellPopulation the cell population
     */
    void SetCellPopulation(std::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation);

    /**
     * Set whether to use a Moore or Von Neumann neighbourhood
     * @param useMooreNeighbourhood whether to use a Moore or Von Neumann neighbourhood
     */
    void SetUseMooreNeighbourhood(bool useMooreNeighbourhood);

};

#endif /* ABSTRACTMIGRATIONRULE_HPP_ */
