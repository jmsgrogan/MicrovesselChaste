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

#ifndef ABSTRACTMICROVESSELSIMULATIONMODIFIER_HPP_
#define ABSTRACTMICROVESSELSIMULATIONMODIFIER_HPP_

#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "AbstractCellPopulation.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"

/**
 * A modifier class for the Microvessel Solver itself, rather than the Cell Based Chaste simulation.
 */
template<unsigned DIM>
class AbstractMicrovesselModifier
{

    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The cell population
     */
    boost::shared_ptr<AbstractCellPopulation<DIM,DIM> > mpCellPopulation;

    /**
     * A discrete continuum solver
     */
    std::vector<boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > > mDiscreteContinuumSolvers;

public:

    /**
     * Default constructor.
     */
    AbstractMicrovesselModifier();

    /**
     * Destructor.
     */
    virtual ~AbstractMicrovesselModifier();

    /**
     * Add a discrete continuum solver
     * @param pDiscreteContinuumSolver a discrete continuum solver
     */
    void AddDiscreteContinuumSolver(boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pDiscreteContinuumSolver);

    /**
     * @return the cell population
     */
    boost::shared_ptr<AbstractCellPopulation<DIM> > GetCellPopulation();

    /**
     * @return the vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > GetVesselNetwork();

    /**
     * @return the number of solvers
     */
    unsigned GetNumberOfDiscreteContinuumSolvers();

    /**
     * Return the indexed solver
     * @param index the indexed solver
     * @return the number of solvers
     */
    boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > GetDiscreteContinuumSolver(unsigned index);

    /**
     * Set the cell population
     * @param pCellPopulation the cell population
     */
    void SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM,DIM> > pCellPopulation);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Overridden SetupSolve() method.
     * Specify what to do in the simulation before the start of the time loop.
     *
     * @param outputDirectory the output directory, relative to where Chaste output is stored
     */
    virtual void SetupSolve(std::string outputDirectory)=0;

    /**
     * Overridden UpdateAtEndOfTimeStep() method.
     * Specify what to do in the simulation at the end of each time step.
     */
    virtual void UpdateAtEndOfTimeStep()=0;
};

#endif /*ABSTRACTMICROVESSELMODIFIER_HPP_*/
