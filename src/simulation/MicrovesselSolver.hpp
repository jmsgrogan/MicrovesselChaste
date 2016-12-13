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

#ifndef MICROVESSELSOLVER_HPP_
#define MICROVESSELSOLVER_HPP_

#include <vector>
#include <string>

#include "StructuralAdaptationSolver.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractCellPopulation.hpp"
#include "AngiogenesisSolver.hpp"
#include "RegressionSolver.hpp"
#include "MaturityCalculator.hpp"
#include "AbstractMicrovesselModifier.hpp"

/**
 * This class manages the solution of vascular tumour growth problems. It steps through time,
 * solves a collection of DiscreteContinuum discrete-continuum PDEs and, if required, updates the vessel network. For linking
 * with discrete cell models it can be added to a VascularTumourGrowth simulation modifier.
 */
template<unsigned DIM>
class MicrovesselSolver
{
    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The frequency of file based output
     */
    unsigned mOutputFrequency;

    /**
     * Filehandler containing output directory information
     */
    boost::shared_ptr<OutputFileHandler> mpOutputFileHandler;

    /**
     * The collection of DiscreteContinuum solvers
     */
    std::vector<boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > > mDiscreteContinuumSolvers;

    /**
     * The structural adaptation solver for the vessel network
     */
    boost::shared_ptr<StructuralAdaptationSolver<DIM> > mpStructuralAdaptationSolver;

    /**
     * The angiogenesis solver for the vessel network
     */
    boost::shared_ptr<AngiogenesisSolver<DIM> > mpAngiogenesisSolver;

    /**
     * The regression solver for the vessel network
     */
    boost::shared_ptr<RegressionSolver<DIM> > mpRegressionSolver;

    /**
     * The maturity calculator for the vessel network
     */
    boost::shared_ptr<MaturityCalculator<DIM> > mpMaturityCalculator;

    /**
     * Can the solution from one DiscreteContinuumSolver be sent directly to
     * another, or is grid sampling needed.
     */
    bool mDiscreteContinuumSolversHaveCompatibleGridIndexing;

    /**
     * Whether to update the PDE for each solve
     */
    bool mUpdatePdeEachSolve;

    /**
     * A collection of modifiers for the solver
     */
    std::vector<boost::shared_ptr<AbstractMicrovesselModifier<DIM> > > mMicrovesselModifiers;

    /**
     * A cell population
     */
    boost::shared_ptr<AbstractCellPopulation<DIM,DIM> > mpCellPopulation;

public:

    /**
     * Constructor.
     */
    MicrovesselSolver();

    /**
     * Destructor.
     */
    virtual ~MicrovesselSolver();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<MicrovesselSolver> Create();

    /**
     * Add a DiscreteContinuum solver to the collection
     * @param pDiscreteContinuumSolver a discrete-continuum solver
     */
    void AddDiscreteContinuumSolver(boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pDiscreteContinuumSolver);

    /**
     * Add a MicrovesselModifier to the collection
     * @param pMicrovesselModifier a microvessel modifier
     */
    void AddMicrovesselModifier(boost::shared_ptr<AbstractMicrovesselModifier<DIM> > pMicrovesselModifier);

    /**
     * Return the current DiscreteContinuum solvers
     * @return the DiscreteContinuum solvers
     */
    std::vector<boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > > GetDiscreteContinuumSolvers();

    /**
     * Increment one step in time
     */
    void Increment();

    /**
     * Run until the specified end time
     */
    void Run();

    /**
     * Set the angiogenesis solver for the network
     * @param pAngiogenesisSolver the solver for structural adaptation
     */
    void SetAngiogenesisSolver(boost::shared_ptr<AngiogenesisSolver<DIM> > pAngiogenesisSolver);

    /**
     * Set the output directory for results
     * @param pFileHandler output file handler containing output directory information
     */
    void SetOutputFileHandler(boost::shared_ptr<OutputFileHandler> pFileHandler);

    /**
     * Set the results output frequency
     * @param frequency the frequency of simulaiton output
     */
    void SetOutputFrequency(unsigned frequency);

    /**
     * Set whether to update the pde at each solve
     * @param doUpdate update the pde at each solve
     */
    void SetUpdatePdeEachSolve(bool doUpdate);

    /**
     * Set the structural adaptation solver for the network
     * @param pStructuralAdaptationSolver the solver for structural adaptation
     */
    void SetStructuralAdaptationSolver(boost::shared_ptr<StructuralAdaptationSolver<DIM> > pStructuralAdaptationSolver);

    /**
     * This is called by the MicrovesselSimulationModifier to set up the simulation
     * @param rCellPopulation the cell population
     * @param cellReferenceLength the cell length scale
     * @param cellReferenceConcentration the cell concentration scale
     * @param rDirectory the output directory for writing to
     */
    void SetupFromModifier(AbstractCellPopulation<DIM,DIM>& rCellPopulation,
                           units::quantity<unit::length> cellReferenceLength,
                           units::quantity<unit::concentration> cellReferenceConcentration,
                           const std::string& rDirectory);

    /**
     * This should be called before running in standalone mode
     */
    void Setup();

    /**
     * Set to true if we know that our discrete continuum solvers all have the same grid
     * @param compatibleIndexing doe the grids have compatible indexing
     */
    void SetDiscreteContinuumSolversHaveCompatibleGridIndexing(bool compatibleIndexing);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the cell population, only needed for visualization
     * @param pCellPopulation the cell population
     */
    void SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM,DIM> > pCellPopulation);

    /**
     * Set the regression solver
     * @param pRegressionSolver the regression solver for the network
     */
    void SetRegressionSolver(boost::shared_ptr<RegressionSolver<DIM> > pRegressionSolver);

    /**
     * Set the maturity calculator
     * @param pMaturityCalculator the maturity calculator for the network
     */
    void SetMaturityCalculator(boost::shared_ptr<MaturityCalculator<DIM> > pMaturityCalculator);

    /**
     * Update the cell data with any PDE solutions corresponding to the supplied labels
     * @param labels labels corresponding to cell data that is to be updated
     */
    void UpdateCellData(std::vector<std::string> labels);

};

#endif /* MICROVESSELSOLVER_HPP_ */
