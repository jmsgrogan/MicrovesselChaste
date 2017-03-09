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

#ifndef SIMPLESTRCUTURALADAPATATIONSOLVER_HPP
#define SIMPLESTRCUTURALADAPATATIONSOLVER_HPP

#include "SmartPointers.hpp"
#include "RadiusCalculator.hpp"
#include "FlowSolver.hpp"
#include "AbstractStructuralAdaptationSolver.hpp"
#include "AbstractVesselNetworkCalculator.hpp"

/**
 * This is a concrete implementation of a structural adaptation solver. It iteratively changes
 * vessel radii in response to a collection of flow based stimuli until the rate of change of
 * the radius is below a specified tolerance.
 */
template<unsigned DIM>
class StructuralAdaptationSolver : public AbstractStructuralAdaptationSolver<DIM>
{

private:

    /**
     * A solver to calculate flow rates and pressures in the network
     */
    boost::shared_ptr<FlowSolver<DIM> > mpFlowSolver;

    /**
     * A calculator to determine radius changes
     */
    boost::shared_ptr<RadiusCalculator<DIM> > mpRadiusCalculator;

    /**
     * Calculators to be run before the flow solve
     */
    std::vector<boost::shared_ptr<AbstractVesselNetworkCalculator<DIM> > > mPreFlowSolveCalculators;

    /**
     * Calculators to be run after the flow solve
     */
    std::vector<boost::shared_ptr<AbstractVesselNetworkCalculator<DIM> > > mPostFlowSolveCalculators;


public:

    /**
     * Constructor.
     */
    StructuralAdaptationSolver();

    /**
     * Destructor.
     */
    virtual ~StructuralAdaptationSolver();

    /**
     * Factor constructor. Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class.
     */
    static boost::shared_ptr<StructuralAdaptationSolver<DIM> > Create();

    /**
     * Get the flow calculator
     * @return the flow solver.
     */
    boost::shared_ptr<FlowSolver<DIM> > GetFlowSolver();

    /**
     * Perform a single iteration to update the radius and calculators
     */
    virtual void Iterate();

    /**
     * Add a vessel network calculator to be run before the flow solve
     * @param pCalculator a vessel network calculator
     */
    void AddPreFlowSolveCalculator(boost::shared_ptr<AbstractVesselNetworkCalculator<DIM> > pCalculator);

    /**
     * Add a vessel network calculator to be run before the flow solve
     * @param pCalculator a vessel network calculator
     */
    void AddPostFlowSolveCalculator(boost::shared_ptr<AbstractVesselNetworkCalculator<DIM> > pCalculator);

    /**
     * Set the flow calculator
     * @param pSolver the flow solver.
     */
    void SetFlowSolver(boost::shared_ptr<FlowSolver<DIM> > pSolver);

    /**
     * Set the radius calculator
     * @param pCalculator the radius calculator.
     */
    void SetRadiusCalculator(boost::shared_ptr<RadiusCalculator<DIM> > pCalculator);

    /**
     * To be called if the vessel network changes between solves
     * @param doFullReset fully reset all the solvers
     */
    void UpdateFlowSolver(bool doFullReset=false);

};

#endif //SIMPLESTRCUTURALADAPATATIONSOLVER_HPP
