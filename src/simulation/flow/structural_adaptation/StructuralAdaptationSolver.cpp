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

#include <fstream>
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "UnitCollection.hpp"
#include "RadiusCalculator.hpp"

template<unsigned DIM>
StructuralAdaptationSolver<DIM>::StructuralAdaptationSolver() :
        AbstractStructuralAdaptationSolver<DIM>(),
        mpFlowSolver(new FlowSolver<DIM>),
        mpRadiusCalculator(new RadiusCalculator<DIM>),
        mPreFlowSolveCalculators(),
        mPostFlowSolveCalculators()
{

}

template<unsigned DIM>
StructuralAdaptationSolver<DIM>::~StructuralAdaptationSolver()
{

}

template<unsigned DIM>
std::shared_ptr<StructuralAdaptationSolver<DIM> > StructuralAdaptationSolver<DIM>::Create()
{
    return std::make_shared<StructuralAdaptationSolver<DIM> >();

}

template<unsigned DIM>
std::shared_ptr<FlowSolver<DIM> > StructuralAdaptationSolver<DIM>::GetFlowSolver()
{
    return mpFlowSolver;
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::SetRadiusCalculator(std::shared_ptr<RadiusCalculator<DIM> > pCalculator)
{
    mpRadiusCalculator = pCalculator;
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::AddPreFlowSolveCalculator(std::shared_ptr<AbstractVesselNetworkCalculator<DIM> > pCalculator)
{
    mPreFlowSolveCalculators.push_back(pCalculator);
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::AddPostFlowSolveCalculator(std::shared_ptr<AbstractVesselNetworkCalculator<DIM> > pCalculator)
{
    mPostFlowSolveCalculators.push_back(pCalculator);
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::SetFlowSolver(std::shared_ptr<FlowSolver<DIM> > pCalculator)
{
    mpFlowSolver = pCalculator;
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::Iterate()
{
    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required before the SA solver can be used.");
    }

    if(SimulationTime::Instance()->GetTimeStepsElapsed()==0)
    {
        // Set up calculators
        for(unsigned idx=0; idx<mPreFlowSolveCalculators.size();idx++)
        {
            mPreFlowSolveCalculators[idx]->SetVesselNetwork(this->mpVesselNetwork);
        }
        for(unsigned idx=0; idx<mPostFlowSolveCalculators.size();idx++)
        {
            mPostFlowSolveCalculators[idx]->SetVesselNetwork(this->mpVesselNetwork);
        }
        if(mpRadiusCalculator)
        {
            mpRadiusCalculator->SetTimestep(this->GetTimeIncrement());
            mpRadiusCalculator->SetVesselNetwork(this->mpVesselNetwork);
        }
    }

    for(unsigned idx=0; idx<mPreFlowSolveCalculators.size();idx++)
    {
        mPreFlowSolveCalculators[idx]->Calculate();
    }

    if(SimulationTime::Instance()->GetTimeStepsElapsed()==0)
    {
        UpdateFlowSolver();
    }

    mpFlowSolver->Update(false);
    mpFlowSolver->Solve();

    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = this->mpVesselNetwork->GetVesselSegments();
    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        segments[idx]->GetFlowProperties()->SetGrowthStimulus(0.0*(1.0/(unit::seconds)));
    }

    for(unsigned idx=0; idx<mPostFlowSolveCalculators.size();idx++)
    {
        mPostFlowSolveCalculators[idx]->Calculate();
    }

    if(mpRadiusCalculator)
    {
        mpRadiusCalculator->SetTimestep(this->GetTimeIncrement());
        mpRadiusCalculator->Calculate();
    }
}

template<unsigned DIM>
void StructuralAdaptationSolver<DIM>::UpdateFlowSolver(bool doFullReset)
{
    if(mpFlowSolver)
    {
        mpFlowSolver->SetVesselNetwork(this->mpVesselNetwork);
        if(doFullReset && !SimulationTime::Instance()->GetTimeStepsElapsed()==0)
        {
            for(unsigned idx=0; idx<mPreFlowSolveCalculators.size();idx++)
            {
                mPreFlowSolveCalculators[idx]->Calculate();
            }
            mpFlowSolver->SetUp();
        }
    }
}

// Explicit instantiation
template class StructuralAdaptationSolver<2> ;
template class StructuralAdaptationSolver<3> ;
