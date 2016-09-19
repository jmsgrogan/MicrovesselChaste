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

#include "Owen2011OxygenBasedCellCycleOdeSystem.hpp"
#include "CellwiseOdeSystemInformation.hpp"
#include "IsNan.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "StalkCellMutationState.hpp"

Owen2011OxygenBasedCellCycleOdeSystem::Owen2011OxygenBasedCellCycleOdeSystem(double oxygenConcentration,
        boost::shared_ptr<AbstractCellMutationState> mutation_state,
        std::vector<double> stateVariables)
: AbstractOdeSystem(4),
  oxygenConcentration(oxygenConcentration),
  pmMutationState(mutation_state)
{
    mpSystemInfo.reset(new CellwiseOdeSystemInformation<Owen2011OxygenBasedCellCycleOdeSystem>);

    mpSystemInfo->SetDefaultInitialCondition(3, oxygenConcentration);

    assert(pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<WildTypeCellMutationState>() || pmMutationState->IsType<QuiescentCancerCellMutationState>()
            || pmMutationState->IsType<StalkCellMutationState>() || pmMutationState->IsType<TipCellMutationState>());

    /*
     % The variables are
     % 0. phi = Cell-cycle phase
     % 1. p53 = p53 concentration
     % 2. VEGF = VEGF concentration
     % 3. O2 = Oxygen concentration
     */

    Init(); // set up parameters

    // Parameter values are taken from the Owen (2011) paper
    if (pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<QuiescentCancerCellMutationState>()) // cancer cell
    {
        Cphi = 1.4;
        Tmin = 1600;
        k8doubledash = 0.002;
    }
    else // normal cells
    {
        Cphi = 3;
        Tmin = 3000;
        k8doubledash = -0.002;
    }


    if (stateVariables != std::vector<double>())
    {
        SetStateVariables(stateVariables);
    }

}

Owen2011OxygenBasedCellCycleOdeSystem::~Owen2011OxygenBasedCellCycleOdeSystem()
{
    // Do nothing
}

void Owen2011OxygenBasedCellCycleOdeSystem::Init()
{
    // Parameter values are taken from the Owen (2011) paper
    k7 = 0.002;
    k7dash = 0.01;
    Cp53 = 4.44;
    k8 = 0.002;
    J5 = 0.04;
    k8dash = 0.01;
    CVEGF = 4.44;
}

void Owen2011OxygenBasedCellCycleOdeSystem::SetMutationState(boost::shared_ptr<AbstractCellMutationState> pMutationState)
{

    if (!pMutationState->IsSubType<AbstractCellMutationState>())
    {
        EXCEPTION("Attempting to give cell a cell mutation state that is not a subtype of AbstractCellMutationState");
    }

    pmMutationState = pMutationState;

}

boost::shared_ptr<AbstractCellMutationState> Owen2011OxygenBasedCellCycleOdeSystem::GetMutationState() const
{
    return pmMutationState;
}

void Owen2011OxygenBasedCellCycleOdeSystem::EvaluateYDerivatives(double time, const std::vector<double>& rY, std::vector<double>& rDY)
{


    assert(pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<WildTypeCellMutationState>() || pmMutationState->IsType<QuiescentCancerCellMutationState>());

    double p53 = rY[1];
    double VEGF = rY[2];
    double oxygenConcentration = rY[3];

    double dphi = 0.0;
    double dp53 = 0.0;
    double dVEGF = 0.0;

    /*
     % The variables are
     % 0. phi = Cell-cycle phase
     % 1. p53 = p53 concentration
     % 2. VEGF = VEGF concentration
     % 3. O2 = Oxygen concentration
     */

    if (pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<WildTypeCellMutationState>())
    {
        dphi = oxygenConcentration/(Tmin*(Cphi + oxygenConcentration));
    }
    else
    {
        assert(pmMutationState->IsType<QuiescentCancerCellMutationState>());
        dphi = 0.0;
    }

    dp53 = k7 - k7dash*p53*oxygenConcentration/(Cp53 + oxygenConcentration);
    dVEGF = k8 + k8doubledash*p53*VEGF/(J5 + VEGF) - k8dash*VEGF*oxygenConcentration/(CVEGF + oxygenConcentration);

    rDY[0] = dphi;
    rDY[1] = dp53;
    rDY[2] = dVEGF;
    rDY[3] = 0.0; // oxygen concentration will not change


    // Rescale time to be in hours
    rDY[0] = 60.0*dphi;
    rDY[1] = 60.0*dp53;
    rDY[2] = 60.0*dVEGF;
    rDY[3] = 0.0; // do not change the oxygen concentration
}

bool Owen2011OxygenBasedCellCycleOdeSystem::CalculateStoppingEvent(double time, const std::vector<double>& rY)
{
    return (rY[0] > 1);
}

template<>
void CellwiseOdeSystemInformation<Owen2011OxygenBasedCellCycleOdeSystem>::Initialise()
{
    this->mVariableNames.push_back("Cell_cycle_phase");
    this->mVariableUnits.push_back("unitless");
    this->mInitialConditions.push_back(0.0);

    this->mVariableNames.push_back("p53");
    this->mVariableUnits.push_back("unitless");
    this->mInitialConditions.push_back(0.0);

    this->mVariableNames.push_back("VEGF");
    this->mVariableUnits.push_back("unitless");
    this->mInitialConditions.push_back(0.0);

    this->mVariableNames.push_back("Oxygen");
    this->mVariableUnits.push_back("mmHg");
    this->mInitialConditions.push_back(20.0);

    this->mInitialised = true;
}

double Owen2011OxygenBasedCellCycleOdeSystem::GetOxygenConcentration() const
{
    return oxygenConcentration;
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(Owen2011OxygenBasedCellCycleOdeSystem)
