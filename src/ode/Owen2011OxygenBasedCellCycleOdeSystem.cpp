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

#include "Owen2011OxygenBasedCellCycleOdeSystem.hpp"
#include "CellwiseOdeSystemInformation.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "BaseUnits.hpp"

Owen2011OxygenBasedCellCycleOdeSystem::Owen2011OxygenBasedCellCycleOdeSystem(units::quantity<unit::concentration> oxygenConcentration,
        boost::shared_ptr<AbstractCellMutationState> mutation_state,
        std::vector<double> stateVariables)
: AbstractOdeSystem(4),
  mOxygenConcentration(oxygenConcentration),
  pmMutationState(mutation_state),
  mReferenceTimeScale(BaseUnits::Instance()->GetReferenceTimeScale()),
  mReferenceConcentrationScale(BaseUnits::Instance()->GetReferenceConcentrationScale())
{
    mpSystemInfo.reset(new CellwiseOdeSystemInformation<Owen2011OxygenBasedCellCycleOdeSystem>);
    mpSystemInfo->SetDefaultInitialCondition(3, mOxygenConcentration/mReferenceConcentrationScale);

    mk7 = Owen11Parameters::mpP53ProductionRateConstant->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mk7dash = Owen11Parameters::mpP53MaxDegradationRate->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mCp53 = Owen11Parameters::mpOxygenTensionForHalfMaxP53Degradation->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mk8 = Owen11Parameters::mpCellVegfProductionRate->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mJ5 = Owen11Parameters::mpVegfEffectOnVegfProduction->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mk8dash = Owen11Parameters::mpMaxCellVegfProductionRate->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    mCVEGF = Owen11Parameters::mpOxygenTensionForHalfMaxVegfDegradation->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");

    // Parameter values are taken from the Owen (2011) paper
    if (pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<QuiescentCancerCellMutationState>()) // cancer cell
    {
        mCphi = Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateCancer->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
        mTmin = Owen11Parameters::mpMinimumCellCyclePeriodCancer->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
        mk8doubledash = Owen11Parameters::mpP53EffectOnVegfProduction->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    }
    else // all other cells
    {
        mCphi = Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateNormal->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
        mTmin = Owen11Parameters::mpMinimumCellCyclePeriodNormal->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
        mk8doubledash = Owen11Parameters::mpP53EffectOnVegfProduction->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
    }

    mReferenceSolubility =
            Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("Owen2011OxygenBasedCellCycleOdeSystem") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");

    /*
     % The variables are
     % 0. phi = Cell-cycle phase
     % 1. p53 = p53 concentration
     % 2. VEGF = VEGF concentration
     % 3. O2 = Oxygen concentration
     */
    if (stateVariables != std::vector<double>())
    {
        SetStateVariables(stateVariables);
    }
}

Owen2011OxygenBasedCellCycleOdeSystem::~Owen2011OxygenBasedCellCycleOdeSystem()
{

}

bool Owen2011OxygenBasedCellCycleOdeSystem::CalculateStoppingEvent(double time, const std::vector<double>& rY)
{
    return (rY[0] > 1);
}

void Owen2011OxygenBasedCellCycleOdeSystem::EvaluateYDerivatives(double time,
                                                                 const std::vector<double>& rY,
                                                                 std::vector<double>& rDY)
{
    /*
     % The variables are
     % 0. phi = Cell-cycle phase
     % 1. p53 = p53 concentration
     % 2. VEGF = VEGF concentration
     % 3. O2 = Oxygen concentration
     */
    double p53 = rY[1];
    double VEGF = rY[2];
    mOxygenConcentration = rY[3]*mReferenceConcentrationScale;

    units::quantity<unit::rate> dphi = 0.0* (1.0/ unit::seconds);
    units::quantity<unit::rate> dp53 = 0.0* (1.0/ unit::seconds);
    units::quantity<unit::rate> dVEGF = 0.0* (1.0/ unit::seconds);
    if (pmMutationState->IsType<CancerCellMutationState>() || pmMutationState->IsType<WildTypeCellMutationState>())
    {
        dphi = mOxygenConcentration/(mTmin*(mCphi*mReferenceSolubility + mOxygenConcentration));
    }
    else
    {
        dphi = 0.0 * (1.0/ unit::seconds);
    }

    dp53 = mk7 - mk7dash*p53*mOxygenConcentration/(mCp53*mReferenceSolubility + mOxygenConcentration);
    dVEGF = mk8 + mk8doubledash*p53*VEGF/(mJ5 + VEGF) - mk8dash*VEGF*mOxygenConcentration/(mCVEGF*mReferenceSolubility + mOxygenConcentration);

    // Non-dimensionalise wrt to the reference time scale
    rDY[0] = dphi*mReferenceTimeScale;
    rDY[1] = dp53*mReferenceTimeScale;
    rDY[2] = dVEGF*mReferenceTimeScale;
    rDY[3] = 0.0;
}

boost::shared_ptr<AbstractCellMutationState> Owen2011OxygenBasedCellCycleOdeSystem::GetMutationState() const
{
    return pmMutationState;
}

units::quantity<unit::concentration> Owen2011OxygenBasedCellCycleOdeSystem::GetOxygenConcentration() const
{
    return mOxygenConcentration;
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
    this->mVariableUnits.push_back("moles m^-3");
    this->mInitialConditions.push_back(0.0);
    this->mInitialised = true;
}

void Owen2011OxygenBasedCellCycleOdeSystem::SetMutationState(boost::shared_ptr<AbstractCellMutationState> pMutationState)
{
    pmMutationState = pMutationState;
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(Owen2011OxygenBasedCellCycleOdeSystem)
