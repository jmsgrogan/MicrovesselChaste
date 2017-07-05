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

#include "BaseUnits.hpp"
#include "GenericAntiAngiogenicTherapy.hpp"

template<unsigned DIM>
GenericAntiAngiogenicTherapy<DIM>::GenericAntiAngiogenicTherapy() :
        AbstractTherapy<DIM>(),
        mCurrentConcentration(0.0*unit::mole_per_metre_cubed),
        mConcentrationAtMaxEffect(1.0*unit::mole_per_metre_cubed),
        mRemovalRate((0.001/3600.0)*unit::per_second),
        mpRegressionSolver(),
        mpSproutingRule(),
        mPreviousTime(0.0*unit::seconds),
        mInitialWSS(0.0*unit::pascals),
        mInitialRegressionTime(0.0*unit::seconds),
        mInitialSproutingProbability(0.0*unit::per_second)
{

}

template<unsigned DIM>
GenericAntiAngiogenicTherapy<DIM>::~GenericAntiAngiogenicTherapy()
{

}

template<unsigned DIM>
void GenericAntiAngiogenicTherapy<DIM>::SetRegressionSolver(std::shared_ptr<WallShearStressBasedRegressionSolver<DIM> > regressionSolver)
{
    mpRegressionSolver = regressionSolver;
}

template<unsigned DIM>
void GenericAntiAngiogenicTherapy<DIM>::SetSproutingRule(std::shared_ptr<AbstractSproutingRule<DIM> > sproutingRule)
{
    mpSproutingRule = sproutingRule;
}

template<unsigned DIM>
void GenericAntiAngiogenicTherapy<DIM>::SetupSolve(std::string outputDirectory)
{
    if(mpRegressionSolver)
    {
        mInitialWSS = mpRegressionSolver->GetLowWallShearStressThreshold();
        mInitialRegressionTime = mpRegressionSolver->GetMaximumTimeWithLowWallShearStress();
    }
    if(mpSproutingRule)
    {
        mInitialSproutingProbability = mpSproutingRule->GetSproutingProbability();
    }
}

template<unsigned DIM>
void GenericAntiAngiogenicTherapy<DIM>::UpdateAtEndOfTimeStep()
{
    QTime current_time = SimulationTime::Instance()->GetTime()*BaseUnits::Instance()->GetReferenceTimeScale();
    for(unsigned idx=0; idx<this->mAdministrationTimes.size();idx++)
    {
        if(Qabs(current_time-this->mAdministrationTimes[idx])<30.0*60.0*unit::seconds)
        {
            mCurrentConcentration = mCurrentConcentration + this->mAdministrationDose;
        }
    }
    mCurrentConcentration = mCurrentConcentration - mCurrentConcentration*mRemovalRate*(current_time-mPreviousTime);
    if(mCurrentConcentration<0.0*unit::mole_per_metre_cubed)
    {
        mCurrentConcentration = 0.0*unit::mole_per_metre_cubed;
    }

    // Modify the sprouting probability and regression time and regression WSS
    double effect_fraction = 0.0;
    if(mCurrentConcentration<mConcentrationAtMaxEffect)
    {
        effect_fraction = 1.0 -  mCurrentConcentration/mConcentrationAtMaxEffect;
    }
    if(mpRegressionSolver)
    {
        mpRegressionSolver->SetLowWallShearStressThreshold(effect_fraction*mInitialWSS);
        mpRegressionSolver->SetMaximumTimeWithLowWallShearStress(effect_fraction*mInitialRegressionTime);
    }
    if(mpSproutingRule)
    {
        mpSproutingRule->SetSproutingProbability(mInitialSproutingProbability*effect_fraction);
    }
}

template class GenericAntiAngiogenicTherapy<2>;
template class GenericAntiAngiogenicTherapy<3>;
