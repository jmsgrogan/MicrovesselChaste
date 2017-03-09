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



#include "WallShearStressBasedRegressionSolver.hpp"
#include "Owen11Parameters.hpp"
#include <vector>

template<unsigned DIM>
WallShearStressBasedRegressionSolver<DIM>::WallShearStressBasedRegressionSolver() :
    RegressionSolver<DIM>(),
    mThresholdWss(Owen11Parameters::mpCriticalWallShearStress->GetValue("WallShearStressBasedRegressionSolver")),
    mMaxTimeWithLowWss(Owen11Parameters::mpMaxTimeWithLowWallShearStress->GetValue("WallShearStressBasedRegressionSolver"))
{

}

template<unsigned DIM>
WallShearStressBasedRegressionSolver<DIM>::~WallShearStressBasedRegressionSolver()
{

}

template<unsigned DIM>
boost::shared_ptr<WallShearStressBasedRegressionSolver<DIM> > WallShearStressBasedRegressionSolver<DIM>::Create()
{
    MAKE_PTR(WallShearStressBasedRegressionSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void WallShearStressBasedRegressionSolver<DIM>::SetMaximumTimeWithLowWallShearStress(units::quantity<unit::time> time)
{
    mMaxTimeWithLowWss = time;
}

template<unsigned DIM>
void WallShearStressBasedRegressionSolver<DIM>::SetLowWallShearStressThreshold(units::quantity<unit::pressure> threshold)
{
    mThresholdWss = threshold;
}

template<unsigned DIM>
void WallShearStressBasedRegressionSolver<DIM>::Increment()
{
    if(!this->mpNetwork)
    {
        EXCEPTION("The regression solver needs an initial vessel network");
    }

    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = this->mpNetwork->GetVessels();
    for(unsigned idx=0;idx<vessels.size(); idx++)
    {
        // if wall shear stress of vessel is below threshold then start regression timer, unless it has already been started
        if (vessels[idx]->GetFlowProperties()->GetWallShearStress() < mThresholdWss)
        {
            if (!(vessels[idx]->GetFlowProperties()->HasRegressionTimerStarted()) && !(vessels[idx]->GetFlowProperties()->HasVesselRegressed(this->mReferenceTime)))
            {
                // increment time that the vessel has had low wall shear stress
                vessels[idx]->GetFlowProperties()->SetTimeUntilRegression(mMaxTimeWithLowWss, this->mReferenceTime);
            }
        }
        else // otherwise rescue vessel
        {
            // wall shear stress above threshold so vessel is not regressing
            vessels[idx]->GetFlowProperties()->ResetRegressionTimer();
        }
    }

    // iterate through all vessels and if regression flag is true then remove from the network
    for(unsigned idx=0;idx<vessels.size(); idx++)
    {
        if (vessels[idx]->GetFlowProperties()->HasVesselRegressed(this->mReferenceTime))
        {
            this->mpNetwork->RemoveVessel(vessels[idx], true);
        }
    }
}

// Explicit instantiation
template class WallShearStressBasedRegressionSolver<2>;
template class WallShearStressBasedRegressionSolver<3>;
