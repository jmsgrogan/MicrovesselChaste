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

#include "RadiusCalculator.hpp"
#include "Owen11Parameters.hpp"

template<unsigned DIM>
RadiusCalculator<DIM>::RadiusCalculator() : AbstractVesselNetworkCalculator<DIM>(),
        mMinRadius(Owen11Parameters::mpMinimumRadius->GetValue("RadiusCalculator")),
        mMaxRadius(Owen11Parameters::mpMaximumRadius->GetValue("RadiusCalculator")),
        mTimeStep(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("RadiusCalculator"))
{

}

template<unsigned DIM>
RadiusCalculator<DIM>::~RadiusCalculator()
{

}

template<unsigned DIM>
void RadiusCalculator<DIM>::SetMinRadius(units::quantity<unit::length> minRadius)
{
    mMinRadius = minRadius;
}

template<unsigned DIM>
void RadiusCalculator<DIM>::SetMaxRadius(units::quantity<unit::length> maxRadius)
{
    mMaxRadius = maxRadius;
}

template<unsigned DIM>
void RadiusCalculator<DIM>::SetTimestep(units::quantity<unit::time>  dt)
{
    mTimeStep = dt;
}

template<unsigned DIM>
void RadiusCalculator<DIM>::Calculate()
{
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
    for (unsigned segment_index = 0; segment_index < segments.size(); segment_index++)
    {
        units::quantity<unit::rate> total_stimulus = segments[segment_index]->GetFlowProperties()->GetGrowthStimulus();
        units::quantity<unit::length> radius = segments[segment_index]->GetRadius();
        radius = radius*(1.0 + mTimeStep * total_stimulus);

        // Only change the radius if there is a stimulus, so we can use this calculator without stimulii
        if(mTimeStep * total_stimulus!=0.0)
        {
            if (radius > mMaxRadius)
            {
                radius = mMaxRadius;
            }
            if (radius < mMinRadius)
            {
                radius = mMinRadius;
            }
        }
        segments[segment_index]->SetRadius(radius);
    }
}

// Explicit instantiation
template class RadiusCalculator<2> ;
template class RadiusCalculator<3> ;
