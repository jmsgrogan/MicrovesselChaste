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



#include "math.h"
#include "WallShearStressCalculator.hpp"

template<unsigned DIM>
WallShearStressCalculator<DIM>::WallShearStressCalculator() : AbstractVesselNetworkCalculator<DIM>()
{

}

template<unsigned DIM>
WallShearStressCalculator<DIM>::~WallShearStressCalculator()
{

}

template <unsigned DIM>
std::shared_ptr<WallShearStressCalculator<DIM> > WallShearStressCalculator<DIM>::Create()
{
    return std::make_shared<WallShearStressCalculator<DIM> >();

}

template<unsigned DIM>
void WallShearStressCalculator<DIM>::Calculate()
{
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
    for (unsigned segment_index = 0; segment_index < segments.size(); segment_index++)
    {
        QFlowRate flow_rate =
                Qabs(segments[segment_index]->GetFlowProperties()->GetFlowRate());
        QDynamicViscosity viscosity =
                segments[segment_index]->GetFlowProperties()->GetViscosity();
        QPressure wall_shear_stress = 8.0 * viscosity * flow_rate / (M_PI * Qpow3(segments[segment_index]->GetRadius()));
        segments[segment_index]->GetFlowProperties()->SetWallShearStress(wall_shear_stress);
    }
}

// Explicit instantiation
template class WallShearStressCalculator<2> ;
template class WallShearStressCalculator<3> ;
