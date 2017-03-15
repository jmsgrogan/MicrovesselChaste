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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#include "GeometryTools.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::MichaelisMentenSteadyStateDiffusionReactionPde() :
            AbstractDiscreteContinuumNonLinearEllipticPde<SPACE_DIM>(),
            mRateConstant(0.0 * unit::mole_per_metre_cubed_per_second),
            mMichaelisMentenThreshold(1.0*unit::mole_per_metre_cubed)
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::Create()
{
    MAKE_PTR(MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM>, pSelf);
    return pSelf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::~MichaelisMentenSteadyStateDiffusionReactionPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearSourceTerm(const ChastePoint<SPACE_DIM>& rX)
{
    return 0.0;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTerm(const ChastePoint<SPACE_DIM>& rX, double u)
{
    if(u<0.0)
    {
        return 0.0;
    }
    else
    {
        return (this->mRateConstant/unit::mole_per_metre_cubed_per_second) * u / (mMichaelisMentenThreshold/(unit::mole_per_metre_cubed) + u);
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTermPrime(const ChastePoint<SPACE_DIM>& rX, double u)
{
    if(u<0.0)
    {
        return (this->mRateConstant / mMichaelisMentenThreshold)*(1.0*unit::seconds);
    }
    else
    {
        units::quantity<unit::concentration> u_scaled = u*unit::mole_per_metre_cubed;
        return (this->mRateConstant * mMichaelisMentenThreshold)/((mMichaelisMentenThreshold + u_scaled)*(mMichaelisMentenThreshold + u_scaled))*(1.0*unit::seconds);
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTerm(unsigned gridIndex,
                                                                                                                                                  units::quantity<unit::concentration> u)
{
    if(u<0.0*unit::mole_per_metre_cubed)
    {
        return 0.0*unit::mole_per_metre_cubed_per_second; //this->mDiscreteConstantSourceStrengths[gridIndex];
    }
    else
    {
        return this->mRateConstant * u / (mMichaelisMentenThreshold + u) ;//+ this->mDiscreteConstantSourceStrengths[gridIndex] + this->mDiscreteLinearSourceStrengths[gridIndex]*u;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTermPrime(unsigned gridIndex,
                                                                                                                                    units::quantity<unit::concentration> u)
{
    if(u<0.0*unit::mole_per_metre_cubed)
    {
        return (this->mRateConstant / mMichaelisMentenThreshold) ;//+ this->mDiscreteLinearSourceStrengths[gridIndex];
    }
    else
    {
        return this->mRateConstant * mMichaelisMentenThreshold/((mMichaelisMentenThreshold + u)*(mMichaelisMentenThreshold + u));// + this->mDiscreteLinearSourceStrengths[gridIndex];
    }

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration> MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetMichaelisMentenThreshold()
{
    return mMichaelisMentenThreshold;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetRateConstant(units::quantity<unit::concentration_flow_rate> constantInUTerm)
{
    mRateConstant = constantInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetMichaelisMentenThreshold(units::quantity<unit::concentration> threshold)
{
    mMichaelisMentenThreshold = threshold;
}

// Explicit instantiation
template class MichaelisMentenSteadyStateDiffusionReactionPde<2>;
template class MichaelisMentenSteadyStateDiffusionReactionPde<3>;
