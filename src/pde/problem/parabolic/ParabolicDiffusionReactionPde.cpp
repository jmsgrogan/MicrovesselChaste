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

#include <algorithm>
#include "ParabolicDiffusionReactionPde.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ParabolicDiffusionReactionPde() :
            AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, ELEMENT_DIM>(),
            mDiscreteNonLinearSourceStrengths(),
            mLinearInUTerm(0.0 * unit::per_second)
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::~ParabolicDiffusionReactionPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::Create()
{
    MAKE_PTR(ParabolicDiffusionReactionPde<ELEMENT_DIM>, pSelf);
    return pSelf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTerm(const ChastePoint<SPACE_DIM>& rX,
        double u, Element<ELEMENT_DIM,SPACE_DIM>* pElement)
{
//    if(this->mDiscreteLinearSourceStrengths.size()>0)
//    {
//        if(pElement->GetIndex() >= this->mDiscreteLinearSourceStrengths.size())
//        {
//            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
//        }
//        units::quantity<unit::rate> scaling_factor = (1.0/BaseUnits::Instance()->GetReferenceTimeScale());
//        return (this->mLinearInUTerm + this->mDiscreteLinearSourceStrengths[pElement->GetIndex()])/scaling_factor;
//    }
//    else
//    {
        units::quantity<unit::rate> scaling_factor = (1.0/BaseUnits::Instance()->GetReferenceTimeScale());
        return this->mLinearInUTerm/scaling_factor;
//    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTerm(unsigned gridIndex,
                                                                                                                                                  units::quantity<unit::concentration> u)
{
    units::quantity<unit::concentration_flow_rate> rate = this->mLinearInUTerm*u ;//+
            //this->mDiscreteConstantSourceStrengths[gridIndex] + this->mDiscreteLinearSourceStrengths[gridIndex]*u;
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTermPrime(unsigned gridIndex,
                                                                                                                                    units::quantity<unit::concentration> u)
{
    units::quantity<unit::rate> rate = this->mLinearInUTerm ;//+ this->mDiscreteLinearSourceStrengths[gridIndex];
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm)
{
    mLinearInUTerm = linearInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths();

    if(!this->mpGridCalculator)
    {
        EXCEPTION("A grid has not been set for the determination of source strengths.");
    }
    unsigned num_locations = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    mDiscreteNonLinearSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(num_locations,
            0.0*unit::mole_per_metre_cubed_per_second);
    for(unsigned idx=0; idx<this->mDiscreteSources.size(); idx++)
    {
        this->mDiscreteSources[idx]->SetGridCalculator(this->mpGridCalculator);
        std::vector<units::quantity<unit::concentration_flow_rate> > result = this->mDiscreteSources[idx]->GetNonlinearTermValues();
        std::transform(mDiscreteNonLinearSourceStrengths.begin( ), mDiscreteNonLinearSourceStrengths.end( ),
                       result.begin( ), mDiscreteNonLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
    }

}

// Explicit instantiation
template class ParabolicDiffusionReactionPde<2>;
template class ParabolicDiffusionReactionPde<3>;
