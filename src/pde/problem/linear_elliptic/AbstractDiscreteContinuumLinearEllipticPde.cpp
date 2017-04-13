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
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumLinearEllipticPde() :
            AbstractLinearEllipticPde<ELEMENT_DIM, ELEMENT_DIM>(),
            AbstractDiscreteContinuumPde<ELEMENT_DIM, ELEMENT_DIM>(),
            mConstantInUTerm(0.0 * unit::mole_per_metre_cubed_per_second),
            mLinearInUTerm(0.0 * unit::per_second),
            mDiscreteConstantSourceStrengths(),
            mDiscreteLinearSourceStrengths()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumLinearEllipticPde()
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
c_matrix<double, SPACE_DIM, SPACE_DIM> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>&)
{
    double dimensionless_diffusivity = this->mDiffusivity*this->mReferenceTimeScale/(this->mReferenceLengthScale*this->mReferenceLengthScale);
    return identity_matrix<double>(SPACE_DIM)*dimensionless_diffusivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiscreteConstantInUSourceTerm(unsigned gridIndex)
{
    return 0.0*unit::mole_per_metre_cubed_per_second;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned gridIndex)
{
    return 0.0 * unit::per_second;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumConstantInUTerm(units::quantity<unit::concentration_flow_rate> constantInUTerm)
{
    mConstantInUTerm = constantInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm)
{
    mLinearInUTerm = linearInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    if(this->mDiscreteSources.size()>0)
    {
        unsigned num_locations = this->mDiscreteSources[0]->GetNumberOfPoints();

        mDiscreteConstantSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(num_locations,
                0.0*unit::mole_per_metre_cubed_per_second);
        mDiscreteLinearSourceStrengths = std::vector<units::quantity<unit::rate> >(num_locations, 0.0*unit::per_second);
        for(unsigned idx=0; idx<this->mDiscreteSources.size(); idx++)
        {
            std::vector<units::quantity<unit::rate> > result = this->mDiscreteSources[idx]->GetLinearInUValues();
            std::transform(mDiscreteLinearSourceStrengths.begin( ), mDiscreteLinearSourceStrengths.end( ),
                           result.begin( ), mDiscreteLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::rate> >( ));

            std::vector<units::quantity<unit::concentration_flow_rate> > result2 = this->mDiscreteSources[idx]->GetConstantInUValues();
            std::transform(mDiscreteConstantSourceStrengths.begin( ), mDiscreteConstantSourceStrengths.end( ),
                           result2.begin( ), mDiscreteConstantSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
        }
    }
}

// Explicit instantiation
template class AbstractDiscreteContinuumLinearEllipticPde<2>;
template class AbstractDiscreteContinuumLinearEllipticPde<3>;
