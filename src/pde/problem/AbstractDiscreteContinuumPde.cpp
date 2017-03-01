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

#include <algorithm>
#include "AbstractDiscreteContinuumPde.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumPde() :
            mDiffusivity(1.0 * unit::metre_squared_per_second),
            mConstantInUTerm(0.0 * unit::mole_per_metre_cubed_per_second),
            mLinearInUTerm(0.0 * unit::per_second),
            mDiscreteSources(),
            mpGridCalculator(),
            mDiscreteConstantSourceStrengths(),
            mDiscreteLinearSourceStrengths(),
            mReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale())
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::AddDiscreteSource(boost::shared_ptr<DiscreteSource<SPACE_DIM> > pDiscreteSource)
{
    mDiscreteSources.push_back(pDiscreteSource);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::diffusivity> AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::ComputeIsotropicDiffusionTerm()
{
    return mDiffusivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<boost::shared_ptr<DiscreteSource<SPACE_DIM> > > AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::GetDiscreteSources()
{
    return mDiscreteSources;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiscreteConstantInUSourceTerm(unsigned gridIndex)
{
    return 0.0*unit::mole_per_metre_cubed_per_second;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned gridIndex)
{
    return 0.0 * unit::per_second;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumConstantInUTerm(units::quantity<unit::concentration_flow_rate> constantInUTerm)
{
    mConstantInUTerm = constantInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm)
{
    mLinearInUTerm = linearInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::SetIsotropicDiffusionConstant(units::quantity<unit::diffusivity> diffusivity)
{
    mDiffusivity = diffusivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::SetGridCalculator(boost::shared_ptr<GridCalculator<SPACE_DIM> > pRegularGrid)
{
    mpGridCalculator = pRegularGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration)
{
    mReferenceConcentration = referenceConcentration;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    if(!mpGridCalculator)
    {
        EXCEPTION("A grid has not been set for the determination of source strengths.");
    }
    mDiscreteConstantSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(mpGridCalculator->GetNumberOfLocations(),
            0.0*unit::mole_per_metre_cubed_per_second);
    mDiscreteLinearSourceStrengths = std::vector<units::quantity<unit::rate> >(mpGridCalculator->GetNumberOfLocations(), 0.0*unit::per_second);

    for(unsigned idx=0; idx<mDiscreteSources.size(); idx++)
    {
        mDiscreteSources[idx]->SetGridCalculator(mpGridCalculator);
        std::vector<units::quantity<unit::rate> > result = mDiscreteSources[idx]->GetLinearInUValues();
        std::transform(mDiscreteLinearSourceStrengths.begin( ), mDiscreteLinearSourceStrengths.end( ),
                       result.begin( ), mDiscreteLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::rate> >( ));

        std::vector<units::quantity<unit::concentration_flow_rate> > result2 = mDiscreteSources[idx]->GetConstantInUValues();
        std::transform(mDiscreteConstantSourceStrengths.begin( ), mDiscreteConstantSourceStrengths.end( ),
                       result2.begin( ), mDiscreteConstantSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
    }
}

// Explicit instantiation
template class AbstractDiscreteContinuumPde<2>;
template class AbstractDiscreteContinuumPde<3>;
