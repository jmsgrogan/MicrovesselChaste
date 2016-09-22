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
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumLinearEllipticPde() :
            AbstractLinearEllipticPde<ELEMENT_DIM, ELEMENT_DIM>(),
            mDiffusionTensor(identity_matrix<double>(SPACE_DIM)),
            mDiffusivity(1.0 * unit::metre_squared_per_second),
            mConstantInUTerm(0.0 * unit::mole_per_metre_cubed_per_second),
            mDiscreteSources(),
            mpRegularGrid(),
            mpMesh(),
            mUseRegularGrid(true),
            mDiscreteConstantSourceStrengths(),
            mReferenceConcentration(1.e-9 * unit::mole_per_metre_cubed)
{
    mDiffusionTensor *= mDiffusivity.value();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumLinearEllipticPde()
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::AddDiscreteSource(boost::shared_ptr<DiscreteSource<SPACE_DIM> > pDiscreteSource)
{
    mDiscreteSources.push_back(pDiscreteSource);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeConstantInUSourceTerm(const ChastePoint<SPACE_DIM>& rX,
                                                                                                        Element<ELEMENT_DIM, SPACE_DIM>* pElement)
{
    if(mDiscreteConstantSourceStrengths.size()>0)
    {
        if(pElement->GetIndex() >= mDiscreteConstantSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        units::quantity<unit::concentration_flow_rate> scaling_factor = mReferenceConcentration/BaseUnits::Instance()->GetReferenceTimeScale();
        return (mConstantInUTerm + mDiscreteConstantSourceStrengths[pElement->GetIndex()]) / scaling_factor;
    }
    else
    {
        units::quantity<unit::concentration_flow_rate> scaling_factor = mReferenceConcentration/BaseUnits::Instance()->GetReferenceTimeScale();
        return mConstantInUTerm/scaling_factor;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeConstantInUSourceTerm(unsigned gridIndex)
{
    if(mDiscreteConstantSourceStrengths.size()>0)
    {
        if(gridIndex >= mDiscreteConstantSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        return mConstantInUTerm + mDiscreteConstantSourceStrengths[gridIndex];
    }
    else
    {
        return mConstantInUTerm;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
c_matrix<double, SPACE_DIM, SPACE_DIM> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>&)
{
    return mDiffusionTensor;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::diffusivity> AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeIsotropicDiffusionTerm()
{
    return mDiffusivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<boost::shared_ptr<DiscreteSource<SPACE_DIM> > > AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::GetDiscreteSources()
{
    return mDiscreteSources;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumConstantInUTerm(units::quantity<unit::concentration_flow_rate> constantInUTerm)
{
    mConstantInUTerm = constantInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetIsotropicDiffusionConstant(units::quantity<unit::diffusivity> diffusivity)
{
    mDiffusivity = diffusivity;
    units::quantity<unit::diffusivity> scaling_factor = (BaseUnits::Instance()->GetReferenceLengthScale()*BaseUnits::Instance()->GetReferenceLengthScale())/BaseUnits::Instance()->GetReferenceTimeScale();
    mDiffusionTensor = identity_matrix<double>(SPACE_DIM)*(mDiffusivity/scaling_factor);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<ELEMENT_DIM, SPACE_DIM> > pRegularGrid)
{
    mpRegularGrid = pRegularGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > pMesh)
{
    mpMesh = pMesh;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetUseRegularGrid(bool useRegularGrid)
{
    mUseRegularGrid = useRegularGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration)
{
    mReferenceConcentration = referenceConcentration;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    if(mUseRegularGrid)
    {
        if(!mpRegularGrid)
        {
            EXCEPTION("A grid has not been set for the determination of source strengths.");
        }
        mDiscreteConstantSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(mpRegularGrid->GetNumberOfPoints(), 0.0*unit::mole_per_metre_cubed_per_second);
        for(unsigned idx=0; idx<mDiscreteSources.size(); idx++)
        {
            mDiscreteSources[idx]->SetRegularGrid(mpRegularGrid);
            std::vector<units::quantity<unit::concentration_flow_rate> > result = mDiscreteSources[idx]->GetConstantInURegularGridValues();
            std::transform(mDiscreteConstantSourceStrengths.begin( ), mDiscreteConstantSourceStrengths.end( ),
                           result.begin( ), mDiscreteConstantSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
        }
    }
    else
    {
        if(!mpMesh)
        {
            EXCEPTION("A mesh has not been set for the determination of source strengths.");
        }

        mDiscreteConstantSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(mpMesh->GetNumElements(), 0.0*unit::mole_per_metre_cubed_per_second);
        for(unsigned idx=0; idx<mDiscreteSources.size(); idx++)
        {
            mDiscreteSources[idx]->SetMesh(mpMesh);
            std::vector<units::quantity<unit::concentration_flow_rate> > result = mDiscreteSources[idx]->GetConstantInUMeshValues();
            std::transform(mDiscreteConstantSourceStrengths.begin( ), mDiscreteConstantSourceStrengths.end( ),
                           result.begin( ), mDiscreteConstantSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
        }
    }
}

// Explicit instantiation
template class AbstractDiscreteContinuumLinearEllipticPde<2>;
template class AbstractDiscreteContinuumLinearEllipticPde<3>;
