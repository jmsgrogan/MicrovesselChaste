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
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::CoupledVegfPelletDiffusionReactionPde() :
            AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, ELEMENT_DIM>(),
            mLinearInUTerm(0.0 * unit::per_second),
            mDiscreteLinearSourceStrengths(),
            mDiscreteNonLinearSourceStrengths(),
            mPelletFreeDecayRate((0.8/3600.0)*unit::per_second),
            mPelletVegfBindingConstant(50.0),
            mInitialVegfInPellet(3.93e-4*unit::mole_per_metre_cubed),
            mCorneaPelletPermeability((1.67e-7/3600.0)*unit::metre_per_second),
            mPelletSurfaceArea(6.79e-7*unit::metres_squared),
            mPelletVolume(1.7e-11*unit::metres_cubed),
            mHalfMaxVegf(6.5e-10*unit::mole_per_metre_cubed)
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::~CoupledVegfPelletDiffusionReactionPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::Create()
{
    MAKE_PTR(CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM>, pSelf);
    return pSelf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate>  CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletFreeDecayRate()
{
    return mPelletFreeDecayRate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::dimensionless>  CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletBindingConstant()
{
    return mPelletVegfBindingConstant;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetInitialVegfInPellet()
{
    return mInitialVegfInPellet;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::membrane_permeability> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetCorneaPelletPermeability()
{
    return mCorneaPelletPermeability;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::area> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletSurfaceArea()
{
    return mPelletSurfaceArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::volume> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletVolume()
{
    return mPelletVolume;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletFreeDecayRate(units::quantity<unit::rate> rate)
{
    mPelletFreeDecayRate = rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletBindingConstant(units::quantity<unit::dimensionless> bindingConstant)
{
    mPelletVegfBindingConstant = bindingConstant;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetInitialVegfInPellet(units::quantity<unit::concentration> initialVegf)
{
    mInitialVegfInPellet = initialVegf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetCorneaPelletPermeability(units::quantity<unit::membrane_permeability> permeability)
{
    mCorneaPelletPermeability = permeability;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletSurfaceArea(units::quantity<unit::area> surfaceArea)
{
    mPelletSurfaceArea = surfaceArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletVolume(units::quantity<unit::volume> volume)
{
    mPelletVolume = volume;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetHalfMaxVegfConcentration(units::quantity<unit::concentration> halfMax)
{
    mHalfMaxVegf = halfMax;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearInUCoeffInSourceTerm(const ChastePoint<SPACE_DIM>& rX,
                                                                                                        Element<ELEMENT_DIM, SPACE_DIM>* pElement)
{
    if(mDiscreteLinearSourceStrengths.size()>0)
    {
        if(pElement->GetIndex() >= mDiscreteLinearSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        units::quantity<unit::rate> scaling_factor = (1.0/BaseUnits::Instance()->GetReferenceTimeScale());
        return (mLinearInUTerm + mDiscreteLinearSourceStrengths[pElement->GetIndex()])/scaling_factor;
    }
    else
    {
        units::quantity<unit::rate> scaling_factor = (1.0/BaseUnits::Instance()->GetReferenceTimeScale());
        return mLinearInUTerm/scaling_factor;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex)
{
    if(mDiscreteLinearSourceStrengths.size()>0)
    {
        if(gridIndex >= mDiscreteLinearSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        return mLinearInUTerm + mDiscreteLinearSourceStrengths[gridIndex];
    }
    else
    {
        return mLinearInUTerm;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm)
{
    mLinearInUTerm = linearInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTerm(unsigned gridIndex,
                                                                                                                                                  units::quantity<unit::concentration> u)
{
    units::quantity<unit::concentration_flow_rate> rate = mLinearInUTerm*u +
            this->mDiscreteConstantSourceStrengths[gridIndex] + this->mDiscreteLinearSourceStrengths[gridIndex]*u;
    rate+= this->mDiscreteNonLinearSourceStrengths[gridIndex]*(u/(u+mHalfMaxVegf));
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeNonlinearSourceTermPrime(unsigned gridIndex,
                                                                                                                                    units::quantity<unit::concentration> u)
{
    units::quantity<unit::rate> rate = mLinearInUTerm +
            this->mDiscreteLinearSourceStrengths[gridIndex];
    rate+= this->mDiscreteNonLinearSourceStrengths[gridIndex]*(mHalfMaxVegf/((u+mHalfMaxVegf)*(u+mHalfMaxVegf)));
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths();
    if(this->mUseRegularGrid)
    {
        if(!this->mpRegularGrid)
        {
            EXCEPTION("A grid has not been set for the determination of source strengths.");
        }
        mDiscreteLinearSourceStrengths = std::vector<units::quantity<unit::rate> >(this->mpRegularGrid->GetNumberOfPoints(), 0.0*unit::per_second);
        for(unsigned idx=0; idx<this->mDiscreteSources.size(); idx++)
        {
            this->mDiscreteSources[idx]->SetRegularGrid(this->mpRegularGrid);
            std::vector<units::quantity<unit::rate> > result = this->mDiscreteSources[idx]->GetLinearInURegularGridValues();
            std::transform(mDiscreteLinearSourceStrengths.begin( ), mDiscreteLinearSourceStrengths.end( ),
                           result.begin( ), mDiscreteLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::rate> >( ));
        }

        mDiscreteNonLinearSourceStrengths = std::vector<units::quantity<unit::concentration_flow_rate> >(this->mpRegularGrid->GetNumberOfPoints(),
                0.0*unit::mole_per_metre_cubed_per_second);
        for(unsigned idx=0; idx<this->mDiscreteSources.size(); idx++)
        {
            this->mDiscreteSources[idx]->SetRegularGrid(this->mpRegularGrid);
            std::vector<units::quantity<unit::concentration_flow_rate> > result = this->mDiscreteSources[idx]->GetNonlinearTermRegularGridValues();
            std::transform(mDiscreteNonLinearSourceStrengths.begin( ), mDiscreteNonLinearSourceStrengths.end( ),
                           result.begin( ), mDiscreteNonLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::concentration_flow_rate> >( ));
        }
    }
    else
    {
        if(!this->mpMesh)
        {
            EXCEPTION("A mesh has not been set for the determination of source strengths.");
        }
        mDiscreteLinearSourceStrengths = std::vector<units::quantity<unit::rate> >(this->mpMesh->GetNumElements(), 0.0*unit::per_second);
        for(unsigned idx=0; idx<this->mDiscreteSources.size(); idx++)
        {
            this->mDiscreteSources[idx]->SetMesh(this->mpMesh);
            std::vector<units::quantity<unit::rate> > result = this->mDiscreteSources[idx]->GetLinearInUMeshValues();
            std::transform(mDiscreteLinearSourceStrengths.begin( ), mDiscreteLinearSourceStrengths.end( ),
                           result.begin( ), mDiscreteLinearSourceStrengths.begin( ),std::plus<units::quantity<unit::rate> >( ));
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::UpdateMultiplierValue()
{

}

// Explicit instantiation
template class CoupledVegfPelletDiffusionReactionPde<2>;
template class CoupledVegfPelletDiffusionReactionPde<3>;
