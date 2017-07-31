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
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "BaseUnits.hpp"
#include "Connor17Parameters.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::CoupledVegfPelletDiffusionReactionPde() :
            AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, ELEMENT_DIM>(),
            mLinearInUTerm(0.0 * unit::per_second),
            mDiscreteNonLinearSourceStrengths(),
            mPelletFreeDecayRate(Connor17Parameters::mpVegfDecayRateInPellet->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mPelletVegfBindingConstant(Connor17Parameters::mpVegfBindingConstant->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mInitialVegfInPellet(Connor17Parameters::mpInitialVegfConcentrationInPellet->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mCurrentVegfInPellet(Connor17Parameters::mpInitialVegfConcentrationInPellet->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mCorneaPelletPermeability(Connor17Parameters::mpCorneaVegfPermeability->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mPelletSurfaceArea(Connor17Parameters::mpPelletSurfaceArea->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mPelletDepth(1_um), // for consistency with some existing tests
            mPelletVolume(Connor17Parameters::mpPelletVolume->GetValue("CoupledVegfPelletDiffusionReactionPde")),
            mHalfMaxVegf(Connor17Parameters::mpVegfAtHalfReceptorOccupancy->GetValue("CoupledVegfPelletDiffusionReactionPde"))
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::~CoupledVegfPelletDiffusionReactionPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::Create()
{
    return std::make_shared<CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM> >();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QRate  CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletFreeDecayRate()
{
    return mPelletFreeDecayRate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QDimensionless  CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletBindingConstant()
{
    return mPelletVegfBindingConstant;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QConcentration CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetInitialVegfInPellet()
{
    return mInitialVegfInPellet;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QConcentration CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetCurrentVegfInPellet()
{
    return mCurrentVegfInPellet;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QMembranePermeability CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetCorneaPelletPermeability()
{
    return mCorneaPelletPermeability;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QLength CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletDepth()
{
    return mPelletDepth;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QArea CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletSurfaceArea()
{
    return mPelletSurfaceArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QVolume CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::GetPelletVolume()
{
    return mPelletVolume;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletFreeDecayRate(QRate rate)
{
    mPelletFreeDecayRate = rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletBindingConstant(QDimensionless bindingConstant)
{
    mPelletVegfBindingConstant = bindingConstant;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetInitialVegfInPellet(QConcentration initialVegf)
{
    mInitialVegfInPellet = initialVegf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetCurrentVegfInPellet(QConcentration currentVegf)
{
    mCurrentVegfInPellet = currentVegf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetCorneaPelletPermeability(QMembranePermeability permeability)
{
    mCorneaPelletPermeability = permeability;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletSurfaceArea(QArea surfaceArea)
{
    mPelletSurfaceArea = surfaceArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletVolume(QVolume volume)
{
    mPelletVolume = volume;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetPelletDepth(QLength depth)
{
    mPelletDepth = depth;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetHalfMaxVegfConcentration(QConcentration halfMax)
{
    mHalfMaxVegf = halfMax;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::SetContinuumLinearInUTerm(QRate linearInUTerm)
{
    mLinearInUTerm = linearInUTerm;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
double CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTerm(const ChastePoint<SPACE_DIM>& rX,
        double u, Element<ELEMENT_DIM,SPACE_DIM>* pElement)
{
    QRate scaling_factor = (1.0/BaseUnits::Instance()->GetReferenceTimeScale());
    QConcentration reference_concentration = BaseUnits::Instance()->GetReferenceConcentrationScale();
    double normalized_half_max_conc = mHalfMaxVegf/reference_concentration;

    double rate = this->mLinearInUTerm/scaling_factor*u;
    if(this->mDiscreteNonLinearSourceStrengths.size()>0)
    {
        if(pElement->GetIndex() >= this->mDiscreteNonLinearSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }

        double extra_source = this->mDiscreteNonLinearSourceStrengths[pElement->GetIndex()]*(u/(u+normalized_half_max_conc))/(scaling_factor*BaseUnits::Instance()->GetReferenceConcentrationScale());
        rate = rate + extra_source;
    }
    if(u<=0.0)
    {
        rate=0.0;
    }
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QConcentrationFlowRate CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTerm(unsigned gridIndex,
                                                                                                                                                  QConcentration u)
{
    QConcentrationFlowRate rate = this->mLinearInUTerm*u;
    if(this->mDiscreteNonLinearSourceStrengths.size()>0)
    {
        rate = rate + this->mDiscreteNonLinearSourceStrengths[gridIndex]*(u/(u+mHalfMaxVegf));
    }
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QRate CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::ComputeSourceTermPrime(unsigned gridIndex,
                                                                                                                                    QConcentration u)
{
    QRate rate = this->mLinearInUTerm;
    if(this->mDiscreteNonLinearSourceStrengths.size()>0)
    {
        rate = rate + this->mDiscreteNonLinearSourceStrengths[gridIndex]*(mHalfMaxVegf/((u+mHalfMaxVegf)*(u+mHalfMaxVegf)));
    }
    return rate;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths()
{
    AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>::UpdateDiscreteSourceStrengths();
    if(this->mDiscreteSources.size()>0)
    {
        unsigned num_locations = this->mDiscreteSources[0]->GetDensityMap()->GetGridCalculator()->GetGrid()->GetNumberOfCells();
        mDiscreteNonLinearSourceStrengths = std::vector<QConcentrationFlowRate>(num_locations,
                0.0*unit::mole_per_metre_cubed_per_second);
        for(auto& source:this->mDiscreteSources)
        {
            std::vector<QConcentrationFlowRate> result = source->GetNonlinearTermValues();
            std::transform(mDiscreteNonLinearSourceStrengths.begin( ), mDiscreteNonLinearSourceStrengths.end( ),
                           result.begin( ), mDiscreteNonLinearSourceStrengths.begin( ), std::plus<QConcentrationFlowRate>( ));
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
