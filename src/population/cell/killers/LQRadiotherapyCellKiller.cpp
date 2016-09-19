/*

Copyright (c) 2005-2015, University of Oxford.
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

#include "LQRadiotherapyCellKiller.hpp"
#include "CancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"

template<unsigned DIM>
LQRadiotherapyCellKiller<DIM>::LQRadiotherapyCellKiller(AbstractCellPopulation<DIM>* pCellPopulation)
: AbstractCellKiller<DIM>(pCellPopulation),
  cancerousLinearRadiosensitivity(0.3),
  cancerousQuadraticRadiosensitivity(0.03),
  normalLinearRadiosensitivity(0.15),
  normalQuadraticRadiosensitivity(0.05),
  mDose(2.0),
  mRadiationTimes(),
  mOerAlphaMax(1.75),
  mOerAlphaMin(1.0),
  mOerBetaMax(3.25),
  mOerBetaMin(1.0),
  mKOer(3.28),
  mAlphaMax(0.3),
  mBetaMax(0.03),
  mUseOer(false)
  {

  }


template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetDoseInjected(double d)
{
    mDose = d;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetTimeOfRadiation(std::vector<double> t)
{
    mRadiationTimes = t;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetOerAlphaMax(double value)
{
    mOerAlphaMax = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetOerAlphaMin(double value)
{
    mOerAlphaMin = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetOerBetaMax(double value)
{
    mOerBetaMax = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetOerBetaMin(double value)
{
    mOerBetaMin = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetOerConstant(double value)
{
    mKOer = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetAlphaMax(double value)
{
    mAlphaMax = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetBetaMax(double value)
{
    mBetaMax = value;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::UseOer(bool useOer)
{
    mUseOer = useOer;
}


template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetCancerousRadiosensitivity(double alpha, double beta)
{
	cancerousLinearRadiosensitivity = alpha;
	cancerousQuadraticRadiosensitivity = beta;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::SetNormalRadiosensitivity(double alpha, double beta)
{
	normalLinearRadiosensitivity = alpha;
	normalQuadraticRadiosensitivity = beta;
}

template<unsigned DIM>
double LQRadiotherapyCellKiller<DIM>::GetNormalLinearRadiosensitivity()
{
	return normalLinearRadiosensitivity;
}

template<unsigned DIM>
double LQRadiotherapyCellKiller<DIM>::GetNormalQuadraticRadiosensitivity()
{
	return normalQuadraticRadiosensitivity;
}

template<unsigned DIM>
double LQRadiotherapyCellKiller<DIM>::GetCancerousLinearRadiosensitivity()
{
	return cancerousLinearRadiosensitivity;
}

template<unsigned DIM>
double LQRadiotherapyCellKiller<DIM>::GetCancerousQuadraticRadiosensitivity()
{
	return cancerousQuadraticRadiosensitivity;
}

template<unsigned DIM>
double LQRadiotherapyCellKiller<DIM>::GetDoseInjected()
{
	return mDose;
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::CheckAndLabelSingleCellForApoptosis(CellPtr pCell)
{
    if(!pCell->GetMutationState()->IsType<StalkCellMutationState>())
    {
        for(unsigned idx=0; idx<mRadiationTimes.size(); idx++)
        {
            if (SimulationTime::Instance()->GetTime() == mRadiationTimes[idx])
            {
                // Model radiation hit
                if (mUseOer)
                {
                    double oxygen = pCell->GetCellData()->GetItem("oxygen");
                    double oer_alpha = (mOerAlphaMax - mOerAlphaMin)*mKOer/(oxygen + mKOer) + mOerAlphaMin;
                    double oer_beta = (mOerBetaMax - mOerBetaMin)*mKOer/(oxygen + mKOer) + mOerBetaMin;
                    double alpha = mAlphaMax/oer_alpha;
                    double beta = mBetaMax/(oer_beta*oer_beta);
                    double death_prob = (1.0 - exp(-alpha*mDose -beta*mDose*mDose));

                    if (!pCell->HasApoptosisBegun() && RandomNumberGenerator::Instance()->ranf() < death_prob)
                    {
                        pCell->StartApoptosis();
                    }
                }
                else
                {
                    double death_prob = (1.0 - exp(-cancerousLinearRadiosensitivity*mDose -cancerousQuadraticRadiosensitivity*mDose*mDose));

                    if (!pCell->HasApoptosisBegun() && RandomNumberGenerator::Instance()->ranf() < death_prob)
                    {
                        pCell->StartApoptosis();
                    }
                }
            }
        }
    }

}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::CheckAndLabelCellsForApoptosisOrDeath()
{
	for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = this->mpCellPopulation->Begin();
			cell_iter != this->mpCellPopulation->End();
			++cell_iter)
	{
		CheckAndLabelSingleCellForApoptosis(*cell_iter);
	}
}

template<unsigned DIM>
void LQRadiotherapyCellKiller<DIM>::OutputCellKillerParameters(out_stream& rParamsFile)
{
	*rParamsFile ;

	// Call method on direct parent class
	AbstractCellKiller<DIM>::OutputCellKillerParameters(rParamsFile);
}

/////////////////////////////////////////////////////////////////////////////
// Explicit instantiation
/////////////////////////////////////////////////////////////////////////////

template class LQRadiotherapyCellKiller<1>;
template class LQRadiotherapyCellKiller<2>;
template class LQRadiotherapyCellKiller<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(LQRadiotherapyCellKiller)
