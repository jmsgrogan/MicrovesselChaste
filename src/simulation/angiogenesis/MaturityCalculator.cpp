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

#include "BaseUnits.hpp"
#include "MaturityCalculator.hpp"

template<unsigned DIM>
MaturityCalculator<DIM>::MaturityCalculator() :
    mpNetwork(),
    mpSolver(),
    mHalfMaxVegf(6.5e-10*unit::mole_per_metre_cubed),
    mMaxRateOfMaturation(0.0005*unit::per_hour),
    mMaxRateOfDematuration(0.08*unit::per_hour)
{

}

template<unsigned DIM>
MaturityCalculator<DIM>::~MaturityCalculator()
{

}

template<unsigned DIM>
void MaturityCalculator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void MaturityCalculator<DIM>::SetVegfSolver(boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver)
{
    mpSolver = pSolver;
}

template<unsigned DIM>
void MaturityCalculator<DIM>::Update()
{
    if(!mpNetwork)
    {
        EXCEPTION("A vessel network is needed for the maturity calculator.");
    }

    if(!mpSolver)
    {
        EXCEPTION("A solver with a vegf field is needed for the maturity calculator.");
    }

    std::vector<DimensionalChastePoint<DIM> > segment_midpoints;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segment_midpoints.push_back(segments[idx]->GetMidPoint());
    }

    std::vector<units::quantity<unit::concentration> > vegf_values = mpSolver->GetConcentrations(segment_midpoints);
    units::quantity<unit::time> time_increment =
            SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();

    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        units::quantity<unit::rate> a_term = mMaxRateOfDematuration*vegf_values[idx]/(mHalfMaxVegf + vegf_values[idx]);
        a_term*=segments[idx]->GetMaturity();
        units::quantity<unit::rate> b_term = mMaxRateOfMaturation*mHalfMaxVegf/(mHalfMaxVegf + vegf_values[idx]);
        b_term*=(1.0-segments[idx]->GetMaturity());

        double exponent = time_increment*(-a_term-b_term);
        double maturity = (segments[idx]->GetMaturity()-(b_term/(a_term+b_term)))*exp(exponent)+(b_term/(a_term+b_term));
        if(maturity>1.0)
        {
            maturity = 1.0;
        }
        segments[idx]->SetMaturity(maturity);
    }
}

// Explicit instantiation
template class MaturityCalculator<2>;
template class MaturityCalculator<3>;
