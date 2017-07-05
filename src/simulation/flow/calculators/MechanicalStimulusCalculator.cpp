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



#include "Owen11Parameters.hpp"
#include "MechanicalStimulusCalculator.hpp"

template<unsigned DIM>
MechanicalStimulusCalculator<DIM>::MechanicalStimulusCalculator() : AbstractVesselNetworkCalculator<DIM>(),
    mTauRef(1.e-6 * unit::pascals),
    mTauP(0.05 * unit::pascals),
    mkp(Owen11Parameters::mpSensitivityToIntravascularPressure->GetValue("MechanicalStimulusCalculator"))
{

}

template<unsigned DIM>
MechanicalStimulusCalculator<DIM>::~MechanicalStimulusCalculator()
{

}

template <unsigned DIM>
std::shared_ptr<MechanicalStimulusCalculator<DIM> > MechanicalStimulusCalculator<DIM>::Create()
{
    return std::make_shared<MechanicalStimulusCalculator<DIM> >();
}

template<unsigned DIM>
units::quantity<unit::pressure> MechanicalStimulusCalculator<DIM>::GetTauP()
{
    return mTauP;
}

template<unsigned DIM>
units::quantity<unit::pressure> MechanicalStimulusCalculator<DIM>::GetTauReference()
{
    return mTauRef;
}

template<unsigned DIM>
void MechanicalStimulusCalculator<DIM>::SetTauRef(units::quantity<unit::pressure> TauRef)
{
    mTauRef = TauRef;
}

template<unsigned DIM>
void MechanicalStimulusCalculator<DIM>::SetTauP(units::quantity<unit::pressure> TauP)
{
    mTauP = TauP;
}

template<unsigned DIM>
void MechanicalStimulusCalculator<DIM>::Calculate()
{
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();

    QLength cm(0.01*unit::metres);
    units::quantity<unit::mass> g(1.e-3*unit::kg);
    units::quantity<unit::force> dyne(g*cm/(unit::seconds*unit::seconds));
    units::quantity<unit::pressure> dyne_per_centi_metre_squared(dyne/(cm*cm));
    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        // Get average pressure in a segment
        units::quantity<unit::pressure> node0_pressure = segments[idx]->GetNode(0)->GetFlowProperties()->GetPressure();
        units::quantity<unit::pressure> node1_pressure = segments[idx]->GetNode(1)->GetFlowProperties()->GetPressure();

        // Empirical Equation. Pressure in mmHg, WSS in dyne/cm2.
        units::quantity<unit::pressure> conversion_pressure(1.0*unit::mmHg);
        double average_pressure_in_mmHg = (node0_pressure + node1_pressure)/(2.0*conversion_pressure);

        // The calculation does not work for low pressures, so we need to specify a cut-off value.
        if (log10(average_pressure_in_mmHg) < 1.0)
        {
            mTauP = 1.4 * dyne_per_centi_metre_squared;
        }
        else
        {
            double inside_exponent = -5000.0*pow(log10(log10(average_pressure_in_mmHg)), 5.4);
            mTauP = (100.0 - 86.0 * exp(inside_exponent)) * dyne_per_centi_metre_squared;
        }

        // The equation is dimensionally inconsistent. Drop out of the units framework.
        double log_term_1 = log10((segments[idx]->GetFlowProperties()->GetWallShearStress()/dyne_per_centi_metre_squared + mTauRef/dyne_per_centi_metre_squared));
        double log_term_2 = log10(mTauP/dyne_per_centi_metre_squared);

        units::quantity<unit::rate> mechanical_stimulus = (log_term_1 - 0.5*log_term_2) * unit::per_second;
        segments[idx]->GetFlowProperties()->SetGrowthStimulus(segments[idx]->GetFlowProperties()->GetGrowthStimulus() + mechanical_stimulus);
    }
}

// Explicit instantiation
template class MechanicalStimulusCalculator<2> ;
template class MechanicalStimulusCalculator<3> ;
