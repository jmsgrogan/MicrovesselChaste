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

#include "SegmentFlowProperties.hpp"

template<unsigned DIM>
SegmentFlowProperties<DIM>::SegmentFlowProperties() : AbstractVesselNetworkComponentFlowProperties<DIM>(),
    mHaematocrit(0.0),
    mFlowRate(0.0*unit::metre_cubed_per_second),
    mImpedance(0.0*unit::pascal_second_per_metre_cubed),
    mViscosity(0.0*unit::poiseuille),
    mWallShearStress(0.0*unit::pascals),
    mStimulus(0.0*unit::per_second)
{
}

template<unsigned DIM>
SegmentFlowProperties<DIM>::~SegmentFlowProperties()
{
}

template<unsigned DIM>
units::quantity<unit::dimensionless> SegmentFlowProperties<DIM>::GetHaematocrit() const
{
    return mHaematocrit;
}

template<unsigned DIM>
units::quantity<unit::flow_rate> SegmentFlowProperties<DIM>::GetFlowRate() const
{
    return mFlowRate;
}

template<unsigned DIM>
units::quantity<unit::flow_impedance> SegmentFlowProperties<DIM>::GetImpedance() const
{
    return mImpedance;
}

template<unsigned DIM>
units::quantity<unit::dynamic_viscosity> SegmentFlowProperties<DIM>::GetViscosity() const
{
    return mViscosity;
}

template<unsigned DIM>
units::quantity<unit::pressure> SegmentFlowProperties<DIM>::GetWallShearStress() const
{
    return mWallShearStress;
}

template<unsigned DIM>
units::quantity<unit::rate> SegmentFlowProperties<DIM>::GetGrowthStimulus() const
{
    return mStimulus;
}

template<unsigned DIM>
std::map<std::string, double> SegmentFlowProperties<DIM>::GetOutputData() const
{
    std::map<std::string, double> output_data;
    output_data["Segment Haematocrit"] = this->GetHaematocrit();
    output_data["Segment Flow Rate m^3/s"] = this->GetFlowRate() / unit::metre_cubed_per_second;
    output_data["Segment Impedance kg/m^4/s"] = this->GetImpedance() / unit::pascal_second_per_metre_cubed;
    output_data["Segment Viscosity Pa.s"] = this->GetViscosity() / unit::poiseuille;
    output_data["Segment Wall Shear Stress Pa"] = this->GetWallShearStress() / unit::pascals;
    output_data["Segment Growth Stimulus s^-1"] = this->GetGrowthStimulus() / unit::per_second;
    return output_data;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetHaematocrit(units::quantity<unit::dimensionless> haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetFlowRate(units::quantity<unit::flow_rate> flowRate)
{
    mFlowRate = flowRate;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetImpedance(units::quantity<unit::flow_impedance> impedance)
{
    mImpedance = impedance;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetViscosity(units::quantity<unit::dynamic_viscosity> viscosity)
{
    mViscosity = viscosity;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetWallShearStress(units::quantity<unit::pressure> value)
{
    mWallShearStress = value;
}

template<unsigned DIM>
void SegmentFlowProperties<DIM>::SetGrowthStimulus(units::quantity<unit::rate> value)
{
    mStimulus = value;
}

// Explicit instantiation
template class SegmentFlowProperties<2>;
template class SegmentFlowProperties<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(SegmentFlowProperties, 2)
EXPORT_TEMPLATE_CLASS1(SegmentFlowProperties, 3)
