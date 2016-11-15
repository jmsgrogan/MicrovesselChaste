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
#include "UnitCollection.hpp"
#include "ParameterCollection.hpp"
#include "BaseParameterInstance.hpp"

//// Typdef in this namespace so that pyplusplus uses the nicer typedef'd name for the class
namespace pyplusplus{
namespace aliases{
typedef units::quantity<unit::dimensionless, double> DimensionlessQuantity;
typedef units::quantity<unit::plane_angle, double> AngleQuantity;
typedef units::quantity<unit::time, double> TimeQuantity;
typedef units::quantity<unit::rate, double> RateQuantity;
typedef units::quantity<unit::length, double> LengthQuantity;
typedef units::quantity<unit::area, double> AreaQuantity;
typedef units::quantity<unit::volume, double> VolumeQuantity;
typedef units::quantity<unit::per_length, double> PerLengthQuantity;
typedef units::quantity<unit::per_area, double> PerAreaQuantity;
typedef units::quantity<unit::mass, double> MassQuantity;
typedef units::quantity<unit::mass_flow_rate, double> MassFlowRateQuantity;
typedef units::quantity<unit::mass_flux, double> MassFluxQuantity;
typedef units::quantity<unit::amount, double> AmountQuantity;
typedef units::quantity<unit::molar_flow_rate, double> MolarFlowRateQuantity;
typedef units::quantity<unit::molar_flux, double> MolarFluxQuantity;
typedef units::quantity<unit::concentration, double> ConcentrationQuantity;
typedef units::quantity<unit::concentration_flux, double> ConcentrationFluxQuantity;
typedef units::quantity<unit::concentration_gradient, double> ConcentrationGradientQuantity;
typedef units::quantity<unit::rate_per_concentration, double> RatePerConcentrationQuantity;
typedef units::quantity<unit::molar_mass, double> MolarMassQuantity;
typedef units::quantity<unit::number_density, double> NumberDensityQuantity;
typedef units::quantity<unit::velocity, double> VelocityQuantity;
typedef units::quantity<unit::force, double> ForceQuantity;
typedef units::quantity<unit::pressure, double> PressureQuantity;
typedef units::quantity<unit::dynamic_viscosity, double> ViscosityQuantity;
typedef units::quantity<unit::flow_rate, double> FlowRateQuantity;
typedef units::quantity<unit::flow_impedance, double> ImpedanceQuantity;
typedef units::quantity<unit::diffusivity, double> DiffusivityQuantity;
typedef units::quantity<unit::diffusivity_per_concentration, double> DiffusivityPerConcentrationQuantity;
typedef units::quantity<unit::solubility, double> SolubilityQuantity;
typedef units::quantity<unit::volumetric_solubility, double> VolumetricSolubilityQuantity;
typedef units::quantity<unit::membrane_permeability, double> MembranePermeabilityQuantity;
//typedef ParameterInstance<unit::mass> ParameterInstanceMass;
//typedef ParameterInstance<unit::time> ParameterInstanceTime;
//typedef ParameterInstance<unit::dynamic_viscosity> ParameterInstanceDynamicViscosity;
//typedef ParameterInstance<unit::pressure> ParameterInstancePressure;
//typedef ParameterInstance<unit::length> ParameterInstanceLength;
}
}//pyplusplus::aliases
//

//namespace chaste{
//template class ParameterInstance<unit::mass>;
//}

//template class ChastePoint<3>;

inline int Instantiation()
{
    return  sizeof(pyplusplus::aliases::MembranePermeabilityQuantity) +
            sizeof(pyplusplus::aliases::VolumetricSolubilityQuantity) +
            sizeof(pyplusplus::aliases::SolubilityQuantity) +
            sizeof(pyplusplus::aliases::DiffusivityPerConcentrationQuantity) +
            sizeof(pyplusplus::aliases::DiffusivityQuantity) +
            sizeof(pyplusplus::aliases::FlowRateQuantity) +
            sizeof(pyplusplus::aliases::ViscosityQuantity) +
            sizeof(pyplusplus::aliases::PressureQuantity) +
            sizeof(pyplusplus::aliases::ForceQuantity) +
            sizeof(pyplusplus::aliases::VelocityQuantity) +
            sizeof(pyplusplus::aliases::NumberDensityQuantity) +
            sizeof(pyplusplus::aliases::MolarMassQuantity) +
            sizeof(pyplusplus::aliases::RatePerConcentrationQuantity) +
            sizeof(pyplusplus::aliases::ConcentrationGradientQuantity) +
            sizeof(pyplusplus::aliases::ConcentrationFluxQuantity) +
            sizeof(pyplusplus::aliases::ConcentrationQuantity) +
            sizeof(pyplusplus::aliases::MolarFluxQuantity) +
            sizeof(pyplusplus::aliases::MolarFlowRateQuantity) +
            sizeof(pyplusplus::aliases::AmountQuantity) +
            sizeof(pyplusplus::aliases::MassFluxQuantity) +
            sizeof(pyplusplus::aliases::MassFlowRateQuantity) +
            sizeof(pyplusplus::aliases::MassQuantity) +
            sizeof(pyplusplus::aliases::PerAreaQuantity) +
            sizeof(pyplusplus::aliases::PerLengthQuantity) +
            sizeof(pyplusplus::aliases::VolumeQuantity) +
            sizeof(pyplusplus::aliases::AreaQuantity) +
            sizeof(pyplusplus::aliases::LengthQuantity) +
            sizeof(pyplusplus::aliases::RateQuantity) +
            sizeof(pyplusplus::aliases::TimeQuantity) +
            sizeof(pyplusplus::aliases::AngleQuantity);
//            sizeof(ParameterInstance<unit::mass>) +
//            sizeof(ParameterInstance<unit::time>) +
//            sizeof(ParameterInstance<unit::dynamic_viscosity>) +
//            sizeof(ParameterInstance<unit::pressure>) +
//            sizeof(ParameterInstance<unit::length>);
}
