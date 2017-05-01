// This file has been generated by Py++.

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


#include "boost/python.hpp"

#include "wrapper_header_collection.hpp"

#include "AreaQuantity.pypp.hpp"

#include "BaseParameterInstance.pypp.hpp"

#include "BaseUnits.pypp.hpp"

#include "ConcentrationFlowRateQuantity.pypp.hpp"

#include "ConcentrationFluxQuantity.pypp.hpp"

#include "ConcentrationGradientQuantity.pypp.hpp"

#include "ConcentrationQuantity.pypp.hpp"

#include "Connor17Parameters.pypp.hpp"

#include "DiffusivityPerConcentrationQuantity.pypp.hpp"

#include "DiffusivityQuantity.pypp.hpp"

#include "DimensionlessQuantity.pypp.hpp"

#include "DynamicViscosityQuantity.pypp.hpp"

#include "FlowRateQuantity.pypp.hpp"

#include "ForceQuantity.pypp.hpp"

#include "GenericParameters.pypp.hpp"

#include "LengthQuantity.pypp.hpp"

#include "MassQuantity.pypp.hpp"

#include "MolarFlowRateQuantity.pypp.hpp"

#include "MolarFluxQuantity.pypp.hpp"

#include "MolarMassQuantity.pypp.hpp"

#include "NumberDensityQuantity.pypp.hpp"

#include "Owen11Parameters.pypp.hpp"

#include "ParameterCollection.pypp.hpp"

#include "ParameterInstanceAreaQuantity.pypp.hpp"

#include "ParameterInstanceConcentrationFlowRateQuantity.pypp.hpp"

#include "ParameterInstanceConcentrationFluxQuantity.pypp.hpp"

#include "ParameterInstanceConcentrationGradientQuantity.pypp.hpp"

#include "ParameterInstanceConcentrationQuantity.pypp.hpp"

#include "ParameterInstanceDiffusivityPerConcentrationQuantity.pypp.hpp"

#include "ParameterInstanceDiffusivityQuantity.pypp.hpp"

#include "ParameterInstanceDimensionlessQuantity.pypp.hpp"

#include "ParameterInstanceDynamicViscosityQuantity.pypp.hpp"

#include "ParameterInstanceFlowImpedanceQuantity.pypp.hpp"

#include "ParameterInstanceFlowRateQuantity.pypp.hpp"

#include "ParameterInstanceForceQuantity.pypp.hpp"

#include "ParameterInstanceLengthQuantity.pypp.hpp"

#include "ParameterInstanceMassQuantity.pypp.hpp"

#include "ParameterInstanceMembranePermeabilityQuantity.pypp.hpp"

#include "ParameterInstanceMolarFlowRateQuantity.pypp.hpp"

#include "ParameterInstanceMolarFluxQuantity.pypp.hpp"

#include "ParameterInstanceMolarMassQuantity.pypp.hpp"

#include "ParameterInstanceNumberDensityQuantity.pypp.hpp"

#include "ParameterInstancePerAreaQuantity.pypp.hpp"

#include "ParameterInstancePerLengthQuantity.pypp.hpp"

#include "ParameterInstancePressureQuantity.pypp.hpp"

#include "ParameterInstanceRatePerConcentrationQuantity.pypp.hpp"

#include "ParameterInstanceRateQuantity.pypp.hpp"

#include "ParameterInstanceSolubilityQuantity.pypp.hpp"

#include "ParameterInstanceTimeQuantity.pypp.hpp"

#include "ParameterInstanceVolumeQuantity.pypp.hpp"

#include "ParameterInstanceVolumetricSolubilityQuantity.pypp.hpp"

#include "PerAreaQuantity.pypp.hpp"

#include "PerLengthQuantity.pypp.hpp"

#include "PressureQuantity.pypp.hpp"

#include "RatePerConcentrationQuantity.pypp.hpp"

#include "RateQuantity.pypp.hpp"

#include "Secomb04Parameters.pypp.hpp"

#include "SolubilityQuantity.pypp.hpp"

#include "TimeQuantity.pypp.hpp"

#include "VelocityQuantity.pypp.hpp"

#include "VolumeQuantity.pypp.hpp"

#include "VolumetricSolubilityQuantity.pypp.hpp"

#include "dimensionless.pypp.hpp"

#include "kg.pypp.hpp"

#include "metre.pypp.hpp"

#include "metre_cubed.pypp.hpp"

#include "metre_cubed_per_mole_per_second.pypp.hpp"

#include "metre_cubed_per_second.pypp.hpp"

#include "metre_per_second.pypp.hpp"

#include "metre_pow5_per_second_per_mole.pypp.hpp"

#include "metre_squared.pypp.hpp"

#include "metre_squared_per_second.pypp.hpp"

#include "mole_per_kg.pypp.hpp"

#include "mole_per_metre_cubed.pypp.hpp"

#include "mole_per_metre_cubed_per_second.pypp.hpp"

#include "mole_per_metre_pow4.pypp.hpp"

#include "mole_per_metre_pow5_per_second.pypp.hpp"

#include "mole_per_metre_squared_per_second.pypp.hpp"

#include "mole_per_second.pypp.hpp"

#include "newton.pypp.hpp"

#include "pascal.pypp.hpp"

#include "pascal_second_per_metre_cubed.pypp.hpp"

#include "per_metre.pypp.hpp"

#include "per_metre_cubed.pypp.hpp"

#include "per_metre_squared.pypp.hpp"

#include "per_pascal.pypp.hpp"

#include "per_second.pypp.hpp"

#include "poiseuille.pypp.hpp"

#include "second.pypp.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_chaste_project_MicrovesselChaste_utility){
    register_BaseParameterInstance_class();

    register_BaseUnits_class();

    register_Connor17Parameters_class();

    register_GenericParameters_class();

    register_Owen11Parameters_class();

    register_ParameterCollection_class();

    register_ParameterInstanceDimensionlessQuantity_class();

    register_ParameterInstancePerLengthQuantity_class();

    register_ParameterInstanceDynamicViscosityQuantity_class();

    register_ParameterInstancePressureQuantity_class();

    register_ParameterInstancePerAreaQuantity_class();

    register_ParameterInstanceSolubilityQuantity_class();

    register_ParameterInstanceMolarFluxQuantity_class();

    register_ParameterInstanceNumberDensityQuantity_class();

    register_ParameterInstanceConcentrationQuantity_class();

    register_ParameterInstanceConcentrationFlowRateQuantity_class();

    register_ParameterInstanceConcentrationGradientQuantity_class();

    register_ParameterInstanceFlowImpedanceQuantity_class();

    register_ParameterInstanceConcentrationFluxQuantity_class();

    register_ParameterInstanceLengthQuantity_class();

    register_ParameterInstanceVolumetricSolubilityQuantity_class();

    register_ParameterInstanceForceQuantity_class();

    register_ParameterInstanceMembranePermeabilityQuantity_class();

    register_ParameterInstanceAreaQuantity_class();

    register_ParameterInstanceDiffusivityQuantity_class();

    register_ParameterInstanceVolumeQuantity_class();

    register_ParameterInstanceFlowRateQuantity_class();

    register_ParameterInstanceRatePerConcentrationQuantity_class();

    register_ParameterInstanceDiffusivityPerConcentrationQuantity_class();

    register_ParameterInstanceMolarMassQuantity_class();

    register_ParameterInstanceMassQuantity_class();

    register_ParameterInstanceRateQuantity_class();

    register_ParameterInstanceMolarFlowRateQuantity_class();

    register_ParameterInstanceTimeQuantity_class();

    register_Secomb04Parameters_class();

    register_DimensionlessQuantity_class();

    bp::implicitly_convertible< boost::units::quantity< boost::units::unit< boost::units::dimensionless_type, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >, boost::units::quantity< boost::units::unit< boost::units::dimensionless_type, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >::value_type >();

    register_PerLengthQuantity_class();

    register_DynamicViscosityQuantity_class();

    register_PressureQuantity_class();

    register_PerAreaQuantity_class();

    register_SolubilityQuantity_class();

    register_MolarFluxQuantity_class();

    register_NumberDensityQuantity_class();

    register_ConcentrationQuantity_class();

    register_ConcentrationFlowRateQuantity_class();

    register_ConcentrationGradientQuantity_class();

    register_ConcentrationFluxQuantity_class();

    register_LengthQuantity_class();

    register_VolumetricSolubilityQuantity_class();

    register_ForceQuantity_class();

    register_VelocityQuantity_class();

    register_AreaQuantity_class();

    register_DiffusivityQuantity_class();

    register_VolumeQuantity_class();

    register_FlowRateQuantity_class();

    register_RatePerConcentrationQuantity_class();

    register_DiffusivityPerConcentrationQuantity_class();

    register_MolarMassQuantity_class();

    register_MassQuantity_class();

    register_RateQuantity_class();

    register_MolarFlowRateQuantity_class();

    register_TimeQuantity_class();

    register_dimensionless_class();

    register_per_metre_class();

    register_poiseuille_class();

    register_pascal_class();

    register_per_metre_squared_class();

    register_mole_per_metre_squared_per_second_class();

    register_per_metre_cubed_class();

    register_mole_per_metre_cubed_class();

    register_mole_per_metre_cubed_per_second_class();

    register_mole_per_metre_pow4_class();

    register_pascal_second_per_metre_cubed_class();

    register_mole_per_metre_pow5_per_second_class();

    register_metre_class();

    register_per_pascal_class();

    register_newton_class();

    register_metre_per_second_class();

    register_metre_squared_class();

    register_metre_squared_per_second_class();

    register_metre_cubed_class();

    register_metre_cubed_per_second_class();

    register_metre_cubed_per_mole_per_second_class();

    register_metre_pow5_per_second_per_mole_class();

    register_mole_per_kg_class();

    register_kg_class();

    register_per_second_class();

    register_mole_per_second_class();

    register_second_class();
}

