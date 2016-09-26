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

#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"

std::string bib_info = "@article{Owen2011, \n author = {Owen, Markus R and Stamper, I Johanna and Muthana, Munitta and Richardson, Giles W and Dobson, Jon and Lewis, Claire E and Byrne, Helen M},"
        "\n journal = {Cancer research}, \n month = {apr}, \n number = {8}, \n pages = {2826--37},"
        "\n title = {{Mathematical modeling predicts synergistic antitumor effects of combining a macrophage-based, hypoxia-targeted gene therapy with chemotherapy.}},"
        "\n volume = {71}, \n year = {2011}}";

units::quantity<unit::pressure> mmHg(1.0*unit::mmHg);
units::quantity<unit::pressure> inlet_pressure(25.0 * unit::mmHg);
const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpInletPressure =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (inlet_pressure,
                                                                                   "Owen11_InletPressure",
                                                                                   "Vessel network inlet pressure$",
                                                                                   "P_{in}",
                                                                                   bib_info));

units::quantity<unit::pressure> outlet_pressure(15.0 * unit::mmHg);
const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOutletPressure =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (outlet_pressure,
                                                                                   "Owen11_OutletPressure",
                                                                                   "Vessel network outlet pressure",
                                                                                   "P_{out}",
                                                                                   bib_info));
units::quantity<unit::length> cm (0.01*unit::metres);
units::quantity<unit::mass> g(1.e-3 *unit::kg);
units::quantity<unit::time> min(60.0*unit::seconds);
units::quantity<unit::dynamic_viscosity> plasma_visocity(0.72*g/(cm*min));
const boost::shared_ptr<ParameterInstance<unit::dynamic_viscosity> > Owen11Parameters::mpPlasmaViscosity =
        boost::shared_ptr<ParameterInstance<unit::dynamic_viscosity> >(new ParameterInstance<unit::dynamic_viscosity> (plasma_visocity,
                                                                                   "Owen11_PlasmaViscosity",
                                                                                   "Blood plasma viscosity",
                                                                                   "\\mu_{plasma}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpMinimumCellCyclePeriodNormal =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (3000.0*min,
                                                                                   "Owen11_MinCellCycleNormal",
                                                                                   "Minimum cell cycle period normal",
                                                                                   "T_{min}_{normal}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpMinimumCellCyclePeriodCancer =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (1600.0*min,
                                                                                   "Owen11_MinCellCycleCancer",
                                                                                   "Minimum cell cycle period cancer",
                                                                                   "T_{min}_{cancer}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpTimeToDeathDueToQuiescence =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (4000.0*min,
                                                                                   "Owen11_TimeDeathQuiescence",
                                                                                   "Time for death due to sustained quiescence",
                                                                                   "T_{death}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateNormal =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (3.0*mmHg,
                                                                                   "Owen11_OxygenAtHalfMaxCycleRateNormal",
                                                                                   "Oxygen partial pressure at half max cell cycle rate normal",
                                                                                   "C_{\\phi}_{normal}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateCancer =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (1.4*mmHg,
                                                                                   "Owen11_OxygenAtHalfMaxCycleRateCancer",
                                                                                   "Oxygen partial pressure at half max cell cycle rate cancer",
                                                                                   "C_{\\phi}_{cancer}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenPartialPressureAtQuiescence =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (8.9*mmHg,
                                                                                   "Owen11_OxygenAtQuiescence",
                                                                                   "Oxygen partial pressure at quiescence",
                                                                                   "C^{enter}_{quiesc}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenPartialPressureLeaveQuiescence =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (9.8*mmHg,
                                                                                   "Owen11_OxygenLeaveQuiescence",
                                                                                   "Oxygen partial pressure to leave quiescence",
                                                                                   "C^{leave}_{quiesc}",
                                                                                   bib_info));

units::quantity<unit::area> micron_sq(1.e-6*unit::metres*1.e-6*unit::metres);
units::quantity<unit::concentration> nano_molar(1.e-9 *unit::mole_per_metre_cubed);
units::quantity<unit::diffusivity_per_concentration> chemotactic_sensitivty(2.e4*micron_sq/(min*nano_molar));
const boost::shared_ptr<ParameterInstance<unit::diffusivity_per_concentration> > Owen11Parameters::mpChemotacticSensitivity =
        boost::shared_ptr<ParameterInstance<unit::diffusivity_per_concentration> >(new ParameterInstance<unit::diffusivity_per_concentration> (chemotactic_sensitivty,
                                                                                   "Owen11_ChemotacticSensitivity",
                                                                                   "Chemotactic sensitivity",
                                                                                   "\\Chi",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Owen11Parameters::mpCellMotilityNormal =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (0.0*micron_sq/min,
                                                                                   "Owen11_CellMotilityNormal",
                                                                                   "Maximum cell motility normal",
                                                                                   "D_{normal}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Owen11Parameters::mpCellMotilityCancer =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (0.5*micron_sq/min,
                                                                                   "Owen11_CellMotilityCancer",
                                                                                   "Maximum cell motility cancer",
                                                                                   "D_{cancer}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Owen11Parameters::mpCellMotilityEndothelial =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (1.0*micron_sq/min,
                                                                                   "Owen11_CellMotilityEndothelial",
                                                                                   "Maximum cell motility endothelial",
                                                                                   "D_{endo}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::membrane_permeability> > Owen11Parameters::mpVesselOxygenPermeability =
        boost::shared_ptr<ParameterInstance<unit::membrane_permeability> >(new ParameterInstance<unit::membrane_permeability> (6.0*cm/min,
                                                                                   "Owen11_VesselOxygenPermeability",
                                                                                   "Vessel permeability to oxygen",
                                                                                   "\\psi_{c}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::membrane_permeability> > Owen11Parameters::mpVesselVegfPermeability =
        boost::shared_ptr<ParameterInstance<unit::membrane_permeability> >(new ParameterInstance<unit::membrane_permeability> (1.e-5*cm/min,
                                                                                   "Owen11_VesselVegfPermeability",
                                                                                   "Vessel permeability to vegf",
                                                                                   "\\psi_{v}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpCellOxygenConsumptionRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (13.0/min,
                                                                                   "Owen11_CellOxygenConsumptionRate",
                                                                                   "Cell oxygen consumption rate",
                                                                                   "k_c^{cell}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpMaximumSproutingRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.00025/min,
                                                                                   "Owen11_MaximumSproutingRate",
                                                                                   "Maximum rate of sprouting",
                                                                                   "P^{max}_{sprout}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::concentration> > Owen11Parameters::mpVegfConventrationAtHalfMaxProbSprouting =
        boost::shared_ptr<ParameterInstance<unit::concentration> >(new ParameterInstance<unit::concentration> (0.5*nano_molar,
                                                                                   "Owen11_VegfConventrationAtHalfMaxProbSprouting",
                                                                                   "VEGF concentration at half maximal vessel sprouting probability",
                                                                                   "V_{sprout}",
                                                                                   bib_info));

units::quantity<unit::length> um(1.e-6*unit::metres);
const boost::shared_ptr<ParameterInstance<unit::length> > Owen11Parameters::mpSproutingExclusionRadius =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> (80.0 * um,
                                                                                   "Owen11_SproutingExclusionRadius",
                                                                                   "Sprouting exclusion radius",
                                                                                   "R_{ex}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpMaxTimeWithLowWallShearStress =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (4000.0 * min,
                                                                                   "Owen11_MaxTimeWithLowWallShearStress",
                                                                                   "Maximum vessel survivial time with low wall shear stress",
                                                                                   "T_{prune}",
                                                                                   bib_info));

units::quantity<unit::force> dyne(g*cm/(unit::seconds*unit::seconds));
const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpCriticalWallShearStress =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (8.0 * dyne/(cm*cm),
                                                                                   "Owen11_CriticalWallShearStress",
                                                                                   "Critical wall shear stress for vessel pruning",
                                                                                   "\\tau_{wall}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpMasterStepTime =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (30.0 * min,
                                                                                   "Owen11_MasterStepTime",
                                                                                   "Master step time",
                                                                                   "\\Delta t",
                                                                                   bib_info));

units::quantity<unit::time> day(60.0*60.0*24.0*unit::seconds);
const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpSimulationDuration =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (200.0 * day,
                                                                                   "Owen11_SimulationDuration",
                                                                                   "Simulation duration",
                                                                                   "T_{final}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Owen11Parameters::mpLatticeSpacing =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> (40.0*um,
                                                                                   "Owen11_LatticeSpacing",
                                                                                   "Lattice spacing",
                                                                                   "\\Delta x",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Owen11Parameters::mpOxygenDiffusivity =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (0.00145*cm*cm/min,
                                                                                   "Owen11_OxygenDiffusivity",
                                                                                   "Oxygen diffusivity ",
                                                                                   "D_{c}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Owen11Parameters::mpVegfDiffusivity =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (1.e-5*cm*cm/min,
                                                                                   "Owen11_VegfDiffusivity",
                                                                                   "Vegf diffusivity ",
                                                                                   "D_{v}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpVegfDecayRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.01/min,
                                                                                   "Owen11_VegfDecayRate",
                                                                                   "Vegf decay rate ",
                                                                                   "\\delta_{v}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::dimensionless> > Owen11Parameters::mpInflowHaematocrit =
        boost::shared_ptr<ParameterInstance<unit::dimensionless> >(new ParameterInstance<unit::dimensionless> (0.45,
                                                                                   "Owen11_InflowHaematocrit",
                                                                                   "Inflow haematocrit",
                                                                                   "H_{in}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpP53ProductionRateConstant =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.002/min,
                                                                                   "Owen11_P53ProductionRateConstant",
                                                                                   "Intracellular p53 production rate constant",
                                                                                   "k_7",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpP53MaxDegradationRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.01/min,
                                                                                   "Owen11_P53MaxDegradationRate",
                                                                                   "Max p53 degradation rate",
                                                                                   "k_dash_7",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenTensionForHalfMaxP53Degradation =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (4.44*mmHg,
                                                                                   "Owen11_OxygenTensionForHalfMaxP53Degradation",
                                                                                   "Tissue oxygen tension for half-max p53 degradation",
                                                                                   "C_{p53}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpCellVegfProductionRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.002/min,
                                                                                   "Owen11_CellVegfProductionRate",
                                                                                   "Basal VEGF production rate in cell",
                                                                                   "k_8",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpMaxCellVegfProductionRate =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.01/min,
                                                                                   "Owen11_MaxCellVegfProductionRate",
                                                                                   "Max VEGF production rate in cell",
                                                                                   "k_8_dash",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpP53EffectOnVegfProduction =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (-0.002/min,
                                                                                   "Owen11_P53EffectOnVegfProduction",
                                                                                   "Effect of P53 on VEGF production",
                                                                                   "k_8_dash_dash",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::dimensionless> > Owen11Parameters::mpVegfEffectOnVegfProduction =
        boost::shared_ptr<ParameterInstance<unit::dimensionless> >(new ParameterInstance<unit::dimensionless> (0.04,
                                                                                   "Owen11_VegfEffectOnVegfProduction",
                                                                                   "Effect of VEGF on VEGF production",
                                                                                   "j_5",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpOxygenTensionForHalfMaxVegfDegradation =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (4.44*mmHg,
                                                                                   "Owen11_OxygenTensionForHalfMaxVegfDegradation",
                                                                                   "Tissue oxygen tension for half-max vegf degradation",
                                                                                   "C_{VEGF}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Owen11Parameters::mpVesselRadiusUpdateTimestep =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (0.1*unit::seconds,
                                                                                   "Owen11_VesselRadiusUpdateTimestep",
                                                                                   "Vessel radius update timestep",
                                                                                   "\\epsilon_t",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::concentration_flow_rate> > Owen11Parameters::mpCellVegfSecretionRate =
        boost::shared_ptr<ParameterInstance<unit::concentration_flow_rate> >(new ParameterInstance<unit::concentration_flow_rate> (0.01*nano_molar/min,
                                                                                   "Owen11_CellVegfSecretionRate",
                                                                                   "Cell vegf secretion rate",
                                                                                   "k_v^{cell}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Owen11Parameters::mpMinimumRadius =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> (1.e-6*unit::metres,
                                                                                   "Owen11_MinimumRadius",
                                                                                   "Minimum possible radius",
                                                                                   "R_{MIN}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Owen11Parameters::mpMaximumRadius =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> (50.e-6*unit::metres,
                                                                                   "Owen11_MaximumRadius",
                                                                                   "Maximum possible radius",
                                                                                   "R_{MAX}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::flow_rate> > Owen11Parameters::mpReferenceFlowRateForMetabolicStimulus =
        boost::shared_ptr<ParameterInstance<unit::flow_rate> >(new ParameterInstance<unit::flow_rate> (4.e-5*cm*cm*cm/min,
                                                                                   "Owen11_ReferenceFlowRateForMetabolicStimulus",
                                                                                   "Reference flow rate for metabolic stimulus",
                                                                                   "Q_{ref}",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpShrinkingTendency =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (1.7/unit::seconds,
                                                                                   "Owen11_ShrinkingTendency",
                                                                                   "Shrinking tendency",
                                                                                   "k_s",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpSensitivityToIntravascularPressure =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (0.5/unit::seconds,
                                                                                   "Owen11_SensitivityToIntravascularPressure",
                                                                                   "Shrinking to intravascaulr pressure",
                                                                                   "k_p",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Owen11Parameters::mpBasalMetabolicStimulus =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (1.7/unit::seconds,
                                                                                   "Owen11_BasalMetabolicStimulus",
                                                                                   "Basal metabolic stimulus",
                                                                                   "k^0_m",
                                                                                   bib_info));

const boost::shared_ptr<ParameterInstance<unit::pressure> > Owen11Parameters::mpReferencePartialPressure =
        boost::shared_ptr<ParameterInstance<unit::pressure> >(new ParameterInstance<unit::pressure> (20.0*mmHg,
                                                                                   "Owen11_ReferencePartialPressure",
                                                                                   "Reference partial pressure of inlet haematocrit vessels",
                                                                                   "C_{Ref}",
                                                                                   bib_info));
