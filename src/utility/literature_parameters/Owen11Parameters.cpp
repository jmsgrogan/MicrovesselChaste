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

#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"

std::string bib_info = "@article{Owen2011, \n author = {Owen, Markus R and Stamper, I Johanna and Muthana, Munitta and Richardson, Giles W and Dobson, Jon and Lewis, Claire E and Byrne, Helen M},"
        "\n journal = {Cancer research}, \n month = {apr}, \n number = {8}, \n pages = {2826--37},"
        "\n title = {{Mathematical modeling predicts synergistic antitumor effects of combining a macrophage-based, hypoxia-targeted gene therapy with chemotherapy.}},"
        "\n volume = {71}, \n year = {2011}}";

QPressure mmHg(1.0*unit::mmHg);
QPressure inlet_pressure(25.0 * unit::mmHg);
const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpInletPressure =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (inlet_pressure,
                                                                                   "Owen11_InletPressure",
                                                                                   "Vessel network inlet pressure$",
                                                                                   "P_{in}",
                                                                                   bib_info));

QPressure outlet_pressure(15.0 * unit::mmHg);
const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOutletPressure =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (outlet_pressure,
                                                                                   "Owen11_OutletPressure",
                                                                                   "Vessel network outlet pressure",
                                                                                   "P_{out}",
                                                                                   bib_info));
QLength cm (0.01*unit::metres);
QMass g(1.e-3 *unit::kg);
QTime min(60.0*unit::seconds);
QDynamicViscosity plasma_visocity(0.72*g/(cm*min));
const std::shared_ptr<ParameterInstance<QDynamicViscosity> > Owen11Parameters::mpPlasmaViscosity =
        std::shared_ptr<ParameterInstance<QDynamicViscosity> >(new ParameterInstance<QDynamicViscosity> (plasma_visocity,
                                                                                   "Owen11_PlasmaViscosity",
                                                                                   "Blood plasma viscosity",
                                                                                   "\\mu_{plasma}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpMinimumCellCyclePeriodNormal =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (3000.0*min,
                                                                                   "Owen11_MinCellCycleNormal",
                                                                                   "Minimum cell cycle period normal",
                                                                                   "T_{min}^{normal}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpMinimumCellCyclePeriodCancer =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (1600.0*min,
                                                                                   "Owen11_MinCellCycleCancer",
                                                                                   "Minimum cell cycle period cancer",
                                                                                   "T_{min}^{cancer}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpTimeToDeathDueToQuiescence =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (4000.0*min,
                                                                                   "Owen11_TimeDeathQuiescence",
                                                                                   "Time for death due to sustained quiescence",
                                                                                   "T_{death}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateNormal =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (3.0*mmHg,
                                                                                   "Owen11_OxygenAtHalfMaxCycleRateNormal",
                                                                                   "Oxygen partial pressure at half max cell cycle rate normal",
                                                                                   "C_{\\phi}^{normal}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenPartialPressureAtHalfMaxCycleRateCancer =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (1.4*mmHg,
                                                                                   "Owen11_OxygenAtHalfMaxCycleRateCancer",
                                                                                   "Oxygen partial pressure at half max cell cycle rate cancer",
                                                                                   "C_{\\phi}^{cancer}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenPartialPressureAtQuiescence =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (8.9*mmHg,
                                                                                   "Owen11_OxygenAtQuiescence",
                                                                                   "Oxygen partial pressure at quiescence",
                                                                                   "C^{enter}_{quiesc}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenPartialPressureLeaveQuiescence =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (9.8*mmHg,
                                                                                   "Owen11_OxygenLeaveQuiescence",
                                                                                   "Oxygen partial pressure to leave quiescence",
                                                                                   "C^{leave}_{quiesc}",
                                                                                   bib_info));

QArea micron_sq(1_um*1_um);
QConcentration nano_molar(1.e-6 *unit::mole_per_metre_cubed);
QDiffusivityPerConcentration chemotactic_sensitivty(2.e4*micron_sq/(min*nano_molar));
const std::shared_ptr<ParameterInstance<QDiffusivityPerConcentration> > Owen11Parameters::mpChemotacticSensitivity =
        std::shared_ptr<ParameterInstance<QDiffusivityPerConcentration> >(new ParameterInstance<QDiffusivityPerConcentration> (chemotactic_sensitivty,
                                                                                   "Owen11_ChemotacticSensitivity",
                                                                                   "Chemotactic sensitivity",
                                                                                   "\\chi",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Owen11Parameters::mpCellMotilityNormal =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (0.0*micron_sq/min,
                                                                                   "Owen11_CellMotilityNormal",
                                                                                   "Maximum cell motility normal",
                                                                                   "D_{normal}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Owen11Parameters::mpCellMotilityCancer =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (0.5*micron_sq/min,
                                                                                   "Owen11_CellMotilityCancer",
                                                                                   "Maximum cell motility cancer",
                                                                                   "D_{cancer}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Owen11Parameters::mpCellMotilityEndothelial =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (1.0*micron_sq/min,
                                                                                   "Owen11_CellMotilityEndothelial",
                                                                                   "Maximum cell motility endothelial",
                                                                                   "D_{endo}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QMembranePermeability> > Owen11Parameters::mpVesselOxygenPermeability =
        std::shared_ptr<ParameterInstance<QMembranePermeability> >(new ParameterInstance<QMembranePermeability> (6.0*cm/min,
                                                                                   "Owen11_VesselOxygenPermeability",
                                                                                   "Vessel permeability to oxygen",
                                                                                   "\\psi_{c}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QMembranePermeability> > Owen11Parameters::mpVesselVegfPermeability =
        std::shared_ptr<ParameterInstance<QMembranePermeability> >(new ParameterInstance<QMembranePermeability> (1.e-5*cm/min,
                                                                                   "Owen11_VesselVegfPermeability",
                                                                                   "Vessel permeability to vegf",
                                                                                   "\\psi_{v}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpCellOxygenConsumptionRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (13.0/min,
                                                                                   "Owen11_CellOxygenConsumptionRate",
                                                                                   "Cell oxygen consumption rate",
                                                                                   "k_c^{cell}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpMaximumSproutingRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.00025/min,
                                                                                   "Owen11_MaximumSproutingRate",
                                                                                   "Maximum rate of sprouting",
                                                                                   "P^{max}_{sprout}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QConcentration> > Owen11Parameters::mpVegfConventrationAtHalfMaxProbSprouting =
        std::shared_ptr<ParameterInstance<QConcentration> >(new ParameterInstance<QConcentration> (0.5*nano_molar,
                                                                                   "Owen11_VegfConventrationAtHalfMaxProbSprouting",
                                                                                   "VEGF concentration at half maximal vessel sprouting probability",
                                                                                   "V_{sprout}",
                                                                                   bib_info));

QLength um(1_um);
const std::shared_ptr<ParameterInstance<QLength>  > Owen11Parameters::mpSproutingExclusionRadius =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  (80.0 * um,
                                                                                   "Owen11_SproutingExclusionRadius",
                                                                                   "Sprouting exclusion radius",
                                                                                   "R_{ex}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpMaxTimeWithLowWallShearStress =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (4000.0 * min,
                                                                                   "Owen11_MaxTimeWithLowWallShearStress",
                                                                                   "Maximum vessel survivial time with low wall shear stress",
                                                                                   "T_{prune}",
                                                                                   bib_info));

QForce dyne(g*cm/(unit::seconds*unit::seconds));
const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpCriticalWallShearStress =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (8.0 * dyne/(cm*cm),
                                                                                   "Owen11_CriticalWallShearStress",
                                                                                   "Critical wall shear stress for vessel pruning",
                                                                                   "\\tau_{wall}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpMasterStepTime =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (30.0 * min,
                                                                                   "Owen11_MasterStepTime",
                                                                                   "Master step time",
                                                                                   "\\Delta t",
                                                                                   bib_info));

QTime day(60.0*60.0*24.0*unit::seconds);
const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpSimulationDuration =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (200.0 * day,
                                                                                   "Owen11_SimulationDuration",
                                                                                   "Simulation duration",
                                                                                   "T_{final}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Owen11Parameters::mpLatticeSpacing =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  (40.0*um,
                                                                                   "Owen11_LatticeSpacing",
                                                                                   "Lattice spacing",
                                                                                   "\\Delta x",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Owen11Parameters::mpOxygenDiffusivity =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (0.00145*cm*cm/min,
                                                                                   "Owen11_OxygenDiffusivity",
                                                                                   "Oxygen diffusivity ",
                                                                                   "D_{c}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Owen11Parameters::mpVegfDiffusivity =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (1.e-5*cm*cm/min,
                                                                                   "Owen11_VegfDiffusivity",
                                                                                   "Vegf diffusivity ",
                                                                                   "D_{v}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpVegfDecayRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.01/min,
                                                                                   "Owen11_VegfDecayRate",
                                                                                   "Vegf decay rate ",
                                                                                   "\\delta_{v}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDimensionless> > Owen11Parameters::mpInflowHaematocrit =
        std::shared_ptr<ParameterInstance<QDimensionless> >(new ParameterInstance<QDimensionless> (0.45,
                                                                                   "Owen11_InflowHaematocrit",
                                                                                   "Inflow haematocrit",
                                                                                   "H_{in}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpP53ProductionRateConstant =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.002/min,
                                                                                   "Owen11_P53ProductionRateConstant",
                                                                                   "Intracellular p53 production rate constant",
                                                                                   "k_7",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpP53MaxDegradationRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.01/min,
                                                                                   "Owen11_P53MaxDegradationRate",
                                                                                   "Max p53 degradation rate",
                                                                                   "k_{*7}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenTensionForHalfMaxP53Degradation =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (4.44*mmHg,
                                                                                   "Owen11_OxygenTensionForHalfMaxP53Degradation",
                                                                                   "Tissue oxygen tension for half-max p53 degradation",
                                                                                   "C_{p53}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpCellVegfProductionRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.002/min,
                                                                                   "Owen11_CellVegfProductionRate",
                                                                                   "Basal VEGF production rate in cell",
                                                                                   "k_8",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpMaxCellVegfProductionRate =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.01/min,
                                                                                   "Owen11_MaxCellVegfProductionRate",
                                                                                   "Max VEGF production rate in cell",
                                                                                   "k_{8*}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpP53EffectOnVegfProduction =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (-0.002/min,
                                                                                   "Owen11_P53EffectOnVegfProduction",
                                                                                   "Effect of P53 on VEGF production",
                                                                                   "k_{8**}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QDimensionless> > Owen11Parameters::mpVegfEffectOnVegfProduction =
        std::shared_ptr<ParameterInstance<QDimensionless> >(new ParameterInstance<QDimensionless> (0.04,
                                                                                   "Owen11_VegfEffectOnVegfProduction",
                                                                                   "Effect of VEGF on VEGF production",
                                                                                   "j_5",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpOxygenTensionForHalfMaxVegfDegradation =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (4.44*mmHg,
                                                                                   "Owen11_OxygenTensionForHalfMaxVegfDegradation",
                                                                                   "Tissue oxygen tension for half-max vegf degradation",
                                                                                   "C_{VEGF}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Owen11Parameters::mpVesselRadiusUpdateTimestep =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (0.1*unit::seconds,
                                                                                   "Owen11_VesselRadiusUpdateTimestep",
                                                                                   "Vessel radius update timestep",
                                                                                   "\\epsilon_t",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QConcentrationFlowRate> > Owen11Parameters::mpCellVegfSecretionRate =
        std::shared_ptr<ParameterInstance<QConcentrationFlowRate> >(new ParameterInstance<QConcentrationFlowRate> (0.01*nano_molar/min,
                                                                                   "Owen11_CellVegfSecretionRate",
                                                                                   "Cell vegf secretion rate",
                                                                                   "k_v^{cell}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Owen11Parameters::mpMinimumRadius =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  (1_um,
                                                                                   "Owen11_MinimumRadius",
                                                                                   "Minimum possible radius",
                                                                                   "R_{MIN}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Owen11Parameters::mpMaximumRadius =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  (50.e-6*unit::metres,
                                                                                   "Owen11_MaximumRadius",
                                                                                   "Maximum possible radius",
                                                                                   "R_{MAX}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QFlowRate> > Owen11Parameters::mpReferenceFlowRateForMetabolicStimulus =
        std::shared_ptr<ParameterInstance<QFlowRate> >(new ParameterInstance<QFlowRate> (4.e-5*cm*cm*cm/min,
                                                                                   "Owen11_ReferenceFlowRateForMetabolicStimulus",
                                                                                   "Reference flow rate for metabolic stimulus",
                                                                                   "Q_{ref}",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpShrinkingTendency =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (1.7/unit::seconds,
                                                                                   "Owen11_ShrinkingTendency",
                                                                                   "Shrinking tendency",
                                                                                   "k_s",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpSensitivityToIntravascularPressure =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (0.5/unit::seconds,
                                                                                   "Owen11_SensitivityToIntravascularPressure",
                                                                                   "Shrinking to intravascaulr pressure",
                                                                                   "k_p",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Owen11Parameters::mpBasalMetabolicStimulus =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (1.7/unit::seconds,
                                                                                   "Owen11_BasalMetabolicStimulus",
                                                                                   "Basal metabolic stimulus",
                                                                                   "k^0_m",
                                                                                   bib_info));

const std::shared_ptr<ParameterInstance<QPressure> > Owen11Parameters::mpReferencePartialPressure =
        std::shared_ptr<ParameterInstance<QPressure> >(new ParameterInstance<QPressure> (20.0*mmHg,
                                                                                   "Owen11_ReferencePartialPressure",
                                                                                   "Reference partial pressure of inlet haematocrit vessels",
                                                                                   "C_{Ref}",
                                                                                   bib_info));
