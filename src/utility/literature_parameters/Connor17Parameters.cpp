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
#include "Connor17Parameters.hpp"

std::string connor_bib_info = "@article{Connor17, \n author = {Connor, Anthony J,},}";



const std::shared_ptr<ParameterInstance<QPerLength> > Connor17Parameters::mpEcsPerLength =
        std::shared_ptr<ParameterInstance<QPerLength> >(new ParameterInstance<QPerLength> (1.2e-5/(1.0*unit::metres),
                                                                                   "Connor17_EcsPerLength",
                                                                                   "Number of endothelial cells per vessel length",
                                                                                   "$/nu$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivity>  > Connor17Parameters::mpMotilityCoefficient =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  ((3.e-15/60.0)*(unit::metres*unit::metres/unit::seconds),
                                                                                   "Connor17_MotilityCoefficient",
                                                                                   "Motility coefficient for tips",
                                                                                   "$/mu$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Connor17Parameters::mpVesselRadius =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  ((5.e-6*unit::metres),
                                                                                   "Connor17_VesselRadius",
                                                                                   "Average vessel radius",
                                                                                   "$R$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QVolume>  > Connor17Parameters::mpPelletVolume =
        std::shared_ptr<ParameterInstance<QVolume>  >(new ParameterInstance<QVolume>  ((1.7e-11*unit::metres*unit::metres*unit::metres),
                                                                                   "Connor17_PelletVolume",
                                                                                   "Pellet volume",
                                                                                   "$/Omega_p$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QArea> > Connor17Parameters::mpPelletSurfaceArea =
        std::shared_ptr<ParameterInstance<QArea> >(new ParameterInstance<QArea> ((6.79e-7*unit::metres*unit::metres),
                                                                                   "Connor17_PelletSurfaceArea",
                                                                                   "Pellet surface area",
                                                                                   "$/sigma_p$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QDiffusivityPerConcentration> > Connor17Parameters::mpChemotacticCoefficient =
        std::shared_ptr<ParameterInstance<QDiffusivityPerConcentration> >(new ParameterInstance<QDiffusivityPerConcentration> (((0.57/3600.0)*(unit::metres*unit::metres/(unit::seconds*unit::mole_per_metre_cubed))),
                                                                                   "Connor17_ChemotacticCoefficient",
                                                                                   "Chemotactic coefficient",
                                                                                   "$/chi$",
                                                                                   connor_bib_info));


const std::shared_ptr<ParameterInstance<QDiffusivity>  > Connor17Parameters::mpVegfDiffusionConstant =
        std::shared_ptr<ParameterInstance<QDiffusivity>  >(new ParameterInstance<QDiffusivity>  (((2.52e-7/3600.0)*(unit::metres*unit::metres/unit::seconds)),
                                                                                   "Connor17_VegfDiffusionConstant",
                                                                                   "VEGF diffusion constant",
                                                                                   "$D_v$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Connor17Parameters::mpVegfDecayConstant =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (((0.8/3600.0)*(1.0/unit::seconds)),
                                                                                   "Connor17_VegfDecayConstant",
                                                                                   "Rate of vegf decay",
                                                                                   "$/lambda_v$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QMolarFlowRate> > Connor17Parameters::mpReductionInVegfPerCell =
        std::shared_ptr<ParameterInstance<QMolarFlowRate> >(new ParameterInstance<QMolarFlowRate> (((4.e-19/3600.0)*(unit::moles/unit::seconds)),
                                                                                   "Connor17_ReductionInVegfPerCell;",
                                                                                   "Rate of reduction in VEGF concentration per endothelial cell",
                                                                                   "$K^v_{EC}$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QConcentration> > Connor17Parameters::mpVegfAtHalfReceptorOccupancy =
        std::shared_ptr<ParameterInstance<QConcentration> >(new ParameterInstance<QConcentration> ((6.5e-10*unit::mole_per_metre_cubed),
                                                                                   "Connor17_VegfAtHalfReceptorOccupancy;",
                                                                                   "VEGF at half receptor occupancy",
                                                                                   "$v_{50}$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QMembranePermeability> > Connor17Parameters::mpVesselVegfPermeability =
        std::shared_ptr<ParameterInstance<QMembranePermeability> >(new ParameterInstance<QMembranePermeability> ((3.e-4*(unit::metres/unit::seconds)),
                                                                                   "Connor17_VesselVegfPermeability;",
                                                                                   "Permability of corneal vasculature to VEGF",
                                                                                   "$P_v$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QConcentration> > Connor17Parameters::mpVegfBloodConcentration =
        std::shared_ptr<ParameterInstance<QConcentration> >(new ParameterInstance<QConcentration> ((0.0*unit::mole_per_metre_cubed),
                                                                                   "Connor17_VegfBloodConcentration;",
                                                                                   "Concentration of VEGF in the blood",
                                                                                   "$v_b$",
                                                                                   connor_bib_info));


const std::shared_ptr<ParameterInstance<QDimensionless> > Connor17Parameters::mpVegfBindingConstant =
        std::shared_ptr<ParameterInstance<QDimensionless> >(new ParameterInstance<QDimensionless> ((30.0),
                                                                                   "Connor17_VegfBindingConstant;",
                                                                                   "VEGF binding constant",
                                                                                   "$/theta_v$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QMembranePermeability> > Connor17Parameters::mpCorneaVegfPermeability =
        std::shared_ptr<ParameterInstance<QMembranePermeability> >(new ParameterInstance<QMembranePermeability> (((1.12e-7/3600.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_CorneaVegfPermeability;",
                                                                                   "Cornea pellet vegf permeability",
                                                                                   "$k^v_p$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QRate> > Connor17Parameters::mpVegfDecayRateInPellet =
        std::shared_ptr<ParameterInstance<QRate> >(new ParameterInstance<QRate> (((0.8/3600.0)*(1.0/unit::seconds)),
                                                                                   "Connor17_VegfDecayRateInPellet;",
                                                                                   "VEGF decay rate in pellet",
                                                                                   "$/lambda^v_p$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QConcentration> > Connor17Parameters::mpInitialVegfConcentrationInPellet =
        std::shared_ptr<ParameterInstance<QConcentration> >(new ParameterInstance<QConcentration> ((3.93e-4*unit::mole_per_metre_cubed),
                                                                                   "Connor17_InitialVegfConcentrationInPellet;",
                                                                                   "Initial VEGF concentration in pellet",
                                                                                   "$[V_t]$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QTime> > Connor17Parameters::mpTimeStep =
        std::shared_ptr<ParameterInstance<QTime> >(new ParameterInstance<QTime> (((10.0*60.0)*unit::seconds),
                                                                                   "Connor17_TimeStep;",
                                                                                   "Time step",
                                                                                   "$/Delta_t$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Connor17Parameters::mpLatticeSpacing =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  ((20.0e-6*unit::metres),
                                                                                   "Connor17_LatticeSpacing;",
                                                                                   "Lattice spacing",
                                                                                   "$/Delta_x$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QLength>  > Connor17Parameters::mpFilopodiaProbeDistance =
        std::shared_ptr<ParameterInstance<QLength>  >(new ParameterInstance<QLength>  ((120.0e-6*unit::metres),
                                                                                   "Connor17_FilopodiaProbeDistance;",
                                                                                   "Filopodia probe distance",
                                                                                   "$r_f$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QDimensionless> > Connor17Parameters::mpFilopodiaSensingAngle =
        std::shared_ptr<ParameterInstance<QDimensionless> >(new ParameterInstance<QDimensionless> ((M_PI/3.0),
                                                                                   "Connor17_FilopodiaSensingAngle;",
                                                                                   "Filopodia sensing angle",
                                                                                   "$/theta_f$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QVelocity> > Connor17Parameters::mpTipVesselAttractionStrength =
        std::shared_ptr<ParameterInstance<QVelocity> >(new ParameterInstance<QVelocity> (((2.375e-7/60.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_TipVesselAttractionStrength;",
                                                                                   "Tip vessel interaction strength",
                                                                                   "$K_{\rho}$",
                                                                                   connor_bib_info));

const std::shared_ptr<ParameterInstance<QVelocity> > Connor17Parameters::mpTipTipAttractionStrength =
        std::shared_ptr<ParameterInstance<QVelocity> >(new ParameterInstance<QVelocity> (((1.615e-7/60.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_TipTipAttractionStrength;",
                                                                                   "Tip tip interaction strength",
                                                                                   "$K_{T}$",
                                                                                   connor_bib_info));
