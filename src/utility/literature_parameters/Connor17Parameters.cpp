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



const boost::shared_ptr<ParameterInstance<unit::per_length> > Connor17Parameters::mpEcsPerLength =
        boost::shared_ptr<ParameterInstance<unit::per_length> >(new ParameterInstance<unit::per_length> (1.2e-5/(1.0*unit::metres),
                                                                                   "Connor17_EcsPerLength",
                                                                                   "Number of endothelial cells per vessel length",
                                                                                   "$/nu$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Connor17Parameters::mpMotilityCoefficient =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> ((3.e-15/60.0)*(unit::metres*unit::metres/unit::seconds),
                                                                                   "Connor17_MotilityCoefficient",
                                                                                   "Motility coefficient for tips",
                                                                                   "$/mu$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Connor17Parameters::mpVesselRadius =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> ((5.e-6*unit::metres),
                                                                                   "Connor17_VesselRadius",
                                                                                   "Average vessel radius",
                                                                                   "$R$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::volume> > Connor17Parameters::mpPelletVolume =
        boost::shared_ptr<ParameterInstance<unit::volume> >(new ParameterInstance<unit::volume> ((1.7e-11*unit::metres*unit::metres*unit::metres),
                                                                                   "Connor17_PelletVolume",
                                                                                   "Pellet volume",
                                                                                   "$/Omega_p$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::area> > Connor17Parameters::mpPelletSurfaceArea =
        boost::shared_ptr<ParameterInstance<unit::area> >(new ParameterInstance<unit::area> ((6.79e-7*unit::metres*unit::metres),
                                                                                   "Connor17_PelletSurfaceArea",
                                                                                   "Pellet surface area",
                                                                                   "$/sigma_p$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::diffusivity_per_concentration> > Connor17Parameters::mpChemotacticCoefficient =
        boost::shared_ptr<ParameterInstance<unit::diffusivity_per_concentration> >(new ParameterInstance<unit::diffusivity_per_concentration> (((0.57/3600.0)*(unit::metres*unit::metres/(unit::seconds*unit::mole_per_metre_cubed))),
                                                                                   "Connor17_ChemotacticCoefficient",
                                                                                   "Chemotactic coefficient",
                                                                                   "$/chi$",
                                                                                   connor_bib_info));


const boost::shared_ptr<ParameterInstance<unit::diffusivity> > Connor17Parameters::mpVegfDiffusionConstant =
        boost::shared_ptr<ParameterInstance<unit::diffusivity> >(new ParameterInstance<unit::diffusivity> (((2.52e-7/3600.0)*(unit::metres*unit::metres/unit::seconds)),
                                                                                   "Connor17_VegfDiffusionConstant",
                                                                                   "VEGF diffusion constant",
                                                                                   "$D_v$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Connor17Parameters::mpVegfDecayConstant =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (((0.8/3600.0)*(1.0/unit::seconds)),
                                                                                   "Connor17_VegfDecayConstant",
                                                                                   "Rate of vegf decay",
                                                                                   "$/lambda_v$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::molar_flow_rate> > Connor17Parameters::mpReductionInVegfPerCell =
        boost::shared_ptr<ParameterInstance<unit::molar_flow_rate> >(new ParameterInstance<unit::molar_flow_rate> (((4.e-19/3600.0)*(unit::moles/unit::seconds)),
                                                                                   "Connor17_ReductionInVegfPerCell;",
                                                                                   "Rate of reduction in VEGF concentration per endothelial cell",
                                                                                   "$K^v_{EC}$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::concentration> > Connor17Parameters::mpVegfAtHalfReceptorOccupancy =
        boost::shared_ptr<ParameterInstance<unit::concentration> >(new ParameterInstance<unit::concentration> ((6.5e-10*unit::mole_per_metre_cubed),
                                                                                   "Connor17_VegfAtHalfReceptorOccupancy;",
                                                                                   "VEGF at half receptor occupancy",
                                                                                   "$v_{50}$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::membrane_permeability> > Connor17Parameters::mpVesselVegfPermeability =
        boost::shared_ptr<ParameterInstance<unit::membrane_permeability> >(new ParameterInstance<unit::membrane_permeability> ((3.e-4*(unit::metres/unit::seconds)),
                                                                                   "Connor17_VesselVegfPermeability;",
                                                                                   "Permability of corneal vasculature to VEGF",
                                                                                   "$P_v$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::concentration> > Connor17Parameters::mpVegfBloodConcentration =
        boost::shared_ptr<ParameterInstance<unit::concentration> >(new ParameterInstance<unit::concentration> ((0.0*unit::mole_per_metre_cubed),
                                                                                   "Connor17_VegfBloodConcentration;",
                                                                                   "Concentration of VEGF in the blood",
                                                                                   "$v_b$",
                                                                                   connor_bib_info));


const boost::shared_ptr<ParameterInstance<unit::dimensionless> > Connor17Parameters::mpVegfBindingConstant =
        boost::shared_ptr<ParameterInstance<unit::dimensionless> >(new ParameterInstance<unit::dimensionless> ((30.0),
                                                                                   "Connor17_VegfBindingConstant;",
                                                                                   "VEGF binding constant",
                                                                                   "$/theta_v$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::membrane_permeability> > Connor17Parameters::mpCorneaVegfPermeability =
        boost::shared_ptr<ParameterInstance<unit::membrane_permeability> >(new ParameterInstance<unit::membrane_permeability> (((1.12e-7/3600.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_CorneaVegfPermeability;",
                                                                                   "Cornea pellet vegf permeability",
                                                                                   "$k^v_p$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::rate> > Connor17Parameters::mpVegfDecayRateInPellet =
        boost::shared_ptr<ParameterInstance<unit::rate> >(new ParameterInstance<unit::rate> (((0.8/3600.0)*(1.0/unit::seconds)),
                                                                                   "Connor17_VegfDecayRateInPellet;",
                                                                                   "VEGF decay rate in pellet",
                                                                                   "$/lambda^v_p$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::concentration> > Connor17Parameters::mpInitialVegfConcentrationInPellet =
        boost::shared_ptr<ParameterInstance<unit::concentration> >(new ParameterInstance<unit::concentration> ((3.93e-4*unit::mole_per_metre_cubed),
                                                                                   "Connor17_InitialVegfConcentrationInPellet;",
                                                                                   "Initial VEGF concentration in pellet",
                                                                                   "$[V_t]$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::time> > Connor17Parameters::mpTimeStep =
        boost::shared_ptr<ParameterInstance<unit::time> >(new ParameterInstance<unit::time> (((10.0*60.0)*unit::seconds),
                                                                                   "Connor17_TimeStep;",
                                                                                   "Time step",
                                                                                   "$/Delta_t$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Connor17Parameters::mpLatticeSpacing =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> ((20.0e-6*unit::metres),
                                                                                   "Connor17_LatticeSpacing;",
                                                                                   "Lattice spacing",
                                                                                   "$/Delta_x$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::length> > Connor17Parameters::mpFilopodiaProbeDistance =
        boost::shared_ptr<ParameterInstance<unit::length> >(new ParameterInstance<unit::length> ((120.0e-6*unit::metres),
                                                                                   "Connor17_FilopodiaProbeDistance;",
                                                                                   "Filopodia probe distance",
                                                                                   "$r_f$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::dimensionless> > Connor17Parameters::mpFilopodiaSensingAngle =
        boost::shared_ptr<ParameterInstance<unit::dimensionless> >(new ParameterInstance<unit::dimensionless> ((M_PI/3.0),
                                                                                   "Connor17_FilopodiaSensingAngle;",
                                                                                   "Filopodia sensing angle",
                                                                                   "$/theta_f$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::velocity> > Connor17Parameters::mpTipVesselAttractionStrength =
        boost::shared_ptr<ParameterInstance<unit::velocity> >(new ParameterInstance<unit::velocity> (((2.375e-7/60.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_TipVesselAttractionStrength;",
                                                                                   "Tip vessel interaction strength",
                                                                                   "$K_{\rho}$",
                                                                                   connor_bib_info));

const boost::shared_ptr<ParameterInstance<unit::velocity> > Connor17Parameters::mpTipTipAttractionStrength =
        boost::shared_ptr<ParameterInstance<unit::velocity> >(new ParameterInstance<unit::velocity> (((1.615e-7/60.0)*(unit::metres/unit::seconds)),
                                                                                   "Connor17_TipTipAttractionStrength;",
                                                                                   "Tip tip interaction strength",
                                                                                   "$K_{T}$",
                                                                                   connor_bib_info));
