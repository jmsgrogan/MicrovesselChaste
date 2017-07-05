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

#ifndef CONNOR17PARAMETERS_HPP_
#define CONNOR17PARAMETERS_HPP_

#include "SmartPointers.hpp"
#include "BaseParameterInstance.hpp"
#include "ParameterInstance.hpp"

/**
 * This struct stores parameter values used in the paper draft Connor et al 2017
 */
struct Connor17Parameters
{
    /**
     * Number of endothelial cells per vessel length \nu
     */
    static const std::shared_ptr<ParameterInstance<QPerLength>  > mpEcsPerLength;

    /**
     * Motility coefficient for tips \mu
     */
    static const std::shared_ptr<ParameterInstance<QDiffusivity>  > mpMotilityCoefficient;

    /**
     * Average vessel radius R
     */
    static const std::shared_ptr<ParameterInstance<QLength>  > mpVesselRadius;

    /**
     * Pellet volume \Omega_p
     */
    static const std::shared_ptr<ParameterInstance<QVolume>  > mpPelletVolume;

    /**
     * Pellet surface area \sigma_p
     */
    static const std::shared_ptr<ParameterInstance<QArea>  > mpPelletSurfaceArea;

    /**
     * Chemotactic coefficient \chi
     */
    static const std::shared_ptr<ParameterInstance<QDiffusivityPerConcentration>  > mpChemotacticCoefficient;

    /**
     * VEGF diffusion constant \D_v
     */
    static const std::shared_ptr<ParameterInstance<QDiffusivity>  > mpVegfDiffusionConstant;

    /**
     * Rate of vegf decay \lambda_v
     */
    static const std::shared_ptr<ParameterInstance<QRate>  > mpVegfDecayConstant;

    /**
     * Rate of reduction in VEGF concentration per endothelial cell K^v_{EC}
     */
    static const std::shared_ptr<ParameterInstance<QMolarFlowRate>  > mpReductionInVegfPerCell;

    /**
     * VEGF at half receptor occupancy v_{50}
     */
    static const std::shared_ptr<ParameterInstance<QConcentration> > mpVegfAtHalfReceptorOccupancy;

    /**
     * Permability of corneal vasculature to VEGF P_v
     */
    static const std::shared_ptr<ParameterInstance<QMembranePermeability> > mpVesselVegfPermeability;

    /**
     * Concentration of VEGF in the blood v_b
     */
    static const std::shared_ptr<ParameterInstance<QConcentration> > mpVegfBloodConcentration;

    /**
     * VEGF binding constant \theta_v
     */
    static const std::shared_ptr<ParameterInstance<QDimensionless> > mpVegfBindingConstant;

    /**
     * Cornea pellet vegf permeability k^v_p
     */
    static const std::shared_ptr<ParameterInstance<QMembranePermeability> > mpCorneaVegfPermeability;

    /**
     * VEGF decay rate in pellet \lambda^v_p
     */
    static const std::shared_ptr<ParameterInstance<QRate>  > mpVegfDecayRateInPellet;

    /**
     * Initial VEGF concentration in pellet
     */
    static const std::shared_ptr<ParameterInstance<QConcentration> > mpInitialVegfConcentrationInPellet;

    /**
     * Time step \Delta T
     */
    static const std::shared_ptr<ParameterInstance<QTime> > mpTimeStep;

    /**
     * Lattice spacing \Delta X
     */
    static const std::shared_ptr<ParameterInstance<QLength>  > mpLatticeSpacing;

    /**
     * Filopodia probe distance r_f
     */
    static const std::shared_ptr<ParameterInstance<QLength>  > mpFilopodiaProbeDistance;

    /**
     * Filopodia sensing angle \theta_f
     */
    static const std::shared_ptr<ParameterInstance<QDimensionless> > mpFilopodiaSensingAngle;

    /**
     * Tip vessel interaction strength K_{\rho}
     */
    static const std::shared_ptr<ParameterInstance<QVelocity> > mpTipVesselAttractionStrength;

    /**
     * Tip tip interaction strength K_T
     */
    static const std::shared_ptr<ParameterInstance<QVelocity> > mpTipTipAttractionStrength;

};

#endif /*CONNOR17PARAMETERS_HPP_*/
