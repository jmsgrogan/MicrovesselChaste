/*

 Copyright (c) 2005-2015, University of Oxford.
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

#include "ViscosityCalculator.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
ViscosityCalculator<DIM>::ViscosityCalculator() : AbstractVesselNetworkCalculator<DIM>(),
    mPlasmaViscosity(3.5e-3 * unit::poiseuille)
{

}

template<unsigned DIM>
ViscosityCalculator<DIM>::~ViscosityCalculator()
{

}

template<unsigned DIM>
void ViscosityCalculator<DIM>::SetDimensionalPlasmaViscosity(units::quantity<unit::dynamic_viscosity> visocity)
{
    mPlasmaViscosity = visocity;
}

template<unsigned DIM>
void ViscosityCalculator<DIM>::SetPlasmaViscositySI(double visocity)
{
    mPlasmaViscosity = visocity * unit::poiseuille;
}

template<unsigned DIM>
void ViscosityCalculator<DIM>::Calculate()
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
    for (unsigned segment_index = 0; segment_index < segments.size(); segment_index++)
    {
        units::quantity<unit::length> radius = segments[segment_index]->GetRadius();
        units::quantity<unit::dimensionless> haematocrit = segments[segment_index]->GetFlowProperties()->GetHaematocrit();

        // This equation assumes the radius is in micron so manually converted. No dimensional checking is done here, it may not
        // even be possible for this equation.
        double micron_radius = (radius/unit::metres)*1.e6;
        double power_term_1 = 1.0 / (1.0 + pow(10.0, -11.0) * pow(2.0 * micron_radius, 12));
        double c = (0.8 + exp(-0.15 * micron_radius)) * (power_term_1 - 1) + power_term_1;
        double mu_45 = 6.0 * exp(-0.17 * micron_radius) + 3.2 - 2.44 * exp(-0.06 * pow(2 * micron_radius, 0.645));

        double power_term_2 = pow((2.0 * micron_radius / (2.0 * micron_radius - 1.1)), 2.0);
        double mu_rel = (1.0
                + (mu_45 - 1.0) * (((pow((1.0 - haematocrit), c)) - 1) / ((pow((1.0 - 0.45), c)) - 1.0)) * power_term_2) * power_term_2;

        segments[segment_index]->GetFlowProperties()->SetViscosity(mPlasmaViscosity*mu_rel);
    }
}

// Explicit instantiation
template class ViscosityCalculator<2> ;
template class ViscosityCalculator<3> ;
