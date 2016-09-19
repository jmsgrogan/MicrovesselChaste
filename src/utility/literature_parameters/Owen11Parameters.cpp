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
