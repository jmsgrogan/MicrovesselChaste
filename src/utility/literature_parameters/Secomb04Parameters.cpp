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
#include "Secomb04Parameters.hpp"

std::string SECOMBO4_BIB_INFO = "@article{Secomb04, \n author = {Secomb, Timothy W and Hsu, Richard and Park, Eric Y H and DeWhirst, Mark W},"
        "\n journal = {Annals of Biomedical Engineering}, \n month = {nov}, \n number = {11}, \n pages = {1519--29},"
        "\n title = {{Green’s function methods for analysis of oxygen delivery to tissue by microvascular networks.}},"
        "\n volume = {32}, \n year = {2004}}";

units::quantity<unit::pressure> SECOMB04_MMHG(1.0*unit::mmHg);
const boost::shared_ptr<ParameterInstance<unit::volumetric_solubility> > Secomb04Parameters::mpOxygenVolumetricSolubility =
        boost::shared_ptr<ParameterInstance<unit::volumetric_solubility> >(new ParameterInstance<unit::volumetric_solubility> (3.1e-5/SECOMB04_MMHG,
                                                                                   "Secomb04_OxygenVolumetricSolubility",
                                                                                   "Oxygen solubility",
                                                                                   "\\alpha_{eff}",
                                                                                   SECOMBO4_BIB_INFO));
