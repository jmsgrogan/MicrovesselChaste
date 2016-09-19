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

#include "ParameterInstance.hpp"

template<class UNIT>
ParameterInstance<UNIT>::ParameterInstance()
    : BaseParameterInstance(),
      mValue()
{

}

template<class UNIT>
ParameterInstance<UNIT>::ParameterInstance(units::quantity<UNIT> value,
                                                     const std::string& rName,
                                                     const std::string& rShortDescription,
                                                     const std::string& rSymbol,
                                                     const std::string& rBibliographicInfromation)
    : BaseParameterInstance(rName, rShortDescription, rSymbol, rBibliographicInfromation),
      mValue(value)
{

}

template<class UNIT>
ParameterInstance<UNIT>::~ParameterInstance()
{

}

template<class UNIT>
boost::shared_ptr<ParameterInstance<UNIT> > ParameterInstance<UNIT>::Create()
{
    MAKE_PTR(ParameterInstance<UNIT>, pSelf);
    return pSelf;
}

template<class UNIT>
units::quantity<UNIT> ParameterInstance<UNIT>::GetValue(const std::string& rCallingClass, bool addToCollection)
{
    if(addToCollection)
    {
        // Register self with the parameter collection if not already in there.
        this->RegisterWithCollection(rCallingClass);
    }

    return mValue;
}

template<class UNIT>
std::string ParameterInstance<UNIT>::GetValueAsString()
{
    std::stringstream ss;
    ss << mValue;
    return ss.str();
}

template<class UNIT>
void ParameterInstance<UNIT>::SetValue(units::quantity<UNIT> value)
{
    mValue = value;
}

// Explicit Instantiation
template class ParameterInstance<unit::dimensionless>;
template class ParameterInstance<unit::time>;
template class ParameterInstance<unit::rate>;
template class ParameterInstance<unit::length>;
template class ParameterInstance<unit::per_length>;
template class ParameterInstance<unit::area>;
template class ParameterInstance<unit::volume>;
template class ParameterInstance<unit::mass>;
template class ParameterInstance<unit::mass_flux>;
template class ParameterInstance<unit::mass_flow_rate>;
template class ParameterInstance<unit::amount>;
template class ParameterInstance<unit::molar_flux>;
template class ParameterInstance<unit::molar_flow_rate>;
template class ParameterInstance<unit::molar_mass>;
template class ParameterInstance<unit::number_density>;
template class ParameterInstance<unit::rate_per_concentration>;
template class ParameterInstance<unit::concentration>;
template class ParameterInstance<unit::concentration_gradient>;
template class ParameterInstance<unit::flow_rate>;
template class ParameterInstance<unit::flow_impedance>;
template class ParameterInstance<unit::pressure>;
template class ParameterInstance<unit::dynamic_viscosity>;
template class ParameterInstance<unit::diffusivity>;
template class ParameterInstance<unit::solubility>;
template class ParameterInstance<unit::membrane_permeability>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::dimensionless)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::time)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::length)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::per_length)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::area)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::volume)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::mass)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::mass_flux)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::mass_flow_rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::amount)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::molar_flux)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::molar_flow_rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::molar_mass)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::number_density)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::rate_per_concentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::concentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::concentration_gradient)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::flow_rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::flow_impedance)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::pressure)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::dynamic_viscosity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::diffusivity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::solubility)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::membrane_permeability)
