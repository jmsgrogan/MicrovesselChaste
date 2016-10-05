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

#ifndef PARAMETERINSTANCE_HPP_
#define PARAMETERINSTANCE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "SmartPointers.hpp"
#include "UnitCollection.hpp"
#include "BaseParameterInstance.hpp"

/**
 * This is a concrete class for storing parameters. It inherits from BaseParameterInstance
 * to allow a heterogenous collection of parameters to be stored in the ParameterCollection. When
 * GetValue is called a class instance will be added to the ParameterCollection singleton. It should
 * also be supplied with a descriptive name for where it is being called from, e.g. by "User" or
 * with a class name.
 */
template<class UNIT>
class ParameterInstance : public BaseParameterInstance
{
    /**
     * Archiving
     */
    friend class boost::serialization::access;

    /**
     * Do the serialize
     * @param ar the archive
     * @param version the archive version number
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<BaseParameterInstance>(*this);
        ar & mValue;
    }

    /**
     * The value of the parameter
     */
    units::quantity<UNIT> mValue;

public:

    /**
     * Constructor
     */
    ParameterInstance();

    /**
     * Constructor with some present values
     * @param value the quantity form the parameter
     * @param rName the named by which it will be keyed in the ParameterCollection
     * @param rShortDescription a short description of the parameter
     * @param rSymbol a symbol, as it may appear in the literature
     * @param rBibliographicInfromation a Bibtex formatted literature source.
     */
    ParameterInstance(units::quantity<UNIT> value,
                              const std::string& rName,
                              const std::string& rShortDescription,
                              const std::string& rSymbol,
                              const std::string& rBibliographicInfromation);

    /**
     * Destructor
     */
    virtual ~ParameterInstance();

    /**
     * Factory constructor method
     * @return a shared pointer to a new instance
     */
    static boost::shared_ptr<ParameterInstance<UNIT> > Create();

    /**
     * Over-ridden method to get the value of the parameter as a "Value Unit" string (e.g. "2.0 kg"). It is used in the
     * ParameterCollection which only stores BaseParameterInstance pointers.
     * @return the values as a string.
     */
    virtual std::string GetValueAsString();

    /**
     * Get the value of the parameter. IMPORTANT. This call is expensive as the instance will try to add itself
     * automatically to the ParameterCollection by default. This requires a map lookup and insert. This should be
     * called once per 'scope' to get the value of the parameter into a temporary.
     * @param rCallingClass the name of the class calling this method, or "User" if it is being called directly in a test or main().
     * @param addToCollection whether to add self to the parameter collection.
     * @return the value of the parameter
     */
    units::quantity<UNIT> GetValue(const std::string& rCallingClass = "User", bool addToCollection = true);

    /**
     * Set the value of the parameter
     * @param value the value of the parameter
     */
    void SetValue(units::quantity<UNIT> value);

};

#include "SerializationExportWrapper.hpp"
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
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::concentration_flow_rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::concentration_gradient)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::flow_rate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::flow_impedance)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::pressure)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::dynamic_viscosity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::diffusivity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::solubility)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::membrane_permeability)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::diffusivity_per_concentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, unit::volumetric_solubility)

#endif /*PARAMETERINSTANCE_HPP_*/
