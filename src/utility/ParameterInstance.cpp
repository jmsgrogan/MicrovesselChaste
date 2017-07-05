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

#include <sstream>
#include "ParameterInstance.hpp"

template<class QUANTITY>
ParameterInstance<QUANTITY>::ParameterInstance()
    : BaseParameterInstance(),
      mValue()
{

}

template<class QUANTITY>
ParameterInstance<QUANTITY>::ParameterInstance(QUANTITY value,
                                                     const std::string& rName,
                                                     const std::string& rShortDescription,
                                                     const std::string& rSymbol,
                                                     const std::string& rBibliographicInfromation)
    : BaseParameterInstance(rName, rShortDescription, rSymbol, rBibliographicInfromation),
      mValue(value)
{

}

template<class QUANTITY>
ParameterInstance<QUANTITY>::~ParameterInstance()
{

}

template<class QUANTITY>
std::shared_ptr<ParameterInstance<QUANTITY> > ParameterInstance<QUANTITY>::Create()
{
    return std::make_shared<ParameterInstance<QUANTITY> >();
}

template<class QUANTITY>
std::shared_ptr<ParameterInstance<QUANTITY> > ParameterInstance<QUANTITY>::Create(QUANTITY value,
                                                                            const std::string& rName,
                                                                            const std::string& rShortDescription,
                                                                            const std::string& rSymbol,
                                                                            const std::string& rBibliographicInfromation)
{
    return std::make_shared<ParameterInstance<QUANTITY> >(value,rName,rShortDescription,rSymbol, rBibliographicInfromation);
}

template<class QUANTITY>
QUANTITY ParameterInstance<QUANTITY>::GetValue(const std::string& rCallingClass, bool addToCollection)
{
    if(addToCollection)
    {
        // Register self with the parameter collection if not already in there.
        this->RegisterWithCollection(rCallingClass);
    }

    return mValue;
}

template<class QUANTITY>
std::string ParameterInstance<QUANTITY>::GetValueAsString()
{
    std::stringstream ss;
    ss << mValue;
    return ss.str();
}

template<class QUANTITY>
void ParameterInstance<QUANTITY>::SetValue(QUANTITY value)
{
    mValue = value;
}

// Explicit Instantiation
template class ParameterInstance<QDimensionless>;
template class ParameterInstance<QTime>;
template class ParameterInstance<QRate>;
template class ParameterInstance<QLength> ;
template class ParameterInstance<QPerLength>;
template class ParameterInstance<QArea>;
template class ParameterInstance<QVolume>;
template class ParameterInstance<QMass>;
template class ParameterInstance<QMassFlux>;
template class ParameterInstance<QMassFlowRate>;
template class ParameterInstance<QAmount>;
template class ParameterInstance<QMolarFlux>;
template class ParameterInstance<QMolarFlowRate>;
template class ParameterInstance<QMolarMass>;
template class ParameterInstance<QNumberDensity>;
template class ParameterInstance<QRatePerConcentration>;
template class ParameterInstance<QConcentration>;
template class ParameterInstance<QConcentrationGradient>;
template class ParameterInstance<QFlowRate>;
template class ParameterInstance<QFlowImpedance>;
template class ParameterInstance<QPressure>;
template class ParameterInstance<QDynamicViscosity>;
template class ParameterInstance<QDiffusivity>;
template class ParameterInstance<QSolubility>;
template class ParameterInstance<QMembranePermeability>;
template class ParameterInstance<QDiffusivityPerConcentration>;
template class ParameterInstance<QVolumetricSolubility>;
template class ParameterInstance<QConcentrationFlowRate>;
template class ParameterInstance<QConcentrationFlux>;
template class ParameterInstance<QPerArea>;
template class ParameterInstance<QForce>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QDimensionless)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QTime)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QRate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QLength)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QPerLength)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QArea)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QVolume)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMass)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMassFlux)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMassFlowRate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QAmount)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMolarFlux)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMolarFlowRate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMolarMass)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QNumberDensity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QRatePerConcentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QConcentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QConcentrationFlowRate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QConcentrationGradient)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QFlowRate)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QFlowImpedance)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QPressure)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QDynamicViscosity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QDiffusivity)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QSolubility)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QMembranePermeability)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QDiffusivityPerConcentration)
EXPORT_TEMPLATE_CLASS1(ParameterInstance, QVolumetricSolubility)
