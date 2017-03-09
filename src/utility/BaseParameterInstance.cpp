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

#include "BaseParameterInstance.hpp"
#include "ParameterCollection.hpp"

BaseParameterInstance::BaseParameterInstance()
    : mName("DefaultParameter"),
      mShortDescription("A short description of the parameter"),
      mSourceInformation("WARNING: No source information given for parameter."),
      mSymbol("-"),
      mBaseValue(0.0)
{

}

BaseParameterInstance::BaseParameterInstance(const std::string& rName,
                                             const std::string& rShortDescription,
                                             const std::string& rSymbol,
                                             const std::string& rBibliographicInfromation)
    : mName(rName),
      mShortDescription(rShortDescription),
      mSourceInformation(rBibliographicInfromation),
      mSymbol(rSymbol),
      mBaseValue(0.0)
{

}

BaseParameterInstance::~BaseParameterInstance()
{

}

boost::shared_ptr<BaseParameterInstance> BaseParameterInstance::Create()
{
    MAKE_PTR(BaseParameterInstance, pSelf);
    return pSelf;
}

void BaseParameterInstance::RegisterWithCollection(const std::string& rCallingClass)
{
    ParameterCollection::Instance()->AddParameter(shared_from_this(), rCallingClass);
}

std::string BaseParameterInstance::GetValueAsString()
{
    std::stringstream ss;
    ss << mBaseValue;
    return ss.str();
}

std::string BaseParameterInstance::GetBibliographicInformation()
{
    return mSourceInformation;
}

std::string BaseParameterInstance::GetName()
{
    return mName;
}

std::string BaseParameterInstance::GetSymbol()
{
    return mSymbol;
}

std::string BaseParameterInstance::GetShortDescription()
{
    return mShortDescription;
}

void BaseParameterInstance::SetBibliographicInformation(const std::string& rSourceInformation)
{
    mSourceInformation = rSourceInformation;
}

void BaseParameterInstance::SetName(const std::string& rName)
{
    mName = rName;
}

void BaseParameterInstance::SetShortDescription(const std::string& rShortDescription)
{
    mShortDescription = rShortDescription;
}

void BaseParameterInstance::SetSymbol(const std::string& rSymbol)
{
    mSymbol = rSymbol;
}

std::ostream& operator <<(std::ostream& stream, const boost::shared_ptr<BaseParameterInstance>& rParameter)
{
    stream<< "<name>" << rParameter->GetName() <<"</name>\n";
    stream<< "<value>" << rParameter->GetValueAsString() << "</value>\n";
    stream<< "<description>" << rParameter->GetShortDescription() << "</description>\n";
    stream<< "<symbol>" << rParameter->GetSymbol() << "</symbol>\n";
    stream<< "<source>" << "\"" <<rParameter->GetBibliographicInformation() << "\"" << "</source>\n";
    return stream;
}
