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

#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include "PetscTools.hpp"

#include "ParameterCollection.hpp"

/** Pointer to the single instance */
boost::shared_ptr<ParameterCollection> ParameterCollection::mpInstance = boost::shared_ptr<ParameterCollection>();

ParameterCollection* ParameterCollection::Instance()
{
    if (!mpInstance)
    {
        mpInstance = boost::shared_ptr<ParameterCollection>(new ParameterCollection);
        std::atexit(Destroy);
    }
    return mpInstance.get();
}

boost::shared_ptr<ParameterCollection> ParameterCollection::SharedInstance()
{
    if (!mpInstance)
    {
        mpInstance = boost::shared_ptr<ParameterCollection>(new ParameterCollection);
        std::atexit(Destroy);
    }
    return mpInstance;
}

ParameterCollection::ParameterCollection()
    : mParameters()
{
    // Make sure there's only one instance - enforces correct serialization
    assert(!mpInstance);

}

void ParameterCollection::DumpToFile(const std::string& rFilename)
{
    if(PetscTools::AmMaster())
    {
        std::ofstream myfile;
        myfile.open(rFilename.c_str());
        myfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
        myfile << "<parameter_collection>" << std::endl;

        typedef std::map<std::string, std::pair<std::string, boost::shared_ptr<BaseParameterInstance> > >::iterator it_type;
        for(it_type iterator = mParameters.begin(); iterator != mParameters.end(); iterator++)
        {
            myfile << "<parameter>" << std::endl;
            myfile << "<added_by>" << (iterator->second).first << "</added_by>"<< std::endl;
            myfile << (iterator->second).second;
            myfile << "</parameter>" << std::endl;
        }
        myfile << "</parameter_collection>" << std::endl;
        myfile.close();
    }
}

boost::shared_ptr<BaseParameterInstance> ParameterCollection::GetParameter(const std::string& rName)
{
    return mParameters[rName].second;
}

void ParameterCollection::AddParameter(boost::shared_ptr<BaseParameterInstance> pParameter, const std::string& rFirstInstantiated)
{
    // Check if the parameter already exists in the map
    std::map<std::string, std::pair<std::string, boost::shared_ptr<BaseParameterInstance> > >::iterator it = mParameters.find(pParameter->GetName());
    if(it != mParameters.end() && rFirstInstantiated != it->first)
    {
        it->second = std::pair<std::string, boost::shared_ptr<BaseParameterInstance> >(rFirstInstantiated, pParameter);
    }
    else
    {
        mParameters[pParameter->GetName()] = std::pair<std::string, boost::shared_ptr<BaseParameterInstance> >(rFirstInstantiated, pParameter);
    }
}

void ParameterCollection::Destroy()
{
    if (mpInstance)
    {
        mpInstance = boost::shared_ptr<ParameterCollection>();
    }
}

