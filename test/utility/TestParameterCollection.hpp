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

#ifndef TESTPARAMETERCOLLECTION_HPP
#define TESTPARAMETERCOLLECTION_HPP

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "SmartPointers.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "UnitCollection.hpp"
#include "OutputFileHandler.hpp"
#include "SerializableSingleton.hpp"
#include "BaseParameterInstance.hpp"
#include "ParameterInstance.hpp"
#include "ParameterCollection.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestParameterCollection : public CxxTest::TestSuite
{

public:

    void TestMixedParameterCollection()
    {
        std::shared_ptr<BaseParameterInstance> my_parameter = BaseParameterInstance::Create();
        my_parameter->SetShortDescription("My Description");
        my_parameter->SetName("Base");

        std::shared_ptr<ParameterInstance<QTime> > my_time_parameter = ParameterInstance<QTime>:: Create();
        QTime few_seconds = 5.0*unit::seconds;
        my_time_parameter->SetShortDescription("My Description For Time Parameter");
        my_time_parameter->SetValue(few_seconds);
        my_time_parameter->SetName("Derived");

        std::shared_ptr<ParameterCollection> my_params = ParameterCollection::SharedInstance();
        OutputFileHandler file_handler("TestMixedParameterCollection", true);
        my_params->AddParameter(my_parameter, "Test");
        my_params->AddParameter(my_time_parameter, "Test");
        my_params->DumpToFile(file_handler.GetOutputDirectoryFullPath() + "parameter_dump.xml");

        ParameterCollection::Destroy();
    }

    void TestArchiving()
    {

        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("ParameterCollection.arch");

        // Save archive
        {
            std::shared_ptr<BaseParameterInstance> my_parameter = BaseParameterInstance::Create();
            my_parameter->SetShortDescription("My Description");
            my_parameter->SetName("Base");

            std::shared_ptr<ParameterInstance<QTime> > my_time_parameter = ParameterInstance<QTime>:: Create();
            QTime few_seconds = 5.0*unit::seconds;
            my_time_parameter->SetShortDescription("My Description For Time Parameter");
            my_time_parameter->SetValue(few_seconds);
            my_time_parameter->SetName("Derived");

            ParameterCollection* p_my_params = ParameterCollection::Instance();
            OutputFileHandler file_handler("TestMixedParameterCollection", true);
            p_my_params->AddParameter(my_parameter, "Test");
            p_my_params->AddParameter(my_time_parameter, "Test");

            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);

            SerializableSingleton<ParameterCollection>* const p_wrapper = p_my_params->GetSerializationWrapper();
            output_arch << p_wrapper;

            ParameterCollection::Destroy();
        }

        // Load archive
        {
            std::shared_ptr<ParameterCollection> p_my_params_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            SerializableSingleton<ParameterCollection>* p_wrapper;
            input_arch >> p_wrapper;

            TS_ASSERT_EQUALS("My Description", ParameterCollection::Instance()->GetParameter("Base")->GetShortDescription());

            std::shared_ptr<ParameterInstance<QTime> > p_derived =
                    std::dynamic_pointer_cast<ParameterInstance<QTime> >(ParameterCollection::Instance()->GetParameter("Derived"));
            TS_ASSERT_EQUALS("My Description For Time Parameter", p_derived->GetShortDescription());
            TS_ASSERT_DELTA(5.0, p_derived->GetValue()/1_s, 1.e-6);
        }
    }

};

#endif // TESTPARAMETERCOLLECTION_HPP
