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

#ifndef TESTPARAMETERINSTANCE_HPP
#define TESTPARAMETERINSTANCE_HPP

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "SmartPointers.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "UnitCollection.hpp"
#include "OutputFileHandler.hpp"
#include "ParameterInstance.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestParameterInstance : public CxxTest::TestSuite
{

public:

    void TestInstance()
    {
        boost::shared_ptr<ParameterInstance<unit::time> > p_my_parameter = ParameterInstance<unit::time>::Create();
        units::quantity<unit::time> few_seconds = 5.0*unit::seconds;
        p_my_parameter->SetShortDescription("My Description For Time Parameter");
        p_my_parameter->SetValue(few_seconds);
        p_my_parameter->SetName("Derived");
        p_my_parameter->SetBibliographicInformation("J. Smith et al., (2003).");

        TS_ASSERT_EQUALS("Derived", p_my_parameter->GetName());
        TS_ASSERT_EQUALS("My Description For Time Parameter", p_my_parameter->GetShortDescription());
        TS_ASSERT_DELTA(p_my_parameter->GetValue()/unit::seconds, 5.0, 1.e-6);
        TS_ASSERT_EQUALS("J. Smith et al., (2003).", p_my_parameter->GetBibliographicInformation());

        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("ParameterInstance.arch");

        // Save archive
        {
            boost::shared_ptr<BaseParameterInstance> p_my_parameter_for_archive = boost::shared_ptr<BaseParameterInstance>(new ParameterInstance<unit::time>);
            units::quantity<unit::time> few_seconds = 5.0*unit::seconds;
            p_my_parameter_for_archive->SetShortDescription("My Description For Time Parameter");
            p_my_parameter_for_archive->SetName("Derived");

            boost::shared_ptr<ParameterInstance<unit::time> > p_my_cast_parameter_for_archive = boost::static_pointer_cast<ParameterInstance<unit::time> >(p_my_parameter_for_archive);
            p_my_cast_parameter_for_archive->SetValue(few_seconds);

            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_my_parameter_for_archive;
        }

        // Load archive
        {
            boost::shared_ptr<BaseParameterInstance> p_my_parameter_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_my_parameter_from_archive;
            TS_ASSERT_EQUALS("My Description For Time Parameter", p_my_parameter_from_archive->GetShortDescription());
            TS_ASSERT_EQUALS("Derived", p_my_parameter_from_archive->GetName());

            boost::shared_ptr<ParameterInstance<unit::time> > p_my_cast_parameter_from_archive = boost::dynamic_pointer_cast<ParameterInstance<unit::time> >(p_my_parameter_from_archive);
            TS_ASSERT_DELTA(p_my_cast_parameter_from_archive->GetValue()/unit::seconds, 5.0, 1.e-6);
        }
    }

    void TestAlternateConstructor()
    {
        units::quantity<unit::time> few_seconds = 5.0*unit::seconds;
        boost::shared_ptr<ParameterInstance<unit::time> > p_my_parameter = ParameterInstance<unit::time>::Create(few_seconds,
                                                                                     "Derived",
                                                                                     "My Description For Time Parameter",
                                                                                     "P",
                                                                                     "J. Smith et al., (2003).");

        TS_ASSERT_EQUALS("Derived", p_my_parameter->GetName());
        TS_ASSERT_EQUALS("My Description For Time Parameter", p_my_parameter->GetShortDescription());
        TS_ASSERT_DELTA(p_my_parameter->GetValue().value(), 5.0, 1.e-6);
        TS_ASSERT_EQUALS("J. Smith et al., (2003).", p_my_parameter->GetBibliographicInformation());
    }

};

#endif // TESTPARAMETERINSTANCE_HPP
