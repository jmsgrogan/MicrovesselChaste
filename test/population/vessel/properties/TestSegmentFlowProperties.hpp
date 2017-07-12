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

#ifndef TESTSEGMENTFLOWPROPERTIES_HPP_
#define TESTSEGMENTFLOWPROPERTIES_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "SmartPointers.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"
#include "SegmentFlowProperties.hpp"
#include "OutputFileHandler.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestSegmentFlowProperties : public CxxTest::TestSuite
{
public:

    void TestSetAndGet() throw (Exception)
    {
        SegmentFlowProperties<3> properties = SegmentFlowProperties<3>();
        properties.SetHaematocrit(10.0);
        properties.SetFlowRate(20.0*unit::metre_cubed_per_second);
        properties.SetImpedance(30.0*unit::pascal_second_per_metre_cubed);
        properties.SetViscosity(40.0*unit::poiseuille);
        properties.SetWallShearStress(50.0*unit::pascals);
        properties.SetGrowthStimulus(60.0*unit::per_second);

        TS_ASSERT_DELTA(double(properties.GetHaematocrit()), 10.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetImpedance()/(1.0*unit::pascal_second_per_metre_cubed), 30.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetFlowRate()/(1.0*unit::metre_cubed_per_second), 20.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetViscosity()/(1.0*unit::poiseuille), 40.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetWallShearStress()/1_Pa, 50.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetGrowthStimulus()/(1.0*unit::per_second), 60.0, 1.e-6);

        TS_ASSERT_DELTA(properties.GetOutputData()["Segment Flow Rate m^3/s"], 20.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
#if BOOST_VERSION >= 105600
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("SegmentFlowProperties.arch");

        // Save archive
        {
            std::shared_ptr<AbstractVesselNetworkComponentProperties<3> > p_properties =
                    std::shared_ptr<AbstractVesselNetworkComponentProperties<3> >(new SegmentFlowProperties<3>());

            std::shared_ptr<SegmentFlowProperties<3> > p_cast_properties =
                    std::dynamic_pointer_cast<SegmentFlowProperties<3> >(p_properties);

            p_cast_properties->SetHaematocrit(10.0);
            p_cast_properties->SetFlowRate(20.0*unit::metre_cubed_per_second);
            p_cast_properties->SetImpedance(30.0*unit::pascal_second_per_metre_cubed);
            p_cast_properties->SetViscosity(40.0*unit::poiseuille);
            p_cast_properties->SetWallShearStress(50.0*unit::pascals);
            p_cast_properties->SetGrowthStimulus(60.0*unit::per_second);

            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_properties;
        }

        // Load archive
        {
            std::shared_ptr<AbstractVesselNetworkComponentProperties<3> > p_properties_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_properties_from_archive;
            std::shared_ptr<SegmentFlowProperties<3> > p_cast_properties =
                    std::dynamic_pointer_cast<SegmentFlowProperties<3> >(p_properties_from_archive);

            TS_ASSERT_DELTA(double(p_cast_properties->GetHaematocrit()), 10.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetImpedance()/(1.0*unit::pascal_second_per_metre_cubed), 30.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetFlowRate()/(1.0*unit::metre_cubed_per_second), 20.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetViscosity()/(1.0*unit::poiseuille), 40.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetWallShearStress()/1_Pa, 50.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetGrowthStimulus()/(1.0*unit::per_second), 60.0, 1.e-6);

            TS_ASSERT_DELTA(p_cast_properties->GetOutputData()["Segment Flow Rate m^3/s"], 20.0, 1.e-6);
        }
#endif
    }
};

#endif /*TESTSEGMENTFLOWPROPERTIES_HPP_*/
