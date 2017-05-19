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

#ifndef TESTNODEFLOWPROPERTIES_HPP_
#define TESTNODEFLOWPROPERTIES_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "SmartPointers.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"
#include "NodeFlowProperties.hpp"
#include "OutputFileHandler.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestNodeFlowProperties : public CxxTest::TestSuite
{
public:

    void TestSetAndGet() throw (Exception)
    {
        NodeFlowProperties<3> properties = NodeFlowProperties<3>();
        properties.SetIsInputNode(true);
        properties.SetIsOutputNode(false);
        properties.SetPressure(20.0*unit::pascals);
        properties.SetUseVelocityBoundaryCondition(false);

        TS_ASSERT(properties.IsInputNode());
        TS_ASSERT(!properties.IsOutputNode());
        TS_ASSERT(!properties.UseVelocityBoundaryCondition());
        TS_ASSERT_DELTA(properties.GetPressure().value(), 20.0, 1.e-6);
        TS_ASSERT_DELTA(properties.GetOutputData()["Node Pressure Pa"], 20.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("NodeFlowProperties.arch");

        // Save archive
        {
            boost::shared_ptr<AbstractVesselNetworkComponentProperties<3> > p_properties =
                    boost::shared_ptr<AbstractVesselNetworkComponentProperties<3> >(new NodeFlowProperties<3>());

            boost::shared_ptr<NodeFlowProperties<3> > p_cast_properties =
                    boost::dynamic_pointer_cast<NodeFlowProperties<3> >(p_properties);

            p_cast_properties->SetIsInputNode(true);
            p_cast_properties->SetIsOutputNode(false);
            p_cast_properties->SetPressure(20.0*unit::pascals);
            p_cast_properties->SetUseVelocityBoundaryCondition(false);

            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_properties;
        }

        // Load archive
        {
            boost::shared_ptr<AbstractVesselNetworkComponentProperties<3> > p_properties_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_properties_from_archive;
            boost::shared_ptr<NodeFlowProperties<3> > p_cast_properties =
                    boost::dynamic_pointer_cast<NodeFlowProperties<3> >(p_properties_from_archive);

            TS_ASSERT(p_cast_properties->IsInputNode());
            TS_ASSERT(!p_cast_properties->IsOutputNode());
            TS_ASSERT(!p_cast_properties->UseVelocityBoundaryCondition());
            TS_ASSERT_DELTA(p_cast_properties->GetPressure().value(), 20.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_properties->GetOutputData()["Node Pressure Pa"], 20.0, 1.e-6);
        }
    }
};

#endif /*TESTNODEFLOWPROPERTIES_HPP_*/
