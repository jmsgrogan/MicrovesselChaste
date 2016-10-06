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

#ifndef TESTVESSELNETWORKREADER_HPP_
#define TESTVESSELNETWORKREADER_HPP_

#include <cxxtest/TestSuite.h>
#include "VesselNetworkReader.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVesselNetworkReader : public CxxTest::TestSuite
{
public:

    void TestReadNetworkFromFile() throw(Exception)
    {
        // Locate the input file
        FileFinder fileFinder("projects/MicrovesselChaste/test/data/tapmeier_network.vtp", RelativeTo::ChasteSourceRoot);
        TS_ASSERT(fileFinder.Exists());
        TS_ASSERT(fileFinder.IsFile());

        // Generate the network
        boost::shared_ptr<VesselNetworkReader<3> > p_network_reader = VesselNetworkReader<3>::Create();
        p_network_reader->SetFileName(fileFinder.GetAbsolutePath());
        boost::shared_ptr<VesselNetwork<3> > p_network = p_network_reader->Read();

        // Write the network to file
        OutputFileHandler output_file_handler("TestVesselNetworkReaders", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("VtkVesselNetwork.vtp");
        p_network->MergeCoincidentNodes();
        p_network->Write(output_filename);
    }

    void TestReadBioNetworkFromFile() throw(Exception)
    {
        // Locate the input file
        FileFinder fileFinder("projects/MicrovesselChaste/test/data/retinal.vtp", RelativeTo::ChasteSourceRoot);
        TS_ASSERT(fileFinder.Exists());
        TS_ASSERT(fileFinder.IsFile());

        // Generate the network
        boost::shared_ptr<VesselNetworkReader<3> > p_network_reader = VesselNetworkReader<3>::Create();
        p_network_reader->SetFileName(fileFinder.GetAbsolutePath());
        p_network_reader->SetRadiusArrayName("Distance");
        boost::shared_ptr<VesselNetwork<3> > p_network = p_network_reader->Read();

        // Write the network to file
        OutputFileHandler output_file_handler("TestVesselNetworkReaders", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("VtkRetinalNetwork.vtp");
        p_network->MergeCoincidentNodes();
        p_network->Write(output_filename);
    }
};

#endif /*TESTVESSELNETWORKREADER_HPP_*/
