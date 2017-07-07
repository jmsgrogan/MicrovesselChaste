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

#ifndef TESTVESSELNETWORKGENERATOR_HPP_
#define TESTVESSELNETWORKGENERATOR_HPP_

#include <cxxtest/TestSuite.h>
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "Timer.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselNetworkGenerator : public CxxTest::TestSuite
{

public:

    void TestGenerateAndWriteMonolithicHexagonalNetwork() throw (Exception)
    {
        // Generate the network
        VesselNetworkGenerator<2> network_generator;
        QLength length(10000.0*unit::microns);
        QLength vessel_length(40.0*unit::microns);
        Timer::Reset();
        std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(length,
                length, vessel_length);

        Timer::PrintAndReset("Network set up");

        // Write the network to file
        OutputFileHandler output_file_handler("TestVesselNetworkGenerator/TestGenerateAndWriteMonolithicHexagonalNetwork");
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("network.vtp");
        p_network->Write(output_filename);
    }

    void TestGenerateAndWriteHexagonalNetwork() throw (Exception)
    {
        // Specify the network dimensions
        QLength vessel_length = 5.0* 1_um;

        // Generate the network
        VesselNetworkGenerator<2> vascular_network_generator;
        std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalUnit(vessel_length);

        // Pattern the unit
        std::vector<unsigned> num_units(2,3);
        vascular_network_generator.PatternUnitByTranslation(vascular_network, num_units);

        // Write the network to file
        OutputFileHandler output_file_handler("TestVesselNetworkGenerator", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexagonalVesselNetwork.vtp");
        vascular_network->Write(output_filename);
    }

    void TestGenerate3dHexagonalNetwork() throw (Exception)
    {
        // Specify the network dimensions
        QLength vessel_length = 40.0* 1_um;

        // Generate the network
        VesselNetworkGenerator<3> vascular_network_generator;
        std::shared_ptr<VesselNetwork<3> > vascular_network = vascular_network_generator.GenerateHexagonalUnit(vessel_length);

        // Pattern the unit
        std::vector<unsigned> num_units(3, 3);
        vascular_network_generator.PatternUnitByTranslation(vascular_network, num_units);

        // Write the network to file
        OutputFileHandler output_file_handler("TestVesselNetworkGenerator", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexagonalVesselNetwork3d.vtp");
        vascular_network->Write(output_filename);
    }

    void TestParallelNetworks() throw (Exception)
    {
        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        QArea target_density(1.e-4/(1_um));
        QLength exclusion_distance(20.0*unit::microns);

        p_part->AddCuboid(1000.0* 1_um,
                          1000.0* 1_um,
                          50.0* 1_um,
                          DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        VesselNetworkGenerator<3> network_generator;
        std::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateParrallelNetwork(p_part,
                                                                                                    target_density,
                                                                                                        VesselDistribution::REGULAR);
        OutputFileHandler output_file_handler("TestVesselNetworkGenerator/Parallel", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("RegularNetwork.vtp");
        p_network->Write(output_filename);

        std::shared_ptr<VesselNetwork<3> > p_network2 = network_generator.GenerateParrallelNetwork(p_part,
                                                                                                     target_density,
                                                                                                        VesselDistribution::UNIFORM);

        std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("UniformNetwork.vtp");
        p_network2->Write(output_filename2);

        std::shared_ptr<VesselNetwork<3> > p_network3 = network_generator.GenerateParrallelNetwork(p_part,
                                                                                                     target_density,
                                                                                                        VesselDistribution::UNIFORM,
                                                                                                        exclusion_distance);

        std::string output_filename3 = output_file_handler.GetOutputDirectoryFullPath().append("UniformExclusionNetwork.vtp");
        p_network3->Write(output_filename3);

        std::shared_ptr<VesselNetwork<3> > p_network4 = network_generator.GenerateParrallelNetwork(p_part,
                                                                                                     target_density,
                                                                                                        VesselDistribution::TWO_LAYER);

        std::string output_filename4 = output_file_handler.GetOutputDirectoryFullPath().append("TwoLayerNetwork.vtp");
        p_network3->Write(output_filename4);

        std::shared_ptr<VesselNetwork<3> > p_network5 = network_generator.GenerateParrallelNetwork(p_part,
                                                                                                     target_density,
                                                                                                        VesselDistribution::TWO_LAYER,
                                                                                                        exclusion_distance);

        std::string output_filename5 = output_file_handler.GetOutputDirectoryFullPath().append("TwoLayerExclusionNetwork.vtp");
        p_network3->Write(output_filename5);
    }
};

#endif /*TESTVESSELNETWORKGENERATOR_HPP_*/
