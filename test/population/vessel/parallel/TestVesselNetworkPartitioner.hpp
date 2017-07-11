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

#ifndef TESTVESSELNETWORKPARTITIONER_HPP_
#define TESTVESSELNETWORKPARTITIONER_HPP_

#include <cxxtest/TestSuite.h>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNetworkPartitioner.hpp"
#include "PetscTools.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "FlowSolver.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselNetworkPartitioner : public CxxTest::TestSuite
{
public:

    void TestSingleVessel() throw (Exception)
    {
        std::string output_directory = "TestVesselNetworkPartitioner";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory);
        std::vector<std::shared_ptr<VesselNode<2> > > nodes;
        nodes.push_back(VesselNode<2>::Create(0.0, 0.0, 0.0, 1_um));
        nodes.push_back(VesselNode<2>::Create(50.0, 0.0, 0.0, 1_um));
        nodes.push_back(VesselNode<2>::Create(150.0, 0.0, 0.0, 1_um));
        nodes.push_back(VesselNode<2>::Create(220.0, 0.0, 0.0, 1_um));

        std::shared_ptr<Vessel<2> > pVessel1 = Vessel<2>::Create(nodes);
        std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(pVessel1);

        VesselNetworkPartitioner<2> partitioner;
        partitioner.SetVesselNetwork(p_network);
        partitioner.Update();

        unsigned rank = PetscTools::GetMyRank();
        p_network->Write(output_file_handler.GetOutputDirectoryFullPath()+"network_" +
                boost::lexical_cast<std::string>(rank)+".vtp", false);
    }

    void TestHexagonalNetwork() throw (Exception)
    {
        std::string output_directory = "TestVesselNetworkPartitioner";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateHexagonalNetwork(500.0_um, 500.0_um, 50.0_um);

        VesselNetworkPartitioner<2> partitioner;
        partitioner.SetVesselNetwork(p_network);
        partitioner.Update();

        unsigned rank = PetscTools::GetMyRank();
        p_network->Write(output_file_handler.GetOutputDirectoryFullPath()+"hex_network_" +
                boost::lexical_cast<std::string>(rank)+".vtp", false);
    }

    void TestParallelFlowProblem() throw (Exception)
    {
        std::string output_directory = "TestVesselNetworkPartitioner/Flow";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        VesselNetworkGenerator<2> generator;

        QLength domain_size = 500.0*1_um;
        QLength vessel_length = 100.0*1_um;
        QLength reference_length = 1_um;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateHexagonalNetwork(domain_size,
                domain_size, vessel_length);

        VesselNetworkPartitioner<2> partitioner;
        partitioner.SetVesselNetwork(p_network);
        partitioner.Update();

        unsigned rank = PetscTools::GetMyRank();
        p_network->Write(output_file_handler.GetOutputDirectoryFullPath()+"pre_network_" +
                boost::lexical_cast<std::string>(rank)+".vtp", false);

        double impedance = 1.e12;
        p_network->GetVesselSegments()[0]->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_network->GetVesselSegments()[0]);
        VesselNetworkPropertyManager<2>::AssignInflows(p_network,
                Vertex<2>(0.0, 0.0, 0.0, reference_length), vessel_length/5.0);
        VesselNetworkPropertyManager<2>::AssignOutflows(p_network,
                Vertex<2>(domain_size/reference_length, domain_size/reference_length, 0.0, reference_length), vessel_length/5.0);
        VesselNetworkPropertyManager<2>::SetInflowPressures(p_network, 3393.0*unit::pascals);
        VesselNetworkPropertyManager<2>::SetOutflowPressures(p_network, 1000.5*unit::pascals);

        FlowSolver<2> solver;
        solver.SetVesselNetwork(p_network);
        solver.Solve();

        p_network->Write(output_file_handler.GetOutputDirectoryFullPath()+"post_network_" +
                boost::lexical_cast<std::string>(rank)+".vtp", false);
    }
};

#endif /*TESTVESSELNETWORKPARTITIONER_HPP_*/
