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

#ifndef TESTVESSELSURFACEGENERATOR_HPP_
#define TESTVESSELSURFACEGENERATOR_HPP_

#include <cxxtest/TestSuite.h>
#include <map>
#include <math.h>
#include <vector>
#include <string>
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "SmartPointers.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "GeometryWriter.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestGenerateVtkVesselSurface : public CxxTest::TestSuite
{
public:

    void TestSingleSegmentVessel() throw(Exception)
    {
        QLength vessel_length = 100.0 * 1_um;
        VesselNetworkGenerator<3> generator;
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, Vertex<3>(0.0, 0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0e-6 * unit::metres);

        // Set up the surface generator
        VesselSurfaceGenerator<3> surface_generator(p_network);

        OutputFileHandler output_file_handler("TestVesselSurfaceGenerator", true);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("SingleSegmentVessel.vtp");

        GeometryWriter writer;
        writer.AddInput(surface_generator.GetVtkSurface());
        writer.SetFileName(output_filename);
        writer.Write();
    }

    void TestMultiSegmentVessel() throw(Exception)
    {
        double vessel_length = 100.0;
        std::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3> ::Create(0.0, 0.0, 0.0);
        std::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3> ::Create(vessel_length, 0.0, 0.0);
        std::shared_ptr<VesselNode<3> > p_node3 = VesselNode<3> ::Create(2.0*vessel_length, vessel_length, 0);
        std::shared_ptr<VesselNode<3> > p_node4 = VesselNode<3> ::Create(3.0*vessel_length, vessel_length, vessel_length);

        p_node1->SetRadius(10.0e-6 * unit::metres);
        p_node2->SetRadius(10.0e-6 * unit::metres);
        p_node3->SetRadius(10.0e-6 * unit::metres);
        p_node4->SetRadius(10.0e-6 * unit::metres);

        std::shared_ptr<VesselSegment<3> > p_segment1 = VesselSegment<3>::Create(p_node1, p_node2);
        std::shared_ptr<VesselSegment<3> > p_segment2 = VesselSegment<3>::Create(p_node2, p_node3);
        std::shared_ptr<VesselSegment<3> > p_segment3 = VesselSegment<3>::Create(p_node3, p_node4);

        std::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(p_segment1);
        p_vessel1->AddSegment(p_segment2);
        p_vessel1->AddSegment(p_segment3);

        std::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);

        VesselSurfaceGenerator<3> surface_generator(p_network);
        OutputFileHandler output_file_handler("TestVesselSurfaceGenerator", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("MultiSegmentVessel.vtp");

        GeometryWriter writer;
        writer.AddInput(surface_generator.GetVtkSurface());
        writer.SetFileName(output_filename);
        writer.Write();
    }

    void TestSinusoidVessel() throw(Exception)
    {
        double vessel_length = 400.0;
        unsigned num_segments= 10;
        double segment_length = vessel_length / double(num_segments);

        std::vector<std::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(vessel_length/10.0, 0.0, 0.0));
        nodes[0]->SetRadius(10.0e-6 * unit::metres);
        for(unsigned idx=0; idx<num_segments+1; idx++)
        {
            double x_position = vessel_length/10.0 * std::cos(double(idx) * segment_length * (2.0 * M_PI/vessel_length));
            double z_position = double(idx) * segment_length + segment_length;
            nodes.push_back(VesselNode<3>::Create(x_position, 0.0, z_position));
            nodes[idx+1]->SetRadius(10.0e-6 * unit::metres);
        }
        nodes.push_back(VesselNode<3>::Create(vessel_length/10.0, 0.0, (double(num_segments) + 1 ) * segment_length + segment_length));
        nodes[nodes.size()-1]->SetRadius(10.0e-6 * unit::metres);

        std::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(nodes);
        std::shared_ptr<VesselNetwork<3> > p_network = std::shared_ptr<VesselNetwork<3> >(new VesselNetwork<3>());
        p_network->AddVessel(p_vessel1);

        VesselSurfaceGenerator<3> surface_generator(p_network);
        OutputFileHandler output_file_handler("TestVesselSurfaceGenerator", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("SinusoidalVessel.vtp");

        GeometryWriter writer;
        writer.AddInput(surface_generator.GetVtkSurface());
        writer.SetFileName(output_filename);
        writer.Write();
    }

    void TestMultiVessel() throw(Exception)
    {
        double vessel_length = 100.0;
        std::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3> ::Create(-vessel_length, 0.0, 0.0);
        std::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3> ::Create(0.0, 0.0, 0.0);
        std::shared_ptr<VesselNode<3> > p_node3 = VesselNode<3> ::Create(vessel_length, 0.0, 0.0);
        std::shared_ptr<VesselNode<3> > p_node4 = VesselNode<3> ::Create(0.0, vessel_length, 0.0);
        std::shared_ptr<VesselNode<3> > p_node5 = VesselNode<3> ::Create(0.0, -vessel_length, 0.0);
        std::shared_ptr<VesselNode<3> > p_node6 = VesselNode<3> ::Create(0.0, 0.0, vessel_length);
        std::shared_ptr<VesselNode<3> > p_node7 = VesselNode<3> ::Create(0.0, 0.0, -vessel_length);

        p_node1->SetRadius(10.0e-6 * unit::metres);
        p_node2->SetRadius(10.0e-6 * unit::metres);
        p_node3->SetRadius(10.0e-6 * unit::metres);
        p_node4->SetRadius(10.0e-6 * unit::metres);
        p_node5->SetRadius(10.0e-6 * unit::metres);
        p_node6->SetRadius(10.0e-6 * unit::metres);
        p_node7->SetRadius(10.0e-6 * unit::metres);

        std::shared_ptr<VesselSegment<3> > p_segment1 = VesselSegment<3>::Create(p_node2, p_node1);
        std::shared_ptr<VesselSegment<3> > p_segment2 = VesselSegment<3>::Create(p_node2, p_node3);
        std::shared_ptr<VesselSegment<3> > p_segment3 = VesselSegment<3>::Create(p_node2, p_node4);
        std::shared_ptr<VesselSegment<3> > p_segment4 = VesselSegment<3>::Create(p_node2, p_node5);
        std::shared_ptr<VesselSegment<3> > p_segment5 = VesselSegment<3>::Create(p_node2, p_node6);
        std::shared_ptr<VesselSegment<3> > p_segment6 = VesselSegment<3>::Create(p_node2, p_node7);

        std::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(p_segment1);
        std::shared_ptr<Vessel<3> > p_vessel2 = Vessel<3>::Create(p_segment2);
        std::shared_ptr<Vessel<3> > p_vessel3 = Vessel<3>::Create(p_segment3);
        std::shared_ptr<Vessel<3> > p_vessel4 = Vessel<3>::Create(p_segment4);
        std::shared_ptr<Vessel<3> > p_vessel5 = Vessel<3>::Create(p_segment5);
        std::shared_ptr<Vessel<3> > p_vessel6 = Vessel<3>::Create(p_segment6);

        std::shared_ptr<VesselNetwork<3> > p_network = std::shared_ptr<VesselNetwork<3> >(new VesselNetwork<3>());
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);
        p_network->AddVessel(p_vessel4);
        p_network->AddVessel(p_vessel5);
        p_network->AddVessel(p_vessel6);

        //Set up the surface generator
        VesselSurfaceGenerator<3> surface_generator(p_network);
        OutputFileHandler output_file_handler("TestVesselSurfaceGenerator", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("MultiVessel.vtp");

        GeometryWriter writer;
        writer.AddInput(surface_generator.GetVtkSurface());
        writer.SetFileName(output_filename);
        writer.Write();
    }

    void Test2dFails() throw(Exception)
    {
        QLength vessel_length = 100.0 * 1_um;
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length, Vertex<2>(0.0, 0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0e-6 * unit::metres);

        // Set up the surface generator
        VesselSurfaceGenerator<2> surface_generator(p_network);

        TS_ASSERT_THROWS_THIS(surface_generator.GetSurface(), "The surface generator currently only works in 3D");
    }
};

#endif /*TESTVESSELSURFACEGENERATOR_HPP_*/
