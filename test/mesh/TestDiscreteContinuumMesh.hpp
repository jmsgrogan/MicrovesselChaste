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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#ifndef TESTDISCRETECONTINUUMMESH_HPP_
#define TESTDISCRETECONTINUUMMESH_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Part.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "VtkMeshWriter.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"

class TestDiscreteContinuumMesh : public CxxTest::TestSuite
{
private:

    boost::shared_ptr<VesselNetwork<3> > SetUpNetwork()
    {
        double vessel_length = 100;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;
        std::vector<boost::shared_ptr<VesselNode<3> > > start_nodes;
        std::vector<boost::shared_ptr<VesselNode<3> > > end_nodes;

        for(unsigned idx =0; idx<num_vessels_per_row; idx++)
        {
            for(unsigned jdx =0; jdx<num_vessels_per_row; jdx++)
            {
                double x_position = (spacing+2.0*radius) * double(idx) + spacing/2.0 + radius;
                double y_position = (spacing+2.0*radius) * double(jdx) + spacing/2.0 + radius;
                start_nodes.push_back(VesselNode<3>::Create(x_position, y_position, 0.0));
                end_nodes.push_back(VesselNode<3>::Create(x_position, y_position, vessel_length));
            }
        }

        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx = 0; idx<start_nodes.size(); idx++)
        {
            start_nodes[idx]->SetRadius(radius * 1.e-6 * unit::metres);
            end_nodes[idx]->SetRadius(radius * 1.e-6 * unit::metres);
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(start_nodes[idx], end_nodes[idx])));
            vessels[idx]->GetSegments()[0]->SetRadius(10.0 * 1.e-6 * unit::metres);
        }

        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessels(vessels);
        return p_network;
    }

public:

    void TestMeshCylinder()
    {
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        boost::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));
        p_part->Extrude(p_circle, 1.e-6 * unit::metres);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(20.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter<3, 3> mesh_writer("TestDiscreteContinuumMesh", "Cylinder", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    // Not Supported With Tetgen <1.5
    void DontTestMeshCylinderWithVesselLine()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        boost::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(100.0* 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Extrude(p_circle, 100.0*1.e-6*unit::metres);
        p_part->AddVesselNetwork(p_network);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter<3, 3> mesh_writer("TestDiscreteContinuumMesh", "CylinderWithVesselLine", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    void TestMeshCylinderWithVesselSurface()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        boost::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(100.0* 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Extrude(p_circle, 100.0*1.e-6*unit::metres);
        p_part->AddVesselNetwork(p_network, true);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter < 3, 3 > mesh_writer("TestDiscreteContinuumMesh", "CylinderWithVesselSurface", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    // Not Supported With Tetgen <1.5
    void DontTestMeshCubeWithVesselLine()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0* 1.e-6 * unit::metres), vessel_length/(2.0* 1.e-6 * unit::metres));
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(vessel_length, vessel_length, vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddVesselNetwork(p_network);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter <3, 3> mesh_writer("TestDiscreteContinuumMesh", "CubeWithVesselLine", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    void TestMeshCubeWithVesselSurface()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0* 1.e-6 * unit::metres), vessel_length/(2.0* 1.e-6 * unit::metres));
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);

        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(2.0 * vessel_length, 2.0 * vessel_length, vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddVesselNetwork(p_network, true);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter<3, 3> mesh_writer("TestDiscreteContinuumMesh", "CubeWithVesselSurface", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    void DontTestMeshCubeWithVesselSurfaceInternal()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0* 1.e-6 * unit::metres), vessel_length/(2.0* 1.e-6 * unit::metres));
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0 * 1.e-6 * unit::metres);

        DimensionalChastePoint<3> translate(0.0, 0.0, -vessel_length/(2.0* 1.e-6 * unit::metres), 1.e-6 * unit::metres);
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(vessel_length, vessel_length, 2.0*vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Translate(translate);
        p_part->AddVesselNetwork(p_network, true);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter<3, 3> mesh_writer("TestDiscreteContinuumMesh", "CubeWithVesselSurface", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    void DontTestParrallelVesselSurfaceCube()
    {
        units::quantity<unit::length> vessel_length = 100.0* 1.e-6 * unit::metres;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;

        double domain_width = num_vessels_per_row * (spacing + 2.0* radius);
        double domain_height = num_vessels_per_row * (spacing + 2.0* radius);
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(domain_width* 1.e-6 * unit::metres, domain_height* 1.e-6 * unit::metres, vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddVesselNetwork(SetUpNetwork(), true);

        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*units::pow<3>(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        VtkMeshWriter < 3, 3 > mesh_writer("TestDiscreteContinuumMesh", "ParrallelVesselSurface", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }
};

#endif /*TESTDISCRETECONTINUUMMESH_HPP_*/
