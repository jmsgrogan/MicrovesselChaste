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

#ifndef TESTPART_HPP_
#define TESTPART_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Part.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestPart : public CxxTest::TestSuite
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

        boost::shared_ptr<VesselNetwork<3> > p_network = boost::shared_ptr<VesselNetwork<3> >(new VesselNetwork<3>);
        p_network->AddVessels(vessels);
        return p_network;
    }

public:

    void TestAddRectangle()
    {
        Part<3> part = Part<3>();
        part.AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));

        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[0]->rGetLocation()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[0]->rGetLocation()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[1]->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[1]->rGetLocation()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[2]->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[2]->rGetLocation()[1], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[3]->rGetLocation()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[3]->rGetLocation()[1], 1.0, 1.e-6);
    }

    void TestAddCuboid()
    {
        Part<3> part = Part<3>();
        part.AddCuboid(1.e-6*unit::metres, 1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
    }

    void TestComposite2DPart()
    {
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));

        OutputFileHandler output_file_handler("TestPart");
        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("Composite2DPart.vtp"));
    }

    void TestExtrudePart()
    {
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        boost::shared_ptr<Polygon> p_circle = p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));
        p_part->Extrude(p_circle, 1.e-6*unit::metres);
        OutputFileHandler output_file_handler("TestPart", false);
        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("ExtrudePart.vtp"));
    }

    void TestAddParrallelVessels3d()
    {
        boost::shared_ptr<VesselNetwork<3> > p_network = SetUpNetwork();

        double vessel_length = 100;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;

        double domain_width = num_vessels_per_row * (spacing + 2.0* radius);
        double domain_height = num_vessels_per_row * (spacing + 2.0* radius);
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(domain_width*1.e-6*unit::metres,
                            domain_height*1.e-6*unit::metres,
                            vessel_length*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network);

        OutputFileHandler output_file_handler("TestPart", false);
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVessels3d.vtp"));
    }

    void TestAddParrallelVesselSurfaces3d()
    {
        boost::shared_ptr<VesselNetwork<3> > p_network = SetUpNetwork();

        double vessel_length = 100;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;

        double domain_width = num_vessels_per_row * (spacing + 2.0* radius);
        double domain_height = num_vessels_per_row * (spacing + 2.0* radius);
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(domain_width*1.e-6*unit::metres,
                            domain_height*1.e-6*unit::metres,
                            vessel_length*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network, true);

        OutputFileHandler output_file_handler("TestPart", false);
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselSurfaces3d.vtp"));
    }

    void TestAddVesselsSurface3dCylinder()
    {
        units::quantity<unit::length> vessel_length = 100.0 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0e-6 * unit::metres);

        Part<3> part = Part<3>();
        boost::shared_ptr<Polygon> p_circle = part.AddCircle(vessel_length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        part.Extrude(p_circle, vessel_length);
        part.AddVesselNetwork(p_network, true);

        OutputFileHandler output_file_handler("TestPart", false);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Vessels3dCylinderSurface.vtp"));
    }
};

#endif /*TESTPART_HPP_*/
