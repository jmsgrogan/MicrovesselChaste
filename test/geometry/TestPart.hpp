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

#ifndef TESTPART_HPP_
#define TESTPART_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Part.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "MappableGridGenerator.hpp"
#include "PetscTools.hpp"
#include "ObjectCommunicator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

template<unsigned DIM>
VesselNetworkPtr<DIM> GetVesselNetwork()
{
    auto p_start_top = VesselNode<DIM>::Create(10.0_um, 60.0_um, double(DIM-2)*50_um);
    auto p_end_top = VesselNode<DIM>::Create(90.0_um, 60.0_um, double(DIM-2)*50_um);
    auto p_start_bottom = VesselNode<DIM>::Create(10.0_um, 20.0_um, double(DIM-2)*50_um);
    auto p_end_bottom = VesselNode<DIM>::Create(100.0_um, 20.0_um, double(DIM-2)*50_um);
    auto p_top_vessel = Vessel<DIM>::Create(VesselSegment<DIM>::Create(p_start_top, p_end_top));
    auto p_bottom_vessel = Vessel<DIM>::Create(VesselSegment<DIM>::Create(p_start_bottom, p_end_bottom));
    auto p_network = VesselNetwork<DIM>::Create();
    p_network->AddVessel(p_top_vessel);
    p_network->AddVessel(p_bottom_vessel);
    p_top_vessel->GetSegment(0)->SetRadius(10_um);
    p_bottom_vessel->GetSegment(0)->SetRadius(10_um);
    return p_network;
}

class TestPart : public CxxTest::TestSuite
{

public:

    void TestAddRectangle()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory);

        Part<3> part;
        part.AddRectangle(1_um, 1_um);

        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[0]->Convert(1_um)[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[0]->Convert(1_um)[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[1]->Convert(1_um)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[1]->Convert(1_um)[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[2]->Convert(1_um)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[2]->Convert(1_um)[1], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[3]->Convert(1_um)[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[3]->Convert(1_um)[1], 1.0, 1.e-6);

        TS_ASSERT_DELTA(part.GetReferenceLengthScale()/1_m, 1.e-6, 1.e-8);
        TS_ASSERT_THROWS_THIS(part.GetFacet(Vertex<3>(4.0_um)), "No facet found at input location");
        part.SetReferenceLengthScale(10_um);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Rectangle.vtp"));
    }

    void TestAddCuboid()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        Part<3> part;
        part.AddCuboid(1_um, 1_um, 1_um);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Cuboid.vtp"));
    }

    void TestAddCylinder()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        Part<3> part;
        part.AddCylinder(1_um, 1_um);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Cylinder.vtp"));
    }

    void TestComposite2DPart()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_part = Part<3>::Create();
        p_part->AddRectangle(1_um, 1_um);
        p_part->AddCircle(0.33_um, Vertex<3>(0.5_um, 0.5_um));

        auto p_part2 = Part<3>::Create();
        p_part2->AddRectangle(1_um, 1_um);
        p_part2->AddPolygon(p_part->GetPolygons()[1], true);

        auto p_part3 = Part<3>::Create();
        p_part3->AddRectangle(1_um, 1_um);
        p_part3->AddPolygon(p_part->GetPolygons()[1], false);

        auto p_part4 = Part<3>::Create();
        p_part4->AddRectangle(1_um, 1_um);
        p_part4->AddPolygon(p_part->GetPolygons()[1], false, p_part4->GetFacets()[0]);

        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("Composite2DPart.vtp"));
    }

    void TestExtrudePart()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_part = Part<3>::Create();
        PolygonPtr<3> p_circle = p_part->AddCircle(0.33_um, Vertex<3>(0.5_um, 0.5_um));
        p_part->Extrude(p_circle, 1_um);
        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("ExtrudePart.vtp"));

        auto p_part2 = Part<2>::Create();
        PolygonPtr<2> p_circle2 = p_part2->AddCircle(0.33_um, Vertex<2>(0.5_um, 0.5_um));
        TS_ASSERT_THROWS_THIS(p_part2->Extrude(p_circle2, 1_um), "Only parts in 3D space can be extruded.");
    }

    void TestAddParrallelVesselsLines2d()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_network = GetVesselNetwork<2>();
        auto p_domain = Part<2>::Create();
        p_domain->AddRectangle(100_um, 100_um);
        p_domain->AddVesselNetwork(p_network);

        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselLines2d.vtp"));
        std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > segment_indices = p_domain->GetSegmentIndices();
        TS_ASSERT_EQUALS(segment_indices.size(), 4u);
    }

    void TestAddParrallelVesselLines3d()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_network = GetVesselNetwork<3>();
        auto p_domain = Part<3>::Create();
        p_domain->AddCuboid(100_um, 100_um, 100_um);
        p_domain->AddVesselNetwork(p_network);
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselLines3d.vtp"));
    }

    void TestAddParrallelVesselsSurfaces2d()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_network = GetVesselNetwork<2>();
        auto p_domain = Part<2>::Create();
        p_domain->AddRectangle(100_um, 100_um);
        TS_ASSERT_THROWS_THIS(p_domain->AddVesselNetwork(p_network, true), "The surface generator currently only works in 3D");
    }

    void TestAddParrallelVesselSurface3d()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        auto p_network = GetVesselNetwork<3>();
        auto p_domain = Part<3>::Create();
        p_domain->AddCuboid(100_um, 100_um, 100_um);
        p_domain->AddVesselNetwork(p_network, true);
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselSurface3d.vtp"));
    }

    void TestAddVesselsSurface3dCylinder()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        QLength vessel_length = 100_um;
        VesselNetworkGenerator<3> generator;
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length);
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5_um);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5_um);

        Part<3> part;
        PolygonPtr<3> p_circle = part.AddCircle(vessel_length);
        part.Extrude(p_circle, vessel_length);
        part.AddVesselNetwork(p_network, true);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Vessels3dCylinderSurface.vtp"));

        Part<3> part2;
        PolygonPtr<3> p_circle2 = part2.AddCircle(vessel_length);
        part2.Extrude(p_circle2, vessel_length);
        part2.AddVesselNetwork(p_network, true, true);
        part2.Write(output_file_handler.GetOutputDirectoryFullPath().append("Vessels3dCylinderSurfaceFill.vtp"));
    }

    void TestBooleanWithNetwork()
    {
        auto p_network = GetVesselNetwork<2>();
        auto p_domain = Part<2>::Create();
        p_domain->AddRectangle(50_um, 50_um);
        p_domain->BooleanWithNetwork(p_network);
        TS_ASSERT_EQUALS(p_network->GetNumberOfVessels(), 1u);
    }

    void TestContainingGridIndices()
    {
        Part<3> part;
        part.AddCuboid(10_um, 10_um, 10_um, Vertex<3>(0.5_um, 0.5_um, 0.5_um));

        std::vector<unsigned> containing_indices = part.GetContainingGridIndices(20, 20, 20, 1_um);
        TS_ASSERT_EQUALS(containing_indices.size(), 10u*10u*10u);
    }

    void TestGetSegmentIndices()
    {
        Part<3> part;
        part.AddCuboid(10_um, 10_um, 10_um);
    }

    void TestPointInPart()
    {
        Part<3> part;
        part.AddCuboid(10_um, 10_um, 10_um);
        TS_ASSERT(part.IsPointInPart(Vertex<3>(0.5_um, 0.5_um, 0.5_um)));

        auto p_facet = part.GetFacet(Vertex<3>(0.5_um, 0.5_um, 0.0_um));

        Part<2> part2;
        part2.AddRectangle(10_um, 10_um);

        vtkSmartPointer<vtkPoints> p_probes = vtkSmartPointer<vtkPoints>::New();
        p_probes->InsertNextPoint(0.5, 0.5, 0.0);
        TS_ASSERT(part2.IsPointInPart(p_probes)[0]);
    }

    void TestTranslate()
    {
        Part<3> part;
        part.AddCuboid(10_um, 10_um, 10_um);

        Vertex<3> translation_vector(10.0_um, 10.0_um, 10.0_um);
        part.Translate(translation_vector);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->rGetVertices()[0]->Convert(1_um)[0], 10.0, 1.e-6);
    }

    void TestLabelling()
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        // Circle 2D
        QLength cornea_radius = 1300_um;
        QLength cornea_thickness = 100_um;
        QLength pellet_thickness = 80_um;
        QLength pellet_height = 1000_um;
        QLength pellet_radius = 150_um;
        QLength delta = pellet_height-cornea_radius+pellet_radius;
        auto p_domain = Part<2> ::Create();
        p_domain->AddCircle(cornea_radius);

        p_domain->AddAttribute("Circle", 1.0);
        std::map<std::string, double> attributes = p_domain->GetAttributes();
        TS_ASSERT_DELTA(attributes["Circle"], 1.0, 1.e-6);

        PolygonPtr<2> p_polygon = p_domain->AddCircle(pellet_radius, Vertex<2>(0.0_m, -delta));
        p_polygon->AddAttribute("Pellet", 1.0);
        p_polygon->AddAttributeToAllEdges("Pellet Boundary", 1.0);
        for(auto& vert:p_domain->GetVertices())
        {
            vert->AddAttribute("Vert", 1.0);
        }

        p_domain->AddHoleMarker(Vertex<2>(0.0_m, -delta));
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_circle_2d.vtp", GeometryFormat::VTP, true);

        TS_ASSERT(!p_domain->EdgeHasAttribute(Vertex<2>(100.0_m, 100.0_m), "Nothing"));

        // Circle 3D
        auto p_circ_3d_domain = Part<3> ::Create();
        PolygonPtr<3> p_circle = p_circ_3d_domain->AddCircle(cornea_radius);
        p_circ_3d_domain->Extrude(p_circle, cornea_thickness);

        auto p_circ_3d_pellet = Part<3> ::Create();
        PolygonPtr<3> p_pellet_circle = p_circ_3d_pellet->AddCircle(pellet_radius, Vertex<3>(0.0_m, -1.0*delta));
        p_circ_3d_pellet->Extrude(p_pellet_circle, pellet_thickness);
        p_circ_3d_pellet->AddAttributeToPolygons("Pellet Interface",  1.0);
        p_circ_3d_domain->AppendPart(p_circ_3d_pellet);
        p_circ_3d_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_circle_3d.vtp",
                GeometryFormat::VTP, true);

        // Plane 2D
        std::vector<std::shared_ptr<Vertex<2> > > points;
        points.push_back(Vertex<2>::Create(0.0_m));
        points.push_back(Vertex<2>::Create(2.0*M_PI*cornea_radius));
        points.push_back(Vertex<2>::Create(2.0*M_PI*cornea_radius, pellet_height));
        points.push_back(Vertex<2>::Create(2.0*M_PI*cornea_radius/2.0 + 2.0*M_PI*pellet_radius/2.0, pellet_height));
        points.push_back(Vertex<2>::Create(2.0*M_PI*cornea_radius/2.0 - 2.0*M_PI*pellet_radius/2.0, pellet_height));
        points.push_back(Vertex<2>::Create(0.0_m, pellet_height));
        auto p_temp_polygon = Polygon<2>::Create(points);
        auto p_2d_planar_domain = Part<2>::Create();
        p_2d_planar_domain->AddPolygon(p_temp_polygon);
        p_2d_planar_domain->AddAttributeToEdgeIfFound(Vertex<2>(2.0*M_PI*cornea_radius/2.0, pellet_height), "Pellet Interface", 1.0);
        p_2d_planar_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_plane_2d.vtp", GeometryFormat::VTP, true);

        // Hemisphere 3D
        MappableGridGenerator<3> grid_generator;
        std::shared_ptr<Part<3> > hemisphere = grid_generator.GenerateHemisphere(cornea_radius,
                cornea_thickness, 20, 20, double(1.0*M_PI), double(0.999*M_PI));

        auto p_hemi_pellet = Part<3> ::Create();
        QLength gap = (cornea_thickness- pellet_thickness)/8.0;
        QLength base = cornea_radius + gap - cornea_thickness;
        p_hemi_pellet->AddCylinder(pellet_radius, pellet_thickness, Vertex<3>(0.0_m, 0.0_m, base));
        p_hemi_pellet->AddAttributeToPolygons("Pellet Interface",  1.0);
        hemisphere->AppendPart(p_hemi_pellet);
        hemisphere->Write(output_file_handler.GetOutputDirectoryFullPath() + "hemisphere.vtp", GeometryFormat::VTP, true);
    }

    void TestArchiving()
    {
#if BOOST_VERSION >= 105600
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("Part.arch");

        auto p_part = Part<3>::Create();
        p_part->AddCuboid(10_um, 10_um, 10_um);

        // Save archive
        {
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_part;
        }

        // Load archive
        {
            std::shared_ptr<Part<3> > p_part_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_part_from_archive;

            // Visual check
            p_part_from_archive->Write(output_file_handler.GetOutputDirectoryFullPath().append("PartFromArchive.vtp"));
        }
#endif
    }

    void TestSendAndReceive()
    {
#if BOOST_VERSION >= 105600
        EXIT_IF_SEQUENTIAL;

        std::string output_directory = "TestPartParallel";
        OutputFileHandler output_file_handler(output_directory, false);

        ObjectCommunicator<Part<2> > part_comm;
        if(PetscTools::GetMyRank() == 0)
        {
            auto p_domain = boost::shared_ptr<Part<2> >(new Part<2> ());
            p_domain->AddCircle(1_m);
            part_comm.SendObject(p_domain, 1, 6789);
        }
        else if(PetscTools::GetMyRank() == 1)
        {
            MPI_Status status;
            boost::shared_ptr<Part<2> > p_recv_domain = part_comm.RecvObject(0, 6789, status);

            GeometryWriter writer;
            writer.AddInput(p_recv_domain->GetVtk());
            writer.SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "receveid_part.vtp");
            writer.Write(false);
        }
#endif
    }
};

#endif /*TESTPART_HPP_*/
