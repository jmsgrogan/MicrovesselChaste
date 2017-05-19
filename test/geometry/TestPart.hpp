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

class TestPart : public CxxTest::TestSuite
{

public:

    void TestAddRectangle() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory);

        Part<3> part = Part<3>();
        part.AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0, 1.e-6*unit::metres));

        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[0]->GetLocation(1.e-6*unit::metres)[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[0]->GetLocation(1.e-6*unit::metres)[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[1]->GetLocation(1.e-6*unit::metres)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[1]->GetLocation(1.e-6*unit::metres)[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[2]->GetLocation(1.e-6*unit::metres)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[2]->GetLocation(1.e-6*unit::metres)[1], 1.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[3]->GetLocation(1.e-6*unit::metres)[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[3]->GetLocation(1.e-6*unit::metres)[1], 1.0, 1.e-6);

        TS_ASSERT_DELTA(part.GetReferenceLengthScale().value(), 1.e-6, 1.e-8);
        TS_ASSERT_THROWS_THIS(part.GetFacet(DimensionalChastePoint<3>(4.0, 0.0, 0.0, 1.e-6*unit::metres)), "No facet found at input location");
        part.SetReferenceLengthScale(10.e-6*unit::metres);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Rectangle.vtp"));
    }

    void TestAddCuboid() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        Part<3> part = Part<3>();
        part.AddCuboid(1.e-6*unit::metres, 1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Cuboid.vtp"));
    }

    void TestAddCylinder() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        Part<3> part = Part<3>();
        part.AddCylinder(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0, 1.e-6*unit::metres), 24);
        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Cylinder.vtp"));
    }

    void TestComposite2DPart() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));

        boost::shared_ptr<Part<3> > p_part2 = Part<3>::Create();
        p_part2->AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part2->AddPolygon(p_part->GetPolygons()[1], true);

        boost::shared_ptr<Part<3> > p_part3 = Part<3>::Create();
        p_part3->AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part3->AddPolygon(p_part->GetPolygons()[1], false);

        boost::shared_ptr<Part<3> > p_part4 = Part<3>::Create();
        p_part4->AddRectangle(1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part4->AddPolygon(p_part->GetPolygons()[1], false, p_part4->GetFacets()[0]);

        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("Composite2DPart.vtp"));
    }

    void TestExtrudePart() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        boost::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));
        p_part->Extrude(p_circle, 1.e-6*unit::metres);
        p_part->Write(output_file_handler.GetOutputDirectoryFullPath().append("ExtrudePart.vtp"));

        boost::shared_ptr<Part<2> > p_part2 = Part<2>::Create();
        boost::shared_ptr<Polygon<2> > p_circle2 = p_part2->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<2>(0.5, 0.5));
        TS_ASSERT_THROWS_THIS(p_part2->Extrude(p_circle2, 1.e-6*unit::metres), "Only parts in 3D space can be extruded.");
    }

    void TestAddParrallelVesselsLines2d() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<VesselNode<2> > p_start_top = VesselNode<2>::Create(0.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_top = VesselNode<2>::Create(90.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_start_bottom = VesselNode<2>::Create(10.0, 20.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_bottom = VesselNode<2>::Create(100.0, 20.0, 0.0);
        boost::shared_ptr<Vessel<2> > p_top_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_top, p_end_top));
        boost::shared_ptr<Vessel<2> > p_bottom_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_bottom, p_end_bottom));
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_top_vessel);
        p_network->AddVessel(p_bottom_vessel);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network);

        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselLines2d.vtp"));

        std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > segment_indices = p_domain->GetSegmentIndices();
        TS_ASSERT_EQUALS(segment_indices.size(), 4u);
    }

    void TestAddParrallelVesselLines3d() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<VesselNode<3> > p_start_top = VesselNode<3>::Create(10.0, 60.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_end_top = VesselNode<3>::Create(90.0, 60.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_start_bottom = VesselNode<3>::Create(10.0, 20.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_end_bottom = VesselNode<3>::Create(90.0, 20.0, 50.0);
        boost::shared_ptr<Vessel<3> > p_top_vessel = Vessel<3>::Create(VesselSegment<3>::Create(p_start_top, p_end_top));
        boost::shared_ptr<Vessel<3> > p_bottom_vessel = Vessel<3>::Create(VesselSegment<3>::Create(p_start_bottom, p_end_bottom));
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_top_vessel);
        p_network->AddVessel(p_bottom_vessel);

        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network);

        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselLines3d.vtp"));
    }

    void TestAddParrallelVesselsSurfaces2d() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<VesselNode<2> > p_start_top = VesselNode<2>::Create(10.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_top = VesselNode<2>::Create(90.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_start_bottom = VesselNode<2>::Create(10.0, 20.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_bottom = VesselNode<2>::Create(90.0, 20.0, 0.0);
        boost::shared_ptr<Vessel<2> > p_top_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_top, p_end_top));
        boost::shared_ptr<Vessel<2> > p_bottom_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_bottom, p_end_bottom));
        p_top_vessel->GetSegment(0)->SetRadius(10.0*1.e-6*unit::metres);
        p_bottom_vessel->GetSegment(0)->SetRadius(10.0*1.e-6*unit::metres);

        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_top_vessel);
        p_network->AddVessel(p_bottom_vessel);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        TS_ASSERT_THROWS_THIS(p_domain->AddVesselNetwork(p_network, true), "The surface generator currently only works in 3D");
    }

    void TestAddParrallelVesselSurface3d() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        boost::shared_ptr<VesselNode<3> > p_start_top = VesselNode<3>::Create(10.0, 60.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_end_top = VesselNode<3>::Create(90.0, 60.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_start_bottom = VesselNode<3>::Create(10.0, 20.0, 50.0);
        boost::shared_ptr<VesselNode<3> > p_end_bottom = VesselNode<3>::Create(90.0, 20.0, 50.0);
        boost::shared_ptr<Vessel<3> > p_top_vessel = Vessel<3>::Create(VesselSegment<3>::Create(p_start_top, p_end_top));
        boost::shared_ptr<Vessel<3> > p_bottom_vessel = Vessel<3>::Create(VesselSegment<3>::Create(p_start_bottom, p_end_bottom));
        p_top_vessel->GetSegment(0)->SetRadius(10.0*1.e-6*unit::metres);
        p_bottom_vessel->GetSegment(0)->SetRadius(10.0*1.e-6*unit::metres);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_top_vessel);
        p_network->AddVessel(p_bottom_vessel);

        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            100.0*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_domain->AddVesselNetwork(p_network, true);
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath().append("ParrallelVesselSurface3d.vtp"));
    }

    void TestAddVesselsSurface3dCylinder() throw(Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        units::quantity<unit::length> vessel_length = 100.0 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0e-6 * unit::metres);

        Part<3> part = Part<3>();
        boost::shared_ptr<Polygon<3> > p_circle = part.AddCircle(vessel_length, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        part.Extrude(p_circle, vessel_length);
        part.AddVesselNetwork(p_network, true);

        part.Write(output_file_handler.GetOutputDirectoryFullPath().append("Vessels3dCylinderSurface.vtp"));
    }

    void TestBooleanWithNetwork() throw(Exception)
    {
        boost::shared_ptr<VesselNode<2> > p_start_top = VesselNode<2>::Create(10.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_top = VesselNode<2>::Create(90.0, 60.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_start_bottom = VesselNode<2>::Create(10.0, 20.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_end_bottom = VesselNode<2>::Create(90.0, 20.0, 0.0);
        boost::shared_ptr<Vessel<2> > p_top_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_top, p_end_top));
        boost::shared_ptr<Vessel<2> > p_bottom_vessel = Vessel<2>::Create(VesselSegment<2>::Create(p_start_bottom, p_end_bottom));
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_top_vessel);
        p_network->AddVessel(p_bottom_vessel);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(50.0*1.e-6*unit::metres,
                            50.0*1.e-6*unit::metres,
                            DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        p_domain->BooleanWithNetwork(p_network);

        TS_ASSERT_EQUALS(p_network->GetNumberOfVessels(), 1u);
    }

    void TestContainingGridIndices() throw(Exception)
    {
        Part<3> part = Part<3>();
        part.AddCuboid(10.e-6*unit::metres, 10.e-6*unit::metres, 10.e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5, 0.5, 1.0e-6*unit::metres));

        std::vector<unsigned> containing_indices = part.GetContainingGridIndices(20, 20, 20, 1.0e-6*unit::metres);
        TS_ASSERT_EQUALS(containing_indices.size(), 10u*10u*10u);
    }

    void TestGetSegmentIndices() throw(Exception)
    {
        Part<3> part = Part<3>();
        part.AddCuboid(10.e-6*unit::metres, 10.e-6*unit::metres, 10.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
    }

    void TestPointInPart() throw(Exception)
    {
        Part<3> part = Part<3>();
        part.AddCuboid(10.e-6*unit::metres, 10.e-6*unit::metres, 10.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        TS_ASSERT(part.IsPointInPart(DimensionalChastePoint<3>(0.5, 0.5, 0.5)));

        Part<2> part2 = Part<2>();
        part2.AddRectangle(10.e-6*unit::metres, 10.e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0));

        vtkSmartPointer<vtkPoints> p_probes = vtkSmartPointer<vtkPoints>::New();
        p_probes->InsertNextPoint(0.5, 0.5, 0.0);
        TS_ASSERT(part2.IsPointInPart(p_probes)[0]);
    }

    void TestTranslate() throw(Exception)
    {
        Part<3> part = Part<3>();
        part.AddCuboid(10.e-6*unit::metres, 10.e-6*unit::metres, 10.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        DimensionalChastePoint<3> translation_vector(10.0, 10.0, 10.0, 1e-6*unit::metres);
        part.Translate(translation_vector);

        TS_ASSERT_DELTA(part.GetPolygons()[0]->GetVertices()[0]->GetLocation(1e-6*unit::metres)[0], 10.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
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

        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(10.e-6*unit::metres, 10.e-6*unit::metres, 10.e-6*unit::metres,
                DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        // Save archive
        {
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_part;
        }

        // Load archive
        {
            boost::shared_ptr<Part<3> > p_part_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_part_from_archive;

            // Visual check
            p_part_from_archive->Write(output_file_handler.GetOutputDirectoryFullPath().append("PartFromArchive.vtp"));
        }
    }

    void TestLabelling() throw (Exception)
    {
        std::string output_directory = "TestPart";
        if(PetscTools::IsParallel())
        {
            output_directory += "Parallel";
        }
        OutputFileHandler output_file_handler(output_directory, false);

        // Circle 2D
        units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
        units::quantity<unit::length> cornea_radius = 1300*1e-6*unit::metres;
        units::quantity<unit::length> cornea_thickness = 100*1e-6*unit::metres;
        units::quantity<unit::length> pellet_thickness = 80*1e-6*unit::metres;
        units::quantity<unit::length> pellet_height = 1000*1e-6*unit::metres;
        units::quantity<unit::length> pellet_radius = 150*1e-6*unit::metres;
        units::quantity<unit::length> delta = pellet_height-cornea_radius+pellet_radius;
        boost::shared_ptr<Part<2> > p_domain = Part<2> ::Create();
        p_domain->AddCircle(cornea_radius, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<Polygon<2> > p_polygon = p_domain->AddCircle(pellet_radius,
                DimensionalChastePoint<2>(0.0, -delta/reference_length, 0.0, reference_length));
        p_polygon->AddAttribute("Pellet", 1.0);
        p_polygon->AddAttributeToAllEdges("Pellet Boundary", 1.0);

        p_domain->AddHoleMarker(DimensionalChastePoint<2>(0.0, -delta/reference_length, 0.0, reference_length));
        p_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_circle_2d.vtp", GeometryFormat::VTP, true);

        // Circle 3D
        boost::shared_ptr<Part<3> > p_circ_3d_domain = Part<3> ::Create();
        boost::shared_ptr<Polygon<3> > p_circle = p_circ_3d_domain->AddCircle(cornea_radius, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        p_circ_3d_domain->Extrude(p_circle, cornea_thickness);

        boost::shared_ptr<Part<3> > p_circ_3d_pellet = Part<3> ::Create();
        boost::shared_ptr<Polygon<3> > p_pellet_circle = p_circ_3d_pellet->AddCircle(pellet_radius,
                        DimensionalChastePoint<3>(0.0, -1.0*delta/reference_length, 0.0, reference_length));
        p_circ_3d_pellet->Extrude(p_pellet_circle, pellet_thickness);
        p_circ_3d_pellet->AddAttributeToPolygons("Pellet Interface",  1.0);
        p_circ_3d_domain->AppendPart(p_circ_3d_pellet);
        p_circ_3d_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_circle_3d.vtp", GeometryFormat::VTP, true);

        // Plane 2D
        std::vector<boost::shared_ptr<DimensionalChastePoint<2> > > points;
        points.push_back(DimensionalChastePoint<2>::Create(0.0, 0.0, 0.0, reference_length));
        points.push_back(DimensionalChastePoint<2>::Create(2.0*M_PI*cornea_radius/reference_length, 0.0, 0.0, reference_length));
        points.push_back(DimensionalChastePoint<2>::Create(2.0*M_PI*cornea_radius/reference_length, pellet_height/reference_length, 0.0,
                reference_length));
        points.push_back(DimensionalChastePoint<2>::Create(2.0*M_PI*cornea_radius/(2.0*reference_length) + 2.0*M_PI*pellet_radius/(2.0*reference_length), pellet_height/reference_length, 0.0,
                reference_length));
        points.push_back(DimensionalChastePoint<2>::Create(2.0*M_PI*cornea_radius/(2.0*reference_length) - 2.0*M_PI*pellet_radius/(2.0*reference_length), pellet_height/reference_length, 0.0,
                reference_length));
        points.push_back(DimensionalChastePoint<2>::Create(0.0, pellet_height/reference_length, 0.0, reference_length));
        boost::shared_ptr<Polygon<2> > p_temp_polygon = Polygon<2>::Create(points);
        boost::shared_ptr<Part<2> > p_2d_planar_domain = Part<2>::Create();
        p_2d_planar_domain->AddPolygon(p_temp_polygon);
        p_2d_planar_domain->AddAttributeToEdgeIfFound(DimensionalChastePoint<2>(2.0*M_PI*cornea_radius/(2.0*reference_length),
                pellet_height/reference_length, 0, reference_length), "Pellet Interface", 1.0);
        p_2d_planar_domain->Write(output_file_handler.GetOutputDirectoryFullPath() + "labelled_plane_2d.vtp", GeometryFormat::VTP, true);

        // Hemisphere 3D
        MappableGridGenerator grid_generator;
        boost::shared_ptr<Part<3> > hemisphere = grid_generator.GenerateHemisphere(cornea_radius,
                cornea_thickness, 20, 20, double(1.0*M_PI), double(0.999*M_PI));

        boost::shared_ptr<Part<3> > p_hemi_pellet = Part<3> ::Create();
        double gap = (cornea_thickness- pellet_thickness)/(2.0*reference_length)/4.0;
        double base = cornea_radius/reference_length + gap - cornea_thickness/reference_length;
        p_hemi_pellet->AddCylinder(pellet_radius,
                                  pellet_thickness,
                                  DimensionalChastePoint<3>(0.0, 0.0, base, reference_length));
        p_hemi_pellet->AddAttributeToPolygons("Pellet Interface",  1.0);
        hemisphere->AppendPart(p_hemi_pellet);
        hemisphere->Write(output_file_handler.GetOutputDirectoryFullPath() + "hemisphere.vtp", GeometryFormat::VTP, true);
    }

    void TestSendAndReceive() throw (Exception)
    {
        EXIT_IF_SEQUENTIAL;

        std::string output_directory = "TestPartParallel";
        OutputFileHandler output_file_handler(output_directory, false);

        ObjectCommunicator<Part<2> > part_comm;
        if(PetscTools::GetMyRank() == 0)
        {
            boost::shared_ptr<Part<2> > p_domain = Part<2> ::Create();
            p_domain->AddCircle(1.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
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
    }
};

#endif /*TESTPART_HPP_*/
