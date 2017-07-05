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

#ifndef TESTDISCRETECONTINUUMMESH_HPP_
#define TESTDISCRETECONTINUUMMESH_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Part.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"
#include "PetscTools.hpp"
#include "MappableGridGenerator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestDiscreteContinuumMesh : public CxxTest::TestSuite
{
private:

    std::shared_ptr<VesselNetwork<3> > SetUpNetwork()
    {
        double vessel_length = 100;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;
        std::vector<std::shared_ptr<VesselNode<3> > > start_nodes;
        std::vector<std::shared_ptr<VesselNode<3> > > end_nodes;

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

        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx = 0; idx<start_nodes.size(); idx++)
        {
            start_nodes[idx]->SetRadius(radius * 1.e-6 * unit::metres);
            end_nodes[idx]->SetRadius(radius * 1.e-6 * unit::metres);
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(start_nodes[idx], end_nodes[idx])));
            vessels[idx]->GetSegments()[0]->SetRadius(10.0 * 1.e-6 * unit::metres);
        }

        std::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessels(vessels);
        return p_network;
    }

public:

    void TestMeshCircleInCirle() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/Circle");
        std::shared_ptr<Part<2> > p_part = Part<2>::Create();
        std::shared_ptr<Polygon<2> > p_circle = p_part->AddCircle(0.33e-6*unit::metres,
                DimensionalChastePoint<2>(0.5, 0.5));
        p_circle->AddAttributeToAllEdges("Outer Boundary", 1.0);

        std::shared_ptr<Polygon<2> > p_circle2 = p_part->AddCircle(0.1e-6*unit::metres,
                DimensionalChastePoint<2>(0.5, 0.5));
        p_part->AddRegionMarker(DimensionalChastePoint<2>(0.5, 0.5), 1.0);
        p_part->GetVtk(true);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp", GeometryFormat::VTP, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(5.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<2> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"circle");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();

        // Add a hole
        std::vector<DimensionalChastePoint<2> > holes;
        holes.push_back(DimensionalChastePoint<2>(0.5, 0.5));
        p_mesh_generator->SetHoles(holes);
        p_mesh_generator->Update();
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"circle_hole");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestMeshCylinder() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/Cylinder");

        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        std::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(0.33e-6*unit::metres, DimensionalChastePoint<3>(0.5, 0.5));
        p_part->Extrude(p_circle, 1.e-6 * unit::metres);
        p_part->AddRegionMarker(DimensionalChastePoint<3>(0.5, 0.5, 1.0, 1.e-6*unit::metres), 2.0);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp");

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(20.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cylinder");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh());
        mesh_writer.Write();

        unsigned local_proc_index = PetscTools::GetMyRank();
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cylinder_part"+
                boost::lexical_cast<std::string>(local_proc_index));
        mesh_writer.SetMesh(vtkUnstructuredGrid::SafeDownCast(p_mesh_generator->GetMesh()->GetVtkGrid()));
        mesh_writer.Write();
    }

    void TestMeshCylinderWithVesselSurface() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/CylinderWithVesselSurface");

        QLength vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0 * 1.e-6 * unit::metres);

        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        std::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(100.0* 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Extrude(p_circle, 100.0*1.e-6*unit::metres);
        p_part->AddVesselNetwork(p_network, true);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp");

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cylinder");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestMeshCylinderWithVesselSurfaceNoHole() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/CylinderWithVesselSurfaceNoHole");

        QLength vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length,
                                                                                        DimensionalChastePoint<3>(0.0, 0.0));
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(5.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(5.0 * 1.e-6 * unit::metres);

        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        std::shared_ptr<Polygon<3> > p_circle = p_part->AddCircle(100.0* 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Extrude(p_circle, 100.0*1.e-6*unit::metres);
        p_part->AddVesselNetwork(p_network, true, false);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp");

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cylinder");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestMeshCubeWithVesselSurface() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/CubeWithVesselSurface");

        QLength vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0* 1.e-6 * unit::metres), vessel_length/(2.0* 1.e-6 * unit::metres));
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);

        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0 * 1.e-6 * unit::metres);

        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(2.0 * vessel_length, 2.0 * vessel_length, vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddVesselNetwork(p_network, true);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp");

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cube");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestMeshCubeWithVesselSurfaceInternal() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/CubeWithVesselSurface", false);

        QLength vessel_length = 100.0* 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        DimensionalChastePoint<3> centre(vessel_length/(2.0* 1.e-6 * unit::metres), vessel_length/(2.0* 1.e-6 * unit::metres));
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(vessel_length, centre);
        p_network->GetVessels()[0]->GetStartNode()->SetRadius(10.0 * 1.e-6 * unit::metres);
        p_network->GetVessels()[0]->GetEndNode()->SetRadius(10.0 * 1.e-6 * unit::metres);

        DimensionalChastePoint<3> translate(0.0, 0.0, -vessel_length/(2.0* 1.e-6 * unit::metres), 1.e-6 * unit::metres);
        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(vessel_length, vessel_length, 2.0*vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->Translate(translate);
        p_part->AddVesselNetwork(p_network, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"cube_internal");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestParrallelVesselSurfaceCube() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/ParrallelVesselSurface");

        QLength vessel_length = 100.0* 1.e-6 * unit::metres;
        double radius = 10.0;
        double spacing = 3.0 * radius;
        unsigned num_vessels_per_row = 5;

        double domain_width = num_vessels_per_row * (spacing + 2.0* radius);
        double domain_height = num_vessels_per_row * (spacing + 2.0* radius);
        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(domain_width* 1.e-6 * unit::metres, domain_height* 1.e-6 * unit::metres, vessel_length, DimensionalChastePoint<3>(0.0, 0.0));
        p_part->AddVesselNetwork(SetUpNetwork(), true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"parallel");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestPatchOnFace() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/PatchOnFace");

        QLength domain_width = 500.0e-6*unit::metres;
        QLength pellet_width = 100.0e-6*unit::metres;
        QLength domain_depth = 50.0e-6*unit::metres;
        QLength pellet_depth = 10.0e-6*unit::metres;
        QLength reference_length = 1.0e-6*unit::metres;

        std::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(domain_width, domain_width, domain_depth, DimensionalChastePoint<3>(0.0, 0.0));

        double left_coord = (domain_width-pellet_width)/(2.0*reference_length);
        double right_coord = (domain_width+pellet_width)/(2.0*reference_length);
        double gap = (domain_depth-pellet_depth)/(2.0*reference_length);

        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > points;
        points.push_back(DimensionalChastePoint<3>::Create(left_coord, domain_width/reference_length, gap, reference_length));
        points.push_back(DimensionalChastePoint<3>::Create(right_coord, domain_width/reference_length, gap, reference_length));
        points.push_back(DimensionalChastePoint<3>::Create(right_coord, domain_width/reference_length, domain_depth/reference_length-gap, reference_length));
        points.push_back(DimensionalChastePoint<3>::Create(left_coord, domain_width/reference_length, domain_depth/reference_length-gap, reference_length));

        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(points);
        p_polygon->AddAttribute("Pellet Interface", 1.0);

        std::vector<std::shared_ptr<Facet<3> > > facets = p_part->GetFacets();
        DimensionalChastePoint<3> probe = p_polygon->GetCentroid();
        c_vector<double, 3> prob_norm = probe.GetLocation(reference_length);

        std::cout << prob_norm[0] << "," << prob_norm[1] << "," << prob_norm[2] << std::endl;

        for(unsigned idx=0;idx<facets.size();idx++)
        {
            QLength distance = facets[idx]->GetCentroid().GetDistance(probe);
            c_vector<double, 3> facet_loc = facets[idx]->GetCentroid().GetLocation(reference_length);

            std::cout << facet_loc[0] << "," << facet_loc[1] << "," << facet_loc[2] << std::endl;
            if(distance/reference_length <1.e-3)
            {
                p_part->AddPolygon(p_polygon, false, facets[idx]);
            }
        }
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"patch.vtp", GeometryFormat::VTP, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(100.0*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"patch");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestPartInPart() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/Hemisphere");

        MappableGridGenerator<3> generator;
        unsigned num_divisions_x = 20;
        unsigned num_divisions_y = 20;
        double azimuth_angle = 1.0 * M_PI;
        double polar_angle = 0.999 * M_PI;
        std::vector<DimensionalChastePoint<3> > holes;

        QLength cornea_radius = 1300*1.e-6*unit::metres;
        QLength cornea_thickness = 100*1.e-6*unit::metres;
        QLength pellet_radius = 200*1.e-6*unit::metres;
        QLength pellet_thickness = 50*1.e-6*unit::metres;
        QLength reference_length = 1.e-6*unit::metres;
        std::shared_ptr<Part<3> > p_domain = generator.GenerateHemisphere(cornea_radius,
                cornea_thickness, num_divisions_x, num_divisions_y, azimuth_angle, polar_angle);

        double gap = (cornea_thickness - pellet_thickness)/(2.0*reference_length)/4.0;
        double base = cornea_radius/reference_length + gap - cornea_thickness/reference_length;

        std::shared_ptr<Part<3> > p_pellet = Part<3>::Create();
        p_pellet->AddCylinder(pellet_radius,pellet_thickness,
                              DimensionalChastePoint<3>(0.0, 0.0, base, reference_length));

        // Rotate the pellet
        double rotation_angle = M_PI/8.0;
        c_vector<double, 3> axis;
        axis[0] = 0.0;
        axis[1] = 1.0;
        axis[2] = 0.0;
        p_pellet->RotateAboutAxis(axis, rotation_angle);

        DimensionalChastePoint<3> centre(0.0, 0.0, base + pellet_thickness/(2.0*reference_length), reference_length);
        centre.RotateAboutAxis(axis, rotation_angle);

        p_domain->AppendPart(p_pellet);
        p_domain->AddHoleMarker(DimensionalChastePoint<3>(centre));

        std::vector<std::shared_ptr<Polygon<3> > > polygons = p_pellet->GetPolygons();
        for(unsigned idx=0;idx<polygons.size();idx++)
        {
            polygons[idx]->AddAttribute("Pellet Interface", 1.0);
        }

        p_domain->Write(file_handler.GetOutputDirectoryFullPath()+"domain.vtp", GeometryFormat::VTP, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        //p_mesh_generator->SetMaxElementArea(1e4*Qpow3(1.e-18 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"mesh");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();
    }

    void TestDistanceMap3D() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/DistanceMap3D");

        MappableGridGenerator<3> generator;
        unsigned num_divisions_x = 20;
        unsigned num_divisions_y = 20;
        double azimuth_angle = 1.0 * M_PI;
        double polar_angle = 0.999 * M_PI;
        std::vector<DimensionalChastePoint<3> > holes;

        QLength cornea_radius = 1300*1.e-6*unit::metres;
        QLength cornea_thickness = 100*1.e-6*unit::metres;
        QLength pellet_radius = 200*1.e-6*unit::metres;
        QLength pellet_thickness = 50*1.e-6*unit::metres;
        QLength reference_length = 1.e-6*unit::metres;
        std::shared_ptr<Part<3> > p_domain = generator.GenerateHemisphere(cornea_radius,
                cornea_thickness, num_divisions_x, num_divisions_y, azimuth_angle, polar_angle);

        double gap = (cornea_thickness - pellet_thickness)/(2.0*reference_length)/4.0;
        double base = cornea_radius/reference_length + gap - cornea_thickness/reference_length;

        std::shared_ptr<Part<3> > p_pellet = Part<3>::Create();
        p_pellet->AddCylinder(pellet_radius,pellet_thickness,
                              DimensionalChastePoint<3>(0.0, 0.0, base, reference_length));

        // Rotate the pellet
        double rotation_angle = M_PI/8.0;
        c_vector<double, 3> axis;
        axis[0] = 0.0;
        axis[1] = 1.0;
        axis[2] = 0.0;
        p_pellet->RotateAboutAxis(axis, rotation_angle);

        DimensionalChastePoint<3> centre(0.0, 0.0, base + pellet_thickness/(2.0*reference_length), reference_length);
        centre.RotateAboutAxis(axis, rotation_angle);

        p_domain->AppendPart(p_pellet);
        p_domain->AddHoleMarker(DimensionalChastePoint<3>(centre));

        std::vector<std::shared_ptr<Polygon<3> > > polygons = p_pellet->GetPolygons();
        for(unsigned idx=0;idx<polygons.size();idx++)
        {
            polygons[idx]->AddAttribute("Pellet Interface", 1.0);
        }

        p_domain->Write(file_handler.GetOutputDirectoryFullPath()+"domain.vtp", GeometryFormat::VTP, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(1e4*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"mesh");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();

        vtkSmartPointer<vtkUnstructuredGrid> p_distance_map =
                vtkUnstructuredGrid::SafeDownCast(p_mesh_generator->GetMesh()->CalculateDistanceMap(p_domain));
        vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_polydata_writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
        p_polydata_writer->SetInputData(p_distance_map);
        std::string output_path = file_handler.GetOutputDirectoryFullPath()+"distance_map.vtu";
        p_polydata_writer->SetFileName(output_path.c_str());
        p_polydata_writer->Write();
    }

    void TestDistanceMap2D() throw(Exception)
    {
        OutputFileHandler file_handler("TestDiscreteContinuumMesh/DistanceMap2D");
        std::shared_ptr<Part<2> > p_part = Part<2>::Create();
        std::shared_ptr<Polygon<2> > p_circle = p_part->AddCircle(0.33e-6*unit::metres,
                DimensionalChastePoint<2>(0.5, 0.5));
        p_circle->AddAttributeToAllEdges("Outer Boundary", 1.0);

        std::shared_ptr<Polygon<2> > p_circle2 = p_part->AddCircle(0.1e-6*unit::metres,
                DimensionalChastePoint<2>(0.5, 0.5));
        p_part->AddRegionMarker(DimensionalChastePoint<2>(0.5, 0.5), 1.0);
        p_part->GetVtk(true);
        p_part->Write(file_handler.GetOutputDirectoryFullPath()+"part.vtp", GeometryFormat::VTP, true);

        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->SetMaxElementArea(1.e-3*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        MultiFormatMeshWriter<2> mesh_writer;
        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"circle");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();

        // Add a hole
        std::vector<DimensionalChastePoint<2> > holes;
        holes.push_back(DimensionalChastePoint<2>(0.5, 0.5));
        p_mesh_generator->SetHoles(holes);
        p_mesh_generator->SetMaxElementArea(1.e-3*Qpow3(1.e-6 * unit::metres));
        p_mesh_generator->Update();

        mesh_writer.SetFileName(file_handler.GetOutputDirectoryFullPath()+"circle_hole");
        mesh_writer.SetMesh(p_mesh_generator->GetMesh(), true);
        mesh_writer.Write();

        vtkSmartPointer<vtkUnstructuredGrid> p_distance_map =
                vtkUnstructuredGrid::SafeDownCast(p_mesh_generator->GetMesh()->CalculateDistanceMap(p_part));
        vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_polydata_writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
        p_polydata_writer->SetInputData(p_distance_map);
        std::string output_path = file_handler.GetOutputDirectoryFullPath()+"distance_map.vtu";
        p_polydata_writer->SetFileName(output_path.c_str());
        p_polydata_writer->Write();
    }
};

#endif /*TESTDISCRETECONTINUUMMESH_HPP_*/
