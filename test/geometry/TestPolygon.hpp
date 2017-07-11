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

#ifndef TESTPOLYGON_HPP_
#define TESTPOLYGON_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "OutputFileHandler.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "Vertex.hpp"
#include "Polygon.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestPolygon : public CxxTest::TestSuite
{
public:

    void TestConstructor() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));

        Polygon<3> polygon1 = Polygon<3>(vertices);
        Polygon<3> polygon2 = Polygon<3>(vertices[0]);
        TS_ASSERT_EQUALS(polygon1.GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(polygon2.GetVertices().size(), 1u);
    }

    void TestFactoryConstructor() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));

        std::shared_ptr<Polygon<3> > p_polygon1 = Polygon<3>::Create(vertices);
        std::shared_ptr<Polygon<3> > p_polygon2 = Polygon<3>::Create(vertices[0]);

        TS_ASSERT_EQUALS(p_polygon1->GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 1u);
    }

    void TestAddingVertices() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));

        std::vector<std::shared_ptr<Vertex<3> > > new_vertices;
        new_vertices.push_back(Vertex<3>::Create(1.0, 1.0, 0.0, 1_um));
        new_vertices.push_back(Vertex<3>::Create(1.0, 2.0, 0.0, 1_um));

        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        p_polygon->AddVertices(new_vertices);
        TS_ASSERT_EQUALS(p_polygon->GetVertices().size(), 5u);

        std::shared_ptr<Polygon<3> > p_polygon2 = Polygon<3>::Create(vertices);
        p_polygon2->AddVertex(new_vertices[0]);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 4u);

        TS_ASSERT_THROWS_THIS(p_polygon2->GetVertex(100), "Requested vertex index out of range");
        TS_ASSERT_THROWS_THIS(p_polygon2->ReplaceVertex(100, vertices[0]), "Requested vertex index out of range");
    }

    void TestVtkMethods() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 1.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));

        std::vector<std::shared_ptr<Vertex<3> > > short_vertices;
        short_vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        short_vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));

        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        std::shared_ptr<Polygon<3> > p_short_polygon = Polygon<3>::Create(short_vertices);
        TS_ASSERT_THROWS_THIS(p_short_polygon->GetNormal(), "At least 3 vertices are required to generate a normal.");

        Vertex<3> centroid = p_polygon->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1_um)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1_um)[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1_um)[2], 0.0, 1.e-6);

        c_vector<double, 3> normal = p_polygon->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[2], 1.0, 1.e-6);

        Vertex<3> test_point1(0.75, 0.75, 0.0);
        Vertex<3> test_point2(1.25, 0.75, 0.0);
        Vertex<3> test_point3(0.75, 0.75, 1.0);

        TS_ASSERT(p_polygon->ContainsPoint(test_point1));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point2));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point3));

        vtkSmartPointer<vtkPolygon> p_vtk_polygon =  p_polygon->GetVtkPolygon();
        std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_pair = p_polygon->GetVtkVertices();
    }

    void TestVtkMethods2d() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<2> > > vertices;
        vertices.push_back(Vertex<2>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<2>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<2>::Create(1.0, 1.0, 0.0, 1_um));
        vertices.push_back(Vertex<2>::Create(0.0, 1.0, 0.0, 1_um));

        std::vector<std::shared_ptr<Vertex<2> > > short_vertices;
        short_vertices.push_back(Vertex<2>::Create(0.0, 0.0, 0.0, 1_um));
        short_vertices.push_back(Vertex<2>::Create(1.0, 0.0, 0.0, 1_um));

        std::shared_ptr<Polygon<2> > p_polygon = Polygon<2>::Create(vertices);
        std::shared_ptr<Polygon<2> > p_short_polygon = Polygon<2>::Create(short_vertices);
        TS_ASSERT_THROWS_THIS(p_short_polygon->GetNormal(), "At least 3 vertices are required to generate a normal.");

        Vertex<2> centroid = p_polygon->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1_um)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1_um)[1], 0.5, 1.e-6);

        c_vector<double, 2> normal = p_polygon->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);

        Vertex<2> test_point1(0.75, 0.75, 0.0);
        Vertex<2> test_point2(1.25, 0.75, 0.0);

        TS_ASSERT(p_polygon->ContainsPoint(test_point1));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point2));

        vtkSmartPointer<vtkPolygon> p_vtk_polygon =  p_polygon->GetVtkPolygon();
        std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_pair = p_polygon->GetVtkVertices();
    }

    void TestTransforms() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 1.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);

        Vertex<3> translation_vector(2.0, 2.0, 0.0);
        Vertex<3> new_position = *vertices[1] + translation_vector;

        p_polygon->Translate(translation_vector);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->GetLocation(1_um)[0], 3.0, 1.e-6);

        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_polygon->RotateAboutAxis(rotation_axis, M_PI/2.0);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->GetLocation(1_um)[1], 3.0, 1.e-6);
    }

    void TestGeometryOperations() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 1.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);

        std::vector<QLength > bbox = p_polygon->GetBoundingBox();
        TS_ASSERT_DELTA(bbox[0]/1_um, 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[1]/1_um, 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[2]/1_um, 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[3]/1_um, 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[4]/1_um, 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[5]/1_um, 0.0, 1.e-6);

        TS_ASSERT_DELTA(p_polygon->GetDistance(Vertex<3>(0.5, 0.5, 0.5, 1_um))/1_um, 0.5e-6, 1.e-8);
        TS_ASSERT_DELTA(p_polygon->GetPlane()->GetNormal()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_polygon->GetPlane()->GetNormal()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(std::abs(p_polygon->GetPlane()->GetNormal()[2]), 1.0, 1.e-6);

        TS_ASSERT_DELTA(p_polygon->GetDistanceToEdges(Vertex<3>(1.5, 0.5, 0.0, 1_um))/1_um, 0.5e-6, 1.e-8);
    }

    void TestLabelling() throw(Exception)
    {
        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 1.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);

        std::string test = "Test";
        bool edge_found = p_polygon->AddAttributeToEdgeIfFound(Vertex<3>(0.5, 0.0, 0.0, 1_um), test, 2.0);
        std::vector<std::map<std::string, double> > edge_attributes = p_polygon->GetEdgeAttributes();
        TS_ASSERT(edge_found);
        TS_ASSERT_DELTA(edge_attributes[0]["Test"], 2.0, 1.e-6);
        TS_ASSERT(p_polygon->EdgeHasAttribute(Vertex<3>(0.5, 0.0, 0.0, 1_um), "Test"));

        std::string poly_label = "TestPoly";
        p_polygon->AddAttribute(poly_label, 2.0);
        TS_ASSERT_DELTA(p_polygon->GetAttributes()[poly_label], 2.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("Polygon.arch");

        std::vector<std::shared_ptr<Vertex<3> > > vertices;
        vertices.push_back(Vertex<3>::Create(0.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(1.0, 0.0, 0.0, 1_um));
        vertices.push_back(Vertex<3>::Create(0.0, 1.0, 0.0, 1_um));
        std::shared_ptr<Polygon<3> > p_polygon1 = Polygon<3>::Create(vertices);

        // Save archive
        {
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_polygon1;
        }

        // Load archive
        {
            std::shared_ptr<Polygon<3> > p_polygon_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_polygon_from_archive;

            // Check that we remember the reference length
            TS_ASSERT_EQUALS(p_polygon_from_archive->GetVertices().size(), 3u);
        }
    }
};

#endif /*TESTPOLYGON_HPP_*/
