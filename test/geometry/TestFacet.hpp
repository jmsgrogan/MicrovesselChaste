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

#ifndef TESTFACET_HPP_
#define TESTFACET_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include "OutputFileHandler.hpp"
#include "Vertex.hpp"
#include "Polygon.hpp"
#include "Facet.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"


std::vector<PolygonPtr<3> > SetUpPolygons()
{
    std::vector<VertexPtr<3> > vertices;
    vertices.push_back(Vertex<3>::Create(0.0_um, 0.0_um));
    vertices.push_back(Vertex<3>::Create(1.0_um, 0.0_um));
    vertices.push_back(Vertex<3>::Create(1.0_um, 1.0_um));
    std::vector<PolygonPtr<3> > polygons;
    for(unsigned idx=0; idx<3; idx++)
    {
        polygons.push_back(Polygon<3>::Create(vertices[idx]));
    }
    return polygons;
}

PolygonPtr<3> SetUpTriangle()
{
    std::vector<VertexPtr<3> > vertices;
    vertices.push_back(Vertex<3>::Create(0.0_um, 0.0_um));
    vertices.push_back(Vertex<3>::Create(1.0_um, 0.0_um));
    vertices.push_back(Vertex<3>::Create(1.0_um, 1.0_um));
    return Polygon<3>::Create(vertices[0]);
}

template<unsigned DIM>
PolygonPtr<DIM> SetUpSquare()
{
    std::vector<VertexPtr<DIM> > vertices;
    vertices.push_back(Vertex<DIM>::Create(0.0_um, 0.0_um));
    vertices.push_back(Vertex<DIM>::Create(1.0_um, 0.0_um));
    vertices.push_back(Vertex<DIM>::Create(1.0_um, 1.0_um));
    vertices.push_back(Vertex<DIM>::Create(0.0_um, 1.0_um));
    return Polygon<DIM>::Create(vertices);
}

class TestFacet : public CxxTest::TestSuite
{

public:

    void TestConstructor() throw (Exception)
    {
        std::vector<PolygonPtr<3> > polygons = SetUpPolygons();
        Facet<3> facet1(polygons);
        Facet<3> facet2(polygons[0]);
        TS_ASSERT_EQUALS(facet1.GetPolygons().size(), 3u);
        TS_ASSERT_EQUALS(facet2.GetPolygons().size(), 1u);
    }

    void TestFactoryConstructor() throw (Exception)
    {
        std::vector<PolygonPtr<3> > polygons = SetUpPolygons();
        auto p_facet1 = Facet<3>::Create(polygons);
        auto p_facet2 = Facet<3>::Create(polygons[0]);
        TS_ASSERT_EQUALS(p_facet1->GetPolygons().size(), 3u);
        TS_ASSERT_EQUALS(p_facet2->GetPolygons().size(), 1u);
    }

    void TestAddingPolygons() throw (Exception)
    {
        std::vector<PolygonPtr<3> > polygons = SetUpPolygons();
        PolygonPtr<3> p_polygon = SetUpTriangle();
        auto p_facet1 = Facet<3>::Create(p_polygon);
        auto p_facet2 = Facet<3>::Create(p_polygon);
        p_facet1->AddPolygons(polygons);
        p_facet2->AddPolygon(polygons[0]);
        TS_ASSERT_EQUALS(p_facet1->GetPolygons().size(), 4u);
        TS_ASSERT_EQUALS(p_facet2->GetPolygons().size(), 2u);
    }

    void TestVtkMethods() throw (Exception)
    {
        PolygonPtr<3> p_polygon = SetUpSquare<3>();
        auto p_facet = Facet<3>::Create(p_polygon);

        TS_ASSERT(p_facet->ContainsPoint(Vertex<3>(0.75_um, 0.75_um, 0.0_um)));
        TS_ASSERT(!p_facet->ContainsPoint(Vertex<3>(1.25_um, 0.75_um, 0.0_um)));
        TS_ASSERT(!p_facet->ContainsPoint(Vertex<3>(0.75_um, 0.75_um, 1.0_um)));

        Vertex<3> centroid = p_facet->GetCentroid();
        c_vector<double, 3> loc = centroid.Convert(1_um);
        TS_ASSERT_DELTA(loc[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(loc[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(loc[2], 0.0, 1.e-6);

        c_vector<double, 3> normal = p_facet->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(std::abs(normal[2]), 1.0, 1.e-6);
    }

    void TestVtkMethods2d() throw (Exception)
    {
        PolygonPtr<2> p_polygon = SetUpSquare<2>();
        auto p_facet = Facet<2>::Create(p_polygon);
        TS_ASSERT(p_facet->ContainsPoint(Vertex<2>(0.75_um, 0.75_um, 0.0_um)));
        TS_ASSERT(!p_facet->ContainsPoint(Vertex<2>(1.25_um, 0.75_um, 0.0_um)));

        Vertex<2> centroid = p_facet->GetCentroid();
        c_vector<double, 2> loc = centroid.Convert(1_um);
        TS_ASSERT_DELTA(loc[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(loc[1], 0.5, 1.e-6);

        c_vector<double, 3> normal = p_facet->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
    }

    void TestTransforms() throw (Exception)
    {
        PolygonPtr<3> p_polygon = SetUpSquare<3>();
        auto p_facet = Facet<3>::Create(p_polygon);

        Vertex<3> translation_vector(2.0_um, 2.0_um, 0.0_um);
        Vertex<3> new_position = *(p_polygon->rGetVertices()[1]) + translation_vector;
        p_facet->Translate(translation_vector);

        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_facet->RotateAboutAxis(rotation_axis, M_PI/2.0);

        // Check extra verts have been updated during rotation
        auto p_extra_poly = Polygon<3>::Create(Vertex<3>::Create(0.5_um, 1.5_um));
        p_facet->AddPolygon(p_extra_poly);
        p_facet->RotateAboutAxis(rotation_axis, -M_PI/2.0);
    }

    void TestGeometryAndLabelMethods() throw (Exception)
    {
        std::vector<VertexPtr<3> > vertices;
        vertices.push_back(Vertex<3>::Create(-0.5_um, 0.0_um));
        vertices.push_back(Vertex<3>::Create(1.0_um, 0.0_um));
        vertices.push_back(Vertex<3>::Create(1.0_um, 1.0_um));
        vertices.push_back(Vertex<3>::Create(-0.5_um, 1.0_um));
        auto p_polygon = Polygon<3>::Create(vertices);
        auto p_facet = Facet<3>::Create(p_polygon);

        std::array<QLength,6> bbox = p_facet->GetBoundingBox();
        TS_ASSERT_DELTA(double(bbox[0]/1_um), -0.5, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[1]/1_um), 1.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[2]/1_um), 0.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[3]/1_um), 1.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[4]/1_um), 0.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[5]/1_um), 0.0, 1.e-6);

        TS_ASSERT_DELTA(double(p_facet->GetDistance(Vertex<3>(0.5_um, 0.5_um, 0.5_um))/1_um), 0.5, 1.e-6);
        TS_ASSERT_DELTA(p_facet->GetPlane()->GetNormal()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_facet->GetPlane()->GetNormal()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(abs(p_facet->GetPlane()->GetNormal()[2]), 1.0, 1.e-6);
    }

    void Test2DMethods() throw (Exception)
    {
        std::vector<std::shared_ptr<Vertex<2> > > vertices;
        vertices.push_back(Vertex<2>::Create(-0.5_um, 0.0_um));
        vertices.push_back(Vertex<2>::Create(1.0_um, 0.0_um));
        vertices.push_back(Vertex<2>::Create(1.0_um, 1.0_um));
        vertices.push_back(Vertex<2>::Create(-0.5_um, 1.0_um));
        auto p_polygon = Polygon<2>::Create(vertices);
        auto p_facet = Facet<2>::Create(p_polygon);

        std::array<QLength,6> bbox = p_facet->GetBoundingBox();
        TS_ASSERT_DELTA(double(bbox[0]/1_um), -0.5, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[1]/1_um), 1.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[2]/1_um), 0.0, 1.e-6);
        TS_ASSERT_DELTA(double(bbox[3]/1_um), 1.0, 1.e-6);

        TS_ASSERT_DELTA(double(p_facet->GetDistance(Vertex<2>(1.5_um, 0.5_um, 0.0_um))/1_um), 0.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
#if BOOST_VERSION >= 105600
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("Facet.arch");

        std::vector<VertexPtr<3> > vertices;
        vertices.push_back(Vertex<3>::Create(0.0_um, 0.0_um));
        vertices.push_back(Vertex<3>::Create(1.0_um, 0.0_um));
        vertices.push_back(Vertex<3>::Create(0.0_um, 1.0_um));
        FacetPtr<3> p_facet1 = Facet<3>::Create(Polygon<3>::Create(vertices));

        // Save archive
        {
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_facet1;
        }

        // Load archive
        {
            FacetPtr<3> p_facet_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_facet_from_archive;

            // Check that we remember the reference length
            TS_ASSERT_EQUALS(p_facet_from_archive->GetPolygons()[0]->rGetVertices().size(), 3u);
        }
#endif
    }
};

#endif /*TESTPLCFACET_HPP_*/
