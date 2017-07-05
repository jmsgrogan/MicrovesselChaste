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
#include "SmartPointers.hpp"
#include "DimensionalChastePoint.hpp"
#include "Polygon.hpp"
#include "Facet.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestFacet : public CxxTest::TestSuite
{
public:

    void TestConstructor()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::vector<std::shared_ptr<Polygon<3> > > polygons;
        for(unsigned idx=0; idx<3; idx++)
        {
            polygons.push_back(Polygon<3>::Create(vertices[idx]));
        }

        Facet<3> facet1 = Facet<3>(polygons);
        Facet<3> facet2 = Facet<3>(polygons[0]);

        TS_ASSERT_EQUALS(facet1.GetPolygons().size(), 3u);
        TS_ASSERT_EQUALS(facet2.GetPolygons().size(), 1u);
    }

    void TestFactoryConstructor()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::vector<std::shared_ptr<Polygon<3> > > polygons;
        for(unsigned idx=0; idx<3; idx++)
        {
            polygons.push_back(Polygon<3>::Create(vertices[idx]));
        }

        std::shared_ptr<Facet<3> > p_facet1 = Facet<3>::Create(polygons);
        std::shared_ptr<Facet<3> > p_facet2 = Facet<3>::Create(polygons[0]);

        TS_ASSERT_EQUALS(p_facet1->GetPolygons().size(), 3u);
        TS_ASSERT_EQUALS(p_facet2->GetPolygons().size(), 1u);
    }

    void TestAddingPolygons()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices[0]);

        std::vector<std::shared_ptr<Polygon<3> > > polygons;
        for(unsigned idx=1; idx<3; idx++)
        {
            polygons.push_back(Polygon<3>::Create(vertices[idx]));
        }

        std::shared_ptr<Facet<3> > p_facet1 = Facet<3>::Create(p_polygon);
        std::shared_ptr<Facet<3> > p_facet2 = Facet<3>::Create(p_polygon);

        p_facet1->AddPolygons(polygons);
        p_facet2->AddPolygon(polygons[0]);

        TS_ASSERT_EQUALS(p_facet1->GetPolygons().size(), 3u);
        TS_ASSERT_EQUALS(p_facet2->GetPolygons().size(), 2u);
    }

    void TestVtkMethods()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        std::shared_ptr<Facet<3> > p_facet = Facet<3>::Create(p_polygon);

        DimensionalChastePoint<3> centroid = p_facet->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[2], 0.0, 1.e-6);

        c_vector<double, 3> normal = p_facet->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(std::abs(normal[2]), 1.0, 1.e-6);
        DimensionalChastePoint<3> test_point1(0.75, 0.75, 0.0);
        DimensionalChastePoint<3> test_point2(1.25, 0.75, 0.0);
        DimensionalChastePoint<3> test_point3(0.75, 0.75, 1.0);

        TS_ASSERT(p_facet->ContainsPoint(test_point1));
        TS_ASSERT(!p_facet->ContainsPoint(test_point2));
        TS_ASSERT(!p_facet->ContainsPoint(test_point3));
    }

    void TestVtkMethods2d()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<2> > > vertices;
        vertices.push_back(DimensionalChastePoint<2>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<2> > p_polygon = Polygon<2>::Create(vertices);
        std::shared_ptr<Facet<2> > p_facet = Facet<2>::Create(p_polygon);

        DimensionalChastePoint<2> centroid = p_facet->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[1], 0.5, 1.e-6);

        c_vector<double, 2> normal = p_facet->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
        DimensionalChastePoint<2> test_point1(0.75, 0.75, 0.0);
        DimensionalChastePoint<2> test_point2(1.25, 0.75, 0.0);
        TS_ASSERT(p_facet->ContainsPoint(test_point1));
        TS_ASSERT(!p_facet->ContainsPoint(test_point2));
    }

    void TestTransforms()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        std::shared_ptr<Facet<3> > p_facet = Facet<3>::Create(p_polygon);

        DimensionalChastePoint<3> translation_vector(2.0, 2.0, 0.0);
        DimensionalChastePoint<3> new_position = *vertices[1] + translation_vector;

        p_facet->Translate(translation_vector);
        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_facet->RotateAboutAxis(rotation_axis, M_PI/2.0);
    }

    void TestGeometryAndLabelMethods()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(-0.5, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(-0.5, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        std::shared_ptr<Facet<3> > p_facet = Facet<3>::Create(p_polygon);

        std::vector<QLength > bbox = p_facet->GetBoundingBox();
        TS_ASSERT_DELTA(bbox[0].value(), -0.5e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[1].value(), 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[2].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[3].value(), 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[4].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[5].value(), 0.0, 1.e-6);

        TS_ASSERT_DELTA(p_facet->GetDistance(DimensionalChastePoint<3>(0.5, 0.5, 0.5,1.e-6*unit::metres )).value(), 0.5e-6, 1.e-8);

        TS_ASSERT_DELTA(p_facet->GetPlane()->GetNormal()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_facet->GetPlane()->GetNormal()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(abs(p_facet->GetPlane()->GetNormal()[2]), 1.0, 1.e-6);

        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > short_vertices;
        short_vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        short_vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<3> > p_short_polygon = Polygon<3>::Create(short_vertices);
        std::shared_ptr<Facet<3> > p_short_facet = Facet<3>::Create(p_short_polygon);
        TS_ASSERT_THROWS_THIS(p_short_facet->GetNormal(), "At least 3 vertices are required to generate a normal.");
    }

    void Test2DMethods()
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<2> > > vertices;
        vertices.push_back(DimensionalChastePoint<2>::Create(-0.5, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(-0.5, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Polygon<2> > p_polygon = Polygon<2>::Create(vertices);
        std::shared_ptr<Facet<2> > p_facet = Facet<2>::Create(p_polygon);

        std::vector<QLength > bbox = p_facet->GetBoundingBox();
        TS_ASSERT_DELTA(bbox[0].value(), -0.5e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[1].value(), 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[2].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[3].value(), 1.e-6, 1.e-8);

        TS_ASSERT_DELTA(p_facet->GetDistance(DimensionalChastePoint<2>(1.5, 0.5, 0.0,1.e-6*unit::metres )).value(), 0.0e-6, 1.e-8);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("Facet.arch");

        std::vector<std::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        std::shared_ptr<Facet<3> > p_facet1 = Facet<3>::Create(Polygon<3>::Create(vertices));

        // Save archive
        {
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_facet1;
        }

        // Load archive
        {
            std::shared_ptr<Facet<3> > p_facet_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_facet_from_archive;

            // Check that we remember the reference length
            TS_ASSERT_EQUALS(p_facet_from_archive->GetPolygons()[0]->GetVertices().size(), 3u);
        }
    }
};

#endif /*TESTPLCFACET_HPP_*/
