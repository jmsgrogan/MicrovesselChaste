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

#ifndef TESTPOLYGON_HPP_
#define TESTPOLYGON_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "DimensionalChastePoint.hpp"
#include "Polygon.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestPolygon : public CxxTest::TestSuite
{
public:

    void TestConstructor()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        Polygon<3> polygon1 = Polygon<3>(vertices);
        Polygon<3> polygon2 = Polygon<3>(vertices[0]);
        TS_ASSERT_EQUALS(polygon1.GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(polygon2.GetVertices().size(), 1u);
    }

    void TestFactoryConstructor()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        boost::shared_ptr<Polygon<3> > p_polygon1 = Polygon<3>::Create(vertices);
        boost::shared_ptr<Polygon<3> > p_polygon2 = Polygon<3>::Create(vertices[0]);

        TS_ASSERT_EQUALS(p_polygon1->GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 1u);
    }

    void TestAddingVertices()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > new_vertices;
        new_vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        new_vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 2.0, 0.0, 1.e-6*unit::metres));

        boost::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        p_polygon->AddVertices(new_vertices);
        TS_ASSERT_EQUALS(p_polygon->GetVertices().size(), 5u);

        boost::shared_ptr<Polygon<3> > p_polygon2 = Polygon<3>::Create(vertices);
        p_polygon2->AddVertex(new_vertices[0]);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 4u);

        TS_ASSERT_THROWS_THIS(p_polygon2->GetVertex(100), "Requested vertex index out of range");
        TS_ASSERT_THROWS_THIS(p_polygon2->ReplaceVertex(100, vertices[0]), "Requested vertex index out of range");
    }

    void TestVtkMethods()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > short_vertices;
        short_vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        short_vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));

        boost::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);
        boost::shared_ptr<Polygon<3> > p_short_polygon = Polygon<3>::Create(short_vertices);
        TS_ASSERT_THROWS_THIS(p_short_polygon->GetNormal(), "At least 3 vertices are required to generate a normal.");

        DimensionalChastePoint<3> centroid = p_polygon->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[2], 0.0, 1.e-6);

        c_vector<double, 3> normal = p_polygon->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[2], 1.0, 1.e-6);

        DimensionalChastePoint<3> test_point1(0.75, 0.75, 0.0);
        DimensionalChastePoint<3> test_point2(1.25, 0.75, 0.0);
        DimensionalChastePoint<3> test_point3(0.75, 0.75, 1.0);

        TS_ASSERT(p_polygon->ContainsPoint(test_point1));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point2));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point3));

        vtkSmartPointer<vtkPolygon> p_vtk_polygon =  p_polygon->GetVtkPolygon();
        std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_pair = p_polygon->GetVtkVertices();
    }

    void TestVtkMethods2d()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<2> > > vertices;
        vertices.push_back(DimensionalChastePoint<2>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<2>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));

        std::vector<boost::shared_ptr<DimensionalChastePoint<2> > > short_vertices;
        short_vertices.push_back(DimensionalChastePoint<2>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        short_vertices.push_back(DimensionalChastePoint<2>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));

        boost::shared_ptr<Polygon<2> > p_polygon = Polygon<2>::Create(vertices);
        boost::shared_ptr<Polygon<2> > p_short_polygon = Polygon<2>::Create(short_vertices);
        TS_ASSERT_THROWS_THIS(p_short_polygon->GetNormal(), "At least 3 vertices are required to generate a normal.");

        DimensionalChastePoint<2> centroid = p_polygon->GetCentroid();
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid.GetLocation(1.e-6*unit::metres)[1], 0.5, 1.e-6);

        c_vector<double, 2> normal = p_polygon->GetNormal();
        TS_ASSERT_DELTA(normal[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(normal[1], 0.0, 1.e-6);

        DimensionalChastePoint<2> test_point1(0.75, 0.75, 0.0);
        DimensionalChastePoint<2> test_point2(1.25, 0.75, 0.0);

        TS_ASSERT(p_polygon->ContainsPoint(test_point1));
        TS_ASSERT(!p_polygon->ContainsPoint(test_point2));

        vtkSmartPointer<vtkPolygon> p_vtk_polygon =  p_polygon->GetVtkPolygon();
        std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_pair = p_polygon->GetVtkVertices();
    }

    void TestTransforms()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        boost::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);

        DimensionalChastePoint<3> translation_vector(2.0, 2.0, 0.0);
        DimensionalChastePoint<3> new_position = *vertices[1] + translation_vector;

        p_polygon->Translate(translation_vector);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->GetLocation(1.e-6*unit::metres)[0], new_position.GetLocation(1.e-6*unit::metres)[0], 1.e-6);

        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_polygon->RotateAboutAxis(rotation_axis, M_PI/2.0);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->GetLocation(1.e-6*unit::metres)[0], -new_position.GetLocation(1.e-6*unit::metres)[1], 1.e-6);
    }

    void TestGeometryOperations()
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 0.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(1.0, 1.0, 0.0, 1.e-6*unit::metres));
        vertices.push_back(DimensionalChastePoint<3>::Create(0.0, 1.0, 0.0, 1.e-6*unit::metres));
        boost::shared_ptr<Polygon<3> > p_polygon = Polygon<3>::Create(vertices);

        std::vector<units::quantity<unit::length> > bbox = p_polygon->GetBoundingBox();
        TS_ASSERT_DELTA(bbox[0].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[1].value(), 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[2].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[3].value(), 1.e-6, 1.e-8);
        TS_ASSERT_DELTA(bbox[4].value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox[5].value(), 0.0, 1.e-6);

        TS_ASSERT_DELTA(p_polygon->GetDistance(DimensionalChastePoint<3>(0.5, 0.5, 0.5, 1.e-6*unit::metres)).value(), 0.5e-6, 1.e-8);
        TS_ASSERT_DELTA(p_polygon->GetPlane()->GetNormal()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_polygon->GetPlane()->GetNormal()[1], 0.0, 1.e-6);
        TS_ASSERT_DELTA(std::abs(p_polygon->GetPlane()->GetNormal()[2]), 1.0, 1.e-6);

        TS_ASSERT_DELTA(p_polygon->GetDistanceToEdges(DimensionalChastePoint<3>(1.5, 0.5, 0.0, 1.e-6*unit::metres)).value(), 0.5e-6, 1.e-8);
    }
};

#endif /*TESTPOLYGON_HPP_*/
