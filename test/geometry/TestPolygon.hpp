/*

 Copyright (c) 2005-2015, University of Oxford.
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
#include "Vertex.hpp"
#include "Polygon.hpp"

class TestPolygon : public CxxTest::TestSuite
{
public:

    void TestConstructor()
    {
        std::vector<boost::shared_ptr<Vertex> > vertices;
        vertices.push_back(Vertex::Create(0.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(0.0, 1.0, 0.0));

        Polygon polygon1 = Polygon(vertices);
        Polygon polygon2 = Polygon(vertices[0]);
        TS_ASSERT_EQUALS(polygon1.GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(polygon2.GetVertices().size(), 1u);
    }

    void TestFactoryConstructor()
    {
        std::vector<boost::shared_ptr<Vertex> > vertices;
        vertices.push_back(Vertex::Create(0.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(0.0, 1.0, 0.0));

        boost::shared_ptr<Polygon> p_polygon1 = Polygon::Create(vertices);
        boost::shared_ptr<Polygon> p_polygon2 = Polygon::Create(vertices[0]);

        TS_ASSERT_EQUALS(p_polygon1->GetVertices().size(), 3u);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 1u);
    }

    void TestAddingVertices()
    {
        std::vector<boost::shared_ptr<Vertex> > vertices;
        vertices.push_back(Vertex::Create(0.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(0.0, 1.0, 0.0));

        std::vector<boost::shared_ptr<Vertex> > new_vertices;
        new_vertices.push_back(Vertex::Create(1.0, 1.0, 0.0));
        new_vertices.push_back(Vertex::Create(1.0, 2.0, 0.0));

        boost::shared_ptr<Polygon> p_polygon = Polygon::Create(vertices);
        p_polygon->AddVertices(new_vertices);
        TS_ASSERT_EQUALS(p_polygon->GetVertices().size(), 5u);

        boost::shared_ptr<Polygon> p_polygon2 = Polygon::Create(vertices);
        p_polygon2->AddVertex(new_vertices[0]);
        TS_ASSERT_EQUALS(p_polygon2->GetVertices().size(), 4u);
    }

    void TestVtkMethods()
    {
        std::vector<boost::shared_ptr<Vertex> > vertices;
        vertices.push_back(Vertex::Create(0.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 1.0, 0.0));
        vertices.push_back(Vertex::Create(0.0, 1.0, 0.0));
        boost::shared_ptr<Polygon> p_polygon = Polygon::Create(vertices);

        DimensionalChastePoint<3> centroid = p_polygon->GetCentroid();
        TS_ASSERT_DELTA(centroid[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(centroid[2], 0.0, 1.e-6);

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

    void TestTransforms()
    {
        std::vector<boost::shared_ptr<Vertex> > vertices;
        vertices.push_back(Vertex::Create(0.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 0.0, 0.0));
        vertices.push_back(Vertex::Create(1.0, 1.0, 0.0));
        vertices.push_back(Vertex::Create(0.0, 1.0, 0.0));
        boost::shared_ptr<Polygon> p_polygon = Polygon::Create(vertices);

        c_vector<double, 3> translation_vector;
        translation_vector[0] = 2.0;
        translation_vector[1] = 2.0;
        translation_vector[2] = 0.0;
        c_vector<double, 3> new_position = vertices[1]->rGetLocation() + translation_vector;

        p_polygon->Translate(translation_vector);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->rGetLocation()[0], new_position[0], 1.e-6);

        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_polygon->RotateAboutAxis(rotation_axis, M_PI/2.0);
        TS_ASSERT_DELTA(p_polygon->GetVertices()[1]->rGetLocation()[0], -new_position[1], 1.e-6);
    }
};

#endif /*TESTPOLYGON_HPP_*/
