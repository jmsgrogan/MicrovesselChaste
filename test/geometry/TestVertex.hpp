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

#ifndef TESTVERTEX_HPP_
#define TESTVERTEX_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "Vertex.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVertex : public CxxTest::TestSuite
{
public:

    void TestConstructor()
    {
        Vertex vertex1 = Vertex(1.0, 2.0, 3.0);
        vertex1.SetIndex(5);

        c_vector<double, 3> coords;
        coords[0] = 4.0;
        coords[1] = 5.0;
        coords[2] = 6.0;
        Vertex vertex2 = Vertex(coords);
        vertex1.SetIndex(10);

        TS_ASSERT_DELTA(vertex1.rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(vertex2.rGetLocation()[1], 5.0, 1.e-6);
    }

    void TestFactoryConstructor()
    {
        boost::shared_ptr<Vertex> p_vertex1 = Vertex::Create(7.0, 8.0, 9.0);

        c_vector<double, 3> coords;
        coords[0] = 10.0;
        coords[1] = 11.0;
        coords[2] = 12.0;
        boost::shared_ptr<Vertex> p_vertex2 = Vertex::Create(coords);

        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[0], 7.0, 1.e-6);
        TS_ASSERT_DELTA(p_vertex2->rGetLocation()[1], 11.0, 1.e-6);
    }

    void TestGetAndSet()
    {
        boost::shared_ptr<Vertex> p_vertex1 = Vertex::Create();
        p_vertex1->SetIndex(5);
        TS_ASSERT_EQUALS(p_vertex1->GetIndex(), 5u);
    }

    void TestTransforms()
    {
        c_vector<double, 3> coords;
        coords[0] = 1.0;
        coords[1] = 2.0;
        coords[2] = 3.0;
        boost::shared_ptr<Vertex> p_vertex1 = Vertex::Create(coords);

        c_vector<double, 3> translation_vector;
        translation_vector[0] = 3.0;
        translation_vector[1] = 4.0;
        translation_vector[2] = 5.0;
        p_vertex1->Translate(translation_vector);

        c_vector<double, 3> new_position = coords + translation_vector;
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[0], new_position[0], 1.e-6);
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[1], new_position[1], 1.e-6);
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[2], new_position[2], 1.e-6);

        c_vector<double, 3> rotation_axis = unit_vector<double>(3, 2);
        p_vertex1->RotateAboutAxis(rotation_axis, M_PI/2.0);
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[0], -new_position[1], 1.e-6);
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[1], new_position[0], 1.e-6);
        TS_ASSERT_DELTA(p_vertex1->rGetLocation()[2], new_position[2], 1.e-6);
    }
};

#endif /*TESTVERTEX_HPP_*/
