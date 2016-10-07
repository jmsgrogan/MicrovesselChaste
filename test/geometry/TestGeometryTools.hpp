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

#ifndef TESTGEOMETRYTOOLS_HPP_
#define TESTGEOMETRYTOOLS_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "GeometryTools.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestGeometryTools : public CxxTest::TestSuite
{

public:

    void TestLineInBoxBothOutside()
    {
        double spacing = 1.0;
        c_vector<double,3> centre;
        centre[0] = 0.5;
        centre[1] = 0.5;
        centre[2] = 0.5;

        c_vector<double,3> point1;
        point1[0] = -1.5;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 1.5;
        point2[1] = 0.5;
        point2[2] = 0.5;

        double length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length, 1.0, 1.e-6);
    }

    void TestLineInBoxBothOutsideNotCrossing()
    {
        double spacing = 1.0;
        c_vector<double,3> centre;
        centre[0] = 0.5;
        centre[1] = 0.5;
        centre[2] = 0.5;

        c_vector<double,3> point1;
        point1[0] = -1.5;
        point1[1] = 1.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 1.5;
        point2[1] = 1.5;
        point2[2] = 0.5;

        double length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length, 0.0, 1.e-6);
    }

    void TestLineInBoxInside()
    {
        double spacing = 1.0;
        c_vector<double,3> centre;
        centre[0] = 0.5;
        centre[1] = 0.5;
        centre[2] = 0.5;

        c_vector<double,3> point1;
        point1[0] = 0.25;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 0.75;
        point2[1] = 0.5;
        point2[2] = 0.5;

        double length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length, 0.5, 1.e-6);
    }

    void TestLineInBoxStartInside()
    {
        double spacing = 1.0;
        c_vector<double,3> centre;
        centre[0] = 0.5;
        centre[1] = 0.5;
        centre[2] = 0.5;

        c_vector<double,3> point1;
        point1[0] = 0.25;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 1.5;
        point2[1] = 0.5;
        point2[2] = 0.5;

        double length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length, 0.75, 1.e-6);
    }

    void TestLineInBoxEndInside()
    {
        double spacing = 1.0;
        c_vector<double,3> centre;
        centre[0] = 0.5;
        centre[1] = 0.5;
        centre[2] = 0.5;

        c_vector<double,3> point1;
        point1[0] = -1.5;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 0.75;
        point2[1] = 0.5;
        point2[2] = 0.5;

        double length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length, 0.75, 1.e-6);
    }

    void TestLineInTetra()
    {
        std::vector<c_vector<double,3> > tetra_points;
        c_vector<double,3> tetra1;
        tetra1[0] = 0.0;
        tetra1[1] = 0.0;
        tetra1[2] = 0.0;
        tetra_points.push_back(tetra1);

        c_vector<double,3> tetra2;
        tetra2[0] = 1.0;
        tetra2[1] = 0.0;
        tetra2[2] = 0.0;
        tetra_points.push_back(tetra2);

        c_vector<double,3> tetra3;
        tetra3[0] = 0.5;
        tetra3[1] = 1.0;
        tetra3[2] = 0.0;
        tetra_points.push_back(tetra3);

        c_vector<double,3> tetra4;
        tetra4[0] = 0.5;
        tetra4[1] = 0.5;
        tetra4[2] = 1.0;
        tetra_points.push_back(tetra4);

        c_vector<double,3> point1;
        point1[0] = -0.5;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point2;
        point2[0] = 1.5;
        point2[1] = 0.5;
        point2[2] = 0.5;

        c_vector<double,3> point3;
        point1[0] = 0.5;
        point1[1] = 0.5;
        point1[2] = 0.5;

        c_vector<double,3> point4;
        point2[0] = 0.5;
        point2[1] = 0.5;
        point2[2] = 0.6;

        double length = LengthOfLineInTetra<3>(point1, point2, tetra_points);
        TS_ASSERT_DELTA(length, 0.25, 1.e-6);

        double length2 = LengthOfLineInTetra<3>(point3, point4, tetra_points);
        TS_ASSERT_DELTA(length2, 0.1, 1.e-6);

        double length3 = LengthOfLineInTetra<3>(point1, point3, tetra_points);
        TS_ASSERT_DELTA(length3, 0.125, 1.e-6);

        double length4 = LengthOfLineInTetra<3>(point3, point1, tetra_points);
        TS_ASSERT_DELTA(length4, 0.125, 1.e-6);
    }
};

#endif /*TESTGEOMETRYTOOLS_HPP_*/
