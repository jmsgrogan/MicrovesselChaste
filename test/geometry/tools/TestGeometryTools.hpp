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

#ifndef TESTGEOMETRYTOOLS_HPP_
#define TESTGEOMETRYTOOLS_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "GeometryTools.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestGeometryTools : public CxxTest::TestSuite
{

public:

    void TestGeometryOperations()
    {
        QLength reference_length = 1.0_m;
        DimensionalChastePoint<3> point1(1.0, 2.0, 0.5, reference_length);
        DimensionalChastePoint<3> point2(2.0, 4.0, 0.5, reference_length);
        DimensionalChastePoint<3> point3(1.5, 4.0, 0.5, reference_length);

        TS_ASSERT_DELTA(GetDistance(point1, point2)/1_m, std::sqrt(5.0), 1.e-6);
        TS_ASSERT_DELTA(GetDistanceToLineSegment(point1, point2, point3)/1_m, 0.4472, 1.e-4);
    }

    void TestLineInBoxBothOutside()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point1(-1.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(1.5, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 1.0, 1.e-6);
    }

    void TestLineInBoxBothOutsideNotCrossing()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point1(-1.5, 1.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(1.5, 1.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.0, 1.e-6);
    }

    void TestLineInBoxInside()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point1(0.25, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(0.75, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.5, 1.e-6);
    }

    void TestLineInBoxStartInside()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point1(0.25, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(1.5, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.75, 1.e-6);
    }

    void TestLineInBoxEndInside()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<3> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point1(-1.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(0.75, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<3>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.75, 1.e-6);
    }

    void TestLineInBoxBothOutside2d()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<2> centre(0.5, 0.5, 0.0, spacing);
        DimensionalChastePoint<2> point1(-1.5, 0.5, 0.0, spacing);
        DimensionalChastePoint<2> point2(1.5, 0.5, 0.0, spacing);
        QLength length = LengthOfLineInBox<2>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 1.0, 1.e-6);
    }

    void TestLineInBoxBothOutsideNotCrossing2d()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<2> centre(0.5, 0.5, 0.0, spacing);
        DimensionalChastePoint<2> point1(-1.5, 1.5, 0.0, spacing);
        DimensionalChastePoint<2> point2(1.5, 1.5, 0.0, spacing);
        QLength length = LengthOfLineInBox<2>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.0, 1.e-6);
    }

    void TestLineInBoxInside2d()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<2> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point1(0.25, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point2(0.75, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<2>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.5, 1.e-6);
    }

    void TestLineInBoxStartInside2d()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<2> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point1(0.25, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point2(1.5, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<2>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.75, 1.e-6);
    }

    void TestLineInBoxEndInside2d()
    {
        QLength spacing = 1.0 * unit::metres;
        DimensionalChastePoint<2> centre(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point1(-1.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<2> point2(0.75, 0.5, 0.5, spacing);
        QLength length = LengthOfLineInBox<2>(point1, point2, centre, spacing);
        TS_ASSERT_DELTA(length/1_m, 0.75, 1.e-6);
    }

    void TestLineInTetra()
    {
        QLength spacing = 1.0 * unit::metres;

        std::vector<DimensionalChastePoint<3> > tetra_points;
        DimensionalChastePoint<3> tetra1(0.0, 0.0, 0.0, spacing);
        tetra_points.push_back(tetra1);
        DimensionalChastePoint<3> tetra2(1.0, 0.0, 0.0, spacing);
        tetra_points.push_back(tetra2);
        DimensionalChastePoint<3> tetra3(0.5, 1.0, 0.0, spacing);
        tetra_points.push_back(tetra3);
        DimensionalChastePoint<3> tetra4(0.5, 0.5, 1.0, spacing);
        tetra_points.push_back(tetra4);

        DimensionalChastePoint<3> point1(-0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point2(1.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point3(0.5, 0.5, 0.5, spacing);
        DimensionalChastePoint<3> point4(0.5, 0.5, 0.6, spacing);

        QLength length = LengthOfLineInTetra<3>(point1, point2, tetra_points);
        TS_ASSERT_DELTA(length/1_m, 0.25, 1.e-6);
        QLength length2 = LengthOfLineInTetra<3>(point3, point4, tetra_points);
        TS_ASSERT_DELTA(length2/1_m, 0.1, 1.e-6);
        QLength length3 = LengthOfLineInTetra<3>(point1, point3, tetra_points);
        TS_ASSERT_DELTA(length3/1_m, 0.125, 1.e-6);
        QLength length4 = LengthOfLineInTetra<3>(point3, point1, tetra_points);
        TS_ASSERT_DELTA(length4/1_m, 0.125, 1.e-6);
    }
};

#endif /*TESTGEOMETRYTOOLS_HPP_*/
