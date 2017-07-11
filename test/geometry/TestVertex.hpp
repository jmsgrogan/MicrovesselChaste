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

#ifndef TESTVertex_HPP_
#define TESTVertex_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "Vertex.hpp"
#include "OutputFileHandler.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVertex: public CxxTest::TestSuite
{
public:

    void TestConstructorsAndScaling()
    {
        // Create points using constructors
        QLength reference_scale1(5_m);
        QLength reference_scale2(10_m);
        Vertex<2> point3 = Vertex<2>(1.0_m, 2.0_m);
        TS_ASSERT_DELTA(point3.Convert(reference_scale1)[0], 0.2, 1.e-6);
        TS_ASSERT_DELTA(point3.Convert(reference_scale1)[1], 0.4, 1.e-6);
        c_vector<double, 2> point4_location;
        point4_location[0] = 2.0/10.0;
        point4_location[1] = 3.0/10.0;
        Vertex<2> point4 = Vertex<2>(point4_location, reference_scale2);

        // Check translation and scaling
        TS_ASSERT_DELTA(point3.Convert(reference_scale2)[0], 0.1, 1.e-6);
        TS_ASSERT_DELTA(point3.Convert(reference_scale2)[1], 0.2, 1.e-6);
        point4.Translate(point3);
        TS_ASSERT_DELTA(point4.Convert(reference_scale2)[0], 3.0/10.0, 1.e-6);
        point4.TranslateTo(point3);
        TS_ASSERT_DELTA(point4.Convert(reference_scale2)[0], 0.1, 1.e-6);
        TS_ASSERT_DELTA(point4.Convert(reference_scale2)[1], 0.2, 1.e-6);

        // Check indices
        point4.SetIndex(10);
        TS_ASSERT_EQUALS(point4.GetIndex(), 10u);

        // Check factory constructors
        auto p_point7 = Vertex<2>::Create(1_m, 2_m, 0_m);
        c_vector<double, 2> point8_location;
        point8_location[0] = 2.0;
        point8_location[1] = 3.0;
        auto p_point8 = Vertex<2>::Create(point8_location, reference_scale2);
        TS_ASSERT_DELTA(p_point7->Convert(reference_scale2)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_point7->Convert(reference_scale2)[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_point8->Convert(reference_scale2)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_point8->Convert(reference_scale2)[1], 3.0, 1.e-6);
    }

    void TestGeometryOperations()
    {
        QLength reference_scale(1.0_m);
        Vertex<2> point1 = Vertex<2>(1.0_m, 2.0_m);
        Vertex<2> point2 = Vertex<2>(2.0_m, 3.0_m);
        TS_ASSERT_DELTA(point1.GetDistance(point2)/1_m, std::sqrt(2.0), 1.e-6);
        TS_ASSERT_DELTA(point1.GetMidPoint(point2).Convert(reference_scale)[0], 1.5, 1.e-6);
        TS_ASSERT_DELTA(point1.GetMidPoint(point2).Convert(reference_scale)[1], 2.5, 1.e-6);
        TS_ASSERT_DELTA(point1.GetNorm2()/1_m, std::sqrt(5.0), 1.e-6);

        QLength reference_scale2(5.0 * unit::metres);
        c_vector<double, 2> scaled_location = point1.Convert(reference_scale2);
        TS_ASSERT_DELTA(scaled_location[0], 1.0/5.0, 1.e-6);
        TS_ASSERT_DELTA(scaled_location[1], 2.0/5.0, 1.e-6);
        c_vector<double, 2> unit_vector = point1.GetUnitVector();
        TS_ASSERT_DELTA(unit_vector[0], 1.0/std::sqrt(5.0), 1.e-6);
        TS_ASSERT_DELTA(unit_vector[1], 2.0/std::sqrt(5.0), 1.e-6);

        c_vector<double, 2> unit_tangent = point1.GetUnitTangent(point2);
        TS_ASSERT_DELTA(unit_tangent[0], std::sqrt(2.0)/2.0, 1.e-6);
        TS_ASSERT_DELTA(unit_tangent[1], std::sqrt(2.0)/2.0, 1.e-6);
        Vertex<2> point3 = Vertex<2>(1.0_m, 2.0_m);
        TS_ASSERT(point1.IsCoincident(point3));
        TS_ASSERT(!point1.IsCoincident(point2));

        Vertex<3> point4 = Vertex<3>(1.0_m, 2.0_m);
        c_vector<double, 3> rotation_axis;
        rotation_axis[0] = 0.0;
        rotation_axis[1] = 0.0;
        rotation_axis[2] = 1.0;
        point4.RotateAboutAxis(rotation_axis, M_PI);
        TS_ASSERT_DELTA(point4.Convert(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point4.Convert(reference_scale)[1], -2.0, 1.e-6);

        Vertex<2> point5 = Vertex<2>(1.0_m, 2.0_m);
        point5.RotateAboutAxis(rotation_axis, M_PI);
        TS_ASSERT_DELTA(point5.Convert(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point5.Convert(reference_scale)[1], -2.0, 1.e-6);
    }

    void TestOverloadedOperators()
    {
        QLength reference_scale(1.0 * unit::metres);
        Vertex<2> point1 = Vertex<2>(1.0_m, 2.0_m);
        Vertex<2> point2 = Vertex<2>(2.0_m, 3.0_m);

        Vertex<2> point3 = point1 + point2;
        TS_ASSERT_DELTA(point3.Convert(reference_scale)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(point3.Convert(reference_scale)[1], 5.0, 1.e-6);

        Vertex<2> point4 = point1 *3.0;
        TS_ASSERT_DELTA(point4.Convert(reference_scale)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(point4.Convert(reference_scale)[1], 6.0, 1.e-6);

        Vertex<2> point5 = point1 / 2.0;
        TS_ASSERT_DELTA(point5.Convert(reference_scale)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(point5.Convert(reference_scale)[1], 1.0, 1.e-6);

        Vertex<2> point6 = point1- point2;
        TS_ASSERT_DELTA(point6.Convert(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point6.Convert(reference_scale)[1], -1.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("Vertex.arch");
        QLength reference_length(10.0*unit::microns);
        // Save archive
        {

            std::shared_ptr<Vertex<3> > p_point = Vertex<3>::Create(1.0_m, 2.0_m, 3.0_m);
            TS_ASSERT_DELTA(p_point->Convert(reference_length)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->Convert(reference_length)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->Convert(reference_length)[2], 3.0, 1.e-6);

            std::ofstream ofs(archive_filename.c_str());
            ofs << std::scientific;
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_point;
        }

        // Load archive
        {
            std::shared_ptr<Vertex<3> > p_point_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_point_from_archive;
            TS_ASSERT_DELTA(p_point_from_archive->Convert(reference_length)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->Convert(reference_length)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->Convert(reference_length)[2], 3.0, 1.e-6);
        }
    }
};

#endif /*TESTVertex_HPP_*/
