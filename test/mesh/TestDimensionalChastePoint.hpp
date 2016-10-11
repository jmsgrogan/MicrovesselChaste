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

#ifndef TESTDIMENSIONALCHASTEPOINT_HPP_
#define TESTDIMENSIONALCHASTEPOINT_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "DimensionalChastePoint.hpp"
#include "OutputFileHandler.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestDimensionalChastePoint: public CxxTest::TestSuite
{
public:

    void TestConstructorsAndScaling()
    {
        // Create points using constructors
        units::quantity<unit::length> reference_scale1(5.0 * unit::metres);
        units::quantity<unit::length> reference_scale2(10.0 * unit::metres);
        DimensionalChastePoint<2> point3 = DimensionalChastePoint<2>(1.0/5.0, 2.0/5.0, 0.0, reference_scale1);
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale1)[0], 0.2, 1.e-6);
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale1)[1], 0.4, 1.e-6);
        c_vector<double, 2> point4_location;
        point4_location[0] = 2.0/10.0;
        point4_location[1] = 3.0/10.0;
        DimensionalChastePoint<2> point4 = DimensionalChastePoint<2>(point4_location, reference_scale2);

        // Check translation and scaling
        point3.SetReferenceLengthScale(point4.GetReferenceLengthScale());
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale2)[0], 0.1, 1.e-6);
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale2)[1], 0.2, 1.e-6);
        point4.Translate(point3);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale2)[0], 3.0/10.0, 1.e-6);
        point4.TranslateTo(point3);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale2)[0], 0.1, 1.e-6);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale2)[1], 0.2, 1.e-6);

        // Check indices
        point4.SetIndex(10);
        TS_ASSERT_EQUALS(point4.GetIndex(), 10u);

        // Check factory constructors
        boost::shared_ptr<DimensionalChastePoint<2> > p_point7 = DimensionalChastePoint<2>::Create(1.0, 2.0, 0.0, reference_scale2);
        c_vector<double, 2> point8_location;
        point8_location[0] = 2.0;
        point8_location[1] = 3.0;
        boost::shared_ptr<DimensionalChastePoint<2> > p_point8 = DimensionalChastePoint<2>::Create(point8_location, reference_scale2);
        TS_ASSERT_DELTA(p_point7->GetLocation(reference_scale2)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_point7->GetLocation(reference_scale2)[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_point8->GetLocation(reference_scale2)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_point8->GetLocation(reference_scale2)[1], 3.0, 1.e-6);
    }

    void TestZeroReferenceLengthExceptions()
    {
        units::quantity<unit::length> reference_scale(0.0 * unit::metres);
        TS_ASSERT_THROWS_THIS(DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale), "Point has zero reference length");

        c_vector<double, 2> point_location;
        point_location[0] = 2.0;
        point_location[1] = 3.0;
        TS_ASSERT_THROWS_THIS(DimensionalChastePoint<2>(point_location, reference_scale), "Point has zero reference length");

        units::quantity<unit::length> reference_scale1(1.0 * unit::metres);
        DimensionalChastePoint<2> point1 = DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale1);
        TS_ASSERT_THROWS_THIS(point1.SetReferenceLengthScale(reference_scale), "Attempted to assign a zero length scale");
    }

    void TestGeometryOperations()
    {
        units::quantity<unit::length> reference_scale(1.0 * unit::metres);
        DimensionalChastePoint<2> point1 = DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale);
        DimensionalChastePoint<2> point2 = DimensionalChastePoint<2>(2.0, 3.0, 0.0, reference_scale);
        TS_ASSERT_DELTA(point1.GetDistance(point2).value(), std::sqrt(2.0), 1.e-6);
        TS_ASSERT_DELTA(point1.GetMidPoint(point2).GetLocation(reference_scale)[0], 1.5, 1.e-6);
        TS_ASSERT_DELTA(point1.GetMidPoint(point2).GetLocation(reference_scale)[1], 2.5, 1.e-6);
        TS_ASSERT_DELTA(point1.GetNorm2().value(), std::sqrt(5.0), 1.e-6);

        units::quantity<unit::length> reference_scale2(5.0 * unit::metres);
        c_vector<double, 2> scaled_location = point1.GetLocation(reference_scale2);
        TS_ASSERT_DELTA(scaled_location[0], 1.0/5.0, 1.e-6);
        TS_ASSERT_DELTA(scaled_location[1], 2.0/5.0, 1.e-6);
        c_vector<double, 2> unit_vector = point1.GetUnitVector();
        TS_ASSERT_DELTA(unit_vector[0], 1.0/std::sqrt(5.0), 1.e-6);
        TS_ASSERT_DELTA(unit_vector[1], 2.0/std::sqrt(5.0), 1.e-6);

        c_vector<double, 2> unit_tangent = point1.GetUnitTangent(point2);
        TS_ASSERT_DELTA(unit_tangent[0], std::sqrt(2.0)/2.0, 1.e-6);
        TS_ASSERT_DELTA(unit_tangent[1], std::sqrt(2.0)/2.0, 1.e-6);
        DimensionalChastePoint<2> point3 = DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale);
        TS_ASSERT(point1.IsCoincident(point3));
        TS_ASSERT(!point1.IsCoincident(point2));

        DimensionalChastePoint<3> point4 = DimensionalChastePoint<3>(1.0, 2.0, 0.0, reference_scale);
        c_vector<double, 3> rotation_axis;
        rotation_axis[0] = 0.0;
        rotation_axis[1] = 0.0;
        rotation_axis[2] = 1.0;
        point4.RotateAboutAxis(rotation_axis, M_PI);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale)[1], -2.0, 1.e-6);

        DimensionalChastePoint<2> point5 = DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale);
        point5.RotateAboutAxis(rotation_axis, M_PI);
        TS_ASSERT_DELTA(point5.GetLocation(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point5.GetLocation(reference_scale)[1], -2.0, 1.e-6);
    }

    void TestOverloadedOperators()
    {
        units::quantity<unit::length> reference_scale(1.0 * unit::metres);
        DimensionalChastePoint<2> point1 = DimensionalChastePoint<2>(1.0, 2.0, 0.0, reference_scale);
        DimensionalChastePoint<2> point2 = DimensionalChastePoint<2>(2.0, 3.0, 0.0, reference_scale);

        DimensionalChastePoint<2> point3 = point1 + point2;
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(point3.GetLocation(reference_scale)[1], 5.0, 1.e-6);

        DimensionalChastePoint<2> point4 = point1 *3.0;
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(point4.GetLocation(reference_scale)[1], 6.0, 1.e-6);

        DimensionalChastePoint<2> point5 = point1 / 2.0;
        TS_ASSERT_DELTA(point5.GetLocation(reference_scale)[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(point5.GetLocation(reference_scale)[1], 1.0, 1.e-6);

        DimensionalChastePoint<2> point6 = point1- point2;
        TS_ASSERT_DELTA(point6.GetLocation(reference_scale)[0], -1.0, 1.e-6);
        TS_ASSERT_DELTA(point6.GetLocation(reference_scale)[1], -1.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("DimensionalChastePoint.arch");
        units::quantity<unit::length> reference_length(10.0*unit::microns);
        // Save archive
        {

            boost::shared_ptr<DimensionalChastePoint<3> > p_point = DimensionalChastePoint<3>::Create(1.0, 2.0, 3.0, reference_length);
            TS_ASSERT_DELTA(p_point->GetLocation(reference_length)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->GetLocation(reference_length)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->GetLocation(reference_length)[2], 3.0, 1.e-6);

            std::ofstream ofs(archive_filename.c_str());
            ofs << std::scientific;
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_point;
        }

        // Load archive
        {
            boost::shared_ptr<DimensionalChastePoint<3> > p_point_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_point_from_archive;
            TS_ASSERT_DELTA(p_point_from_archive->GetReferenceLengthScale().value(), 10.e-6, 1.e-8);
            TS_ASSERT_DELTA(p_point_from_archive->GetLocation(reference_length)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->GetLocation(reference_length)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->GetLocation(reference_length)[2], 3.0, 1.e-6);
        }
    }
};

#endif /*TESTDIMENSIONALCHASTEPOINT_HPP_*/
