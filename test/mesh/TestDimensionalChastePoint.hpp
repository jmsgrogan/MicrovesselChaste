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
        DimensionalChastePoint<2> point1 = DimensionalChastePoint<2>(1.0, 2.0);
        c_vector<double, 2> point2_location;
        point2_location[0] = 2.0;
        point2_location[1] = 3.0;
        DimensionalChastePoint<2> point2 = DimensionalChastePoint<2>(point2_location);

        TS_ASSERT_DELTA(point1.rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(point2.rGetLocation()[1], 3.0, 1.e-6);

        units::quantity<unit::length> reference_scale1(5.0 * unit::metres);
        units::quantity<unit::length> reference_scale2(10.0 * unit::metres);
        DimensionalChastePoint<2> point3 = DimensionalChastePoint<2>(1.0/5.0, 2.0/5.0, 0.0, reference_scale1);
        TS_ASSERT_DELTA(point3.rGetLocation()[0], 0.2, 1.e-6);
        TS_ASSERT_DELTA(point3.rGetLocation()[1], 0.4, 1.e-6);
        c_vector<double, 2> point4_location;
        point4_location[0] = 2.0/10.0;
        point4_location[1] = 3.0/10.0;
        DimensionalChastePoint<2> point4 = DimensionalChastePoint<2>(point4_location, reference_scale2);

        TS_ASSERT_DELTA(point4.GetScalingFactor(point3), 0.5, 1.e-6);
        point3.SetReferenceLengthScale(point4.GetReferenceLengthScale());
        TS_ASSERT_DELTA(point3.rGetLocation()[0], 0.1, 1.e-6);
        TS_ASSERT_DELTA(point3.rGetLocation()[1], 0.2, 1.e-6);

        point4.Translate(point3);
        TS_ASSERT_DELTA(point4.rGetLocation()[0], 3.0/10.0, 1.e-6);
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("DimensionalChastePoint.arch");

        // Save archive
        {
            units::quantity<unit::length> reference_length(10.0*unit::microns);
            boost::shared_ptr<DimensionalChastePoint<3> > p_point =
                    boost::shared_ptr<DimensionalChastePoint<3> >(new DimensionalChastePoint<3>(1.0, 2.0, 3.0, reference_length));

            TS_ASSERT_DELTA(p_point->rGetLocation()[0], 1.0/10.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->rGetLocation()[1], 2.0/10.0, 1.e-6);
            TS_ASSERT_DELTA(p_point->rGetLocation()[2], 3.0/10.0, 1.e-6);

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
            TS_ASSERT_DELTA(p_point_from_archive->rGetLocation()[0], 1.0/10.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->rGetLocation()[1], 2.0/10.0, 1.e-6);
            TS_ASSERT_DELTA(p_point_from_archive->rGetLocation()[2], 3.0/10.0, 1.e-6);
        }
    }
};

#endif /*TESTDIMENSIONALCHASTEPOINT_HPP_*/
