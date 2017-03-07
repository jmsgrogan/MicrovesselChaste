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

#ifndef TESTREGULARGRID_HPP_
#define TESTREGULARGRID_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGridWriter.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestRegularGrid : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestIndexing() throw(Exception)
    {
        // Set up a 2d grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 5;
        dimensions[1] = 7;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        TS_ASSERT_EQUALS(p_grid->GetGlobalGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid->GetGlobalGridIndex(3,1,0), 8u)
        TS_ASSERT_EQUALS(p_grid->GetGlobalGridIndex(0,3,0), 15u)

        // Set up a 3d grid
        boost::shared_ptr<RegularGrid<3> > p_grid_3d = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions3d;
        dimensions3d[0] = 5;
        dimensions3d[1] = 7;
        dimensions3d[2] = 4;
        p_grid_3d->SetDimensions(dimensions3d);

        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(3,1,2), 78u)
        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(0,3,3), 120u)
    }

    void TestNeighbourCalculation()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 5;
        dimensions[1] = 7;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        // Get neighbours
        std::vector<std::vector<unsigned> > neighbours =  p_grid->rGetNeighbourData();
        for(unsigned idx=0; idx<2; idx++)
        {
            TS_ASSERT(neighbours[0][idx] == 1 or neighbours[0][idx] == 5)
        }
        for(unsigned idx=0; idx<4; idx++)
        {
            TS_ASSERT(neighbours[8][idx] == 13 or neighbours[8][idx] == 9 or neighbours[8][idx] == 3 or neighbours[8][idx] == 7)
        }

        // Set up a 3d grid
         boost::shared_ptr<RegularGrid<3> > p_grid_3d = RegularGrid<3>::Create();
         c_vector<unsigned, 3> dimensions3d;
         dimensions3d[0] = 5;
         dimensions3d[1] = 7;
         dimensions3d[2] = 4;
         p_grid_3d->SetDimensions(dimensions3d);
         // Get neighbours
         std::vector<std::vector<unsigned> > neighbours_3d =  p_grid_3d->rGetNeighbourData();
         for(unsigned idx=0; idx<3; idx++)
         {
             TS_ASSERT(neighbours_3d[0][idx] == 1u or neighbours_3d[0][idx] == 5u or neighbours_3d[0][idx] == 35u)
         }
         for(unsigned idx=0; idx<6; idx++)
         {
             TS_ASSERT(neighbours_3d[43][idx] == 8u or neighbours_3d[43][idx] == 42u or neighbours_3d[43][idx] == 44u or
                       neighbours_3d[43][idx] == 78u or neighbours_3d[43][idx] == 38u or neighbours_3d[43][idx] == 48u)
         }
    }

    void TestMooreNeighbourCalculation()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 5;
        dimensions[1] = 7;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        // Get neighbours
        std::vector<std::vector<unsigned> > neighbours =  p_grid->rGetMooreNeighbourData();
        for(unsigned idx=0; idx<3; idx++)
        {
            TS_ASSERT(neighbours[0][idx] == 1 or neighbours[0][idx] == 5 or neighbours[0][idx] == 6)
        }
        for(unsigned idx=0; idx<8; idx++)
        {
            TS_ASSERT(neighbours[8][idx] == 13 or neighbours[8][idx] == 9 or neighbours[8][idx] == 3 or neighbours[8][idx] == 7
                    or neighbours[8][idx] == 12 or neighbours[8][idx] == 14 or neighbours[8][idx] == 2 or neighbours[8][idx] == 4)
        }
    }

    void TestGetPointBoundingBox2d()
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 11;
        dimensions[1] = 11;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        p_grid->SetSpacing(1.0*unit::metres);

        c_vector<double,6> bbox1 = p_grid->GetPointBoundingBox(0, 0, 0);
        TS_ASSERT_DELTA(bbox1[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox1[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox1[2], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox1[3], 0.5, 1.e-6);

        c_vector<double,6> bbox2 = p_grid->GetPointBoundingBox(1, 0, 0);
        TS_ASSERT_DELTA(bbox2[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[1], 1.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[2], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox2[3], 0.5, 1.e-6);

        c_vector<double,6> bbox3 = p_grid->GetPointBoundingBox(1, 1, 0);
        TS_ASSERT_DELTA(bbox3[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[1], 1.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[2], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[3], 1.5, 1.e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestGetPointBoundingBox3d()
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 11;
        dimensions[1] = 11;
        dimensions[2] = 11;
        p_grid->SetDimensions(dimensions);
        p_grid->SetSpacing(1.0*unit::metres);

        c_vector<double,6> bbox1 = p_grid->GetPointBoundingBox(0, 0, 0);
        TS_ASSERT_DELTA(bbox1[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox1[1], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox1[2], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox1[3], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox1[4], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox1[5], 0.5, 1.e-6);

        c_vector<double,6> bbox2 = p_grid->GetPointBoundingBox(1, 0, 1);
        TS_ASSERT_DELTA(bbox2[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[1], 1.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[2], 0.0, 1.e-6);
        TS_ASSERT_DELTA(bbox2[3], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[4], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox2[5], 1.5, 1.e-6);

        c_vector<double,6> bbox3 = p_grid->GetPointBoundingBox(1, 1, 1);
        TS_ASSERT_DELTA(bbox3[0], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[1], 1.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[2], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[3], 1.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[4], 0.5, 1.e-6);
        TS_ASSERT_DELTA(bbox3[5], 1.5, 1.e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestWriteGrid2d()
    {
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 5;
        dimensions[1] = 7;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        boost::shared_ptr<OutputFileHandler> p_file_handler =
                boost::shared_ptr<OutputFileHandler>(new OutputFileHandler("TestRegularGrid/TestWrite2d"));
        p_grid->Write(p_file_handler);
    }
};

#endif /*TESTREGULARGRID_HPP_*/
