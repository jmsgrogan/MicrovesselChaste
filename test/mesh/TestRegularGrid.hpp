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

#ifndef TESTREGULARGRID_HPP_
#define TESTREGULARGRID_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGridWriter.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

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
        unsigned test_index = 0;
        if (p_grid->GetLocalIndex(test_index)>=0) // non-owing procs ignore
        {
            for(unsigned idx=0; idx<2; idx++)
            {
                TS_ASSERT(neighbours[p_grid->GetLocalIndex(test_index)][idx] == 1 or
                        neighbours[p_grid->GetLocalIndex(test_index)][idx] == 5)
            }
        }
        test_index = 8;
        if (p_grid->GetLocalIndex(test_index)>=0)
        {
            unsigned global_index = p_grid->GetLocalIndex(test_index);
            for(unsigned idx=0; idx<4; idx++)
            {
                TS_ASSERT(neighbours[global_index][idx] == 13 or neighbours[global_index][idx] == 9 or
                        neighbours[global_index][idx] == 3 or neighbours[global_index][idx] == 7)
            }
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

         test_index = 0;
         if (p_grid->GetLocalIndex(test_index)>=0)
         {
             unsigned global_index = p_grid->GetLocalIndex(test_index);
             for(unsigned idx=0; idx<3; idx++)
             {
                 TS_ASSERT(neighbours_3d[global_index][idx] == 1u or neighbours_3d[global_index][idx] == 5u or
                         neighbours_3d[global_index][idx] == 35u)
             }
         }

         test_index = 43;
         if (p_grid->GetLocalIndex(test_index)>=0)
         {
             for(unsigned idx=0; idx<6; idx++)
             {
                 TS_ASSERT(neighbours_3d[test_index][idx] == 8u or
                         neighbours_3d[test_index][idx] == 42u or neighbours_3d[test_index][idx] == 44u or
                           neighbours_3d[test_index][idx] == 78u or
                           neighbours_3d[test_index][idx] == 38u or neighbours_3d[test_index][idx] == 48u)
             }
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
        unsigned test_index = 0;
        if (p_grid->GetLocalIndex(test_index)>=0) // non-owing procs ignore
        {
            for(unsigned idx=0; idx<3; idx++)
            {
                TS_ASSERT(neighbours[test_index][idx] == 1 or neighbours[test_index][idx] == 5 or neighbours[test_index][idx] == 6)
            }
        }

        test_index = 8;
        if (p_grid->GetLocalIndex(test_index)>=0) // non-owing procs ignore
        {
            for(unsigned idx=0; idx<8; idx++)
            {
                TS_ASSERT(neighbours[test_index][idx] == 13 or neighbours[test_index][idx] == 9 or
                        neighbours[test_index][idx] == 3 or neighbours[test_index][idx] == 7
                        or neighbours[test_index][idx] == 12 or neighbours[test_index][idx] == 14 or
                        neighbours[test_index][idx] == 2 or neighbours[test_index][idx] == 4)
            }
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
        dimensions[0] = 10;
        dimensions[1] = 6;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        boost::shared_ptr<OutputFileHandler> p_file_handler =
                boost::shared_ptr<OutputFileHandler>(new OutputFileHandler("TestRegularGrid/TestWrite2d"));

        // Check point data writing
        std::vector<double> local_index;
        for(unsigned idx=0;idx<p_grid->GetNumberOfPoints();idx++)
        {
            local_index.push_back(idx);
        }
        p_grid->AddPointData(local_index, "Local Index");
        p_grid->Write(p_file_handler);
    }

    void TestWriteGrid3d()
    {
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 10;
        dimensions[1] = 10;
        dimensions[2] = 6;
        p_grid->SetDimensions(dimensions);

        boost::shared_ptr<OutputFileHandler> p_file_handler =
                boost::shared_ptr<OutputFileHandler>(new OutputFileHandler("TestRegularGrid/TestWrite3d"));

        // Check point data writing
        std::vector<double> local_index;
        for(unsigned idx=0;idx<p_grid->GetNumberOfPoints();idx++)
        {
            local_index.push_back(idx);
        }
        p_grid->AddPointData(local_index, "Local Index");
        p_grid->Write(p_file_handler);
    }

    void TestBoundingDomain2d()
    {
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 10;
        dimensions[1] = 6;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        boost::shared_ptr<OutputFileHandler> p_file_handler =
                boost::shared_ptr<OutputFileHandler>(new OutputFileHandler("TestRegularGrid/TestBoundingDomain"));
        GeometryWriter writer;
        writer.SetFileName(p_file_handler->GetOutputDirectoryFullPath()+
                "domain_bounds"+boost::lexical_cast<std::string>(PetscTools::GetMyRank())+".vtp");
        writer.AddInput(p_grid->GetBoundingGeometry());
        writer.Write(false);
    }

    void TestBoundingDomain3d()
    {
        boost::shared_ptr<OutputFileHandler> p_file_handler =
                boost::shared_ptr<OutputFileHandler>(new OutputFileHandler("TestRegularGrid/TestBoundingDomain", false));
        boost::shared_ptr<RegularGrid<3> > p_grid3d = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions3d;
        dimensions3d[0] = 10;
        dimensions3d[1] = 10;
        dimensions3d[2] = 10;
        p_grid3d->SetDimensions(dimensions3d);

        GeometryWriter writer;
        writer.SetFileName(p_file_handler->GetOutputDirectoryFullPath()+
                "domain_bounds3d"+boost::lexical_cast<std::string>(PetscTools::GetMyRank())+".vtp");
        writer.AddInput(p_grid3d->GetBoundingGeometry());
        writer.Write(false);
    }
};

#endif /*TESTREGULARGRID_HPP_*/
