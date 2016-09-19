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

#ifndef TESTREGULARGRID_HPP_
#define TESTREGULARGRID_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "RandomNumberGenerator.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "MutableMesh.hpp"
#include "CellsGenerator.hpp"
#include "TransitCellProliferativeType.hpp"
#include "UniformCellCycleModel.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestRegularGrid : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestIndexing() throw(Exception)
    {
        // Set up a 2d grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 5;
        extents[1] = 7;
        extents[2] = 1;
        p_grid->SetExtents(extents);

        TS_ASSERT_EQUALS(p_grid->Get1dGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid->Get1dGridIndex(3,1,0), 8u)
        TS_ASSERT_EQUALS(p_grid->Get1dGridIndex(0,3,0), 15u)

        // Set up a 3d grid
        boost::shared_ptr<RegularGrid<3> > p_grid_3d = RegularGrid<3>::Create();
        std::vector<unsigned> extents_3d(3);
        extents_3d[0] = 5;
        extents_3d[1] = 7;
        extents_3d[2] = 4;
        p_grid_3d->SetExtents(extents_3d);

        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(3,1,2), 78u)
        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(0,3,3), 120u)
    }

    void TestNeighbourCalculation()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 5;
        extents[1] = 7;
        extents[2] = 1;
        p_grid->SetExtents(extents);

        // Get neighbours
        std::vector<std::vector<unsigned> > neighbours =  p_grid->GetNeighbourData();
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
         std::vector<unsigned> extents_3d(3);
         extents_3d[0] = 5;
         extents_3d[1] = 7;
         extents_3d[2] = 4;
         p_grid_3d->SetExtents(extents_3d);

         // Get neighbours
         std::vector<std::vector<unsigned> > neighbours_3d =  p_grid_3d->GetNeighbourData();
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

    void TestPointPointMapGeneration()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 101;
        extents[1] = 101;
        extents[2] = 101;
        p_grid->SetExtents(extents);

        // Set up points
        RandomNumberGenerator::Instance()->Reseed(1000);
        std::vector<DimensionalChastePoint<3> > points(10000);
        for (unsigned idx = 0; idx < 10000; idx++)
        {
            DimensionalChastePoint<3> location(RandomNumberGenerator::Instance()->ranf() * 100.0,
                                               RandomNumberGenerator::Instance()->ranf() * 100.0,
                                               RandomNumberGenerator::Instance()->ranf() * 100.0);
            points[idx] = location;
        }

        // Get a point-point map
        std::vector<std::vector<unsigned> > map = p_grid->GetPointPointMap(points);

        // Make sure all the points are accounted for
        unsigned sum = 0;
        for (unsigned idx = 0; idx < map.size(); idx++)
        {
            sum += map[idx].size();
        }
        TS_ASSERT_EQUALS(sum, 10000u);
    }

    void TestPointCellMapGeneration()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 101;
        extents[1] = 101;
        extents[2] = 1;
        p_grid->SetExtents(extents);
        p_grid->SetSpacing(0.333 * 1.e-6 * unit::metres);

        // Set up cells
        HoneycombMeshGenerator generator(10, 10);    // Parameters are: cells across, cells up
        MutableMesh<2, 2>* p_mesh = generator.GetMesh();
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);
        CellsGenerator<UniformCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumNodes(), p_transit_type);

        MeshBasedCellPopulation<2> cell_population(*p_mesh, cells);

        // Get a point-cell map
        p_grid->SetCellPopulation(cell_population);
        std::vector<std::vector<CellPtr> > map = p_grid->GetPointCellMap();

        // Make sure all the cells are accounted for
        unsigned sum = 0;
        for (unsigned idx = 0; idx < map.size(); idx++)
        {
            sum += map[idx].size();
        }
        TS_ASSERT_EQUALS(sum, 100u);
    }

    void TestInterpolateGridValues() throw (Exception)
    {
        // Set up a grid
        RandomNumberGenerator::Instance()->Reseed(1000);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 101;
        extents[1] = 101;
        extents[2] = 101;
        p_grid->SetExtents(extents);
        double spacing = 0.33;
        p_grid->SetSpacing(spacing* 1.e-6 * unit::metres);

        // Set up a function increasing quadratically from bottom front left to top back right
        std::vector<double> my_grid_func(extents[0] * extents[1] * extents[2]);
        for (unsigned idx = 0; idx < extents[2]; idx++)
        {
            for (unsigned jdx = 0; jdx < extents[1]; jdx++)
            {
                for (unsigned kdx = 0; kdx < extents[0]; kdx++)
                {
                    double value = spacing * spacing * double(kdx * kdx + jdx * jdx + idx * idx);
                    unsigned grid_index = kdx + jdx * extents[0] + idx * extents[0] * extents[1];
                    my_grid_func[grid_index] = value;
                }
            }
        }

        // Set up some sample points
        std::vector<DimensionalChastePoint<3> > points(100);
        for (unsigned idx = 0; idx < 100; idx++)
        {
            DimensionalChastePoint<3> location(RandomNumberGenerator::Instance()->ranf() * 30.0,
                                               RandomNumberGenerator::Instance()->ranf() * 30.0,
                                               RandomNumberGenerator::Instance()->ranf() * 30.0);
            points[idx] = location;
        }

        // Get the interpolated values
        std::vector<double> interpolated_vals = p_grid->InterpolateGridValues(points, my_grid_func);

        // Get the max error
        double max_error = 0.0;
        for (unsigned idx = 0; idx < interpolated_vals.size(); idx++)
        {
            double analytical = points[idx][0] * points[idx][0] + points[idx][1] * points[idx][1]
                    + points[idx][2] * points[idx][2];
            double error = std::abs((analytical - interpolated_vals[idx]) / analytical);
            if (error > max_error)
            {
                max_error = error;
            }
        }
        TS_ASSERT(max_error < 0.1);
        std::cout << "Max Error: " << max_error << std::endl;
    }

    void TestInterpolateGridValuesWithVtk() throw (Exception)
    {
        // Set up a grid
        RandomNumberGenerator::Instance()->Reseed(1000);
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        std::vector<unsigned> extents(3);
        extents[0] = 101;
        extents[1] = 101;
        extents[2] = 101;
        p_grid->SetExtents(extents);
        double spacing = 0.33;
        p_grid->SetSpacing(spacing* 1.e-6 * unit::metres);

        // Set up a function increasing quadratically from bottom front left to top back right
        std::vector<double> my_grid_func(extents[0] * extents[1] * extents[2]);
        for (unsigned idx = 0; idx < extents[2]; idx++)
        {
            for (unsigned jdx = 0; jdx < extents[1]; jdx++)
            {
                for (unsigned kdx = 0; kdx < extents[0]; kdx++)
                {
                    double value = spacing * spacing * double(kdx * kdx + jdx * jdx + idx * idx);
                    unsigned grid_index = kdx + jdx * extents[0] + idx * extents[0] * extents[1];
                    my_grid_func[grid_index] = value;
                }
            }
        }

        // Set up some sample points
        // Set up some sample points
        std::vector<DimensionalChastePoint<3> > points(100);
        for (unsigned idx = 0; idx < 100; idx++)
        {
            DimensionalChastePoint<3> location(RandomNumberGenerator::Instance()->ranf() * 30.0,
                                               RandomNumberGenerator::Instance()->ranf() * 30.0,
                                               RandomNumberGenerator::Instance()->ranf() * 30.0);
            points[idx] = location;
        }

        // Get the interpolated values
        std::vector<double> interpolated_vals = p_grid->InterpolateGridValues(points, my_grid_func, true);

        // Get the max error
        double max_error = 0.0;
        for (unsigned idx = 0; idx < interpolated_vals.size(); idx++)
        {
            double analytical = points[idx][0] * points[idx][0] + points[idx][1] * points[idx][1]
                    + points[idx][2] * points[idx][2];
            double error = std::abs((analytical - interpolated_vals[idx]) / analytical);
            if (error > max_error)
            {
                max_error = error;
            }
            std::cout << "vals: " << analytical << "," << interpolated_vals[idx] << std::endl;

        }
        std::cout << "Max Error: " << max_error << std::endl;
        TS_ASSERT(max_error < 0.1);
    }
};

#endif /*TESTREGULARGRID_HPP_*/
