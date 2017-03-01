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

#ifndef TESTGRIDCALCULATOR_HPP_
#define TESTGRIDCALCULATOR_HPP_

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
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "GridCalculator.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestRegularGrid : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestPointPointMapGeneration()
    {
        // Set up a grid
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 101;
        dimensions[1] = 101;
        dimensions[2] = 101;
        p_grid->SetDimensions(dimensions);

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
        boost::shared_ptr<GridCalculator<3> > p_grid_calc = GridCalculator<3>::Create();
        p_grid_calc->SetGrid(p_grid);
        std::vector<std::vector<unsigned> > map = p_grid_calc->GetPointMap(points);

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
        EXIT_IF_PARALLEL;    // HoneycombMeshGenerator doesn't work in parallel

        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 101;
        dimensions[1] = 101;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
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
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetCellPopulation(cell_population, 1.e-6 * unit::metres);
        std::vector<std::vector<CellPtr> > map = p_grid_calc->GetCellMap();

        // Make sure all the cells are accounted for
        unsigned sum = 0;
        for (unsigned idx = 0; idx < map.size(); idx++)
        {
            sum += map[idx].size();
        }
        TS_ASSERT_EQUALS(sum, 100u);
    }

    void PointSegmentMapGeneration()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestRegularGrid"));
        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 101;
        dimensions[1] = 101;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(2.0*unit::microns);
        p_grid->SetSpacing(spacing);

        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> width(100.0*unit::microns);
        units::quantity<unit::length> height(100.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(width,
                                                                                                    height,
                                                                                                    spacing);
        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        p_grid->Write(p_handler);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->GetSegmentMap();

        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }
};

#endif /*TESTGRIDCALCULATOR_HPP_*/
