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
#include "Timer.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestGridCalculator : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestPointSegmentMapGenerationRegularGrid()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointSegmentMapGenerationRegularGrid"));

        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 11;
        dimensions[1] = 11;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->SetSpacing(spacing);

        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> length(92.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(length,
                                                                                                DimensionalChastePoint<2>(50.0, 4.0));
        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap();

        std::vector<std::pair<unsigned, unsigned> > global_index_value_pairs;
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(4, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(5, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(6, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(16, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(27, 1));

        for(unsigned idx=0;idx<global_index_value_pairs.size();idx++)
        {
            if(p_grid->GetLocalIndex(global_index_value_pairs[idx].first)>=0)
            {
                TS_ASSERT(map[p_grid->GetLocalIndex(global_index_value_pairs[idx].first)].size()==
                        global_index_value_pairs[idx].second);
            }
        }

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, false, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void TestPointSegmentMapGenerationRegularGridWithJiggle()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointSegmentMapGenerationRegularGridWithJiggle"));

        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 11;
        dimensions[1] = 11;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->SetSpacing(spacing);

        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> length(92.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(length,
                                                                                                DimensionalChastePoint<2>(45.0, 4.0));
        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap();
        std::vector<std::pair<unsigned, unsigned> > global_index_value_pairs;
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(3, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(4, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(5, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(15, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(26, 1));
        for(unsigned idx=0;idx<global_index_value_pairs.size();idx++)
        {
            if(p_grid->GetLocalIndex(global_index_value_pairs[idx].first)>=0)
            {
                TS_ASSERT(map[p_grid->GetLocalIndex(global_index_value_pairs[idx].first)].size()==
                        global_index_value_pairs[idx].second);
            }
        }

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, false, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void xTestPointSegmentMapGenerationRegularGridWithSurface()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointSegmentMapGenerationRegularGridWithSurface"));

        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 11;
        dimensions[1] = 11;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->SetSpacing(spacing);

        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> length(92.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(length,
                                                                                                DimensionalChastePoint<2>(50.0, 4.0));
        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap();

        std::vector<std::pair<unsigned, unsigned> > global_index_value_pairs;
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(4, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(5, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(6, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(16, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(27, 1));

        for(unsigned idx=0;idx<global_index_value_pairs.size();idx++)
        {
            if(p_grid->GetLocalIndex(global_index_value_pairs[idx].first)>=0)
            {
                TS_ASSERT(map[p_grid->GetLocalIndex(global_index_value_pairs[idx].first)].size()==
                        global_index_value_pairs[idx].second);
            }
        }

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, false, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void TestPointSegmentMapGenerationRegularGridHex()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointSegmentMapGenerationRegularGridHex"));

        // Set up a grid
        Timer::Reset();
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 1000;
        dimensions[1] = 1000;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->SetSpacing(spacing);

        Timer::PrintAndReset("Grid set up");
        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> length(6000.0*unit::microns);
        units::quantity<unit::length> vessel_length(40.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(length, length, vessel_length);
        Timer::PrintAndReset("Network set up");
        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap();
        Timer::PrintAndReset("Map Calculated");

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, false, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void xTestPointPointMapGeneration()
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

    void xTestPointCellMapGeneration()
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
        std::vector<std::vector<CellPtr> > map = p_grid_calc->rGetCellMap();

        // Make sure all the cells are accounted for
        unsigned sum = 0;
        for (unsigned idx = 0; idx < map.size(); idx++)
        {
            sum += map[idx].size();
        }
        TS_ASSERT_EQUALS(sum, 100u);
    }

//    void TestInterpolateGridValuesWithVtk() throw (Exception)
//    {
//        // Set up a grid
//        RandomNumberGenerator::Instance()->Reseed(1000);
//        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
//        c_vector<unsigned, 3> dimensions;
//        dimensions[0] = 101;
//        dimensions[1] = 101;
//        dimensions[2] = 101;
//        p_grid->SetDimensions(dimensions);
//        double spacing = 0.33;
//        p_grid->SetSpacing(spacing* 1.e-6 * unit::metres);
//
//        // Set up a function increasing quadratically from bottom front left to top back right
//        std::vector<double> my_grid_func(dimensions[0] * dimensions[1] * dimensions[2]);
//        for (unsigned idx = 0; idx < dimensions[2]; idx++)
//        {
//            for (unsigned jdx = 0; jdx < dimensions[1]; jdx++)
//            {
//                for (unsigned kdx = 0; kdx < dimensions[0]; kdx++)
//                {
//                    double value = spacing * spacing * double(kdx * kdx + jdx * jdx + idx * idx);
//                    unsigned grid_index = kdx + jdx * dimensions[0] + idx * dimensions[0] * dimensions[1];
//                    my_grid_func[grid_index] = value;
//                }
//            }
//        }
//
//        // Set up some sample points
//        std::vector<DimensionalChastePoint<3> > points(100);
//        for (unsigned idx = 0; idx < 100; idx++)
//        {
//            DimensionalChastePoint<3> location(RandomNumberGenerator::Instance()->ranf() * 30.0,
//                                               RandomNumberGenerator::Instance()->ranf() * 30.0,
//                                               RandomNumberGenerator::Instance()->ranf() * 30.0);
//            points[idx] = location;
//        }
//
//        // Get the interpolated values
//        std::vector<double> interpolated_vals = p_grid->InterpolateGridValues(points, my_grid_func, true);
//
//        // Get the max error
//        double max_error = 0.0;
//        for (unsigned idx = 0; idx < interpolated_vals.size(); idx++)
//        {
//            double analytical = points[idx].GetLocation(1.e-6*unit::metres)[0] * points[idx].GetLocation(1.e-6*unit::metres)[0] +
//                    points[idx].GetLocation(1.e-6*unit::metres)[1] * points[idx].GetLocation(1.e-6*unit::metres)[1]
//                    + points[idx].GetLocation(1.e-6*unit::metres)[2] * points[idx].GetLocation(1.e-6*unit::metres)[2];
//            double error = std::abs((analytical - interpolated_vals[idx]) / analytical);
//            if (error > max_error)
//            {
//                max_error = error;
//            }
//            std::cout << "vals: " << analytical << "," << interpolated_vals[idx] << std::endl;
//
//        }
//        std::cout << "Max Error: " << max_error << std::endl;
//        TS_ASSERT(max_error < 0.1);
//    }
};

#endif /*TESTGRIDCALCULATOR_HPP_*/
