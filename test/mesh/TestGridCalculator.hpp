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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkVoxel.h>
#include <vtkTetra.h>
#include <vtkWedge.h>
#include <vtkUnstructuredGrid.h>
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
#include "NodesOnlyMesh.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "GridCalculator.hpp"
#include "Timer.hpp"
#include "Node.hpp"

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
        p_grid->AddPointData(map_values, "Map Values");
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
        p_grid->AddPointData(map_values, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void TestPointSegmentMapGenerationRegularGridWithSurface()
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
        units::quantity<unit::length> length(100.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(length,
                                                                                                DimensionalChastePoint<2>(45.0, -0.1));
        units::quantity<unit::length> radius(11.0*unit::microns);
        p_network->SetSegmentRadii(radius);

        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap(true, true);

        std::vector<std::pair<unsigned, unsigned> > global_index_value_pairs;
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(3, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(4, 1));
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
        p_grid->AddPointData(map_values, "Map Values");
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
        dimensions[0] = 100;
        dimensions[1] = 100;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);
        units::quantity<unit::length> spacing(10.0*unit::microns);
        p_grid->SetSpacing(spacing);

        // Set up vessel network
        VesselNetworkGenerator<2> network_generator;
        units::quantity<unit::length> length(1000.0*unit::microns);
        units::quantity<unit::length> vessel_length(40.0*unit::microns);
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(length, length, vessel_length);

        // Get a point-segment map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetVesselNetwork(p_network);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > map = p_grid_calc->rGetSegmentMap();

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, "Map Values");
        p_grid->Write(p_handler);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "/network.vtp");
    }

    void TestPointPointMapGeneration()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointPointMapGeneration"));

        // Set up a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 10;
        dimensions[1] = 10;
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        // Set up points
        std::vector<DimensionalChastePoint<2> > points;
        points.push_back(DimensionalChastePoint<2>(5.0, 0.0));
        points.push_back(DimensionalChastePoint<2>(5.3, 0.0));
        points.push_back(DimensionalChastePoint<2>(5.3, 0.2));
        points.push_back(DimensionalChastePoint<2>(5.0, 5.0));
        points.push_back(DimensionalChastePoint<2>(0.0, 5.0));
        // Get a point-point map
        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);
        std::vector<std::vector<unsigned> > map = p_grid_calc->GetPointMap(points);

        std::vector<std::pair<unsigned, unsigned> > global_index_value_pairs;
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(4, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(5, 3));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(6, 0));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(50, 1));
        global_index_value_pairs.push_back(std::pair<unsigned, unsigned>(55, 1));
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
        p_grid->AddPointData(map_values, "Map Values");
        p_grid->Write(p_handler);
    }

    void TestPointCellMapGeneration()
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestGridCalculator/TestPointCellMapGeneration"));

        // Set up a grid
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 10;
        dimensions[1] = 10;
        dimensions[2] = 10;
        p_grid->SetDimensions(dimensions);
        p_grid->SetSpacing(1.0 * 1.e-6 * unit::metres);

        // Set up cells
        std::vector<Node<3>*> nodes;
        for(unsigned kdx=0;kdx<10;kdx++)
        {
            for(unsigned jdx=0;jdx<10;jdx++)
            {
                for(unsigned idx=0;idx<10;idx++)
                {
                    unsigned index = idx + jdx*10 +kdx*10*10;
                    nodes.push_back(new Node<3>(index,  false,  double(idx), double(jdx), double(kdx)));
                }
            }
        }
        NodesOnlyMesh<3> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, 1.5);
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);
        CellsGenerator<UniformCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_transit_type);

        NodeBasedCellPopulation<3> cell_population(mesh, cells);

        // Get a point-cell map
        boost::shared_ptr<GridCalculator<3> > p_grid_calc = GridCalculator<3>::Create();
        p_grid_calc->SetGrid(p_grid);
        p_grid_calc->SetCellPopulation(cell_population, 1.e-6 * unit::metres, BaseUnits::Instance()->GetReferenceConcentrationScale());
        std::vector<std::vector<CellPtr> > map = p_grid_calc->rGetCellMap();

        // Make sure all the cells are accounted for
        unsigned sum = 0;
        for (unsigned idx = 0; idx < map.size(); idx++)
        {
            sum += map[idx].size();
        }
        TS_ASSERT_EQUALS(sum, 1000u);

        // Write out the map
        std::vector<double> map_values;
        for(unsigned idx=0;idx<map.size();idx++)
        {
            map_values.push_back(map[idx].size());
        }
        p_grid->AddPointData(map_values, "Map Values");
        p_grid->Write(p_handler);

        for (unsigned i=0; i<nodes.size(); i++)
        {
            delete nodes[i];
        }
    }

    void TestVtkLocationMethods()
    {
        vtkSmartPointer<vtkPoints> p_tri_points = vtkSmartPointer<vtkPoints>::New();
        p_tri_points->InsertNextPoint(0.0, 0.0, -1.e-3);
        p_tri_points->InsertNextPoint(1.0, 0.0, -1.e-3);
        p_tri_points->InsertNextPoint(0.5, 1.0, -1.e-3);
        p_tri_points->InsertNextPoint(0.0, 0.0, 1.e-3);
        p_tri_points->InsertNextPoint(1.0, 0.0, 1.e-3);
        p_tri_points->InsertNextPoint(0.5, 1.0, 1.e-3);

        vtkSmartPointer<vtkWedge> p_triangle = vtkSmartPointer<vtkWedge>::New();
        p_triangle->GetPointIds()->SetId(0, 0);
        p_triangle->GetPointIds()->SetId(1, 1);
        p_triangle->GetPointIds()->SetId(2, 2);
        p_triangle->GetPointIds()->SetId(3, 3);
        p_triangle->GetPointIds()->SetId(4, 4);
        p_triangle->GetPointIds()->SetId(5, 5);

        vtkSmartPointer<vtkUnstructuredGrid> p_tri_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
        p_tri_grid->Allocate(1, 1);
        p_tri_grid->InsertNextCell(p_triangle->GetCellType(), p_triangle->GetPointIds());
        p_tri_grid->SetPoints(p_tri_points);

        double p1[3];
        p1[0] = 0.0;
        p1[1] = 0.5;
        p1[2] = 0.0;

        double p2[3];
        p2[0] = 2.0;
        p2[1] = 0.5;
        p2[2] = 0.0;

        double t, x[3], pcoords[3];
        int subId;
        int intercept = p_tri_grid->GetCell(0)->IntersectWithLine(p1, p2, 1.e-3, t, x, pcoords, subId);
        TS_ASSERT(intercept==1);
        TS_ASSERT_DELTA(x[0], 0.25, 1.e-6);
        TS_ASSERT_DELTA(x[1], 0.5, 1.e-6);

    }
};

#endif /*TESTGRIDCALCULATOR_HPP_*/
