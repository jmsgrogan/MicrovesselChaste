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

#ifndef TESTCABASEDCELLPOPULATIONWITHVESSELS_HPP
#define TESTCABASEDCELLPOPULATIONWITHVESSELS_HPP

#include <cxxtest/TestSuite.h>
#include <boost/lexical_cast.hpp>
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNetwork.hpp"
#include "CaBasedCellPopulation.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "DifferentiatedCellProliferativeType.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"
#include "UniformCellCycleModel.hpp"
#include "NodeLocationWriter.hpp"
#include "CellLabelWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "CellsGenerator.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "SimulationTime.hpp"
#include "PottsMesh.hpp"
#include "PottsMeshGenerator.hpp"
#include "CellPopulationMigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "RegularGrid.hpp"
#include "DefaultCellProliferativeType.hpp"

#include "FakePetscSetup.hpp"

class TestAngiogenesisWithCaPopulation : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestGrowSingleVessel() throw (Exception)
    {
        std::string output_directory = "TestAngiogenesisWithCaPopulation";
        MAKE_PTR_ARGS(OutputFileHandler, p_file_handler, (output_directory, false));


        // Set up the vessel grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        std::vector<unsigned> extents(3,1);
        extents[0] = 20;
        extents[1] = 20;
        p_grid->SetExtents(extents);
        p_grid->Write(p_file_handler);

        // Create the vessel network: single vessel in middle of domain
        VesselNetworkGenerator<2> network_generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSingleVessel(10*1.e-6*unit::metres, DimensionalChastePoint<2>(10.0, 0.0));

        // Write the initial network to file
        std::string output_filename = p_file_handler->GetOutputDirectoryFullPath().append("InitialVesselNetwork.vtp");

        DimensionalChastePoint<2> tip_position(10.0, 10.0);
        p_network->GetNearestNode(tip_position)->SetIsMigrating(true);
        p_network->Write(output_filename);

        // Set up the cell population
        PottsMeshGenerator<2> generator(20, 0, 0, 20, 0, 0);
        PottsMesh<2>* p_mesh = generator.GetMesh();
        std::vector<unsigned> location_indices;
        for (unsigned index = 0; index < p_mesh->GetNumNodes(); index++)
        {
            location_indices.push_back(index);
        }

        std::vector<CellPtr> cells;
        MAKE_PTR(DefaultCellProliferativeType, p_diff_type);
        MAKE_PTR(StalkCellMutationState, p_EC_state);
        MAKE_PTR(TipCellMutationState, p_EC_Tip_state);

        CellsGenerator<UniformCellCycleModel, 1> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumNodes(), p_diff_type);
        MAKE_PTR_ARGS(CaBasedCellPopulation<2>, p_cell_population, (*p_mesh, cells, location_indices));

        VesselNetworkCellPopulationInteractor<2> interactor = VesselNetworkCellPopulationInteractor<2>();
        interactor.SetVesselNetwork(p_network);
        interactor.PartitionNetworkOverCells(*p_cell_population);
        interactor.KillNonVesselOverlappingCells(*p_cell_population);
        interactor.LabelVesselsInCellPopulation(*p_cell_population, p_EC_Tip_state, p_EC_state);

        std::string output_filename2 = p_file_handler->GetOutputDirectoryFullPath().append("AssociatedVesselNetwork.vtp");
        p_network->Write(output_filename2);
        p_cell_population->AddCellWriter<CellLabelWriter>();
        p_cell_population->AddCellWriter<CellMutationStatesWriter>();
        p_cell_population->AddPopulationWriter<NodeLocationWriter>();

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(10.0, 10);
        AngiogenesisSolver<2> angiogenesis_solver;
        angiogenesis_solver.SetMigrationRule(CellPopulationMigrationRule<2>::Create());
        angiogenesis_solver.SetCellPopulation(p_cell_population);
        angiogenesis_solver.SetVesselNetwork(p_network);
        angiogenesis_solver.SetOutputFileHandler(p_file_handler);
        angiogenesis_solver.SetVesselGrid(p_grid);
        angiogenesis_solver.Run(true);
    }
};

#endif /*TESTCABASEDCELLPOPULATIONWITHVESSELS_HPP*/
