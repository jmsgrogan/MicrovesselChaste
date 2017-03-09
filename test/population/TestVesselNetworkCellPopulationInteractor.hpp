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



#ifndef TESTVESSELNETWORKCELLPOPULATIONINTERACTOR_HPP
#define TESTVESSELNETWORKCELLPOPULATIONINTERACTOR_HPP

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
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "CellLabelWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "NodeLocationWriter.hpp"
#include "CellsGenerator.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "MacrophageMutationState.hpp"
#include "SimulationTime.hpp"
#include "PottsMesh.hpp"
#include "PottsMeshGenerator.hpp"
#include "ConstBoundaryCondition.hpp"
#include "DefaultCellProliferativeType.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"

#include "FakePetscSetup.hpp"

class TestVesselNetworkCellPopulationInteractor : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestSetUpLatticeBasedVessel() throw (Exception)
    {
        // Create the mesh
        PottsMeshGenerator<3> generator(20, 0, 0, 20, 0, 0, 21, 0, 0);
        PottsMesh<3>* p_mesh = generator.GetMesh();

        // Create the vessel network: single vessel in middle of domain
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateSingleVessel(20*1.e-6*unit::metres,
                                                                                                DimensionalChastePoint<3>(10.0, 10.0, 0.0));

        // Write the initial network to file
        std::string output_directory = "TestVesselNetworkCellPopulationInteractor";
        OutputFileHandler output_file_handler(output_directory, false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InitialVesselNetwork.vtp");
        p_network->Write(output_filename);

        // Create cell population
        // use OnLatticeVascularTumourCellPopulationGenerator to generate cells and associate cells
        // with vessels

        // create endothelial cell population
        std::vector<unsigned> location_indices;
        for (unsigned index=0; index < p_mesh->GetNumNodes(); index++)
        {
            location_indices.push_back(index);
        }

        std::vector<CellPtr> cells;
        MAKE_PTR(DefaultCellProliferativeType, p_diff_type);
        MAKE_PTR(StalkCellMutationState, p_EC_state);
        CellsGenerator<Owen2011OxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumNodes(), p_diff_type);
        CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);

        VesselNetworkCellPopulationInteractor<3> interactor = VesselNetworkCellPopulationInteractor<3>();
        interactor.SetVesselNetwork(p_network);
        interactor.PartitionNetworkOverCells(cell_population, 1.e-6*unit::metres);
        interactor.LabelVesselsInCellPopulation(cell_population, 1.e-6*unit::metres, p_EC_state, p_EC_state);

        TS_ASSERT_EQUALS(p_network->GetNumberOfNodes(), 21u);
        TS_ASSERT_EQUALS(p_network->GetNumberOfVessels(), 1u);
        TS_ASSERT_EQUALS(p_network->GetNumberOfVesselNodes(), 2u);

        std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("AssociatedVesselNetwork.vtp");
        p_network->Write(output_filename2);

        // VTK writing needs a simulation time
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
        cell_population.AddCellWriter<CellLabelWriter>();
        cell_population.AddCellWriter<CellMutationStatesWriter>();
        cell_population.AddPopulationWriter<NodeLocationWriter>();
        cell_population.OpenWritersFiles(output_file_handler);
        cell_population.WriteResultsToFiles(output_directory);
        cell_population.CloseWritersFiles();
    }
};

#endif /*TESTVESSELNETWORKCELLPOPULATIONINTERACTOR_HPP*/
