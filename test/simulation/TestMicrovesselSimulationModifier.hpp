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

#ifndef TESTSPHEROIDWITHANGIOGENESIS_HPP_
#define TESTSPHEROIDWITHANGIOGENESIS_HPP_

#include <cxxtest/TestSuite.h>
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "PottsMeshGenerator.hpp"
#include "Cell.hpp"
#include "CellsGenerator.hpp"
#include "FixedG1GenerationalCellCycleModel.hpp"
#include "CaBasedCellPopulation.hpp"
#include "CellMutationStatesCountWriter.hpp"
#include "CellProliferativeTypesCountWriter.hpp"
#include "CellProliferativePhasesCountWriter.hpp"
#include "CellProliferativePhasesWriter.hpp"
#include "CellAncestorWriter.hpp"
#include "CellAgesWriter.hpp"
#include "CellVolumesWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "OnLatticeSimulation.hpp"
#include "Node.hpp"
#include "Polygon.hpp"
#include "NodesOnlyMesh.hpp"
#include "OffLatticeSimulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "FakePetscSetup.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "VesselNetworkGenerator.hpp"
#include "SimpleOxygenBasedCellCycleModel.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteSource.hpp"
#include "CellLabelWriter.hpp"
#include "Vessel.hpp"
#include "VesselNode.hpp"
#include "GeometryTools.hpp"
#include "RandomNumberGenerator.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "RegularGrid.hpp"
#include "UnitCollection.hpp"

class TestSpheroidWithAngiogenesis : public AbstractCellBasedTestSuite
{

    boost::shared_ptr<Part<3> > GetSimulationDomain()
    {
        double domain_x = 800.0;
        double domain_y = 800.0;
        double domain_z = 200.0;
        boost::shared_ptr<Part<3> > p_domain = Part<3> ::Create();
        p_domain->AddCuboid(domain_x*1.e-6*unit::metres,
                            domain_y*1.e-6*unit::metres,
                            domain_z*1.e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0));
        return p_domain;
    }

    boost::shared_ptr<VesselNetwork<3> > GetVesselNetwork()
    {
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        std::vector<boost::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<81; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 50.0, 100.0));
        }
        boost::shared_ptr<Vessel<3> > p_vessel_1 = Vessel<3>::Create(bottom_nodes);
        std::vector<boost::shared_ptr<VesselNode<3> > > top_nodes;
        for(unsigned idx=0; idx<81; idx++)
        {
            top_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 750.0, 100.0));
        }
        boost::shared_ptr<Vessel<3> > p_vessel_2 = Vessel<3>::Create(top_nodes);

        // Set up flow properties
        p_network->AddVessel(p_vessel_1);
        p_network->AddVessel(p_vessel_2);
        p_network->GetVessels()[0]->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_network->GetVessels()[0]->GetStartNode()->GetFlowProperties()->SetPressure(3000.0 * unit::pascals);
        p_network->GetVessels()[0]->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_network->GetVessels()[0]->GetEndNode()->GetFlowProperties()->SetPressure(1000.0 * unit::pascals);

        p_network->GetVessels()[1]->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_network->GetVessels()[1]->GetStartNode()->GetFlowProperties()->SetPressure(3000.0 * unit::pascals);
        p_network->GetVessels()[1]->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_network->GetVessels()[1]->GetEndNode()->GetFlowProperties()->SetPressure(1000.0 * unit::pascals);

        p_network->UpdateSegments();
        p_network->SetSegmentRadii(10.0 * 1.e-6 * unit::metres);
        std::vector<boost::shared_ptr<VesselSegment<3> > > segments = p_network->GetVesselSegments();
        for(unsigned idx=0; idx<segments.size(); idx++)
        {
            segments[idx]->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        }
        return p_network;
    }

    boost::shared_ptr<Part<3> > GetInitialTumourCellRegion()
    {
        double radius = 100.0;
        double depth = 200.0;
        boost::shared_ptr<Part<3> > p_domain = Part<3> ::Create();
        boost::shared_ptr<Polygon> circle = p_domain->AddCircle(radius*1.e-6*unit::metres, DimensionalChastePoint<3>(400.0, 400.0, 0.0));
        p_domain->Extrude(circle, depth*1.e-6*unit::metres);
        return p_domain;
    }

    boost::shared_ptr<FiniteDifferenceSolver<3> > GetOxygenSolver(boost::shared_ptr<Part<3> > p_domain,
                                                                  boost::shared_ptr<VesselNetwork<3> > p_network)
    {
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_oxygen_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> oxygen_diffusivity(0.0033 * unit::metre_squared_per_second);
        p_oxygen_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);

        boost::shared_ptr<DiscreteSource<3> > p_cell_oxygen_sink = DiscreteSource<3>::Create();
//        p_cell_oxygen_sink->SetType(SourceType::CELL);
//        p_cell_oxygen_sink->SetSource(SourceStrength::PRESCRIBED);
//        p_cell_oxygen_sink->SetValue(1.e-6 * unit::);
        p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_vessel_ox_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        p_vessel_ox_boundary_condition->SetValue(40.0 * unit::mole_per_metre_cubed);
        p_vessel_ox_boundary_condition->SetType(BoundaryConditionType::VESSEL_LINE);
        p_vessel_ox_boundary_condition->SetSource(BoundaryConditionSource::PRESCRIBED);
        p_vessel_ox_boundary_condition->SetNetwork(p_network);

        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 40.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = FiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetGrid(p_grid);
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->AddBoundaryCondition(p_vessel_ox_boundary_condition);
        return p_oxygen_solver;
    }

    boost::shared_ptr<FiniteDifferenceSolver<3> > GetVegfSolver(boost::shared_ptr<Part<3> > p_domain,
                                                                  boost::shared_ptr<VesselNetwork<3> > p_network)
    {
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_vegf_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> oxygen_diffusivity(0.0033 * unit::metre_squared_per_second);

        p_vegf_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);
        p_vegf_pde->SetContinuumLinearInUTerm(-1.e-7*unit::per_second);

        boost::shared_ptr<DiscreteSource<3> > p_cell_vegf_source = DiscreteSource<3>::Create();
//        p_cell_vegf_source->SetType(SourceType::CELL);
//        p_cell_vegf_source->SetSource(SourceStrength::PRESCRIBED);
//        p_cell_vegf_source->SetValue(-1.e-4);
        p_vegf_pde->AddDiscreteSource(p_cell_vegf_source);

        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 40.0 * 1.e-6 * unit::metres);

        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = FiniteDifferenceSolver<3>::Create();
        p_vegf_solver->SetGrid(p_grid);
        p_vegf_solver->SetPde(p_vegf_pde);
        return p_vegf_solver;
    }

public:

    void TestCaBasedSpheroid() throw (Exception)
    {
        // Create the simulation domain
        boost::shared_ptr<Part<3> > p_domain = GetSimulationDomain();

        // Create a lattice for the cell population
        double spacing = 40.0;
        unsigned num_x = unsigned(p_domain->GetBoundingBox()[1]/spacing) + 1;
        unsigned num_y = unsigned(p_domain->GetBoundingBox()[3]/spacing) + 1;
        unsigned num_z = unsigned(p_domain->GetBoundingBox()[5]/spacing) + 1;
        PottsMeshGenerator<3> generator(num_x, 0, 0, num_y, 0, 0, num_z, 0, 0);
        PottsMesh<3>* p_mesh = generator.GetMesh();
        p_mesh->Scale(spacing, spacing, spacing);

        // Create a tumour cells in a cylinder in the middle of the domain
        boost::shared_ptr<Part<3> > p_tumour_cell_region = GetInitialTumourCellRegion();
        std::vector<unsigned> location_indices = p_tumour_cell_region->GetContainingGridIndices(num_x, num_y, num_z, spacing);

        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasic(cells, location_indices.size());

        // Create cell population
        CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create the vessel network
        boost::shared_ptr<VesselNetwork<3> > p_network = GetVesselNetwork();

        // Create the oxygen pde solver
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = GetOxygenSolver(p_domain, p_network);
        p_oxygen_solver->GetGrid()->SetVesselNetwork(p_network);
        p_oxygen_solver->GetGrid()->SetCellPopulation(cell_population);
        p_oxygen_solver->Setup();

        // Create the vegf pde solver
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = GetVegfSolver(p_domain, p_network);
        p_vegf_solver->GetGrid()->SetVesselNetwork(p_network);
        p_vegf_solver->GetGrid()->SetCellPopulation(cell_population);
        p_vegf_solver->Setup();

        // Create the angiogenesis solver
        boost::shared_ptr<MicrovesselSolver<3> > p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
        p_vascular_tumour_solver->SetVesselNetwork(p_network);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);

        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_simulation_modifier = boost::shared_ptr<MicrovesselSimulationModifier<3> >(new MicrovesselSimulationModifier<3>);
        p_simulation_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);

        std::vector<std::string> update_labels = std::vector<std::string>();
        update_labels.push_back("oxygen");
        p_simulation_modifier->SetCellDataUpdateLabels(update_labels);

        OnLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory("TestAngiogenesisSimulationModifier/CaBased");
        simulator.SetDt(1.0);
        simulator.SetEndTime(4.0);
        simulator.AddSimulationModifier(p_simulation_modifier);
        simulator.Solve();
    }

    void TestNodeBasedSpheroid() throw (Exception)
    {
        // Create the domain
        boost::shared_ptr<Part<3> > p_domain = GetSimulationDomain();

        // Create nodes corresponding to cell positions
        double spacing = 40.0;
        unsigned num_x = unsigned(p_domain->GetBoundingBox()[1]/spacing) + 1;
        unsigned num_y = unsigned(p_domain->GetBoundingBox()[3]/spacing) + 1;
        unsigned num_z = unsigned(p_domain->GetBoundingBox()[5]/spacing) + 1;

        // Create a tumour cells in a cylinder in the middle of the domain
        boost::shared_ptr<Part<3> > p_tumour_cell_region = GetInitialTumourCellRegion();
        std::vector<unsigned> location_indices = p_tumour_cell_region->GetContainingGridIndices(num_x, num_y, num_z, spacing);

        std::vector<Node<3>*> nodes;
        for(unsigned idx=0; idx<location_indices.size(); idx++)
        {
            c_vector<double, 3> location = Grid::GetLocationOf1dIndex(location_indices[idx], num_x, num_y, spacing);
            nodes.push_back(new Node<3>(idx, location, false));
        }

        NodesOnlyMesh<3> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, 1.5 * spacing);
        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasic(cells, mesh.GetNumNodes());
        NodeBasedCellPopulation<3> cell_population(mesh, cells);
        cell_population.SetAbsoluteMovementThreshold(2.0 * spacing);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create the vessel network
        boost::shared_ptr<VesselNetwork<3> > p_network = GetVesselNetwork();

        // Create the oxygen pde solver
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = GetOxygenSolver(p_domain, p_network);
        p_oxygen_solver->GetGrid()->SetVesselNetwork(p_network);
        p_oxygen_solver->GetGrid()->SetCellPopulation(cell_population);
        p_oxygen_solver->Setup();

        // Create the vegf pde solver
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = GetVegfSolver(p_domain, p_network);
        p_vegf_solver->GetGrid()->SetVesselNetwork(p_network);
        p_vegf_solver->GetGrid()->SetCellPopulation(cell_population);
        p_vegf_solver->Setup();

        // Create the angiogenesis solver
        boost::shared_ptr<MicrovesselSolver<3> > p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
        p_vascular_tumour_solver->SetVesselNetwork(p_network);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);

        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_simulation_modifier = boost::shared_ptr<MicrovesselSimulationModifier<3> >(new MicrovesselSimulationModifier<3>);
        p_simulation_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);

        std::vector<std::string> update_labels = std::vector<std::string>();
        update_labels.push_back("oxygen");

        p_simulation_modifier->SetCellDataUpdateLabels(update_labels);

        OffLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory("TestAngiogenesisSimulationModifier/NodeBased");
        simulator.SetDt(1.0);
        simulator.SetEndTime(4.0);
        simulator.AddSimulationModifier(p_simulation_modifier);

        MAKE_PTR(GeneralisedLinearSpringForce<3>, p_force);
        simulator.AddForce(p_force);
        simulator.Solve();

        // Tidy up
        for (unsigned i=0; i<nodes.size(); i++)
        {
            delete nodes[i];
        }
    }
};

#endif /* TESTSPHEROIDWITHANGIOGENESIS_HPP_ */