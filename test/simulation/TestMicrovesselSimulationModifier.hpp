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

#ifndef TESTMICROVESSELSIMULATIONMODIFIER_HPP_
#define TESTMICROVESSELSIMULATIONMODIFIER_HPP_

#include <cxxtest/TestSuite.h>
#include "DiscreteContinuumLinearEllipticPde.hpp"
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
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
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
#include "CellBasedDiscreteSource.hpp"
#include "VesselNetworkPropertyManager.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestMicrovesselSimulationModifier : public AbstractCellBasedTestSuite
{

    std::shared_ptr<Part<3> > GetSimulationDomain()
    {
        QLength domain_x(800_um);
        QLength domain_y(800_um);
        QLength domain_z(200_um);
        auto p_domain = Part<3> ::Create();
        p_domain->AddCuboid(domain_x, domain_y, domain_z);
        return p_domain;
    }

    std::shared_ptr<VesselNetwork<3> > GetVesselNetwork()
    {
        auto p_network = VesselNetwork<3>::Create();
        std::vector<std::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<81; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10_um, 50_um, 100_um));
        }
        auto p_vessel_1 = Vessel<3>::Create(bottom_nodes);
        std::vector<std::shared_ptr<VesselNode<3> > > top_nodes;
        for(unsigned idx=0; idx<81; idx++)
        {
            top_nodes.push_back(VesselNode<3>::Create(double(idx)*10_um, 750_um, 100_um));
        }
        std::shared_ptr<Vessel<3> > p_vessel_2 = Vessel<3>::Create(top_nodes);

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
        VesselNetworkPropertyManager<3>::SetSegmentRadii(p_network, 10_um);
        std::vector<std::shared_ptr<VesselSegment<3> > > segments = p_network->GetVesselSegments();
        for(unsigned idx=0; idx<segments.size(); idx++)
        {
            segments[idx]->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        }
        return p_network;
    }

    std::shared_ptr<Part<3> > GetInitialTumourCellRegion()
    {
        QLength radius(100_um);
        QLength depth(200_um);
        auto p_domain = Part<3> ::Create();
        std::shared_ptr<Polygon<3> > circle = p_domain->AddCircle(radius, Vertex<3>(400_um, 400_um, 0_um));
        p_domain->Extrude(circle, depth);
        return p_domain;
    }

    std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > GetOxygenSolver(std::shared_ptr<Part<3> > p_domain,
                                                                  std::shared_ptr<VesselNetwork<3> > p_network)
    {
        auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        QDiffusivity oxygen_diffusivity(0.0033 * unit::metre_squared_per_second);
        p_oxygen_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);

        auto p_cell_oxygen_sink = CellBasedDiscreteSource<3>::Create();
        p_cell_oxygen_sink->SetConstantInUConsumptionRatePerCell(-1.e-9 * unit::mole_per_second);
        p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        auto p_vessel_ox_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        p_vessel_ox_boundary_condition->SetValue(40.0e-6 * unit::mole_per_metre_cubed);

        auto p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 50_um);

        auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<3>::Create();
        p_oxygen_solver->SetGrid(p_grid);
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->AddBoundaryCondition(p_vessel_ox_boundary_condition);
        p_oxygen_solver->SetLabel("oxygen");
        return p_oxygen_solver;
    }

    std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > GetVegfSolver(std::shared_ptr<Part<3> > p_domain,
                                                                  std::shared_ptr<VesselNetwork<3> > p_network)
    {
        auto p_vegf_pde = DiscreteContinuumLinearEllipticPde<3>::Create();
        QDiffusivity vegf_diffusivity(0.0033 * unit::metre_squared_per_second);

        p_vegf_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_vegf_pde->SetContinuumLinearInUTerm(-1.e-7*unit::per_second);

        auto p_cell_vegf_source = CellBasedDiscreteSource<3>::Create();
        p_cell_vegf_source->SetConstantInUConsumptionRatePerCell(1.e-6 * unit::mole_per_second);
        p_vegf_pde->AddDiscreteSource(p_cell_vegf_source);

        auto p_vessel_vegf_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        p_vessel_vegf_boundary_condition->SetValue(3.e-3_nM);

        auto p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 50_um);

        auto p_vegf_solver = SimpleLinearEllipticFiniteDifferenceSolver<3>::Create();
        p_vegf_solver->SetGrid(p_grid);
        p_vegf_solver->SetPde(p_vegf_pde);
        p_vegf_solver->AddBoundaryCondition(p_vessel_vegf_boundary_condition);
        p_vegf_solver->SetLabel("vegf");
        return p_vegf_solver;
    }

public:

    void TestCaBasedSpheroid()
    {
        // Create the simulation domain
        std::shared_ptr<Part<3> > p_domain = GetSimulationDomain();

        // Create a lattice for the cell population
        QLength spacing(40_um);
        QLength cell_lenth_scale(1_um);
        unsigned num_x = unsigned(p_domain->GetBoundingBox()[1]/spacing) + 1;
        unsigned num_y = unsigned(p_domain->GetBoundingBox()[3]/spacing) + 1;
        unsigned num_z = unsigned(p_domain->GetBoundingBox()[5]/spacing) + 1;
        PottsMeshGenerator<3> generator(num_x, 0, 0, num_y, 0, 0, num_z, 0, 0);
        PottsMesh<3>* p_mesh = generator.GetMesh();
        p_mesh->Scale(spacing/cell_lenth_scale, spacing/cell_lenth_scale, spacing/cell_lenth_scale);

        // Create a tumour cells in a cylinder in the middle of the domain
        std::shared_ptr<Part<3> > p_tumour_cell_region = GetInitialTumourCellRegion();
        std::vector<unsigned> location_indices = p_tumour_cell_region->GetContainingGridIndices(num_x, num_y, num_z, spacing);

        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasic(cells, location_indices.size());

        // Create cell population
        CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create the vessel network
        std::shared_ptr<VesselNetwork<3> > p_network = GetVesselNetwork();

        // Create the oxygen pde solver
        std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > p_oxygen_solver = GetOxygenSolver(p_domain, p_network);
        p_oxygen_solver->Setup();

        // Create the vegf pde solver
        std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > p_vegf_solver = GetVegfSolver(p_domain, p_network);
        p_vegf_solver->Setup();

        // Create the angiogenesis solver
        auto p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
        p_vascular_tumour_solver->SetVesselNetwork(p_network);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);

        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_simulation_modifier =
                boost::shared_ptr<MicrovesselSimulationModifier<3> >(new MicrovesselSimulationModifier<3>);
        p_simulation_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);

        std::vector<std::string> update_labels = std::vector<std::string>();
        update_labels.push_back("oxygen");
        p_simulation_modifier->SetCellDataUpdateLabels(update_labels);

        OnLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory("TestMicrovesselSimulationModifier/CaBased");
        simulator.SetDt(1.0);
        simulator.SetEndTime(4.0);
        simulator.AddSimulationModifier(p_simulation_modifier);
        simulator.Solve();
    }

<<<<<<< HEAD
    void TestNodeBasedSpheroid()
=======
    void TestNodeBasedSpheroid()
>>>>>>> 162b5879d69a3f7e0d629626fb53c4ff6bc0d5d3
    {
        // Create the domain
        std::shared_ptr<Part<3> > p_domain = GetSimulationDomain();

        // Create nodes corresponding to cell positions
        QLength spacing(40_um);
        QLength cell_lenth_scale(40_um);
        unsigned num_x = unsigned(p_domain->GetBoundingBox()[1]/spacing) + 1;
        unsigned num_y = unsigned(p_domain->GetBoundingBox()[3]/spacing) + 1;
        unsigned num_z = unsigned(p_domain->GetBoundingBox()[5]/spacing) + 1;

        // Create a tumour cells in a cylinder in the middle of the domain
        std::shared_ptr<Part<3> > p_tumour_cell_region = GetInitialTumourCellRegion();
        std::vector<unsigned> location_indices = p_tumour_cell_region->GetContainingGridIndices(num_x, num_y, num_z, spacing);
        auto p_grid = RegularGrid<3>::Create();

        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 1.5*num_x;
        dimensions[1] = 1.5*num_y;
        dimensions[2] = 1.5*num_z;
        p_grid->SetDimensions(dimensions);
        p_grid->SetSpacing(spacing);

        std::vector<Node<3>*> nodes;
        for(unsigned idx=0; idx<location_indices.size(); idx++)
        {
            Vertex<3> location = p_grid->GetPoint(location_indices[idx]);
            nodes.push_back(new Node<3>(idx, location.Convert(cell_lenth_scale), false));
        }

        NodesOnlyMesh<3> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, 1.5 * spacing/cell_lenth_scale);
        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasic(cells, mesh.GetNumNodes());
        NodeBasedCellPopulation<3> cell_population(mesh, cells);
        cell_population.SetAbsoluteMovementThreshold(2.0 * spacing/cell_lenth_scale);
        cell_population.AddCellWriter<CellLabelWriter>();

        // Create the vessel network
        std::shared_ptr<VesselNetwork<3> > p_network = GetVesselNetwork();

        // Create the oxygen pde solver
        std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > p_oxygen_solver = GetOxygenSolver(p_domain, p_network);
        p_oxygen_solver->Setup();

        // Create the vegf pde solver
        std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> > p_vegf_solver = GetVegfSolver(p_domain, p_network);
        p_vegf_solver->Setup();

        // Create the angiogenesis solver
        auto p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
        p_vascular_tumour_solver->SetVesselNetwork(p_network);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);

        boost::shared_ptr<MicrovesselSimulationModifier<3> > p_simulation_modifier = boost::shared_ptr<MicrovesselSimulationModifier<3> >(new MicrovesselSimulationModifier<3>);
        p_simulation_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);

        std::vector<std::string> update_labels = std::vector<std::string>();
        update_labels.push_back("oxygen");

        p_simulation_modifier->SetCellDataUpdateLabels(update_labels);

        OffLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory("TestMicrovesselSimulationModifier/NodeBased");
        simulator.SetDt(0.02);
        simulator.SetEndTime(0.08);
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

#endif /* TESTMICROVESSELSIMULATIONMODIFIER_HPP_ */
