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

#ifndef TESTOWEN2011TUMOURSPHEROIDSIMULATIONS_HPP_
#define TESTOWEN2011TUMOURSPHEROIDSIMULATIONS_HPP_

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

#include "PetscSetupAndFinalize.hpp"

class TestOwen2011TumourSpheroidSimulations : public AbstractCellBasedWithTimingsTestSuite
{
public:

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
            boost::shared_ptr<VascularNetwork<3> > p_network = GetVesselNetwork();

            // Create the oxygen pde solver
            boost::shared_ptr<FiniteDifferenceSolver<3> > p_oxygen_solver = GetOxygenSolver(p_domain, p_network);

            // Create the vegf pde solver
            boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = GetVegfSolver(p_domain, p_network);

            // Create the angiogenesis solver
            boost::shared_ptr<MicrovesselSolver<3> > p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
            p_vascular_tumour_solver->SetVesselNetwork(p_network);
            p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
            p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);

            boost::shared_ptr<MicrovesselSimulationModifier<3> > p_simulation_modifier = boost::shared_ptr<MicrovesselSimulationModifier<3> >(new MicrovesselSimulationModifier<3>);
            p_simulation_modifier->SetMicrovesselSolver(p_vascular_tumour_solver);

            OffLatticeSimulation<3> simulator(cell_population);
            simulator.SetOutputDirectory("TestAngiogenesisSimulationModifier/NodeBased");
            simulator.SetDt(1.0);
            simulator.SetEndTime(10.0);
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

#endif /*TESTOWEN2011TUMOURSPHEROIDSIMULATIONS_HPP_*/
