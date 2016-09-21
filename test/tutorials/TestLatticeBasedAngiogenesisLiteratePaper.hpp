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

#ifndef TESTLATTICEBASEDANGIOGENESISLITERATEPAPER_HPP_
#define TESTLATTICEBASEDANGIOGENESISLITERATEPAPER_HPP_

/* = Introduction =
 * This tutorial is designed to introduce a lattice based angiogenesis problem similar to
 * vascular tumour models described in [http://epubs.siam.org/doi/abs/10.1137/040603760 Alarcon et al., 2005] and
 * [http://www.ncbi.nlm.nih.gov/pubmed/21363914 Owen et al. 2011]. It is a 2D simulation using cellular automaton
 * for cells, a regular grid for vessel movement and the same grid for the solution of partial differential equations
 * using the finite difference method.
 *
 * = The Test =
 * Start by introducing the necessary header files. The first contain functionality for setting up unit tests,
 * smart pointer tools and output management.
 */
#include <vector>
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
/*
 * Dimensional analysis.
 */
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
/*
 * Vessel networks.
 */
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
/*
 * Cells.
 */
#include "Owen11CellPopulationGenerator.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "CaBasedCellPopulation.hpp"
#include "OnLatticeSimulation.hpp"
#include "ApoptoticCellKiller.hpp"
/*
 * Flow.
 */
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
/*
 * Grids and PDEs.
 */
#include "RegularGrid.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "LinearSteadyStateDiffusionReactionPde.hpp"
/*
 * Angiogenesis.
 */
#include "Owen2011SproutingRule.hpp"
#include "Owen2011MigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
/*
 * Runs the full simulation.
 */
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
/*
 * This should appear last.
 */
#include "PetscSetupAndFinalize.hpp"
class TestLatticeBasedAngiogenesisLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void Test2dLatticeBased() throw (Exception)
    {
        /*
         * Set up output file management
         */
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestLatticeBasedAngiogenesisLiteratePaper"));
        /*
         * Set up the base units. These are microns and seconds.
         */
        units::quantity<unit::length> reference_length(1.0 * unit::microns);
        units::quantity<unit::time> reference_time(1.0 * unit::seconds);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        /*
         * Set up the lattice (grid), we will use the same dimensions as [http://www.ncbi.nlm.nih.gov/pubmed/21363914 Owen et al. 2011].
         * Note that we are using hard-coded parameters from that paper. We will see how to dump these to file for inspection later.
         * We annotate the parameter with "User" so we know we have instantiated it ourselves. Some solvers use parameter defaults
         * and will annotate the parameter with their own name.
         * We can write the lattice to file for quick visualization with Paraview.
         */
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        units::quantity<unit::length> grid_spacing = Owen11Parameters::mpLatticeSpacing->GetValue("User");
        p_grid->SetSpacing(grid_spacing);
        std::vector<unsigned> extents(3, 1);
        extents[0] = 51; // num x
        extents[1] = 51; // num_y
        p_grid->SetExtents(extents);
        /*
         * We can write the lattice to file for quick visualization with Paraview.
         *
         * [[Image(source:/chaste/projects/Microvessel/test/tutorials/images/Lattice_Tutorial_Initial_Grid.png, 20%, align=center, border=1)]]
         *
         */
        p_grid->Write(p_handler);
        /*
         * Next, set up the vessel network, this will initially consist of two, large counter-flowing vessels. Also set the inlet
         * and outlet pressures and flags and vessel (segment) radii.
         */
        boost::shared_ptr<VesselNode<2> > p_node11 = VesselNode<2>::Create(0.0, 400.0, 0.0, reference_length);
        boost::shared_ptr<VesselNode<2> > p_node12 = VesselNode<2>::Create(2000.0, 400.0, 0.0, reference_length);
        p_node11->GetFlowProperties()->SetIsInputNode(true);
        p_node11->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
        p_node12->GetFlowProperties()->SetIsOutputNode(true);
        p_node12->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
        boost::shared_ptr<VesselNode<2> > p_node13 = VesselNode<2>::Create(2000.0, 1600.0, 0.0, reference_length);
        boost::shared_ptr<VesselNode<2> > p_node14 = VesselNode<2>::Create(0.0, 1600.0, 0.0, reference_length);
        p_node13->GetFlowProperties()->SetIsInputNode(true);
        p_node13->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
        p_node14->GetFlowProperties()->SetIsOutputNode(true);
        p_node14->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
        boost::shared_ptr<Vessel<2> > p_vessel1 = Vessel<2>::Create(p_node11, p_node12);
        boost::shared_ptr<Vessel<2> > p_vessel2 = Vessel<2>::Create(p_node13, p_node14);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        units::quantity<unit::length> large_vessel_radius(25.0 * unit::microns);
        p_network->SetSegmentRadii(large_vessel_radius);
        /*
         * We can write the network to file for quick visualization with Paraview.
         *
         * [[Image(source:/chaste/projects/Microvessel/test/tutorials/images/Lattice_Angiogenesis_Tutorial_Grid_Vessels.png, 20%, align=center, border=1)]]
         *
         */
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "initial_network.vtp");
        /*
         * Next, set up the cell populations. We will setup up a population similar to that used in the Owen et al., 2011 paper. That is, a grid
         * filled with normal cells and a tumour spheroid in the middle. We can use a generator for this purpose. The generator simply sets up
         * the population using conventional Cell Based Chaste methods.
         */
        boost::shared_ptr<Owen11CellPopulationGenerator<2> > p_cell_population_genenerator = Owen11CellPopulationGenerator<2>::Create();
        p_cell_population_genenerator->SetRegularGrid(p_grid);
        p_cell_population_genenerator->SetVesselNetwork(p_network);
        units::quantity<unit::length> tumour_radius(200.0 * unit::microns);
        p_cell_population_genenerator->SetTumourRadius(tumour_radius);
        boost::shared_ptr<CaBasedCellPopulation<2> > p_cell_population = p_cell_population_genenerator->Update();
        /*
         * At this point the simulation domain will look as follows:
         *
         * [[Image(source:/chaste/projects/Microvessel/test/tutorials/images/Lattice_Based_Tutorial_Cell_Setup.png, 20%, align=center, border=1)]]
         *
         * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources
         */
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<2> > p_oxygen_pde = LinearSteadyStateDiffusionReactionPde<2>::Create();
        p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
        boost::shared_ptr<CellBasedDiscreteSource<2> > p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
        p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
        p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_ox_boundary = DiscreteContinuumBoundaryCondition<2>::Create();
        p_ox_boundary->SetType(BoundaryConditionType::VESSEL_LINE);
        p_ox_boundary->SetSource(BoundaryConditionSource::PRESCRIBED);
        units::quantity<unit::pressure> vessel_oxygen_partial_pressure(20.0*unit::mmHg);
        units::quantity<unit::concentration> vessel_oxygen_concentration =
                Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User") * vessel_oxygen_partial_pressure;
        p_ox_boundary->SetValue(vessel_oxygen_concentration);
        p_ox_boundary->SetNetwork(p_network);

        boost::shared_ptr<FiniteDifferenceSolver<2> > p_oxygen_solver = FiniteDifferenceSolver<2>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->AddBoundaryCondition(p_ox_boundary);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetGrid(p_grid);
         /*
         * The microvessel solver will manage all aspects of the vessel solve
         */
        boost::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFrequency(5);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        /*
         * The microvessel solution modifier will link the vessel and cell solvers
         */
        boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier = MicrovesselSimulationModifier<2>::Create();
        p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
        std::vector<std::string> update_labels;
        update_labels.push_back("oxygen");
        p_microvessel_modifier->SetCellDataUpdateLabels(update_labels);
        /*
         * The full simulation is run as a typical Cell Based Chaste simulation
         */
        OnLatticeSimulation<2> simulator(*p_cell_population);
        simulator.AddSimulationModifier(p_microvessel_modifier);
        /*
         * Add a killer to remove apoptotic cells
         */
        boost::shared_ptr<ApoptoticCellKiller<2> > p_apoptotic_cell_killer(new ApoptoticCellKiller<2>(p_cell_population.get()));
        simulator.AddCellKiller(p_apoptotic_cell_killer);
        /*
         * Another modifier for updating cell cycle quantities
         */
        boost::shared_ptr<Owen2011TrackingModifier<2> > p_owen11_tracking_modifier(new Owen2011TrackingModifier<2>);
        simulator.AddSimulationModifier(p_owen11_tracking_modifier);
        /*
         * Set up the remainder of the simulation
         */
        simulator.SetOutputDirectory("TestLatticeBasedAngiogenesisLiteratePaper");
        simulator.SetSamplingTimestepMultiple(5);
        simulator.SetDt(0.5);
        simulator.SetEndTime(200);
        /*
         * Do the solve
         */
        simulator.Solve();
        /*
         * Dump the parameters to file
         */
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    }
};

#endif /*TESTLATTICEBASEDANGIOGENESISLITERATEPAPER_HPP_*/
