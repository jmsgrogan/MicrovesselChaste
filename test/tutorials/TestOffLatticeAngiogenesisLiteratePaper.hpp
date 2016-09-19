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

#ifndef TESOFFLATTICEANGIOGENESISLITERATEPAPER_HPP_
#define TESOFFLATTICEANGIOGENESISLITERATEPAPER_HPP_

/* = An Off Lattice Angiogenesis Tutorial =
 * This tutorial demonstrates most features of the Angiogenesis Project code. It can be used
 * to get a rough idea of how the code works, and then individual components can
 * be looked at in more detailed, dedicated tutorials.
 *
 * The following is covered:
 * * Defining domains using geometry primitives
 * * Setting up vessel networks and cell populations
 * * Setting up PDEs and flow problems
 * * Setting up an angiogenesis solver
 * * Interacting with Cell Based Chaste
 *
 * = The Test =
 * Start by introducing the necessary header files. The first contain functionality for setting up unit tests,
 * smart pointer tools and output management.
 */
#include <cxxtest/TestSuite.h>
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "FileFinder.hpp"
#include "RandomNumberGenerator.hpp"
/*
 * Dimensional analysis.
 */
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
/*
 * Geometry tools.
 */
#include "MappableGridGenerator.hpp"
#include "Part.hpp"
/*
 * Vessel networks.
 */
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
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
#include "DiscreteContinuumMesh.hpp"
#include "VtkMeshWriter.hpp"
#include "FiniteElementSolver.hpp"
#include "DiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
/*
 * Angiogenesis
 */
#include "OffLatticeSproutingRule.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
/*
 * Runs the full simulation
 */
#include "MicrovesselSolver.hpp"
/*
 * This should appear last.
 */
#include "PetscSetupAndFinalize.hpp"
class TestOffLatticeAngiogenesisLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:
    /*
     * = Test 1 - Angiogenesis With No Cells =
     * In the first example angiogenesis is simulated without interaction with a cell population.
     */
    void TestVesselsOnly() throw(Exception)
    {
        /*
         * We will work in microns
         */
        units::quantity<unit::length> reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        /*
         * Create  a cylindrical domain, outside of which no vessels can move. Write it to
         * file for visualization.
         */
        units::quantity<unit::length> domain_radius(0.005 * unit::metres);
        units::quantity<unit::length> domain_height(0.001 * unit::metres);
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCylinder(domain_radius, domain_height, DimensionalChastePoint<3>(domain_radius/reference_length,
                                                                                      domain_radius/reference_length, 0.0));
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestOffLatticeAngiogenesisLiteratePaper/TestVesselsOnly"));
        p_domain->Write(p_handler->GetOutputDirectoryFullPath()+"domain.vtp");
        /*
         * Set up a vessel network, which will span the base of the domain.
         */
        units::quantity<unit::length> target_width = 2.2*domain_radius;
        units::quantity<unit::length> target_height = 2.2*domain_radius;
        units::quantity<unit::length> vessel_length(300.0*unit::microns);
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);
        DimensionalChastePoint<3> translation_vector(0.0, 0.0, 100.0, reference_length);
        p_network->Translate(translation_vector);
        /*
         * Remove any vessel with both nodes outside the domain.
         */
        p_domain->BooleanWithNetwork(p_network);
        /*
         * Set a random selection of nodes as inlets and outlets
         */
        RandomNumberGenerator::Instance()->Reseed(1101001);
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes = p_network->GetNodes();
        units::quantity<unit::pressure> inlet_pressure = Owen11Parameters::mpInletPressure->GetValue();
        units::quantity<unit::pressure> outlet_pressure = Owen11Parameters::mpOutletPressure->GetValue();
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            if(nodes[idx]->GetNumberOfSegments()==1)
            {
                unsigned rand_num = RandomNumberGenerator::Instance()->randMod(10);
                if(rand_num==0)
                {
                    nodes[idx]->GetFlowProperties()->SetIsInputNode(true);
                    nodes[idx]->GetFlowProperties()->SetPressure(inlet_pressure);
                }
                else if(rand_num<=1)
                {
                    nodes[idx]->GetFlowProperties()->SetIsOutputNode(true);
                    nodes[idx]->GetFlowProperties()->SetPressure(outlet_pressure);
                }
            }
        }
        /*
         * Use the structural adaptation solver to iterate until the flow reaches steady state
         */
        units::quantity<unit::length> vessel_radius(40.0*unit::microns);
        p_network->SetSegmentRadii(vessel_radius);
        units::quantity<unit::dynamic_viscosity> viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"initial_network.vtp");

        BaseUnits::Instance()->SetReferenceTimeScale(60.0*unit::seconds);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();

        StructuralAdaptationSolver<3> structural_adaptation_solver;
        structural_adaptation_solver.SetVesselNetwork(p_network);
        structural_adaptation_solver.SetWriteOutput(true);
        structural_adaptation_solver.SetOutputFileName(p_handler->GetOutputDirectoryFullPath()+"adaptation_data.dat");
        structural_adaptation_solver.SetTolerance(0.0001);
        structural_adaptation_solver.SetMaxIterations(10);
        structural_adaptation_solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_haematocrit_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_wss_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        structural_adaptation_solver.SetTimeIncrement(0.01 * unit::seconds);
        structural_adaptation_solver.Solve();
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"network_initial_sa.vtp");
        /*
         * Vessels Release Oxygen, Depending on the amount of haematocrit. Continuum Cells consume oxygen.
         * Set up the oxygen field.
         */
        boost::shared_ptr<DiscreteContinuumMesh<3> > p_mesh = DiscreteContinuumMesh<3>::Create();
        p_mesh->SetDomain(p_domain);
        p_mesh->SetMaxElementArea(1.e12);
        p_mesh->Update();
        /*
         * Set up the oxygen pde
         */
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_oxygen_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> oxygen_diffusivity(1.e-6*unit::metre_squared_per_second);
        p_oxygen_pde->SetIsotropicDiffusionConstant(oxygen_diffusivity);
        /*
         * Add continuum sink term for cells
         */
        units::quantity<unit::rate> oxygen_consumption_rate(1.e-6*unit::per_second);
        p_oxygen_pde->SetContinuumLinearInUTerm(oxygen_consumption_rate);
        /*
         * Add discrete source terms for vessels
         */
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_oxygen_source = VesselBasedDiscreteSource<3>::Create();
        p_vessel_oxygen_source->SetVesselPermeability(1.0 * unit::metre_per_second);
        p_vessel_oxygen_source->SetOxygenConcentrationPerUnitHaematocrit(1.0*unit::mole_per_metre_cubed);
        p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);
        /*
         * Set up the vegf pde
         */
        boost::shared_ptr<LinearSteadyStateDiffusionReactionPde<3> > p_vegf_pde = LinearSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(1.e-6*unit::metre_squared_per_second);
        p_vegf_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        units::quantity<unit::rate> vegf_decay_rate(1.e-6*unit::per_second);
        p_vegf_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        /*
         * Add a continuum source term with rate dependent on the oxygen concentration
         */
        boost::shared_ptr<DiscreteSource<3> > p_oxygen_dependent_vegf_source = DiscreteSource<3>::Create();
        p_oxygen_dependent_vegf_source->SetType(SourceType::SOLUTION);
        p_oxygen_dependent_vegf_source->SetLinearInUSinkRatePerSolutionQuantity(-1.0*unit::metre_cubed_per_mole_per_second);
        p_vegf_pde->AddDiscreteSource(p_oxygen_dependent_vegf_source);
        /*
         * Add a perfect sink at the bottom of the domain by means of a zero Dirichlet boundary condition. We can apply this
         * directly on the domain geometry using a Facet locator to manually label the boundary. We then set the boundary condition
         * using this label.
         */
        p_domain->GetFacet(DimensionalChastePoint<3>(domain_radius/reference_length,
                                                     domain_radius/reference_length, 0.0))->SetLabel("vegf_boundary");
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_vegf_perfect_sink = DiscreteContinuumBoundaryCondition<3>::Create();
        p_vegf_perfect_sink->SetType(BoundaryConditionType::FACET);
        p_vegf_perfect_sink->SetDomain(p_domain);
        p_vegf_perfect_sink->SetValue(0.0*unit::mole_per_metre_cubed);
        p_vegf_perfect_sink->SetLabelName("vegf_boundary");
        /*
         * Set up the PDE solvers for the oxygen and vegf problems
         */
        boost::shared_ptr<FiniteElementSolver<3> > p_oxygen_solver = FiniteElementSolver<3>::Create();
        p_oxygen_solver->SetPde(p_oxygen_pde);
        p_oxygen_solver->SetLabel("oxygen");
        p_oxygen_solver->SetMesh(p_mesh);
        boost::shared_ptr<FiniteElementSolver<3> > p_vegf_solver = FiniteElementSolver<3>::Create();
        p_vegf_solver->SetPde(p_vegf_pde);
        p_vegf_solver->SetLabel("vegf");
        p_vegf_solver->SetMesh(p_mesh);
        p_vegf_solver->AddBoundaryCondition(p_vegf_perfect_sink);
        /*
         * Set up the `AngiogenesisSolver`
         */
//        boost::shared_ptr<AngiogenesisSolver<3> > p_angiogenesis_solver = AngiogenesisSolver<3>::Create();
//        p_angiogenesis_solver->SetVesselNetwork(p_network);
//        p_angiogenesis_solver->a
        /*
         * Set up the `MicrovesselSolver` which coordinates all solves. Note that for sequentially
         * coupled PDE solves, the solution propagates in the order that the PDE solvers are added to the `MicrovesselSolver`.
         */
        boost::shared_ptr<MicrovesselSolver<3> > p_vascular_tumour_solver = MicrovesselSolver<3>::Create();
        p_vascular_tumour_solver->SetVesselNetwork(p_network);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_vascular_tumour_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        p_vascular_tumour_solver->SetOutputFileHandler(p_handler);
        p_vascular_tumour_solver->SetOutputFrequency(1);
        p_vascular_tumour_solver->SetDiscreteContinuumSolversHaveCompatibleGridIndexing(true);
        /*
         * Reset the simulation time and run the solver.
         */
        SimulationTime::Instance()->Destroy();
        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30.0, 1);
        p_vascular_tumour_solver->Run();
    }
};

#endif /*TESTMICROPOCKET_HPP_*/
