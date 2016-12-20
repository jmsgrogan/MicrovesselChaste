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

#ifndef TESTTIPSENSINGPAPER_HPP_
#define TESTTIPSENSINGPAPER_HPP_


#include <vector>

#include "TipAttractionLatticeBasedMigrationRule.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "RandomNumberGenerator.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "MetabolicStimulusCalculator.hpp"
#include "ShrinkingStimulusCalculator.hpp"
#include "ViscosityCalculator.hpp"
#include "RegularGrid.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "Owen2011SproutingRule.hpp"
#include "Owen2011MigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "OnLatticeSimulation.hpp"
#include "MicrovesselVtkScene.hpp"
#include "VtkSceneMicrovesselModifier.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestTipSensingPaper : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void Test2dLatticeBased() throw (Exception)
    {
        /*
         * Set up output file management and seed the random number generator.
         */
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestTipSensingPaper2d_b"));
        RandomNumberGenerator::Instance()->Reseed(12345);

        /*
         * Set up units
         */
        units::quantity<unit::length> reference_length(1.0 * unit::microns);
        units::quantity<unit::time> reference_time(1.0* unit::hours);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);

        /*
         * Set up the lattice (grid)
         */
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2.0e-3*unit::metres, 1.24e-3*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 20.0e-6*unit::metres);

        /*
         * Set up the vessel network
         */
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        for (unsigned idx=0; idx<100; idx++)
        {
            nodes.push_back(VesselNode<2>::Create(float(idx)*20.0, 200.0, 0.0, reference_length));
        }

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
        nodes[nodes.size()-1]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[nodes.size()-1]->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
        boost::shared_ptr<Vessel<2> > p_vessel1 = Vessel<2>::Create(nodes);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);

        /*
         * Write to file
         */
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "initial_network.vtp");

        /*
         * Next set up the PDE for VEGF.
         */
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_vegf_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_vegf_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_vegf_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_vegf_pde->SetMultiplierValue(initial_vegf_concentration);

        /*
        * Vessels bind to vegf
        */
        boost::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_vegf_sink = VesselBasedDiscreteSource<2>::Create();
        units::quantity<unit::concentration> vegf_blood_concentration(0.0*unit::mole_per_metre_cubed);
        p_vessel_vegf_sink->SetReferenceConcentration(vegf_blood_concentration);
        p_vessel_vegf_sink->SetVesselPermeability((3.e-4/3600.0)*unit::metre_per_second);
        p_vessel_vegf_sink->SetReferenceHaematocrit(0.45);
        p_vessel_vegf_sink->SetUptakeRatePerCell(-(4.e-22/3600.0)*unit::mole_per_second);
        p_vegf_pde->AddDiscreteSource(p_vessel_vegf_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */
        boost::shared_ptr<FiniteDifferenceSolver<2> > p_vegf_solver = FiniteDifferenceSolver<2>::Create();
        p_vegf_solver->SetParabolicPde(p_vegf_pde);
        p_vegf_solver->SetLabel("vegf");
        p_vegf_solver->SetGrid(p_grid);

        /*
         * Next set up the flow problem. Assign a blood plasma viscosity to the vessels. The actual viscosity will
         * depend on haematocrit and diameter. This solver manages growth and shrinkage of vessels in response to
         * flow related stimuli.
         */
        units::quantity<unit::length> max_vessel_radius(5.0 * unit::microns);
        p_network->SetSegmentRadii(max_vessel_radius);
        units::quantity<unit::dynamic_viscosity> viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue("User");
        p_network->SetSegmentViscosity(viscosity);
        /*
        * Set up the pre- and post flow calculators.
        */
        boost::shared_ptr<VesselImpedanceCalculator<2> > p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<2> > p_haematocrit_calculator = ConstantHaematocritSolver<2>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        /*
        * Set up and configure the structural adaptation solver.
        */
        boost::shared_ptr<StructuralAdaptationSolver<2> > p_structural_adaptation_solver = StructuralAdaptationSolver<2>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(2);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);

        /*
         * Set up an angiogenesis solver and add sprouting and migration rules.
         */
        boost::shared_ptr<AngiogenesisSolver<2> > p_angiogenesis_solver = AngiogenesisSolver<2>::Create();
        boost::shared_ptr<TipAttractionLatticeBasedMigrationRule<2> > p_migration_rule = TipAttractionLatticeBasedMigrationRule<2>::Create();
        boost::shared_ptr<Owen2011SproutingRule<2> > p_sprouting_rule = Owen2011SproutingRule<2>::Create();
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_sprouting_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_sprouting_rule->SetSproutingProbability(1000.0*unit::per_second);
        p_migration_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_migration_rule->SetUseMooreNeighbourhood(true);
        p_migration_rule->SetUseTipAttraction(true);
        p_angiogenesis_solver->SetVesselGrid(p_grid);
        p_angiogenesis_solver->SetVesselNetwork(p_network);

         /*
         * The microvessel solver will manage all aspects of the vessel solve.
         */
        boost::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFrequency(1);
        p_microvessel_solver->SetOutputFileHandler(p_handler);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        p_microvessel_solver->SetAngiogenesisSolver(p_angiogenesis_solver);

        /*
         * Set the simulation time and run the solver. The result is shown at the top of the tutorial.
         */
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(10.0, 100.0);
        p_microvessel_solver->Run();
        /*
         * Dump the parameters to file for inspection.
         */
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    }

    void xTestLatticeBased() throw (Exception)
    {
        /*
         * Set up output file management and seed the random number generator.
         */
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestTipSensingPaper"));
        RandomNumberGenerator::Instance()->Reseed(12345);

        /*
         * Set up units
         */
        units::quantity<unit::length> reference_length(1.0 * unit::microns);
        units::quantity<unit::time> reference_time(1.0* unit::hours);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);

        /*
         * Set up the lattice (grid)
         */
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(2.0e-3*unit::metres, 1.24e-3*unit::metres, 100.0e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 20.0e-6*unit::metres);

        /*
         * Set up the vessel network
         */
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        for (unsigned idx=0; idx<100; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(float(idx)*20.0, 200.0, 40.0, reference_length));
        }

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue("User"));
        nodes[nodes.size()-1]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[nodes.size()-1]->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue("User"));
        boost::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(nodes);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);

        /*
         * Write to file
         */
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "initial_network.vtp");

        /*
         * Next set up the PDE for VEGF.
         */
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<3> > p_vegf_pde = CoupledVegfPelletDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_vegf_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_vegf_pde->SetContinuumLinearInUTerm(vegf_decay_rate);
        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_vegf_pde->SetMultiplierValue(initial_vegf_concentration);

        /*
        * Vessels bind to vegf
        */
        boost::shared_ptr<VesselBasedDiscreteSource<3> > p_vessel_vegf_sink = VesselBasedDiscreteSource<3>::Create();
        units::quantity<unit::concentration> vegf_blood_concentration(0.0*unit::mole_per_metre_cubed);
        p_vessel_vegf_sink->SetReferenceConcentration(vegf_blood_concentration);
        p_vessel_vegf_sink->SetVesselPermeability((3.e-4/3600.0)*unit::metre_per_second);
        p_vessel_vegf_sink->SetReferenceHaematocrit(0.45);

        p_vegf_pde->AddDiscreteSource(p_vessel_vegf_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */
        boost::shared_ptr<FiniteDifferenceSolver<3> > p_vegf_solver = FiniteDifferenceSolver<3>::Create();
        p_vegf_solver->SetParabolicPde(p_vegf_pde);
        p_vegf_solver->SetLabel("vegf");
        p_vegf_solver->SetGrid(p_grid);

        /*
         * Next set up the flow problem. Assign a blood plasma viscosity to the vessels. The actual viscosity will
         * depend on haematocrit and diameter. This solver manages growth and shrinkage of vessels in response to
         * flow related stimuli.
         */
        units::quantity<unit::length> max_vessel_radius(5.0 * unit::microns);
        p_network->SetSegmentRadii(max_vessel_radius);
        units::quantity<unit::dynamic_viscosity> viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue("User");
        p_network->SetSegmentViscosity(viscosity);
        /*
        * Set up the pre- and post flow calculators.
        */
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        /*
        * Set up and configure the structural adaptation solver.
        */
        boost::shared_ptr<StructuralAdaptationSolver<3> > p_structural_adaptation_solver = StructuralAdaptationSolver<3>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(2);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);

        /*
         * Set up an angiogenesis solver and add sprouting and migration rules.
         */
        boost::shared_ptr<AngiogenesisSolver<3> > p_angiogenesis_solver = AngiogenesisSolver<3>::Create();
        boost::shared_ptr<Owen2011SproutingRule<3> > p_sprouting_rule = Owen2011SproutingRule<3>::Create();
        boost::shared_ptr<TipAttractionLatticeBasedMigrationRule<3> > p_migration_rule = TipAttractionLatticeBasedMigrationRule<3>::Create();
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_sprouting_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_migration_rule->SetDiscreteContinuumSolver(p_vegf_solver);
        p_angiogenesis_solver->SetVesselGrid(p_grid);
        p_angiogenesis_solver->SetVesselNetwork(p_network);

         /*
         * The microvessel solver will manage all aspects of the vessel solve.
         */
        boost::shared_ptr<MicrovesselSolver<3> > p_microvessel_solver = MicrovesselSolver<3>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFrequency(1);
        p_microvessel_solver->SetOutputFileHandler(p_handler);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        p_microvessel_solver->SetAngiogenesisSolver(p_angiogenesis_solver);

        /*
         * Set the simulation time and run the solver. The result is shown at the top of the tutorial.
         */
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(40.0, 200);
        p_microvessel_solver->Run();
        /*
         * Dump the parameters to file for inspection.
         */
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    }
};

#endif /*TESTTIPSENSINGPAPER_HPP_*/
