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

#ifndef TESTBETTERIDGEHAEMATOCRITSOLVER_HPP
#define TESTBETTERIDGEHAEMATOCRITSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "VesselImpedanceCalculator.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "SimulationTime.hpp"
#include "BetteridgeHaematocritSolver.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "RegularGrid.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "MicrovesselSolver.hpp"
#include "OnLatticeSimulation.hpp"

#include "Owen11CellPopulationGenerator.hpp"

#include "MicrovesselVtkScene.hpp"

#include "CaBasedCellPopulation.hpp"

#include "PottsBasedCellPopulation.hpp"
#include "PottsMeshGenerator.hpp"
#include "CellsGenerator.hpp"
#include "UniformG1GenerationalCellCycleModel.hpp"
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"




#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedTestSuite.hpp"
#include <cmath>
#include <iostream>
#include "CellBasedSimulationArchiver.hpp"
#include "SmartPointers.hpp"
#include "AdhesionPottsUpdateRule.hpp"
#include "CellsGenerator.hpp"
#include "CylindricalHoneycombMeshGenerator.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "MeshBasedCellPopulationWithGhostNodes.hpp"
#include "NagaiHondaForce.hpp"
#include "SimpleTargetAreaModifier.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "OnLatticeSimulation.hpp"
#include "PlaneBoundaryCondition.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "PottsMeshGenerator.hpp"
#include "RandomCellKiller.hpp"
#include "RepulsionForce.hpp"
#include "UniformG1GenerationalCellCycleModel.hpp"
#include "SurfaceAreaConstraintPottsUpdateRule.hpp"
#include "TysonNovakCellCycleModel.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "VolumeConstraintPottsUpdateRule.hpp"
#include "VoronoiDataWriter.hpp"




#include <vector>
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "RandomNumberGenerator.hpp"
#include "UnitCollection.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "BaseUnits.hpp"
#include "VesselNode.hpp"
#include "VesselNetwork.hpp"
#include "CancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "Owen11CellPopulationGenerator.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "CaBasedCellPopulation.hpp"
#include "ApoptoticCellKiller.hpp"
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
#include "CellBasedDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "Owen2011SproutingRule.hpp"
#include "Owen2011MigrationRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "OnLatticeSimulation.hpp"
#include "MicrovesselVtkScene.hpp"
#include "VtkSceneMicrovesselModifier.hpp"

//#include "FakePetscSetup.hpp"
//#include "PetscSetupAndFinalize.hpp"





#include "PetscAndVtkSetupAndFinalize.hpp"

//class TestBetteridgeHaematocritSolver : public CxxTest::TestSuite
class TestBetteridgeHaematocritSolver : public AbstractCellBasedTestSuite
{

public:


void TestInvertedPiBetteridgeHaematocrit() throw(Exception)
{ 
    
    PottsMeshGenerator<2> generator(200, 1, 200, 100, 1, 100); //**Changed**//
    PottsMesh<2>* p_mesh = generator.GetMesh(); //**Changed**//
    std::vector<CellPtr> cells;
       
    //MAKE_PTR(TransitCellProliferativeType, p_transit_type);

    CellsGenerator<UniformG1GenerationalCellCycleModel, 2> cells_generator;

    cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumElements());//, p_transit_type);     //Here it fails
    PottsBasedCellPopulation<2> cell_population(*p_mesh, cells);//**Changed**//
    //cell_population.SetTemperature(1.0); 

    
     //  MAKE_PTR(SurfaceAreaConstraintPottsUpdateRule<2>, p_surface_area_update_rule); //**Changed**//
      // simulator.AddUpdateRule(p_surface_area_update_rule); //**Changed**//
       //MAKE_PTR(AdhesionPottsUpdateRule<2>, p_adhesion_update_rule); //**Changed**//
       //simulator.AddUpdateRule(p_adhesion_update_rule); //**Changed**//

   
       //MAKE_PTR_ARGS(RandomCellKiller<2>, p_cell_killer, (&cell_population, 0.01));
       //simulator.AddCellKiller(p_cell_killer);



    //auto p_grid = RegularGrid<2>::Create();
    std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();

    QLength grid_spacing = 10.0 * 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 21; // num x
    dimensions[1] = 11; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);


    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;


    

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 85.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 85.0 * 1_um);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
        /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);
    //TS_ASSERT_EQUALS(vascular_network->GetNumberOfNodes(), 6u);
    //TS_ASSERT_EQUALS(vascular_network->GetNumberOfVessels(), 5u);
        //vascular_network->MergeCoincidentNodes();
        //vascular_network->UpdateAll();

    // Generate the network

  
    //double radius = 20.0;
    //p_segment_2->SetRadius(radius*1.e-6*unit::metres);
    //double haematocrit = 0.50;
    //p_segment_2->GetFlowProperties()->SetHaematocrit(haematocrit);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment_2);

    

    std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  10.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  10.0 && (*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] < 60.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
	     }

             if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
	     {
                
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  10.0)                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] >   10.0 && (*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  60.0 )                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		     if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
              
            }
            /*if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 3)
            {
	  	                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2500.0*unit::pascals);
	    }
	    if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 3)
            {
	  	                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2500.0*unit::pascals);
	    }*/
        

	}
 


    //std::vector<std::shared_ptr<VesselSegment<2> > > segments = vascular_network->GetVesselSegments();
    //for(unsigned idx=0; idx<segments.size(); idx++)
    //{
     //   segments[idx]->GetFlowProperties()->SetViscosity(1.e-3*unit::poiseuille);
    //}
    QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
    VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
    VesselNetworkPropertyManager<2>::SetSegmentViscosity(vascular_network, viscosity);
    VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver = FlowSolver<2>();
    solver.SetVesselNetwork(vascular_network);
    solver.SetUseDirectSolver(false);
    solver.SetUp();
    solver.Solve();

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_InvPi", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi.vtp");
    vascular_network->Write(output_filename);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo.vtp");
    vascular_network->Write(output_filename2);

    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    //auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    //p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    //p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);
        /*
        * Vessels release oxygen depending on their haematocrit levels
        */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
    //          GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    //QConcentration vessel_oxygen_concentration = 0.0 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    

   //QConcentration other_concentration = 1000000.0 * unit::mole_per_metre_cubed;


    auto p_cell_population_genenerator = Owen11CellPopulationGenerator<2>::Create();
    std::shared_ptr<GridCalculator<2> > pGridCalc = GridCalculator<2>::Create();
    pGridCalc->SetGrid(p_grid);
    p_cell_population_genenerator->SetGridCalculator(pGridCalc);
    p_cell_population_genenerator->SetVesselNetwork(vascular_network);

    std::shared_ptr<CaBasedCellPopulation<2> > p_cell_population = p_cell_population_genenerator->Update();
    //pGridCalc->SetCellPopulation(*p_cell_population,reference_length,other_concentration);
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    //p_oxygen_solver->SetCellPopulation(*p_cell_population,reference_length,other_concentration); 
    
    auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(100.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */


    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);
    p_oxygen_solver->SetVesselNetwork(vascular_network);



    //std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
    std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(vascular_network);
    p_microvessel_solver->SetOutputFrequency(5);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        //p_microvessel_solver->AddDiscreteContinuumSolver(p_vegf_solver);
        //p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        //p_microvessel_solver->SetRegressionSolver(p_regression_solver);
        //p_microvessel_solver->SetAngiogenesisSolver(p_angiogenesis_solver);


    //boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier = MicrovesselSimulationModifier<2>::Create();
    //auto p_microvessel_modifier = MicrovesselSimulationModifier<2>::Create();
    boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier =
                boost::shared_ptr<MicrovesselSimulationModifier<2> >(new MicrovesselSimulationModifier<2> ());
    p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
    
OnLatticeSimulation<2> simulator(*p_cell_population);//**Changed**//
simulator.AddSimulationModifier(p_microvessel_modifier);
        //simulator.SetSamplingTimestepMultiple(5);
        simulator.SetDt(0.1);
        simulator.SetEndTime(0.1);
      simulator.SetOutputDirectory("InvertedPi_with_cell_pop"); //**Changed**//
//MAKE_PTR(VolumeConstraintPottsUpdateRule<2>, p_volume_constraint_update_rule); //**Changed**//
 //   simulator.AddUpdateRule(p_volume_constraint_update_rule); //**Changed**//
       simulator.Solve();




 bool write=true;
 p_oxygen_solver->SetWriteSolution(write);
 std::string output_filename3 = output_file_handler.GetOutputDirectoryFullPath().append("HexOxygen.vtp");
 const std::string& rDirectory = "TestBetteridgeHaematocritSolver_InvPi_oxygen";
 std::shared_ptr<OutputFileHandler> pOutputFileHandler(new OutputFileHandler(rDirectory));
 *pOutputFileHandler = rDirectory;

 p_oxygen_solver->SetFileHandler(pOutputFileHandler);

 p_oxygen_solver->Solve(); //now the problem must be here

      //  std::cout << "Hello4\n";  
   

    //p_oxygen_solver->Write(output_filename3);
        /*
         * Dump the parameters to file for inspection.
         */
    //ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    

}






void TestInvertedPiBetteridgeHaematocrit_polished() throw(Exception)
{ 
    
    std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();

    QLength grid_spacing = 10.0 * 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 21; // num x
    dimensions[1] = 11; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);


    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;


    

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 85.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 85.0 * 1_um);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
        /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);

    std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  10.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  10.0 && (*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] < 60.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
	     }

             if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
	     {
                
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  10.0)                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] >   10.0 && (*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  60.0 )                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		     if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
              
            }
            

	}
 


    QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
    VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
    VesselNetworkPropertyManager<2>::SetSegmentViscosity(vascular_network, viscosity);
    VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver = FlowSolver<2>();
    solver.SetVesselNetwork(vascular_network);
    solver.SetUseDirectSolver(false);
    solver.SetUp();
    solver.Solve();

    OutputFileHandler output_file_handler("Polished_TestBetteridgeHaematocritSolver_InvPi", false);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

std::string output_filename3a = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3a.vtp");
    vascular_network->Write(output_filename3a);

    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
    //          GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    //p_vessel_oxygen_source->SetVesselNetwork(vascular_network);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);


    auto p_cell_population_genenerator = Owen11CellPopulationGenerator<2>::Create();
    std::shared_ptr<GridCalculator<2> > pGridCalc = GridCalculator<2>::Create();
    pGridCalc->SetGrid(p_grid);
    p_cell_population_genenerator->SetGridCalculator(pGridCalc);
    //p_cell_population_genenerator->SetVesselNetwork(vascular_network);


    std::shared_ptr<CaBasedCellPopulation<2> > p_cell_population = p_cell_population_genenerator->Update();
   
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
 
    
    auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */

      
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);
    //p_oxygen_solver->SetVesselNetwork(vascular_network);

    std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(vascular_network);
    p_microvessel_solver->SetOutputFrequency(1);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    
    boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier =
                boost::shared_ptr<MicrovesselSimulationModifier<2> >(new MicrovesselSimulationModifier<2> ());
    p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
    
OnLatticeSimulation<2> simulator(*p_cell_population);//**Changed**//
simulator.AddSimulationModifier(p_microvessel_modifier);
        //simulator.SetSamplingTimestepMultiple(5);
        simulator.SetDt(0.1);
        simulator.SetEndTime(0.1);
      simulator.SetOutputDirectory("Polished_InvertedPi_with_cell_pop"); //**Changed**//
//MAKE_PTR(VolumeConstraintPottsUpdateRule<2>, p_volume_constraint_update_rule); //**Changed**//
 //   simulator.AddUpdateRule(p_volume_constraint_update_rule); //**Changed**//
       simulator.Solve();


     

           std::string output_filename3b = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3b.vtp");
     vascular_network->Write(output_filename3b);

}



void TestInvertedPiBetteridgeHaematocrit_minimal() throw(Exception)
{ 
    
    std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();

    QLength grid_spacing = 10.0*1_um;
    //QLength grid_spacing = 0.25* 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 21; // num x
    dimensions[1] = 11; // num_y
//dimensions[0] = 801;
//dimensions[1] = 401;
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);


    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;


    

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 80.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 80.0 * 1_um);

    p_node_1->GetFlowProperties()->SetIsInputNode(true);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
      
    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(6.0 * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(3.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(3.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
 
 
         /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);

    QLength vessel_radius(1.0*GenericParameters::mpCapillaryRadius->GetValue());
    VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

    OutputFileHandler output_file_handler("Minimal_TestBetteridgeHaematocritSolver_InvPi", false);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

std::string output_filename3a = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3a.vtp");
    vascular_network->Write(output_filename3a);



    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
    //          GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    //p_vessel_oxygen_source->SetVesselNetwork(vascular_network);
    p_vessel_oxygen_source->SetVesselPermeability(1.0*Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);




    std::shared_ptr<RegularGrid<2> > p_grid_2 = RegularGrid<2>::Create();

    QLength grid_spacing_2 = 10.0* 1_um;
    p_grid_2->SetSpacing(grid_spacing_2);
c_vector<unsigned, 3> dimensions_2;
    dimensions_2[0] = 21; // num x
    dimensions_2[1] = 11; // num_y
    dimensions_2[2] = 1;
    p_grid_2->SetDimensions(dimensions_2);

    auto p_cell_population_genenerator = Owen11CellPopulationGenerator<2>::Create();
    std::shared_ptr<GridCalculator<2> > pGridCalc = GridCalculator<2>::Create();
    pGridCalc->SetGrid(p_grid_2);
    p_cell_population_genenerator->SetGridCalculator(pGridCalc);
    //p_cell_population_genenerator->SetVesselNetwork(vascular_network);


    std::shared_ptr<CaBasedCellPopulation<2> > p_cell_population = p_cell_population_genenerator->Update();
   
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
 
    
    auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */

      
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);
    //p_oxygen_solver->SetVesselNetwork(vascular_network);

    std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(vascular_network);
    p_microvessel_solver->SetOutputFrequency(1);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);

    boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier =
                boost::shared_ptr<MicrovesselSimulationModifier<2> >(new MicrovesselSimulationModifier<2> ());
    p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);

OnLatticeSimulation<2> simulator(*p_cell_population);//**Changed**//
simulator.AddSimulationModifier(p_microvessel_modifier);
        //simulator.SetSamplingTimestepMultiple(5);
        simulator.SetDt(0.1);
        simulator.SetEndTime(0.1);
      simulator.SetOutputDirectory("Minimal_InvertedPi_with_cell_pop"); //**Changed**//
//MAKE_PTR(VolumeConstraintPottsUpdateRule<2>, p_volume_constraint_update_rule); //**Changed**//
 //   simulator.AddUpdateRule(p_volume_constraint_update_rule); //**Changed**//
    std::cout << "Step45\n";
       simulator.Solve();


         std::cout << "Step5\n";

           std::string output_filename3b = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3b.vtp");
     vascular_network->Write(output_filename3b);

}


void TestInvertedPiBetteridgeHaematocrit_minimal_playing() throw(Exception)
{ 
    
    std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();

    QLength grid_spacing =10.0* 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 21; // num x
    dimensions[1] = 16; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);


    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;


    

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 65.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 65.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 65.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 65.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 115.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 115.0 * 1_um);

    p_node_1->GetFlowProperties()->SetIsInputNode(true);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
      
    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
 
 
         /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);

    

    OutputFileHandler output_file_handler("Playing_Minimal_TestBetteridgeHaematocritSolver_InvPi", false);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

std::string output_filename3a = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3a.vtp");
    vascular_network->Write(output_filename3a);

    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
    //          GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    //p_vessel_oxygen_source->SetVesselNetwork(vascular_network);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);


    std::shared_ptr<RegularGrid<2> > p_grid_2 = RegularGrid<2>::Create();

    QLength grid_spacing_2 = 10.0* 1_um;
    p_grid_2->SetSpacing(grid_spacing_2);
c_vector<unsigned, 3> dimensions_2;
    dimensions_2[0] = 21; // num x
    dimensions_2[1] = 16; // num_y
    dimensions_2[2] = 1;
    p_grid_2->SetDimensions(dimensions_2);

    auto p_cell_population_genenerator = Owen11CellPopulationGenerator<2>::Create();
    std::shared_ptr<GridCalculator<2> > pGridCalc = GridCalculator<2>::Create();
    pGridCalc->SetGrid(p_grid_2);
    p_cell_population_genenerator->SetGridCalculator(pGridCalc);
    //p_cell_population_genenerator->SetVesselNetwork(vascular_network);


    std::shared_ptr<CaBasedCellPopulation<2> > p_cell_population = p_cell_population_genenerator->Update();
   
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
 
    
    auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);

        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */

      
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);
    //p_oxygen_solver->SetVesselNetwork(vascular_network);

    std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(vascular_network);
    p_microvessel_solver->SetOutputFrequency(1);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    
    boost::shared_ptr<MicrovesselSimulationModifier<2> > p_microvessel_modifier =
                boost::shared_ptr<MicrovesselSimulationModifier<2> >(new MicrovesselSimulationModifier<2> ());
    p_microvessel_modifier->SetMicrovesselSolver(p_microvessel_solver);
    
OnLatticeSimulation<2> simulator(*p_cell_population);//**Changed**//
simulator.AddSimulationModifier(p_microvessel_modifier);
        //simulator.SetSamplingTimestepMultiple(5);
        simulator.SetDt(0.1);
        simulator.SetEndTime(0.1);
      simulator.SetOutputDirectory("Playing_Minimal_InvertedPi_with_cell_pop"); //**Changed**//
//MAKE_PTR(VolumeConstraintPottsUpdateRule<2>, p_volume_constraint_update_rule); //**Changed**//
 //   simulator.AddUpdateRule(p_volume_constraint_update_rule); //**Changed**//
       simulator.Solve();


     

           //std::string output_filename3b = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo3b.vtp");
     //vascular_network->Write(output_filename3b);

}




void TestTwoVesselNetwork() throw(Exception)
{
    std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0_um);
    std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(80_um);
    std::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(160_um);
    p_node1->GetFlowProperties()->SetIsInputNode(true);

    std::shared_ptr<VesselSegment<2> > p_segment1(VesselSegment<2>::Create(p_node1, p_node2));
    std::shared_ptr<VesselSegment<2> > p_segment2(VesselSegment<2>::Create(p_node2, p_node3));

    std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_segment1));
    std::shared_ptr<Vessel<2> > p_vessel2(Vessel<2>::Create(p_segment2));
    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->Calculate();

    TS_ASSERT_DELTA(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(), 0.45, 1e-6);
    TS_ASSERT_DELTA(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(), 0.45/2.0, 1e-6);

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_TwoVessel", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("TwoVessel_Hemo.vtp");
    p_network->Write(output_filename);
}

void TestBifurcationInflowNetwork() throw(Exception)
{
    std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0_um,80.0_um);
    std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(0.0_um,-80.0_um);
    std::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(80_um,0.0_um);
    std::shared_ptr<VesselNode<2> > p_node4 = VesselNode<2>::Create(193_um,0.0_um);
    p_node1->GetFlowProperties()->SetIsInputNode(true);
    //p_node2->GetFlowProperties()->SetIsInputNode(true);
    //p_node1->GetFlowProperties()->SetHaematocrit(0.4);
    //p_node2->GetFlowProperties()->SetHaematocrit(0.5);

    std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_node1, p_node3));
    std::shared_ptr<Vessel<2> > p_vessel2(Vessel<2>::Create(p_node2, p_node3));
    //p_vessel1->GetSegments()[0]->GetFlowProperties()->SetHaematocrit(0.2);
    //p_vessel2->GetSegments()[0]->GetFlowProperties()->SetHaematocrit(0.5);
    std::shared_ptr<Vessel<2> > p_vessel3(Vessel<2>::Create(p_node3, p_node4));
    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(3.0 * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);
    p_network->AddVessel(p_vessel3);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->Calculate();

    //TS_ASSERT_DELTA(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.2, 1e-6);
    //TS_ASSERT_DELTA(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.5, 1e-6);
    //TS_ASSERT_DELTA(p_vessel3->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.4, 1e-6);

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_BifurcInflow", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("BifurcInflow_Hemo.vtp");
    p_network->Write(output_filename);
}

void TestTwoInTwoOutNetwork() throw(Exception)
{
    std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0_um);
    std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(100.0_um);
    std::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(0.0_um, 100.0_um);
    std::shared_ptr<VesselNode<2> > p_node4 = VesselNode<2>::Create(200_um, 100.0_um);
    std::shared_ptr<VesselNode<2> > p_node5 = VesselNode<2>::Create(200.0_um, 0.0_um);
    p_node1->GetFlowProperties()->SetIsInputNode(true);
    p_node3->GetFlowProperties()->SetIsInputNode(true);

    std::shared_ptr<VesselSegment<2> > p_segment1(VesselSegment<2>::Create(p_node1, p_node2));
    std::shared_ptr<VesselSegment<2> > p_segment2(VesselSegment<2>::Create(p_node3, p_node2));
    std::shared_ptr<VesselSegment<2> > p_segment3(VesselSegment<2>::Create(p_node2, p_node4));
    std::shared_ptr<VesselSegment<2> > p_segment4(VesselSegment<2>::Create(p_node2, p_node5));

    std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_segment1));
    std::shared_ptr<Vessel<2> > p_vessel2(Vessel<2>::Create(p_segment2));
    std::shared_ptr<Vessel<2> > p_vessel3(Vessel<2>::Create(p_segment3));
    std::shared_ptr<Vessel<2> > p_vessel4(Vessel<2>::Create(p_segment4));
    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);
    p_network->AddVessel(p_vessel3);
    p_network->AddVessel(p_vessel4);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetUseHigherConnectivityBranches(true);
    p_haematocrit_calculator->Calculate();

    TS_ASSERT_DELTA(double(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),0.45, 1e-6);
    TS_ASSERT_DELTA(double(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),0.45, 1e-6);
    TS_ASSERT_DELTA(double(p_vessel3->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),0.45, 1e-6);
    TS_ASSERT_DELTA(double(p_vessel4->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),0.45, 1e-6);

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_TwoInTwoOut", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("TwoInTwoOut_Hemo.vtp");
    p_network->Write(output_filename);
}

void TestBifurcationOutflowNetwork() throw(Exception)
{
    std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0_um);
    std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(80.0_um);
    std::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(160.0_um);
    std::shared_ptr<VesselNode<2> > p_node4 = VesselNode<2>::Create(200.0_um);
    p_node4->GetFlowProperties()->SetIsInputNode(true);

    std::shared_ptr<VesselSegment<2> > p_segment1(VesselSegment<2>::Create(p_node1, p_node3));
    std::shared_ptr<VesselSegment<2> > p_segment2(VesselSegment<2>::Create(p_node2, p_node3));
    std::shared_ptr<VesselSegment<2> > p_segment3(VesselSegment<2>::Create(p_node3, p_node4));

    std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_segment1));
    std::shared_ptr<Vessel<2> > p_vessel2(Vessel<2>::Create(p_segment2));
    std::shared_ptr<Vessel<2> > p_vessel3(Vessel<2>::Create(p_segment3));
    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(-1.0 * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(-1.0 * unit::metre_cubed_per_second);
    p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(-1.0 * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);
    p_network->AddVessel(p_vessel3);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->Calculate();

    TS_ASSERT_DELTA(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.45/2.0, 1e-6);
    TS_ASSERT_DELTA(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.45/2.0, 1e-6);
    TS_ASSERT_DELTA(p_vessel3->GetSegments()[0]->GetFlowProperties()->GetHaematocrit(),0.45, 1e-6);
    
    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_BifurcOutflow", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("BifurcOutflow_Hemo.vtp");
    p_network->Write(output_filename);
}

void TestBifurcationOutflowNetworkBiasedFlow() throw(Exception)
{
    std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0_um,0.0_um);
    std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(113.0_um,0.0_um);
    std::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(193.0_um,80.0_um);
    std::shared_ptr<VesselNode<2> > p_node4 = VesselNode<2>::Create(193.0_um,-80.0_um);
    p_node1->GetFlowProperties()->SetIsInputNode(true);

    std::shared_ptr<VesselSegment<2> > p_segment1(VesselSegment<2>::Create(p_node1, p_node2));
    std::shared_ptr<VesselSegment<2> > p_segment2(VesselSegment<2>::Create(p_node2, p_node3));
    std::shared_ptr<VesselSegment<2> > p_segment3(VesselSegment<2>::Create(p_node2, p_node4));

    std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_segment1));
    std::shared_ptr<Vessel<2> > p_vessel2(Vessel<2>::Create(p_segment2));
    std::shared_ptr<Vessel<2> > p_vessel3(Vessel<2>::Create(p_segment3));

    double parent_flow_rate = 3.0;
    double competitor_flow_rate = 2.0;
    double my_flow_rate = 1.0;

    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(parent_flow_rate * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(my_flow_rate * unit::metre_cubed_per_second);
    p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(competitor_flow_rate * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);
    p_network->AddVessel(p_vessel3);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->Calculate();

    //double parent_haematocrit = 0.45;

    //double haematocrit_ratio = 1.0 + (1.0 - parent_haematocrit)*(competitor_flow_rate/my_flow_rate - 1.0);
    //double competitor_haematocrit = (parent_flow_rate * parent_haematocrit)/ ((1.0/haematocrit_ratio)*my_flow_rate + competitor_flow_rate);
    //double my_haematocrit = competitor_haematocrit * (1.0/haematocrit_ratio);

    //TS_ASSERT_DELTA(double(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),my_haematocrit, 1e-3);
    //TS_ASSERT_DELTA(double(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),competitor_haematocrit, 1e-3);
    //TS_ASSERT_DELTA(double(p_vessel3->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),parent_haematocrit, 1e-6);

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_BifurcOutflowBiased", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("BifurcOutflowBiased_Hemo.vtp");
    p_network->Write(output_filename);
}




void TestInvPiFlow() throw(Exception)
{




    QLength vessel_length = 50.0 * 1_um;

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 85.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 85.0 * 1_um);
    //set input/output nodes and corresponding pressures
    p_node_1->GetFlowProperties()->SetIsInputNode(true);
   
    /*
     * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
     * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
     */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
    /*
     * Now add the vessels to a vessel network.
     */


    double parent_flow_rate = 9.0;
    double daughter1_flow_rate = 6.0;
    double daughter2_flow_rate = 3.0;

    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(parent_flow_rate * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(daughter1_flow_rate * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(daughter2_flow_rate * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);


    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_InvPi_ImposedFluxes", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi_ImposedFluxes_Hemo_before.vtp");
    vascular_network->Write(output_filename);


    //QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
    //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    //QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
    //VesselNetworkPropertyManager<2>::SetSegmentViscosity(vascular_network, viscosity);
    //VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
    //impedance_calculator.SetVesselNetwork(vascular_network);
    //impedance_calculator.Calculate();

    //FlowSolver<2> solver;
    //solver.SetVesselNetwork(vascular_network);
    //solver.SetUp();
    //solver.Solve();



    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();


    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("InvPi_ImposedFluxes_Hemo_after.vtp");
    vascular_network->Write(output_filename2);

    //double parent_haematocrit = 0.45;

    //double haematocrit_ratio = 1.0 + (1.0 - parent_haematocrit)*(competitor_flow_rate/my_flow_rate - 1.0);
    //double competitor_haematocrit = (parent_flow_rate * parent_haematocrit)/ ((1.0/haematocrit_ratio)*my_flow_rate + competitor_flow_rate);
    //double my_haematocrit = competitor_haematocrit * (1.0/haematocrit_ratio);

    //TS_ASSERT_DELTA(double(p_vessel1->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),my_haematocrit, 1e-3);
    //TS_ASSERT_DELTA(double(p_vessel2->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),competitor_haematocrit, 1e-3);
    //TS_ASSERT_DELTA(double(p_vessel3->GetSegments()[0]->GetFlowProperties()->GetHaematocrit()),parent_haematocrit, 1e-6);
}





void TestHexagonalNetworkBetteridgeHaematocrit() throw(Exception)
{
    // Specify the network dimensions
    QLength vessel_length = 80.0 * 1_um;

    // Generate the network
    VesselNetworkGenerator<2> vascular_network_generator;
    std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalNetwork(800.0 * 1_um,
                                                                                                                    1000.0 * 1_um,
                                                                                                                    vessel_length);

    std::vector<std::shared_ptr<VesselNode<2> > > nodes;
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(0_um,5_um)));
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(5_um,0_um)));
    std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));

    double radius = 10.0;
    p_segment->SetRadius(radius*2.e-6*unit::metres);
    double haematocrit = 0.45;
    p_segment->GetFlowProperties()->SetHaematocrit(haematocrit);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment);

    std::pair<Vertex<2>, Vertex<2> > network_extents =
            VesselNetworkGeometryCalculator<2>::GetExtents(vascular_network);
    double y_middle = (network_extents.first.Convert(1_um)[1]) / 2.0;
    double x_middle = (network_extents.first.Convert(1_um)[0]) / 2.0;

    std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
    {
        if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
        {
            if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[1] >  y_middle)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  x_middle)
                {
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                }
            }
        }
        if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
        {
            if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[1] >  y_middle)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  x_middle)
                {
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                }
            }
        }
        if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
        {
            if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[1] <=  y_middle)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  x_middle)
                {
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                }
            }
        }
        if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
        {
            if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[1] <=  y_middle)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  x_middle)
                {
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                }
            }
        }
    }

    std::vector<std::shared_ptr<VesselSegment<2> > > segments = vascular_network->GetVesselSegments();
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx]->GetFlowProperties()->SetViscosity(1.e-3*unit::poiseuille);
    }

    VesselImpedanceCalculator<2> impedance_calculator;
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver;
    solver.SetVesselNetwork(vascular_network);
    solver.SetUp();
    solver.Solve();

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexNet.vtp");
    vascular_network->Write(output_filename);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("HexNetHemo.vtp");
    vascular_network->Write(output_filename2);
}


void TestHexagonalNetworkBetteridgeHaematocrit_better() throw(Exception)
{


    auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = Owen11Parameters::mpLatticeSpacing->GetValue("User");
    QLength grid_spacing = 40.0 * 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 23; // num x
    dimensions[1] = 25; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    // Specify the network dimensions
    QLength vessel_length = 40.0 * 1_um;

    // Generate the network
    VesselNetworkGenerator<2> vascular_network_generator;
    std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalNetwork(1000.0 * 1_um,
                                                                                                                    1000.0 * 1_um,
                                                                                                                    vessel_length);

    std::vector<std::shared_ptr<VesselNode<2> > > nodes;
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(0_um,5_um)));
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(5_um,0_um)));
    std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));

    double radius = 10.0;
    p_segment->SetRadius(radius*2.e-6*unit::metres);
    double haematocrit = 0.45;
    p_segment->GetFlowProperties()->SetHaematocrit(haematocrit);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment);

    std::pair<Vertex<2>, Vertex<2> > network_extents =
            VesselNetworkGeometryCalculator<2>::GetExtents(vascular_network);
    double y_max = (network_extents.second.Convert(1_um)[1]);
    double x_max = (network_extents.second.Convert(1_um)[0]);
    double y_min = (network_extents.first.Convert(1_um)[1]);
    double x_min = (network_extents.first.Convert(1_um)[0]);
    std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[1] <  y_min + vessel_length)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  x_min + vessel_length)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[1] <  y_min + vessel_length)
                {
                    //if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  x_middle)
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  x_min + vessel_length)                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[1] >  y_max - vessel_length)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  x_max - vessel_length)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[1] >  y_max - vessel_length)
                {
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] >  x_max - vessel_length)
                    //if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  x_middle)
                    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
                }
            }
        }


    std::vector<std::shared_ptr<VesselSegment<2> > > segments = vascular_network->GetVesselSegments();
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx]->GetFlowProperties()->SetViscosity(1.e-3*unit::poiseuille);
    }

    VesselImpedanceCalculator<2> impedance_calculator;
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver;
    solver.SetVesselNetwork(vascular_network);
    solver.SetUp();
    solver.Solve();

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_better", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexNet.vtp");
    vascular_network->Write(output_filename);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("HexNetHemo.vtp");
    vascular_network->Write(output_filename2);


    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    //auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    //p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    //p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);
        /*
        * Vessels release oxygen depending on their haematocrit levels
        */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
     //           GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    //p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);
        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid); 
    p_oxygen_solver->SetVesselNetwork(vascular_network);
    bool write=true;
    p_oxygen_solver->SetWriteSolution(write);
    
    //std::string output_filename3 = output_file_handler.GetOutputDirectoryFullPath().append("HexOxygen.vtp");
    //const std::string& rDirectory = 
    const std::string& rDirectory = "TestBetteridgeHaematocritSolver_oxygen";
    std::shared_ptr<OutputFileHandler> pOutputFileHandler(new OutputFileHandler(rDirectory));
    //*pOutputFileHandler = rDirectory;

    p_oxygen_solver->SetFileHandler(pOutputFileHandler);

    p_oxygen_solver->Solve();

    //p_grid->Write(pOutputFileHandler)
    //auto p_microvessel_solver = MicrovesselSolver<2>::Create();
   // p_microvessel_solver->SetVesselNetwork(vascular_network);
   // p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        /*
         * The full simulation is run as a typical Cell Based Chaste simulation
         */
    //p_microvessel_solver->SetOutputDirectory("Oxygen");;

        /*
         * Do the solve. A sample solution is shown at the top of this test.
         */



    //p_oxygen_solver->Write(output_filename3);
        /*
         * Dump the parameters to file for inspection.
         */
    //ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    

}




void TestInvertedPiBetteridgeHaematocrit_FlowOnly() throw(Exception)
{
    //create nodes
    QLength vessel_length = 50.0 * 1_um;

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 85.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 85.0 * 1_um);
    //set input/output nodes and corresponding pressures
    p_node_1->GetFlowProperties()->SetIsInputNode(true);
    p_node_1->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
    p_node_4->GetFlowProperties()->SetIsOutputNode(true);
    p_node_4->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
    p_node_5->GetFlowProperties()->SetIsOutputNode(true);
    p_node_5->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
    p_node_6->GetFlowProperties()->SetIsOutputNode(true);
    p_node_6->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
    /*
     * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
     * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
     */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
    /*
     * Now add the vessels to a vessel network.
     */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);
    // set up network properties and solve for pressure distribution
    QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
    VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
    VesselNetworkPropertyManager<2>::SetSegmentViscosity(vascular_network, viscosity);
    VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver = FlowSolver<2>();
    solver.SetVesselNetwork(vascular_network);
    solver.SetUp();
    solver.Solve();
    //output
    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_InvPi_FlowOnly", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi.vtp");
    vascular_network->Write(output_filename);
}






void TestInvertedPiBetteridgeHaematocrit_ChangingParameters() throw(Exception)
{


    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5.0 * 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 41; // num x
    dimensions[1] = 22; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);



    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;



    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(2.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um + vessel_length);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(2.0*vessel_length, 35.0 * 1_um + vessel_length);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
        /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);
    //TS_ASSERT_EQUALS(vascular_network->GetNumberOfNodes(), 6u);
    //TS_ASSERT_EQUALS(vascular_network->GetNumberOfVessels(), 5u);
        //vascular_network->MergeCoincidentNodes();
        //vascular_network->UpdateAll();

    // Generate the network

  
    //double radius = 20.0;
    //p_segment_2->SetRadius(radius*1.e-6*unit::metres);
    //double haematocrit = 0.50;
    //p_segment_2->GetFlowProperties()->SetHaematocrit(haematocrit);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment_2);

    

    std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] <  10.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] >  10.0 && (*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] < 60.0)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetStartNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
	     }

             if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
	     {
                
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  10.0)                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
		    if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] >   10.0 && (*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] <  60.0 )                    
		    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
		     if((*vessel_iterator)->GetEndNode()->rGetLocation().Convert(1_um)[0] > 60.0)
		    {
		  	(*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
		    }
              
            }
            /*if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 3)
            {
	  	                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2500.0*unit::pascals);
	    }
	    if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 3)
            {
	  	                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2500.0*unit::pascals);
	    }*/
        

	}
 


    //std::vector<std::shared_ptr<VesselSegment<2> > > segments = vascular_network->GetVesselSegments();
    //for(unsigned idx=0; idx<segments.size(); idx++)
    //{
     //   segments[idx]->GetFlowProperties()->SetViscosity(1.e-3*unit::poiseuille);
    //}
    QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
    //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius*2.0);
    VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    //p_segment_3->SetRadius(50.e-6*unit::metres);
    QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
    VesselNetworkPropertyManager<2>::SetSegmentViscosity(vascular_network, viscosity);
    VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
    impedance_calculator.SetVesselNetwork(vascular_network);
    impedance_calculator.Calculate();
    FlowSolver<2> solver = FlowSolver<2>();
    solver.SetVesselNetwork(vascular_network);
    solver.SetUseDirectSolver(false);
    solver.SetUp();
    solver.Solve();

    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_InvPi_Changing", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi.vtp");
    vascular_network->Write(output_filename);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo.vtp");
    vascular_network->Write(output_filename2);


    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    //auto p_cell_oxygen_sink = CellBasedDiscreteSource<2>::Create();
    //p_cell_oxygen_sink->SetLinearInUConsumptionRatePerCell(Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));
    //p_oxygen_pde->AddDiscreteSource(p_cell_oxygen_sink);
        /*
        * Vessels release oxygen depending on their haematocrit levels
        */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    //QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
     //           GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    //QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
    //            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    QConcentration vessel_oxygen_concentration = 0.02768 * unit::mole_per_metre_cubed;
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    //p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);
        /*
        * Set up a finite difference solver and pass it the pde and grid.
        */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid); 
    p_oxygen_solver->SetVesselNetwork(vascular_network);
    bool write=true;
    p_oxygen_solver->SetWriteSolution(write);
    
    //std::string output_filename3 = output_file_handler.GetOutputDirectoryFullPath().append("HexOxygen.vtp");
    //const std::string& rDirectory = 
    const std::string& rDirectory = "TestBetteridgeHaematocritSolver_InvPi_oxygen_changing";
    std::shared_ptr<OutputFileHandler> pOutputFileHandler(new OutputFileHandler(rDirectory));
    //*pOutputFileHandler = rDirectory;

    p_oxygen_solver->SetFileHandler(pOutputFileHandler);

    p_oxygen_solver->Solve();

    //p_grid->Write(pOutputFileHandler)
    //auto p_microvessel_solver = MicrovesselSolver<2>::Create();
   // p_microvessel_solver->SetVesselNetwork(vascular_network);
   // p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        /*
         * The full simulation is run as a typical Cell Based Chaste simulation
         */
    //p_microvessel_solver->SetOutputDirectory("Oxygen");;

        /*
         * Do the solve. A sample solution is shown at the top of this test.
         */



    //p_oxygen_solver->Write(output_filename3);
        /*
         * Dump the parameters to file for inspection.
         */
    //ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath()+"parameter_collection.xml");
    

}


void TestInvertedPiBetteridgeHaematocrit_FlowBC() throw(Exception)
{


    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5.0 * 1_um;
    p_grid->SetSpacing(grid_spacing);

    c_vector<unsigned, 3> dimensions;
    dimensions[0] = 41; // num x
    dimensions[1] = 22; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);



    QLength reference_length(1.0 * unit::microns);
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    // Specify the network dimensions
    QLength vessel_length = 50.0 * 1_um;



    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(3.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_4 = VesselNode<2>::Create(4.0*vessel_length, 35.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_5 = VesselNode<2>::Create(vessel_length, 85.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_6 = VesselNode<2>::Create(3.0*vessel_length, 85.0 * 1_um);

    p_node_1->GetFlowProperties()->SetIsInputNode(true);
        /*
         * Next make vessel segments and vessels. Vessel segments are straight-line features which contain a `VesselNode` at each end. Vessels
         * can be constructed from multiple vessel segments by adding them in order, but in this case each vessel just has a single segment.
         */
    std::shared_ptr<VesselSegment<2> > p_segment_1 = VesselSegment<2>::Create(p_node_1, p_node_2);
    std::shared_ptr<Vessel<2> > p_vessel_1 = Vessel<2>::Create(p_segment_1);
    std::shared_ptr<VesselSegment<2> > p_segment_2 = VesselSegment<2>::Create(p_node_2, p_node_3);
    std::shared_ptr<Vessel<2> > p_vessel_2 = Vessel<2>::Create(p_segment_2);
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);

    p_segment_5->SetRadius(1.e-4*unit::metres);

    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);




    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);

        /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);


    OutputFileHandler output_file_handler("TestBetteridgeHaematocritSolver_InvPi_FlowBC", false);

    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    std::string output_filename2 = output_file_handler.GetOutputDirectoryFullPath().append("InvPiHemo_FlowBC.vtp");
    vascular_network->Write(output_filename2);


  
    

}





};

#endif // TESTBETTERIDGEHAEMATOCRITSOLVER_HPP
