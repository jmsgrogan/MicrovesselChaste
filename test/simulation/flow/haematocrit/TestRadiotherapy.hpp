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

#ifndef TESTNOCELLSBETTERIDGENONTRIVVIS_HPP
#define TESTNOCELLSBETTERIDGENONTRIVVIS_HPP

#include <cxxtest/TestSuite.h>
#include <boost/lexical_cast.hpp>
#include "VesselImpedanceCalculator.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "SimulationTime.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "BetteridgeHaematocritSolver.hpp"
#include "LinnengarHaematocritSolver.hpp"
#include "YangHaematocritSolver.hpp"
#include "ModifiedPriesHaematocritSolver.hpp"
#include "CFL_ModifiedPriesHaematocritSolver.hpp"
#include "CFL_HardCoded_ModifiedPriesHaematocritSolver.hpp"
#include "CFL_Edin_ModifiedPriesHaematocritSolver.hpp"
#include "CFL_Ed_WithX0_ModifiedPriesHaematocritSolver.hpp"
#include "Pries_WithX0_ModifiedPriesHaematocritSolver.hpp"
#include "UnitCollection.hpp"
#include "RegularGrid.hpp"
#include "SimulationTime.hpp"
#include "MicrovesselSolver.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "Owen11Parameters.hpp"
#include "ViscosityCalculator.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include <sstream>

//#include <iostream>

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestNoCellsBetteridgeNontrivVis : public CxxTest::TestSuite
{

public:

void xTestNoCellsDichotomousWithCFL_LengthsFollowRadii_WithMemory()
{
    unsigned order=5;
    double theta = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.0;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_select_few_with_memory"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii_MoreSpreaded(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 10_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-10;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}


void xTestNoRadiotherapy_Without_Memory()
{
    unsigned order=5;
    double theta = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.0;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("NoRadiotherapy_select_few_without_memory"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii_MoreSpreaded(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 10_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-10;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}



void xTestRadiotherapy_Without_Memory()
{
    unsigned order=5;
    double theta = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.0;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   //for (unsigned k_aux=5; k_aux<6; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_select_few_without_memory"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkForRadiotherapy(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
          Vertex<2>(0.5*domain_side_length_x,2.0*vessel_length));
    p_radiated_vessel->SetRadius(5.0_um);

  //  VesselPtr<2> p_radiated_vessel_2 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
   //         Vertex<2>(0.5*domain_side_length_x,1.8*vessel_length));
   // p_radiated_vessel_2->SetRadius(0.000000000001_um);

    //VesselPtr<2> p_radiated_vessel_3 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
    //        Vertex<2>(0.4*domain_side_length_x,1.8*vessel_length));
    //p_radiated_vessel_3->SetRadius(0.000000000001_um);




    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 100;
    double tolerance2 = 1.e-10;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}


void xTestRadiotherapy_Without_Memory_RandomKilling()
{
    unsigned order=5;
    double theta = 1.0;
    //unsigned NV_Total = 2*(pow(2,order+1)-1);
    unsigned NV_ToKillFrom = 2*(pow(2,order+1)-pow(2,3));
    unsigned KilledVessels = 0;

    double dimless_length = 1.0;
    double dimless_length_ToKill = 1.0;
    double dose = 0.1;
    unsigned NV_ToKill = (unsigned)(dose*NV_ToKillFrom);
    //unsigned NV_ToKill = 1;
    std::cout << "To kill from " << NV_ToKillFrom << " Vessels to kill: " << NV_ToKill;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}

    for(unsigned j_aux=1; j_aux<order-2; j_aux++)
    	{
	dimless_length_ToKill += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(j_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(j_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.0;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   //for (unsigned k_aux=5; k_aux<6; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    QLength domain_side_length_x_ToKill = dimless_length_ToKill*lambda*max_radius;
    QLength domain_side_length_x_ToKill_SecondEndPoint = domain_side_length_x - domain_side_length_x_ToKill;

    QLength LeftEnd_x = domain_side_length_x_ToKill;
    QLength RightEnd_x = domain_side_length_x_ToKill_SecondEndPoint;
    QLength BottomEnd_y = 0.0*domain_side_length_y;
    QLength TopEnd_y = domain_side_length_y;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "  and the part to kill starts at " << domain_side_length_x_ToKill << "meters \n";
    std::cout << "Left point is: " << LeftEnd_x << ", Right point is: " << RightEnd_x << " Bottom end is: " << BottomEnd_y << " Top end is: " << TopEnd_y << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_select_few_without_memory_Random"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkForRadiotherapy(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    srand(unsigned(time(NULL)));

  while( KilledVessels < NV_ToKill ) {
    double x_norm = (double)rand()/RAND_MAX;
    double y_norm = (double)rand()/RAND_MAX;
    QLength x_Kill = LeftEnd_x + x_norm*(RightEnd_x - LeftEnd_x);
    QLength y_Kill = BottomEnd_y + y_norm*(TopEnd_y - BottomEnd_y);
    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
         Vertex<2>(x_Kill,y_Kill));
    if (p_radiated_vessel->GetIsAlive()==true) {
    	p_radiated_vessel->SetToDie();
    	p_radiated_vessel->SetRadius(0.00000000000001_um);
	KilledVessels++;
        }
   std::cout << "Tried again and so far killed"  << KilledVessels << "\n";
   }

//double x_norm_1 = (double)rand()/RAND_MAX;
 //   double y_norm_1 = (double)rand()/RAND_MAX;
//double x_norm_2 = (double)rand()/RAND_MAX;
 //   double y_norm_2 = (double)rand()/RAND_MAX;
//    std::cout << " x to kill: " << x_Kill << " and y to kill: " << y_Kill; //<< " X1 normed is: " << x_norm_1 << " and Y1 normed is: " << y_norm_1 << " X2 normed is: " << x_norm_2 << " and Y2 normed is: " << y_norm_2;


//std::cout << "Is it alive? " << p_radiated_vessel->GetIsAlive();

    //getchar();

  //  VesselPtr<2> p_radiated_vessel_2 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
   //         Vertex<2>(0.5*domain_side_length_x,1.8*vessel_length));
   // p_radiated_vessel_2->SetRadius(0.000000000001_um);

    //VesselPtr<2> p_radiated_vessel_3 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
    //        Vertex<2>(0.4*domain_side_length_x,1.8*vessel_length));
    //p_radiated_vessel_3->SetRadius(0.000000000001_um);




    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 100;
    double tolerance2 = 1.e-10;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}



void xTestRadiotherapy_Without_Memory_RandomKilling_RadiusHeterogeneity()
{
    unsigned order=5;
    double theta = 1.0;
    //unsigned NV_Total = 2*(pow(2,order+1)-1);
    unsigned NV_ToKillFrom = 2*(pow(2,order+1)-pow(2,3));
    unsigned KilledVessels = 0;

    double dimless_length = 1.0;
    double dimless_length_ToKill = 1.0;
    double dose = 0.00;
    unsigned NV_ToKill = (unsigned)(dose*NV_ToKillFrom);
    //unsigned NV_ToKill = 1;
    std::cout << "To kill from " << NV_ToKillFrom << " Vessels to kill: " << NV_ToKill;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}

    for(unsigned j_aux=1; j_aux<order; j_aux++)
    	{
	dimless_length_ToKill += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(j_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(j_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.3;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   //for (unsigned k_aux=5; k_aux<6; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    QLength domain_side_length_x_ToKill = dimless_length_ToKill*lambda*max_radius;
    QLength domain_side_length_x_ToKill_SecondEndPoint = domain_side_length_x - domain_side_length_x_ToKill;

    QLength LeftEnd_x = domain_side_length_x_ToKill;
    QLength RightEnd_x = domain_side_length_x_ToKill_SecondEndPoint;
    QLength BottomEnd_y = 0.0*domain_side_length_y;
    QLength TopEnd_y = domain_side_length_y;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "  and the part to kill starts at " << domain_side_length_x_ToKill << "meters \n";
    std::cout << "Left point is: " << LeftEnd_x << ", Right point is: " << RightEnd_x << " Bottom end is: " << BottomEnd_y << " Top end is: " << TopEnd_y << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_Dichotomous_RandomRadii_Dose"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkForRadiotherapyRandomHeterogeneity(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    srand(unsigned(time(NULL)));

  while( KilledVessels < NV_ToKill ) {
    double x_norm = (double)rand()/RAND_MAX;
    double y_norm = (double)rand()/RAND_MAX;
    QLength x_Kill = LeftEnd_x + x_norm*(RightEnd_x - LeftEnd_x);
    QLength y_Kill = BottomEnd_y + y_norm*(TopEnd_y - BottomEnd_y);
    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
         Vertex<2>(x_Kill,y_Kill));
    if (p_radiated_vessel->GetIsAlive()==true) {
    	p_radiated_vessel->SetToDie();
    	p_radiated_vessel->SetRadius(0.00000000000001_um);
	KilledVessels++;
        }
   std::cout << "Tried again and so far killed"  << KilledVessels << "\n";
   }

//double x_norm_1 = (double)rand()/RAND_MAX;
 //   double y_norm_1 = (double)rand()/RAND_MAX;
//double x_norm_2 = (double)rand()/RAND_MAX;
 //   double y_norm_2 = (double)rand()/RAND_MAX;
//    std::cout << " x to kill: " << x_Kill << " and y to kill: " << y_Kill; //<< " X1 normed is: " << x_norm_1 << " and Y1 normed is: " << y_norm_1 << " X2 normed is: " << x_norm_2 << " and Y2 normed is: " << y_norm_2;


//std::cout << "Is it alive? " << p_radiated_vessel->GetIsAlive();

    //getchar();

  //  VesselPtr<2> p_radiated_vessel_2 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
   //         Vertex<2>(0.5*domain_side_length_x,1.8*vessel_length));
   // p_radiated_vessel_2->SetRadius(0.000000000001_um);

    //VesselPtr<2> p_radiated_vessel_3 = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
    //        Vertex<2>(0.4*domain_side_length_x,1.8*vessel_length));
    //p_radiated_vessel_3->SetRadius(0.000000000001_um);




    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    //auto p_haematocrit_calculator = BetteridgeHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 100;
    double tolerance2 = 1.e-6;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}



void TestRadiotherapy_Without_Memory_RandomKilling_Hexagonal()
{
srand(unsigned(time(NULL)));
for(unsigned ii=0;ii<10;ii++)
{
	int flip=  rand() % 2;// assign random numbers
	std::cout<< "ii is: " << ii << ", and flip is: " <<flip << "  ";
}

    //unsigned NV_Total = 6*4*9;
    unsigned NV_ToKillFrom = 6*3*7;
    unsigned KilledVessels = 0;

    double dose = 0.0;
    unsigned NV_ToKill = (unsigned)(dose*NV_ToKillFrom);
    //unsigned NV_ToKill = 1;
    //std::cout << "To kill from " << NV_ToKillFrom << " Vessels to kill: " << NV_ToKill;

   // QLength max_radius = 50_um;
    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.38;
    double initial_haematocrit = 0.38;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;
   


// Specify the domain
    QLength vessel_length = 100_um;
    //int units_in_y = 6;
    //QLength domain_side_length_x = 1600_um;
    //QLength domain_side_length_y = 1280_um;
    QLength domain_side_length_x = 800_um;
    QLength domain_side_length_y = 800_um;

   // QLength domain_side_length_x_ToKill = dimless_length_ToKill*lambda*max_radius;
  //  QLength domain_side_length_x_ToKill_SecondEndPoint = domain_side_length_x - domain_side_length_x_ToKill;

   // QLength LeftEnd_x = domain_side_length_x_ToKill;
  //  QLength RightEnd_x = domain_side_length_x_ToKill_SecondEndPoint;
  //  QLength BottomEnd_y = 0.0*domain_side_length_y;
  //  QLength TopEnd_y = domain_side_length_y;
   // std::cout << "Here, Domain length is:" << domain_side_length_x << "meters \n";
 //   std::cout << "Left point is: " << LeftEnd_x << ", Right point is: " << RightEnd_x << " Bottom end is: " << BottomEnd_y << " Top end is: " << TopEnd_y << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_Hexagonal_Full_Betteridge_BottomTop", true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetworkEquilateral(domain_side_length_x, domain_side_length_y, vessel_length,false);
    std::string output_file_initial = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork.vtp");
    //QLength real_length_x = 1280_um;
    //QLength real_length_y = 1280_um;
    QLength real_length_x = domain_side_length_x;
    QLength real_length_y = domain_side_length_y;
    QLength offset_x = 300_um;
    QLength offset_y = 200_um;
    // Assign radii
    QLength vessel_radius = 20_um;
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->SetRadius(vessel_radius);
   // p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);


    QLength unit_height = sqrt(2.0)*vessel_length;


    unsigned units_in_y_direction = floor(domain_side_length_y/unit_height);

    for(unsigned i_aux=0; i_aux < units_in_y_direction+1;i_aux++)
    {
    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(-vessel_length, double(i_aux)*unit_height));
    //VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
        //    Vertex<2>(domain_side_length_x, domain_side_length_y));
	VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(domain_side_length_x, double(i_aux)*unit_height));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    }

/*
   VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(-vessel_length, 0.0_um));
   p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);

   //VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
    //      Vertex<2>(domain_side_length_x, 0.0_um));
VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
        Vertex<2>(domain_side_length_x, domain_side_length_y));
   //p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
   //p_inlet_node_2->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);
*/

    srand(unsigned(time(NULL)));

    while( KilledVessels < NV_ToKill ) {
      double x_norm = (double)rand()/RAND_MAX;
      double y_norm = (double)rand()/RAND_MAX;
      QLength x_Kill = offset_x + x_norm*(real_length_x-2.0*offset_x);
      QLength y_Kill = offset_y + y_norm*(real_length_y-2.0*offset_y);
      std::cout << x_Kill << y_Kill;
      //VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
       //  Vertex<2>(x_Kill,y_Kill));
    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
        Vertex<2>(x_Kill,y_Kill));
      if (p_radiated_vessel->GetIsAlive()==true && p_radiated_vessel->GetId() >24) {
    	p_radiated_vessel->SetToDie();
    	p_radiated_vessel->SetRadius(0.00000000000001_um);
	KilledVessels++;
        }
      std::cout << "Tried again and so far killed"  << KilledVessels << "\n";
    }


//    std::cout << " x to kill: " << x_Kill << " and y to kill: " << y_Kill; //<< " X1 normed is: " << x_norm_1 << " and Y1 normed is: " << y_norm_1 << " X2 normed is: " << x_norm_2 << " and Y2 normed is: " << y_norm_2;

//std::cout << "Is it alive? " << p_radiated_vessel->GetIsAlive();





    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((real_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((real_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    //auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_haematocrit_calculator = BetteridgeHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();
    //flow_solver.SetUseDirectSolver(false);
    unsigned max_iter = 400;
    double tolerance2 = 1.e-6;
  p_network->Write(output_file_initial);
    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));


//p_impedance_calculator->Calculate();
        //flow_solver.SetUp();
        //flow_solver.Solve();

        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
      
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        //p_network->Write(output_file_initial);
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;


}



void xTestRadiotherapy_Without_Memory_RandomKilling_Square()
{

    //unsigned NV_Total = 6*4*9;
    unsigned NV_ToKillFrom = 6*4*8;
    unsigned KilledVessels = 0;

    double dose = 0.0;
    unsigned NV_ToKill = (unsigned)(dose*NV_ToKillFrom);
    //unsigned NV_ToKill = 1;
    //std::cout << "To kill from " << NV_ToKillFrom << " Vessels to kill: " << NV_ToKill;

   // QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;
   


// Specify the domain
    QLength vessel_length = 100_um;
    //int units_in_y = 6;
    QLength real_length_x = 300_um;
    QLength real_length_y = 300_um;
    //QLength domain_side_length_x = 100_um;
    //QLength domain_side_length_y = 100_um;

   // QLength domain_side_length_x_ToKill = dimless_length_ToKill*lambda*max_radius;
  //  QLength domain_side_length_x_ToKill_SecondEndPoint = domain_side_length_x - domain_side_length_x_ToKill;

   // QLength LeftEnd_x = domain_side_length_x_ToKill;
  //  QLength RightEnd_x = domain_side_length_x_ToKill_SecondEndPoint;
  //  QLength BottomEnd_y = 0.0*domain_side_length_y;
  //  QLength TopEnd_y = domain_side_length_y;
   // std::cout << "Here, Domain length is:" << domain_side_length_x << "meters \n";
 //   std::cout << "Left point is: " << LeftEnd_x << ", Right point is: " << RightEnd_x << " Bottom end is: " << BottomEnd_y << " Top end is: " << TopEnd_y << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_Square_KillDose_Betteridge_BothInOutAtBottom", true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSquareNetwork_TopOutput(real_length_x, real_length_y, vessel_length,false);
    std::string output_file_initial = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork.vtp");
    std::string output_file_initial2 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork2.vtp");
    std::string output_file_initial3 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork3.vtp");
    std::string output_file_initial4 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork4.vtp");
    std::string output_file_initial5 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork5.vtp");
    std::string output_file_initial6 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork6.vtp");
    std::string output_file_initial7 = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork7.vtp");


    //QLength offset_x = 300_um;
    //QLength offset_y = 200_um;
    // Assign flow properties
    auto p_segment0 = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    p_segment0->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment0->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    /**auto p_segment1 = p_network->GetVesselSegments()[1];
    p_segment1->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    auto p_segment2 = p_network->GetVesselSegments()[2];
    p_segment2->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    auto p_segment3 = p_network->GetVesselSegments()[3];
    p_segment3->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    auto p_segment4 = p_network->GetVesselSegments()[4];
    p_segment4->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    auto p_segment5 = p_network->GetVesselSegments()[5];
    p_segment5->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
*/
    //QLength unit_height = sqrt(2.0)*vessel_length;


    //unsigned units_in_y_direction = floor(domain_side_length_y/unit_height);

  //  for(unsigned i_aux=0; i_aux < units_in_y_direction+1;i_aux++)
  //  {
   // VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
    //        Vertex<2>(-vessel_length, double(i_aux)*unit_height));
   // VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
   //         Vertex<2>(domain_side_length_x, domain_side_length_y));
//	VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
  //        Vertex<2>(domain_side_length_x, double(i_aux)*unit_height));
    //p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
 //   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
   // p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    //p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    //}


   VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(-vessel_length, 0.0_um));
   p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);

 //  VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
   //       Vertex<2>(real_length_x+vessel_length, 0.0_um));
   VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(real_length_x+vessel_length, real_length_y));

   //p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
   //p_inlet_node_2->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    srand(unsigned(time(NULL)));

    while( KilledVessels < NV_ToKill ) {
      double x_norm = (double)rand()/RAND_MAX;
      double y_norm = (double)rand()/RAND_MAX;
      //QLength x_Kill = offset_x + x_norm*(real_length_x-2.0*offset_x);
      //QLength y_Kill = offset_y + y_norm*(real_length_y-2.0*offset_y);
      QLength x_Kill = x_norm*real_length_x;
      QLength y_Kill = y_norm*real_length_y;
      std::cout << x_Kill << y_Kill;
      //VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
       //  Vertex<2>(x_Kill,y_Kill));
    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
        Vertex<2>(x_Kill,y_Kill));
      if (p_radiated_vessel->GetIsAlive()==true && p_radiated_vessel->GetId() >24) {
    	p_radiated_vessel->SetToDie();
    	p_radiated_vessel->SetRadius(0.00000000000001_um);
	KilledVessels++;
        }
      std::cout << "Tried again and so far killed"  << KilledVessels << "\n";
    }


//    std::cout << " x to kill: " << x_Kill << " and y to kill: " << y_Kill; //<< " X1 normed is: " << x_norm_1 << " and Y1 normed is: " << y_norm_1 << " X2 normed is: " << x_norm_2 << " and Y2 normed is: " << y_norm_2;

std::cout << "Is it alive? ";





    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((real_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((real_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    //auto p_haematocrit_calculator = BetteridgeHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();
    //flow_solver.SetUseDirectSolver(false);


    unsigned max_iter = 400;
    double tolerance2 = 1.e-6;
    p_network->Write(output_file_initial);
    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
	    p_network->Write(output_file_initial2);
        p_impedance_calculator->Calculate();
    p_network->Write(output_file_initial3);
        flow_solver.SetUp();
    p_network->Write(output_file_initial4);
        flow_solver.Solve();
    p_network->Write(output_file_initial5);
        p_haematocrit_calculator->Calculate();
    p_network->Write(output_file_initial6);
        p_viscosity_calculator->Calculate();
        // Get the residual
    p_network->Write(output_file_initial7);
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;

}






void xTestRadiotherapy_Without_Memory_RandomKilling_Square_Debug()
{

    //unsigned NV_Total = 6*4*9;
    unsigned NV_ToKillFrom = 6*4*8;
    unsigned KilledVessels = 0;

    double dose = 0.0;
    unsigned NV_ToKill = (unsigned)(dose*NV_ToKillFrom);
    //unsigned NV_ToKill = 1;
    //std::cout << "To kill from " << NV_ToKillFrom << " Vessels to kill: " << NV_ToKill;

   // QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;
   


// Specify the domain
    QLength vessel_length = 100_um;
    //int units_in_y = 6;
    QLength domain_side_length_x = 500_um;
    QLength domain_side_length_y = 500_um;

   // QLength domain_side_length_x_ToKill = dimless_length_ToKill*lambda*max_radius;
  //  QLength domain_side_length_x_ToKill_SecondEndPoint = domain_side_length_x - domain_side_length_x_ToKill;

   // QLength LeftEnd_x = domain_side_length_x_ToKill;
  //  QLength RightEnd_x = domain_side_length_x_ToKill_SecondEndPoint;
  //  QLength BottomEnd_y = 0.0*domain_side_length_y;
  //  QLength TopEnd_y = domain_side_length_y;
   // std::cout << "Here, Domain length is:" << domain_side_length_x << "meters \n";
 //   std::cout << "Left point is: " << LeftEnd_x << ", Right point is: " << RightEnd_x << " Bottom end is: " << BottomEnd_y << " Top end is: " << TopEnd_y << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_Square_KillDose_Betteridge_BothInOutAtBottom", true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateSquareNetwork(domain_side_length_x, domain_side_length_y, vessel_length,false);
    std::string output_file_initial = p_file_handler->GetOutputDirectoryFullPath().append("InitialNetwork.vtp");
    QLength real_length_x = 500_um;
    QLength real_length_y = 500_um;

   VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(-vessel_length, 0.0_um));
   p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);

   VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
          Vertex<2>(domain_side_length_x+vessel_length, 0.0_um));
   //p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
   //p_inlet_node_2->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    srand(unsigned(time(NULL)));

    while( KilledVessels < NV_ToKill ) {
      double x_norm = (double)rand()/RAND_MAX;
      double y_norm = (double)rand()/RAND_MAX;
      //QLength x_Kill = offset_x + x_norm*(real_length_x-2.0*offset_x);
      //QLength y_Kill = offset_y + y_norm*(real_length_y-2.0*offset_y);
      QLength x_Kill = x_norm*real_length_x;
      QLength y_Kill = y_norm*real_length_y;
      std::cout << x_Kill << y_Kill;
      //VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
       //  Vertex<2>(x_Kill,y_Kill));
    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
        Vertex<2>(x_Kill,y_Kill));
      if (p_radiated_vessel->GetIsAlive()==true && p_radiated_vessel->GetId() >24) {
    	p_radiated_vessel->SetToDie();
    	p_radiated_vessel->SetRadius(0.00000000000001_um);
	KilledVessels++;
        }
      std::cout << "Tried again and so far killed"  << KilledVessels << "\n";
    }


//    std::cout << " x to kill: " << x_Kill << " and y to kill: " << y_Kill; //<< " X1 normed is: " << x_norm_1 << " and Y1 normed is: " << y_norm_1 << " X2 normed is: " << x_norm_2 << " and Y2 normed is: " << y_norm_2;

std::cout << "Is it alive? ";





    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((real_length_x)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((real_length_y)/(grid_spacing))+1; // num_y
    dimensions[2] = 1;
    p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
   auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    //auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_haematocrit_calculator = BetteridgeHaematocritSolver<2>::Create();
//auto p_haematocrit_calculator = CFL_Ed_WithX0_ModifiedPriesHaematocritSolver<2>::Create(); //Even for large lambda...if we kill one vessel in the centre...this gives crazy results, negative haematocrits etc 
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
  p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 4;
    //double tolerance2 = 1.e-6;
    p_network->Write(output_file_initial);
    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));

p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        //p_viscosity_calculator->Calculate();

    for(unsigned idx=0;idx<max_iter;idx++)
    {
	std::cout << "Bla";
        //p_impedance_calculator->Calculate();
        //flow_solver.SetUp();
        //flow_solver.Solve();
        //p_haematocrit_calculator->Calculate();
        //p_viscosity_calculator->Calculate();
        // Get the residual
        //double max_difference = 0.0;
        //double h_for_max = 0.0;
        //double prev_for_max = 0.0;
	/**
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
*/
            // Output intermediate results
/**
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
*/
        //}
	

        //if(idx==max_iter-1)
        //{
        //    EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
       // }
    }

 /**
    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
*/
}


void xTestRadiotherapy_Without_Memory_FE()
{
    unsigned order=5;
    double theta = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength max_radius = 50_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;



    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.0;
    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;

   for (unsigned k_aux=1; k_aux<2; k_aux++)
   {
    lambda = 4.0+double(k_aux)*4.0;


// Specify the domain
    QLength vessel_length = 0.9*lambda*max_radius*pow(2.0,-1.0/3.0);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("Radiotherapy_select_few_without_memory_FE"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii_MoreSpreaded(order, vessel_length, max_radius, alpha,theta, lambda);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, 2.0*vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    VesselPtr<2> p_radiated_vessel = VesselNetworkGeometryCalculator<2>::GetNearestVessel(p_network,
            Vertex<2>(0.5*domain_side_length_x,2.0*vessel_length));
    p_radiated_vessel->SetRadius(0.000000000001_um);



    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = 5_um;
    //p_grid->SetSpacing(grid_spacing);
    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = unsigned((domain_side_length_x)/(grid_spacing))+1; // num x
    //dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+1; // num_y
    //dimensions[2] = 1;
    //p_grid->SetDimensions(dimensions);

    /**
     * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
     */
    auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
    p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
    p_oxygen_pde->SetContinuumLinearInUTerm(-1.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

    /**
    * Vessels release oxygen depending on their haematocrit levels
    */
    auto p_vessel_oxygen_source = VesselBasedDiscreteSource<2>::Create();
    QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("User");
    QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
            Owen11Parameters::mpReferencePartialPressure->GetValue("User");
    p_vessel_oxygen_source->SetReferenceConcentration(vessel_oxygen_concentration);
    p_vessel_oxygen_source->SetVesselPermeability(Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
    p_vessel_oxygen_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
    p_oxygen_pde->AddDiscreteSource(p_vessel_oxygen_source);

    /*
    * Set up a finite element solver and pass it the pde and grid.
    */

// Set up the grid
    std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
    p_domain->AddRectangle(domain_side_length_x, domain_side_length_y);
    //Vertex<2> translation_vector(-1.0*vessel_length/2.0);
    //p_domain->Translate(translation_vector);    


    std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator =
                DiscreteContinuumMeshGenerator<2>::Create();
    p_mesh_generator->SetDomain(p_domain);
    //QLength spacing(1.0*unit::microns);
    p_mesh_generator->SetMaxElementArea(Qpow3(0.05*vessel_length));
    p_mesh_generator->Update();
    

    auto p_oxygen_solver = SimpleLinearEllipticFiniteElementSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_mesh_generator->GetMesh());

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = Pries_WithX0_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    //p_haematocrit_calculator->SetLinnM(1.35);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-10;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
            if(difference>max_difference)
            {
                max_difference = difference;
                h_for_max = current_haematocrit;
                prev_for_max = previous_haematocrit[jdx];
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
        if(max_difference<=tolerance2)
        {
            std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
            break;
        }
        else
        {
            // Output intermediate results
            if(idx%1==0)
            {
                std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
                std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
                std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
                p_network->Write(output_file);
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
    }

    SimulationTime::Instance()->SetStartTime(0.0);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
    auto p_microvessel_solver = MicrovesselSolver<2>::Create();
    p_microvessel_solver->SetVesselNetwork(p_network);
    p_microvessel_solver->SetOutputFileHandler(p_file_handler);
    p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
    p_microvessel_solver->Run();

    std::vector<double> solution = p_oxygen_solver->GetSolution();
    double average_oxygen = 0.0;
    for(unsigned jdx=0;jdx<solution.size();jdx++)
    {
        average_oxygen += solution[jdx];
    }
    average_oxygen /= double(solution.size());
    std::cout << "Average oxygen: " << average_oxygen << std::endl;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    SimulationTime::Instance()->Destroy();
    std::cout << vessel_oxygen_concentration;
    }
}


};

#endif // TESTNOCELLSBETTERIDGENONTRIVVIS_HPP
