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

#ifndef TESTWITHORWITHOUTMEMORY_HPP
#define TESTWITHORWITHOUTMEMORY_HPP

#include <cxxtest/TestSuite.h>
#include <boost/lexical_cast.hpp>
#include <ctime>
#include "VesselImpedanceCalculator.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "SimulationTime.hpp"
#include "PriesHaematocritSolver.hpp"
#include "PriesWithMemoryHaematocritSolverNonLinear.hpp"
#include "PriesWithMemoryHaematocritSolver.hpp"
#include "UnitCollection.hpp"
#include "RegularGrid.hpp"
#include "SimulationTime.hpp"
#include "MicrovesselSolver.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
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

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestDifferentPriesHaematocritSolvers : public CxxTest::TestSuite
{

public:


void TestDifferentSolversWithTheSameInitialConditionsHexagonalNetworks()
{
  // Test for checking the distane between the solutions for Haematocrit using
  // the two pries with memory haematocrit solvers using hexagonal networks.
    // order of the dichotomous network
    unsigned order=5;

    double y_scale = 4.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    {
	     dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    }
    QLength input_radius = 50_um;

    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.0;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda = 10.0;
   double twicelambda = 2*lambda;
   double vessel_length = input_radius*twicelambda;


   // Length of the vertical projection of first-order vessels
   QLength main_vert_length = twicelambda*input_radius*pow(2.0,-1.0/2.0);
   // vertical size of the domain
   QLength domain_side_length_y = y_scale*main_vert_length;

   std::ostringstream strs;
   strs << lambda;
   std::string str_lambda = strs.str();
   // horizontal size of the domain
   QLength domain_side_length_x = dimless_length*2.0*twicelambda*input_radius;
   auto p_file_handler = std::make_shared<OutputFileHandler>("Test_Different_Solvers_Hexagonal", true);
   // Generating the two identical networks to test both methods
   std::shared_ptr<VesselNetwork<2> > p_network_1 = network_generator.GenerateHexagonalNetworkRadius(domain_side_length_x, domain_side_length_y, vessel_length, input_radius);
   std::shared_ptr<VesselNetwork<2> > p_network_2 = network_generator.GenerateHexagonalNetworkRadius(domain_side_length_x, domain_side_length_y, vessel_length, input_radius);
   std::vector<std::shared_ptr<Vessel<2>>> vessels_1 = p_network_1->GetVessels();
   std::vector<std::shared_ptr<Vessel<2>>> vessels_2 = p_network_2->GetVessels();
   for(unsigned i = 0; i < vessels_1.size(); i++)
   {
     vessels_1[i]->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
     vessels_2[i]->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
   }

   // identify input and output nodes and assign them properties

   unsigned max_height = y_scale;

   for(unsigned idx = 0; idx <= max_height; idx++)
   {
     double scale = idx;
     VesselNodePtr<2> p_inlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_1, Vertex<2>(0.0_um, scale*main_vert_length));
     VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_1, Vertex<2>(domain_side_length_x, scale*main_vert_length));
     VesselNodePtr<2> p_inlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_2, Vertex<2>(0.0_um, scale*main_vert_length));
     VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_2, Vertex<2>(domain_side_length_x, scale*main_vert_length));
     p_inlet_node_1->GetFlowProperties()->SetIsInputNode(true);
     p_inlet_node_1->GetFlowProperties()->SetPressure(3320.0_Pa);
     p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
     p_outlet_node_1->GetFlowProperties()->SetPressure(2090.0_Pa);
     p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
     p_inlet_node_2->GetFlowProperties()->SetPressure(3320.0_Pa);
     p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
     p_outlet_node_2->GetFlowProperties()->SetPressure(2090.0_Pa);
   }

   // Setting the solvers for the two networks.
   auto p_haematocrit_calculator_linear = PriesWithMemoryHaematocritSolver<2>::Create();
   auto p_haematocrit_calculator_nonlinear = PriesWithMemoryHaematocritSolverNonLinear<2>::Create();
   auto p_impedance_calculator_linear = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator_linear = ViscosityCalculator<2>::Create();
   p_viscosity_calculator_linear->SetPlasmaViscosity(viscosity);
   auto p_impedance_calculator_nonlinear = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator_nonlinear = ViscosityCalculator<2>::Create();
   p_viscosity_calculator_nonlinear->SetPlasmaViscosity(viscosity);

   p_haematocrit_calculator_linear->SetVesselNetwork(p_network_1);
   p_haematocrit_calculator_linear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator_linear->SetVesselNetwork(p_network_1);
   p_viscosity_calculator_linear->SetVesselNetwork(p_network_1);
   p_viscosity_calculator_linear->Calculate();
   p_impedance_calculator_linear->Calculate();

   p_haematocrit_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_haematocrit_calculator_nonlinear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_viscosity_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_viscosity_calculator_nonlinear->Calculate();
   p_impedance_calculator_nonlinear->Calculate();

   FlowSolver<2> flow_solver_linear;
   flow_solver_linear.SetVesselNetwork(p_network_1);
   flow_solver_linear.SetUp();

   FlowSolver<2> flow_solver_nonlinear;
   flow_solver_nonlinear.SetVesselNetwork(p_network_2);
   flow_solver_nonlinear.SetUp();

   unsigned max_iter = 1000;
   double tolerance2 = 1.e-10;

   std::vector<VesselSegmentPtr<2> > segments_linear = p_network_1->GetVesselSegments();
   std::vector<double> previous_haematocrit(segments_linear.size(), double(initial_haematocrit));
   // Timing the speed of the algorithm
   clock_t begin = clock();
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator_linear->Calculate();
        p_impedance_calculator_linear->Calculate();
        flow_solver_linear.SetUp();
        flow_solver_linear.Solve();
        p_haematocrit_calculator_linear->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments_linear.size();jdx++)
        {
            double current_haematocrit = segments_linear[jdx]->GetFlowProperties()->GetHaematocrit();
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }
   clock_t end = clock();

   std::cout << "Time elapsed = " << 1.0e6*(end - begin)/CLOCKS_PER_SEC << std::endl;

   std::cout << "Sup Flow = " << flow_solver_linear.CheckSolution() << std::endl;
   std::cout << "Sup RBC = " << p_haematocrit_calculator_linear->CheckSolution() << std::endl;

   std::string file_suffix = "first_solution.vtp";
   std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
   p_network_1->Write(output_file);

   std::vector<VesselSegmentPtr<2> > segments_nonlinear = p_network_2->GetVesselSegments();
   std::fill(previous_haematocrit.begin(), previous_haematocrit.end(), initial_haematocrit);

   std::cout << "Now beginning to use the new solver" << std::endl;

   // Timing the speed of the algorithm
   begin = clock();
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator_nonlinear->Calculate();
        p_impedance_calculator_nonlinear->Calculate();
        flow_solver_nonlinear.SetUp();
        flow_solver_nonlinear.Solve();
        p_haematocrit_calculator_nonlinear->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments_nonlinear.size();jdx++)
        {
            double current_haematocrit = segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit();
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }
   end = clock();

   std::cout << "Time elapsed = " << 1.0e6*(end - begin)/CLOCKS_PER_SEC << std::endl;

   std::cout << "Sup Flow = " << flow_solver_nonlinear.CheckSolution() << std::endl;
   std::cout << "Sup RBC = " << p_haematocrit_calculator_nonlinear->CheckSolution() << std::endl;

   file_suffix = "second_solution.vtp";
   output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
   p_network_2->Write(output_file);

   QDimensionless Max_Haematocrit_Difference = 0.0;
   QDimensionless Haematocrit_L2_Norm = 0.0;
   unsigned Max_index;
   double absolute_flow;

   // Calculating the L infinty and L2 norm for the two solutions.
   for(unsigned jdx = 0; jdx < segments_linear.size(); jdx++)
   {
     absolute_flow += abs(segments_linear[jdx]->GetFlowProperties()->GetFlowRate() -
     segments_nonlinear[jdx]->GetFlowProperties()->GetFlowRate());
     if(Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() -
     segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit()) > Max_Haematocrit_Difference)
     {
       Max_Haematocrit_Difference = Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() -
       segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit());
       Max_index = jdx;
     }
     Haematocrit_L2_Norm = Haematocrit_L2_Norm + pow(Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() - segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit()), 2.0);
   }
   Haematocrit_L2_Norm = pow(Haematocrit_L2_Norm, 0.5);

   std::cout << "Maximum haematocrit difference = " << Max_Haematocrit_Difference << " with vessel id = " << Max_index << std::endl;
   std::cout << "Haematocrit L2 Norm = " << Haematocrit_L2_Norm << std::endl;
}


void TestDifferentSolversWithTheSameInitialConditionsDichotomousNetwork()
{
  // Test for checking the distane between the solutions for Haematocrit using
  // the two pries with memory haematocrit solvers using dichotomous networks.
    // order of the dichotomous network
    unsigned order=5;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength input_radius = 50_um;

    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda = 10.0;
   double twicelambda = 2.0*lambda;


   // Length of the vertical projection of first-order vessels
   QLength main_vert_length = 0.9*twicelambda*input_radius*pow(2.0,-1.0/3.0);
   // vertical size of the domain

   std::ostringstream strs;
   strs << lambda;
   std::string str_lambda = strs.str();
   // horizontal size of the domain
   QLength domain_side_length_x = dimless_length*2.0*twicelambda*input_radius;
   auto p_file_handler = std::make_shared<OutputFileHandler>("Test_Different_Solvers_Dichotomous_Network_" + str_lambda, true);
   // Generating two identical networks for testing the two methods.
   std::shared_ptr<VesselNetwork<2> > p_network_1 = network_generator.GenerateForkingNetworkNoCorners(order, main_vert_length, input_radius, twicelambda);
   std::shared_ptr<VesselNetwork<2> > p_network_2 = network_generator.GenerateForkingNetworkNoCorners(order, main_vert_length, input_radius, twicelambda);
   // identify input and output nodes and assign them properties

   VesselNodePtr<2> p_inlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_1,
     Vertex<2>(0.0_um,2.0*main_vert_length));
   VesselNodePtr<2> p_inlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_2,
     Vertex<2>(0.0_um,2.0*main_vert_length));
   VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_1,
     Vertex<2>(domain_side_length_x, 2.0*main_vert_length));
   VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network_2,
     Vertex<2>(domain_side_length_x, 2.0*main_vert_length));
   p_inlet_node_1->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node_1->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node_1->GetFlowProperties()->SetPressure(2090.0_Pa);

   p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node_2->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node_2->GetFlowProperties()->SetPressure(2090.0_Pa);

   // Seting the solvers for the two methods.
   auto p_haematocrit_calculator_linear = PriesWithMemoryHaematocritSolver<2>::Create();
   auto p_haematocrit_calculator_nonlinear = PriesWithMemoryHaematocritSolverNonLinear<2>::Create();
   auto p_impedance_calculator_linear = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator_linear = ViscosityCalculator<2>::Create();
   p_viscosity_calculator_linear->SetPlasmaViscosity(viscosity);
   auto p_impedance_calculator_nonlinear = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator_nonlinear = ViscosityCalculator<2>::Create();
   p_viscosity_calculator_nonlinear->SetPlasmaViscosity(viscosity);

   p_haematocrit_calculator_linear->SetVesselNetwork(p_network_1);
   p_haematocrit_calculator_linear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator_linear->SetVesselNetwork(p_network_1);
   p_viscosity_calculator_linear->SetVesselNetwork(p_network_1);
   p_viscosity_calculator_linear->Calculate();
   p_impedance_calculator_linear->Calculate();

   p_haematocrit_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_haematocrit_calculator_nonlinear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_viscosity_calculator_nonlinear->SetVesselNetwork(p_network_2);
   p_viscosity_calculator_nonlinear->Calculate();
   p_impedance_calculator_nonlinear->Calculate();

   FlowSolver<2> flow_solver_linear;
   flow_solver_linear.SetVesselNetwork(p_network_1);
   flow_solver_linear.SetUp();

   FlowSolver<2> flow_solver_nonlinear;
   flow_solver_nonlinear.SetVesselNetwork(p_network_2);
   flow_solver_nonlinear.SetUp();

   unsigned max_iter = 1000;
   double tolerance2 = 1.e-5;

   std::vector<VesselSegmentPtr<2> > segments_linear = p_network_1->GetVesselSegments();
   std::vector<double> previous_haematocrit(segments_linear.size(), double(initial_haematocrit));
   // Clock for testing the speed of the algorithm
   clock_t begin = clock();
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator_linear->Calculate();
        p_impedance_calculator_linear->Calculate();
        flow_solver_linear.SetUp();
        flow_solver_linear.Solve();
        p_haematocrit_calculator_linear->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments_linear.size();jdx++)
        {
            double current_haematocrit = segments_linear[jdx]->GetFlowProperties()->GetHaematocrit();
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
            }
        }
   }
   clock_t end = clock();

   std::cout << "Time elapsed = " << 1.0e6*(end - begin)/CLOCKS_PER_SEC << std::endl;

   std::cout << "Sup Flow = " << flow_solver_linear.CheckSolution() << std::endl;
   std::cout << "Sup RBC = " << p_haematocrit_calculator_linear->CheckSolution() << std::endl;

   std::string file_suffix = "first_solution.vtp";
   std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
   p_network_1->Write(output_file);

   std::vector<VesselSegmentPtr<2> > segments_nonlinear = p_network_2->GetVesselSegments();
   std::fill(previous_haematocrit.begin(), previous_haematocrit.end(), initial_haematocrit);
   // Clock for testing the speed of the algorithm
   begin = clock();
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator_nonlinear->Calculate();
        p_impedance_calculator_nonlinear->Calculate();
        flow_solver_nonlinear.SetUp();
        flow_solver_nonlinear.Solve();
        p_haematocrit_calculator_nonlinear->Calculate();
        // Get the residual
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments_nonlinear.size();jdx++)
        {
            double current_haematocrit = segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit();
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
            }
        }
   }
   end = clock();

   std::cout << "Time elapsed = " << 1.0e6*(end - begin)/CLOCKS_PER_SEC << std::endl;

   std::cout << "Sup Flow = " << flow_solver_nonlinear.CheckSolution() << std::endl;
   std::cout << "Sup RBC = " << p_haematocrit_calculator_nonlinear->CheckSolution() << std::endl;

   file_suffix = "second_solution.vtp";
   output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
   p_network_2->Write(output_file);

   QDimensionless Max_Haematocrit_Difference = 0.0;
   QDimensionless Haematocrit_L2_Norm = 0.0;
   unsigned Max_index;
   double absolute_flow;

   // Calculating the L infinity and L2 norm for the two solutions
   for(unsigned jdx = 0; jdx < segments_linear.size(); jdx++)
   {
     absolute_flow += abs(segments_linear[jdx]->GetFlowProperties()->GetFlowRate() -
     segments_nonlinear[jdx]->GetFlowProperties()->GetFlowRate());
     if(Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() -
     segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit()) > Max_Haematocrit_Difference)
     {
       Max_Haematocrit_Difference = Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() -
       segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit());
       Max_index = jdx;
     }
     Haematocrit_L2_Norm = Haematocrit_L2_Norm + pow(Qabs(segments_linear[jdx]->GetFlowProperties()->GetHaematocrit() - segments_nonlinear[jdx]->GetFlowProperties()->GetHaematocrit()), 2.0);
   }
   Haematocrit_L2_Norm = pow(Haematocrit_L2_Norm, 0.5);

   std::cout << "Maximum haematocrit difference = " << Max_Haematocrit_Difference << " with vessel id = " << Max_index << std::endl;
   std::cout << "Haematocrit L2 Norm = " << Haematocrit_L2_Norm << std::endl;
}


void TestLinearMethodAsInitialConditions()
{
  // Test using the solution for the old method for solving the haematocrit as
  // the initial conditions for the new algorithm using dichotomous networks.
    // order of the dichotomous network
    unsigned order=5;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    {
	     dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    }
    QLength input_radius = 50_um;

    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda = 4.0;
   double twicelambda = 2.0*lambda;


   // Length of the vertical projection of first-order vessels
   QLength main_vert_length = 0.9*twicelambda*input_radius*pow(2.0,-1.0/3.0);
   // vertical size of the domain

   // horizontal size of the domain
   QLength domain_side_length_x = dimless_length*2.0*twicelambda*input_radius;
   auto p_file_handler = std::make_shared<OutputFileHandler>("Test_Linear_Method_As_Initial_Conditions", true);
   // Generate the network for testing both methods
   std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateForkingNetworkNoCorners(order, main_vert_length, input_radius, twicelambda);
   // identify input and output nodes and assign them properties

   VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
     Vertex<2>(0.0_um,2.0*main_vert_length));
   VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
     Vertex<2>(domain_side_length_x, 2.0*main_vert_length));
   p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

   // Setting up the solvers
   auto p_haematocrit_calculator_linear = PriesWithMemoryHaematocritSolver<2>::Create();
   auto p_haematocrit_calculator_nonlinear = PriesWithMemoryHaematocritSolverNonLinear<2>::Create();
   auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
   p_viscosity_calculator->SetPlasmaViscosity(viscosity);

   p_haematocrit_calculator_linear->SetVesselNetwork(p_network);
   p_haematocrit_calculator_linear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator->SetVesselNetwork(p_network);
   p_viscosity_calculator->SetVesselNetwork(p_network);
   p_viscosity_calculator->Calculate();
   p_impedance_calculator->Calculate();

   p_haematocrit_calculator_nonlinear->SetVesselNetwork(p_network);
   p_haematocrit_calculator_nonlinear->SetHaematocrit(inlet_haematocrit);

   FlowSolver<2> flow_solver;
   flow_solver.SetVesselNetwork(p_network);
   flow_solver.SetUp();

   unsigned max_iter = 1000;
   double tolerance2 = 1.e-10;

   std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
   std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator->Calculate();
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator_linear->Calculate();
        double total_flow = 0.0;
        double absolute_flow = 0.0;
        double total_red_blood = 0.0;
        for(unsigned jdx = 0; jdx < segments.size();jdx++)
        {
          total_flow += segments[jdx]->GetFlowProperties()->GetFlowRate();
          absolute_flow += abs(segments[jdx]->GetFlowProperties()->GetFlowRate());
          total_red_blood += segments[jdx]->GetFlowProperties()->GetFlowRate()*segments[jdx]->GetFlowProperties()->GetHaematocrit();
        }
        std::cout << "Total flow = " << total_flow << std::endl;
        std::cout << "Absolute flow = " << absolute_flow << std::endl;
        std::cout << "Total flow/Absolute flow = " << total_flow/absolute_flow << std::endl;
        std::cout << "Total Red blood = " << total_red_blood << std::endl;
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }

   std::vector<QDimensionless> input_haematocrit(segments.size(), 0.0);
   for(unsigned idx = 0; idx < segments.size(); idx++)
   {
     input_haematocrit[idx] = segments[idx]->GetFlowProperties()->GetHaematocrit();
   }
   std::fill(previous_haematocrit.begin(), previous_haematocrit.end(), initial_haematocrit);

   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator->Calculate();
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator_nonlinear->Calculate();
        double total_flow = 0.0;
        double absolute_flow = 0.0;
        double total_red_blood = 0.0;
        for(unsigned jdx = 0; jdx < segments.size();jdx++)
        {
          total_flow += segments[jdx]->GetFlowProperties()->GetFlowRate();
          absolute_flow += abs(segments[jdx]->GetFlowProperties()->GetFlowRate());
          total_red_blood += segments[jdx]->GetFlowProperties()->GetFlowRate()*segments[jdx]->GetFlowProperties()->GetHaematocrit();
        }
        std::cout << "Total flow = " << total_flow << std::endl;
        std::cout << "Absolute flow = " << absolute_flow << std::endl;
        std::cout << "Total flow/Absolute flow = " << total_flow/absolute_flow << std::endl;
        std::cout << "Total Red blood = " << total_red_blood << std::endl;
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }
   QDimensionless Max_Haematocrit_Difference = 0.0;
   QDimensionless Haematocrit_L2_Norm = 0.0;
   unsigned Max_index;

   // Calculating the L infinity norm and L2 norm for the two solutions
   for(unsigned jdx = 0; jdx < segments.size(); jdx++)
   {
     if(Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() - input_haematocrit[jdx]) > Max_Haematocrit_Difference)
     {
       Max_Haematocrit_Difference = Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() -
       input_haematocrit[jdx]);
       Max_index = jdx;
     }
     Haematocrit_L2_Norm = Haematocrit_L2_Norm + pow(Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() - input_haematocrit[jdx]), 2.0);
   }
   Haematocrit_L2_Norm = pow(Haematocrit_L2_Norm, 0.5);

   std::cout << "Maximum haematocrit difference = " << Max_Haematocrit_Difference << " with vessel id = " << Max_index << std::endl;
   std::cout << "Haematocrit L2 Norm = " << Haematocrit_L2_Norm << std::endl;

}


void TestNonLinearMethodAsInitialConditions()
{
  // Test using the solution for the new method for solving the haematocrit as
  // the initial conditions for the old method using dichotomous networks.
    // order of the dichotomous network
    unsigned order=5;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength input_radius = 50_um;

    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda = 4.0;
   double twicelambda = 8.0;


   // Length of the vertical projection of first-order vessels
   QLength main_vert_length = 0.9*twicelambda*input_radius*pow(2.0,-1.0/3.0);
   // vertical size of the domain

   std::ostringstream strs;
   strs << lambda;
   std::string str_lambda = strs.str();
   // horizontal size of the domain
   QLength domain_side_length_x = dimless_length*2.0*twicelambda*input_radius;
   auto p_file_handler = std::make_shared<OutputFileHandler>("Test_NonLinear_Method_As_Initial_Conditions", true);
   // Generating the network for testing the two methods.
   std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateForkingNetworkNoCorners(order, main_vert_length, input_radius, twicelambda);
   // identify input and output nodes and assign them properties
   VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
     Vertex<2>(0.0_um,2.0*main_vert_length));
   VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
     Vertex<2>(domain_side_length_x, 2.0*main_vert_length));
   p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
   p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
   p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
   p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

   // Setting up solvers for testing the methods
   auto p_haematocrit_calculator_linear = PriesWithMemoryHaematocritSolver<2>::Create();
   auto p_haematocrit_calculator_nonlinear = PriesWithMemoryHaematocritSolverNonLinear<2>::Create();
   auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
   auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
   p_viscosity_calculator->SetPlasmaViscosity(viscosity);

   p_haematocrit_calculator_linear->SetVesselNetwork(p_network);
   p_haematocrit_calculator_linear->SetHaematocrit(inlet_haematocrit);
   p_impedance_calculator->SetVesselNetwork(p_network);
   p_viscosity_calculator->SetVesselNetwork(p_network);
   p_viscosity_calculator->Calculate();
   p_impedance_calculator->Calculate();

   p_haematocrit_calculator_nonlinear->SetVesselNetwork(p_network);
   p_haematocrit_calculator_nonlinear->SetHaematocrit(inlet_haematocrit);

   FlowSolver<2> flow_solver;
   flow_solver.SetVesselNetwork(p_network);
   flow_solver.SetUp();

   unsigned max_iter = 1000;
   double tolerance2 = 1.e-10;

   std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
   std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator->Calculate();
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator_nonlinear->Calculate();
        double total_flow = 0.0;
        double absolute_flow = 0.0;
        double total_red_blood = 0.0;
        for(unsigned jdx = 0; jdx < segments.size();jdx++)
        {
          total_flow += segments[jdx]->GetFlowProperties()->GetFlowRate();
          absolute_flow += abs(segments[jdx]->GetFlowProperties()->GetFlowRate());
          total_red_blood += segments[jdx]->GetFlowProperties()->GetFlowRate()*segments[jdx]->GetFlowProperties()->GetHaematocrit();
        }
        std::cout << "Total flow = " << total_flow << std::endl;
        std::cout << "Absolute flow = " << absolute_flow << std::endl;
        std::cout << "Total flow/Absolute flow = " << total_flow/absolute_flow << std::endl;
        std::cout << "Total Red blood = " << total_red_blood << std::endl;
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }

   std::vector<QDimensionless> input_haematocrit(segments.size(), 0.0);
   for(unsigned idx = 0; idx < segments.size(); idx++)
   {
     input_haematocrit[idx] = segments[idx]->GetFlowProperties()->GetHaematocrit();
   }
   std::fill(previous_haematocrit.begin(), previous_haematocrit.end(), initial_haematocrit);

   // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
   for(unsigned idx=0;idx<max_iter;idx++)
   {
        p_viscosity_calculator->Calculate();
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_haematocrit_calculator_linear->Calculate();
        double total_flow = 0.0;
        double absolute_flow = 0.0;
        double total_red_blood = 0.0;
        for(unsigned jdx = 0; jdx < segments.size();jdx++)
        {
          total_flow += segments[jdx]->GetFlowProperties()->GetFlowRate();
          absolute_flow += abs(segments[jdx]->GetFlowProperties()->GetFlowRate());
          total_red_blood += segments[jdx]->GetFlowProperties()->GetFlowRate()*segments[jdx]->GetFlowProperties()->GetHaematocrit();
        }
        std::cout << "Total flow = " << total_flow << std::endl;
        std::cout << "Absolute flow = " << absolute_flow << std::endl;
        std::cout << "Total flow/Absolute flow = " << total_flow/absolute_flow << std::endl;
        std::cout << "Total Red blood = " << total_red_blood << std::endl;
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
            }
        }

        if(idx==max_iter-1)
        {
            EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
        }
   }
   QDimensionless Max_Haematocrit_Difference = 0.0;
   QDimensionless Haematocrit_L2_Norm = 0.0;
   unsigned Max_index;

   // Calculating the the L infinity and L2 norm for the two solutions
   for(unsigned jdx = 0; jdx < segments.size(); jdx++)
   {
     if(Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() - input_haematocrit[jdx]) > Max_Haematocrit_Difference)
     {
       Max_Haematocrit_Difference = Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() -
       input_haematocrit[jdx]);
       Max_index = jdx;
     }
     Haematocrit_L2_Norm = Haematocrit_L2_Norm + pow(Qabs(segments[jdx]->GetFlowProperties()->GetHaematocrit() - input_haematocrit[jdx]), 2.0);
   }
   Haematocrit_L2_Norm = pow(Haematocrit_L2_Norm, 0.5);

   std::cout << "Maximum haematocrit difference = " << Max_Haematocrit_Difference << " with vessel id = " << Max_index << std::endl;
   std::cout << "Haematocrit L2 Norm = " << Haematocrit_L2_Norm << std::endl;
}



};

#endif // TESTWITHORWITHOUTMEMORY_HPP
