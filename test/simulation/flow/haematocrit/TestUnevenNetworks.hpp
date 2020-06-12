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
#include "VesselImpedanceCalculator.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "SimulationTime.hpp"
#include "PriesHaematocritSolver.hpp"
#include "PriesWithMemoryHaematocritSolver.hpp"
#include "PriesWithMemoryHaematocritSolverNonLinear.hpp"
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

class TestUnevenNetworks : public CxxTest::TestSuite
{

public:

// Some constants used by all tests.
const double alpha = 2.0;
const double y_scale = 32.0;
// const unsigned order = 4;


// void TestHexagonalNetworkNonLinear()
// {
//     // This test is for testing the new haematocrit solver on hexagonal networks
//     double dimless_length = 1.0;
//
//     for(unsigned i_aux=1; i_aux<order+1; i_aux++)
//     {
// 	     dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
//     }
//     QLength input_radius = 50_um;
//
//     QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
//
//     double inlet_haematocrit = 0.45;
//     double initial_haematocrit = 0.45;
//
//     // Generate the network
//
//     VesselNetworkGenerator<2> network_generator;
//
//     // alpha is quotient between the length and diameter...in vessel network generator, we use twice this value as an input parameter
//     double twicealpha = 2.0*alpha;
//
//
//     // Length of the vertical projection of first-order vessels
//     QLength main_vert_length = 0.9*twicealpha*input_radius*pow(2.0,-1.0/3.0);
//     // vertical size of the domain
//     QLength domain_side_length_y = y_scale*main_vert_length;
//
//     std::ostringstream strs;
//     strs << alpha;
//     std::string str_alpha = strs.str();
//     // horizontal size of the domain
//     QLength domain_side_length_x = dimless_length*2.0*twicealpha*input_radius;
//     auto p_file_handler = std::make_shared<OutputFileHandler>("HexagonalNetwork_Solved_Nonlinear", true);
//     // Generating the hexagonal network
//     std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(domain_side_length_x, domain_side_length_y, main_vert_length);
//
//     unsigned max_height = y_scale;
//
//     // Setting the inlet and outlet nodes for the network
//     for(unsigned idx = 0; idx <= max_height; idx++)
//     {
//       double scale = idx;
//       VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network, Vertex<2>(0.0_um, scale*main_vert_length));
//       VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network, Vertex<2>(domain_side_length_x, scale*main_vert_length));
//       p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
//       p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
//       p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
//       p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);
//     }
//
//     //    auto p_haematocrit_calculator = PriesWithMemoryHaematocritSolver<2>::Create();
//     auto p_haematocrit_calculator = PriesWithMemoryHaematocritSolverNonLinear<2>::Create();
//     auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
//     auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
//     p_viscosity_calculator->SetPlasmaViscosity(viscosity);
//
//     p_haematocrit_calculator->SetVesselNetwork(p_network);
//     p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
//     p_impedance_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->Calculate();
//     p_impedance_calculator->Calculate();
//
//     FlowSolver<2> flow_solver;
//     flow_solver.SetVesselNetwork(p_network);
//     flow_solver.SetUp();
//
//     unsigned max_iter = 1000;
//     double tolerance2 = 1.e-10;
//
//     std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
//     std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
//     // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
//     for(unsigned idx=0;idx<max_iter;idx++)
//     {
//         p_viscosity_calculator->Calculate();
//         p_impedance_calculator->Calculate();
//         flow_solver.SetUp();
//         flow_solver.Solve();
//         p_haematocrit_calculator->Calculate();
//         // Get the residual
//         double max_difference = 0.0;
//         double h_for_max = 0.0;
//         double prev_for_max = 0.0;
//         for(unsigned jdx=0;jdx<segments.size();jdx++)
//         {
//             double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
//             double difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
//             if(difference>max_difference)
//             {
//                 max_difference = difference;
//                 h_for_max = current_haematocrit;
//                 prev_for_max = previous_haematocrit[jdx];
//             }
//             previous_haematocrit[jdx] = current_haematocrit;
//         }
//         std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << std::endl;
//         if(max_difference<=tolerance2)
//         {
//             std::cout << "Converged after: " << idx << " iterations. " <<  std::endl;
//             break;
//         }
//         else
//         {
//             // Output intermediate results
//             if(idx%1==0)
//             {
//                 std::cout << "Max Difference at iter: " << idx << " is " << max_difference << std::endl;
//                 std::string file_suffix = "IntermediateHaematocrit_" + std::to_string(idx) + ".vtp";
//                 std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append(file_suffix);
//                 p_network->Write(output_file);
//             }
//         }
//
//         if(idx==max_iter-1)
//         {
//             EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
//         }
//     }
//     std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
//     p_network->Write(output_file);
// }


void TestHexagonalNetworkLinear()
{
  // for(unsigned multiplier = 32; multiplier < 33; multiplier++)
  for(unsigned order = 4; order < 33; order++)
  {
  // double y_scale = multiplier*1.0;
  // This test is for testing the new haematocrit solver on hexagonal networks
  double dimless_length = 1.0;

  for(unsigned i_aux=1; i_aux<order+1; i_aux++)
  {
     dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
  }
  QLength input_radius = 50_um;

  QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

  double inlet_haematocrit = 0.45;

  // Generate the network

  VesselNetworkGenerator<2> network_generator;

  // alpha is quotient between the length and diameter...in vessel network generator, we use twice this value as an input parameter
  double twicealpha = 2.0*alpha;


  // Length of the vertical projection of first-order vessels
  QLength main_vert_length = 0.9*twicealpha*input_radius*pow(2.0,-1.0/3.0);
  // vertical size of the domain
  QLength domain_side_length_y = y_scale*main_vert_length;

  std::ostringstream strs;
  strs << alpha;
  std::string str_alpha = strs.str();
  // horizontal size of the domain
  QLength domain_side_length_x = dimless_length*2.0*twicealpha*input_radius;
  // for model without memory effects, use:
  auto p_file_handler = std::make_shared<OutputFileHandler>("HexagonalNetwork_Solved_Linear_Single", true);
  // Generating the hexagonal network
  std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(domain_side_length_x, domain_side_length_y, main_vert_length);

  // Setting the inlet and outlet nodes for the network
  unsigned max_height = y_scale;

  for(unsigned idx = 0; idx <= max_height; idx = idx + 2)
  {
    double scale = idx;
    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network, Vertex<2>(0.0_um, scale*main_vert_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network, Vertex<2>(domain_side_length_x, scale*main_vert_length));

    if(p_inlet_node->GetNumberOfSegments() == 1 && idx != 0)
    {
      p_network->RemoveVessel(p_inlet_node->GetSegment(0)->GetVessel(), true);
    }
    else if(p_inlet_node->GetNumberOfSegments() == 1)
    {
      p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
      p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    }

    if(p_outlet_node->GetNumberOfSegments() == 1 && idx != max_height)
    {
      VesselNodePtr<2> p_connecting_node = p_outlet_node->GetSegment(0)->GetVessel()->GetStartNode();
      if(p_connecting_node == p_outlet_node)
      {
        p_connecting_node = p_outlet_node->GetSegment(0)->GetVessel()->GetEndNode();
      }
      if(p_connecting_node->GetNumberOfSegments() < 3)
      {
        p_network->RemoveVessel(p_connecting_node->GetSegment(0)->GetVessel(), true);
      }
      p_network->RemoveVessel(p_outlet_node->GetSegment(0)->GetVessel(), true);
    }
    else if(p_outlet_node->GetNumberOfSegments() == 1)
    {
      p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
      p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);
    }
  }


  auto p_haematocrit_calculator = PriesWithMemoryHaematocritSolver<2>::Create();
  auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
  auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
  p_viscosity_calculator->SetPlasmaViscosity(viscosity);

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
  std::vector<double> previous_haematocrit(segments.size(), double(inlet_haematocrit));
  // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
  for(unsigned idx=0;idx<max_iter;idx++)
  {
      // std::cout << "Calculating Viscosity" << std::endl;
      p_viscosity_calculator->Calculate();
      // std::cout << "Calculating impedance" << std::endl;
      p_impedance_calculator->Calculate();
      // std::cout << "Calculating flow" << std::endl;
      flow_solver.Update();
      flow_solver.Solve();
      // std::cout << "Calculating haematocrit" << std::endl;
      p_haematocrit_calculator->Calculate();
      double total_flow = 0.0;
      double total_red_blood = 0.0;
      for(unsigned jdx = 0; jdx < segments.size();jdx++)
      {
        total_flow += segments[jdx]->GetFlowProperties()->GetFlowRate();
        total_red_blood += segments[jdx]->GetFlowProperties()->GetFlowRate()*segments[jdx]->GetFlowProperties()->GetHaematocrit();
      }
      std::cout << "y_scale = " << y_scale << std::endl;
      std::cout << "order = " << order << std::endl;
      std::cout << "Sup flow = " << flow_solver.CheckSolution() << std::endl;
      std::cout << "Sup Red blood = " << p_haematocrit_calculator->CheckSolution() << std::endl;
      std::cout << "Average Haematocrit = " << p_haematocrit_calculator->AverageHaematocrit() << std::endl;
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
  std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
  p_network->Write(output_file);
  }
}


};

#endif // TESTWITHORWITHOUTMEMORY_HPP
