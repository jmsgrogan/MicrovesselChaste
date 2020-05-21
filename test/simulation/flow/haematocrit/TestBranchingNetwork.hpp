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

class TestBranchingNetwork : public CxxTest::TestSuite
{


public:
    /** The following is to test that the Pries (without memory) lambda=4 figure can be faithfully reproduced.
        See bottom of RunNoCellsDichotomousWithOrWithoutMemoryEffects() for where this is called.*/

void TestBranchingNetworkStructure()
{
    // order of the dichotomous network
    unsigned order=2;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux-1)));
    	}
    QLength input_radius = 100_um;

    QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;

   double lambda;
   double twicelambda;

  lambda = 10.0;
  // lambda is quotient between the length and diameter...in vessel network generator, we use twice this value as an input parameter
  twicelambda = 2.0*lambda;


    // Length of the vertical projection of first-order vessels
    double main_vert_length = 0.9*twicelambda*input_radius*pow(2.0,-1.0/3.0);

    std::ostringstream strs;
    strs << "Branching_Network" << lambda;

    std::string str_directory_name = strs.str();
    // horizontal size of the domain
    QLength domain_side_length_x = dimless_length*2.0*twicelambda*input_radius;
    auto p_file_handler = std::make_shared<OutputFileHandler>(str_directory_name, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateBranchingNetwork(order, main_vert_length, input_radius, twicelambda);

    // identify input and output nodes and assign them properties

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,2.0*main_vert_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);

    VesselNodePtr<2> p_outlet_node;

    for(int i = 0; i <= int(pow(2.0, order+1)); i++)
    {
      QLength vessel_position = float(i)*main_vert_length*pow(2.0, -float(order+1)+2.0);
      p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,Vertex<2>(domain_side_length_x, vessel_position));
      p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
      p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);
    }



    // Switch between solvers for Pries or newer "with memory"
    std::shared_ptr<AbstractHaematocritSolver<2>> p_abs_haematocrit_calculator;

    auto p_haematocrit_calculator = PriesHaematocritSolver<2>::Create();
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_abs_haematocrit_calculator = p_haematocrit_calculator;

    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-5;
//
    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.SetUp();
        flow_solver.Solve();
        p_abs_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        unsigned max_difference_index = 0;
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
                max_difference_index = jdx;
            }
            previous_haematocrit[jdx] = current_haematocrit;
        }
        std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << " at index " << max_difference_index << std::endl;
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

    std::cout << "Sup flow =" << flow_solver.CheckSolution() << std::endl;
    std::cout << "Sup RBC = " << p_abs_haematocrit_calculator->CheckSolution() << std::endl;

    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);
    TS_ASSERT_DELTA(0.45, 0.45, 1e-6);
    std::vector<std::shared_ptr<Vessel<2> > > network_vessels =  p_network->GetVessels();
    for(unsigned k = 0; k < p_network->GetNumberOfVessels(); k++)
    {
      std::cout << "Vessel index = " << k << std::endl;
      std::cout << "Vessel length = " << network_vessels[k]->GetLength() << std::endl;
      std::cout << "Vessel radius = " << network_vessels[k]->GetRadius() << std::endl;
      std::cout << "Vessel haematocrit = " << network_vessels[k]->GetFlowProperties()->GetHaematocrit() << std::endl;
    }

}

};

#endif // TESTWITHORWITHOUTMEMORY_HPP
