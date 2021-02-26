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

#ifndef TESTTRIANGLENETWORK_HPP
#define TESTTRIANGLENETWORK_HPP

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
#include "GardnerHaematocritSolver.hpp"
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

class TestTriangleNetwork : public CxxTest::TestSuite
{


public:


void FindPressureDifferences(QDimensionless step_size)
{
	QDynamicViscosity viscosity = 1.0*unit::poiseuille;
	QDimensionless inlet_haematocrit = 0.6;
	QDimensionless max_difference, difference;
	QPressure lower_pressure, higher_pressure;
	
	QLength radius_length = 5.0;
	QLength vessel_length = 10000.0;
	unsigned k_max = 1.0/step_size +1;
	VesselNetworkGenerator<2> network_generator;
	QDimensionless radius_ratio = step_size;
   for (unsigned k_aux=1; k_aux<k_max; k_aux++)
   {
	std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateTriangleNetwork(vessel_length,radius_length,radius_ratio);
	std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::ostringstream strs;
    strs << "Triangle_network" << std::endl;

    std::string str_directory_name = strs.str();
    // horizontal size of the domain
    auto p_file_handler = std::make_shared<OutputFileHandler>(str_directory_name, true);

    // identify input and output nodes and assign them properties

    VesselNodePtr<2> p_inlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,Vertex<2>(0.0_um,0.0_um));
	VesselNodePtr<2> p_inlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,Vertex<2>(0.0_um,vessel_length));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,Vertex<2>(vessel_length*5.0/2.0, sqrt(3.0)*vessel_length/2.0));
    p_inlet_node_1->GetFlowProperties()->SetIsInputNode(true);
	p_inlet_node_2->GetFlowProperties()->SetIsInputNode(true);
	p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(0.0_Pa);

    //auto p_haematocrit_calculator = GardnerHaematocritSolver<2>::Create();
	auto p_haematocrit_calculator = PriesWithMemoryHaematocritSolver<2>::Create();
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);

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
    double tolerance2 = 1.e-8;

	segments[0]->GetFlowProperties()->SetHaematocrit(0.6);
	segments[1]->GetFlowProperties()->SetHaematocrit(0.6);
	segments[2]->GetFlowProperties()->SetHaematocrit(0.0);
	segments[3]->GetFlowProperties()->SetHaematocrit(0.0);
	segments[4]->GetFlowProperties()->SetHaematocrit(0.3);
	segments[5]->GetFlowProperties()->SetHaematocrit(0.0);
	
	QPressure lower_pressure = 1000_Pa;
	QPressure higher_pressure = 5000_Pa;

    std::vector<QDimensionless> previous_haematocrit(segments.size(), QDimensionless(0.0));
	std::vector<QDimensionless> current_haematocrit(segments.size(), QDimensionless(0.0));
	std::vector<QDimensionless> first_haematocrit(segments.size(), QDimensionless(0.0));
    // iteration to solve the nonlinear problem follows (haematocrit problem is coupled to the flow problem via viscosity/impedance)
	for(unsigned i=0;i < 1;i++)
	{
		QPressure new_pressure = (lower_pressure + higher_pressure)/2.0;
		p_inlet_node_1->GetFlowProperties()->SetPressure(new_pressure);
		p_inlet_node_1->GetFlowProperties()->SetPressure(10000.0 - new_pressure);
		for(unsigned idx=0;idx<max_iter;idx++)
    	{	
			p_viscosity_calculator->Calculate();
        	p_impedance_calculator->Calculate();
			flow_solver.Update();
        	flow_solver.Solve();
			for(unsigned jdx = 0; jdx < segments.size();jdx++){
				std::cout << "Haematocrit in vessel " << jdx << ": "<< segments[jdx]->GetFlowProperties()->GetHaematocrit() << std::endl;
				std::cout << "Impedance in vessel " << jdx << ": "<< segments[jdx]->GetFlowProperties()->GetImpedance() << std::endl;
				std::cout << "Flow in vessel " << jdx << ": "<< segments[jdx]->GetFlowProperties()->GetFlowRate() << std::endl;
			}
        	p_haematocrit_calculator->Calculate();
        	// Get the residual
        	unsigned max_difference_index = 0;
        	double max_difference = 0.0;
        	double h_for_max = 0.0;
        	double prev_for_max = 0.0;
        	for(unsigned jdx=0;jdx<segments.size();jdx++)
        	{
        	    double current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
				std::cout << "Previous haematocrit: " << previous_haematocrit[jdx] << std::endl;
				std::cout << "Current haematocrit: " << current_haematocrit <<std::endl;
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
        	if(max_difference<=tolerance2 && idx > 0)
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

		first_haematocrit[0] = current_haematocrit[0];
		first_haematocrit[1] = current_haematocrit[1]; 
		first_haematocrit[2] = current_haematocrit[2]; 
		first_haematocrit[3] = current_haematocrit[3]; 
		first_haematocrit[4] = current_haematocrit[4];
		first_haematocrit[5] = current_haematocrit[5];

		segments[0]->GetFlowProperties()->SetHaematocrit(0.6);
		segments[1]->GetFlowProperties()->SetHaematocrit(0.6);
		segments[2]->GetFlowProperties()->SetHaematocrit(0.0);
		segments[3]->GetFlowProperties()->SetHaematocrit(0.9);
		segments[4]->GetFlowProperties()->SetHaematocrit(0.0);
		segments[5]->GetFlowProperties()->SetHaematocrit(0.0);

		for(unsigned idx=0;idx<max_iter;idx++)
    	{	
        	p_impedance_calculator->Calculate();
        	flow_solver.SetUp();
        	flow_solver.Solve();
        	p_haematocrit_calculator->Calculate();
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
        	if(max_difference<=tolerance2 && idx > 0)
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

		double max_difference = 0.0;
		for(unsigned k=0;k<segments.size();k++)
        {	
			double current_haematocrit = segments[k]->GetFlowProperties()->GetHaematocrit();
			double previous_haematocrit = first_haematocrit[k];
            double difference = std::abs(current_haematocrit - previous_haematocrit);
            if(difference>max_difference)
            {
                max_difference = difference;
            }
        }
		if(max_difference > tolerance2)
		{
			higher_pressure = new_pressure;
		}
		else
		{
			lower_pressure = new_pressure;
		}

	}
	
	QPressure pressure_lower_bound = lower_pressure;

	segments[0]->GetFlowProperties()->SetHaematocrit(0.6);
	segments[1]->GetFlowProperties()->SetHaematocrit(0.6);
	segments[2]->GetFlowProperties()->SetHaematocrit(0.9);
	segments[3]->GetFlowProperties()->SetHaematocrit(0.0);
	segments[4]->GetFlowProperties()->SetHaematocrit(0.0);
	segments[5]->GetFlowProperties()->SetHaematocrit(0.0);
	
	
	lower_pressure = 5000_Pa;
	higher_pressure = 9000_Pa;

    for(unsigned i=0;i<1;i++)
	{
		QPressure new_pressure = (lower_pressure+higher_pressure)/2.0;
		p_inlet_node_1->GetFlowProperties()->SetPressure(new_pressure);
		p_inlet_node_1->GetFlowProperties()->SetPressure(10000.0 - new_pressure);
		for(unsigned idx=0;idx<max_iter;idx++)
    	{	
        	p_impedance_calculator->Calculate();
        	flow_solver.SetUp();
        	flow_solver.Solve();
        	p_haematocrit_calculator->Calculate();
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
        	if(max_difference<=tolerance2 && idx > 0)
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

		first_haematocrit[0] = current_haematocrit[0];
		first_haematocrit[1] = current_haematocrit[1]; 
		first_haematocrit[2] = current_haematocrit[2]; 
		first_haematocrit[3] = current_haematocrit[3]; 
		first_haematocrit[4] = current_haematocrit[4];
		first_haematocrit[5] = current_haematocrit[5];

		segments[0]->GetFlowProperties()->SetHaematocrit(0.6);
		segments[1]->GetFlowProperties()->SetHaematocrit(0.6);
		segments[2]->GetFlowProperties()->SetHaematocrit(0.0);
		segments[3]->GetFlowProperties()->SetHaematocrit(0.9);
		segments[4]->GetFlowProperties()->SetHaematocrit(0.0);
		segments[5]->GetFlowProperties()->SetHaematocrit(0.0);

		for(unsigned idx=0;idx<max_iter;idx++)
    	{	
        	p_impedance_calculator->Calculate();
        	flow_solver.SetUp();
        	flow_solver.Solve();
        	p_haematocrit_calculator->Calculate();
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
        	if(max_difference<=tolerance2 && idx > 0)
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

		double max_difference = 0.0;
		for(unsigned k=0;k<segments.size();k++)
        {	
			double current_haematocrit = segments[k]->GetFlowProperties()->GetHaematocrit();
			double previous_haematocrit = first_haematocrit[k];
            double difference = std::abs(current_haematocrit - previous_haematocrit);
            if(difference>max_difference)
            {
                max_difference = difference;
            }
        }
		if(max_difference > tolerance2)
		{
			lower_pressure = new_pressure;
		}
		else
		{
			higher_pressure = new_pressure;
		}
		
		
	}
	
	QPressure pressure_higher_bound = higher_pressure;

	std::cout<< radius_ratio << "," << QPressure(std::abs(pressure_higher_bound - pressure_lower_bound)) << std::endl;
	radius_ratio = radius_ratio + step_size;



    }
}

    void TestTriangleNetworksWithDifferentSteps()
    {
		FindPressureDifferences(1.0);
    }

};

#endif // TESTTRIANGLENETWORK_HPP