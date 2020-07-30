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
#include "GardnerSimplifiedHaematocritSolver.hpp"
#include "PriesHaematocritSolver.hpp"
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

class TestNetworkComparison : public CxxTest::TestSuite
{


public:

void TestRegularToForkedBifurcationUnit()
{
    QLength input_radius = 50_um;
    QLength vessel_length = 1000_um;
    QDimensionless alpha = 0.75;

    // QDynamicViscosity viscosity = 1.0*unit::poiseuille;
    QDynamicViscosity viscosity = 1.0;

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    // Generate the network

    VesselNetworkGenerator<2> network_generator;


    // Length of the vertical projection of first-order vessels
    QLength domain_side_length_x = vessel_length*(2.0 + 1.0/sqrt(2.0));
    // vertical size of the domain
    // QLength domain_side_length_y = (1.0+1.0/sqrt(2.0)) * vessel_length;

    // horizontal size of the domain
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestRegularToForkedBifurcationUnit", true);

    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateTrueForkedBifurcationUnit(vessel_length, input_radius, alpha);

    // identify input and output nodes and assign them properties

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um,vessel_length));
    VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, (1.0-1.0/sqrt(2.0)) * vessel_length));
    VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, (1.0+1.0/sqrt(2.0)) * vessel_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(1.0_Pa);
    p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node_1->GetFlowProperties()->SetPressure(0.0_Pa);
    p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node_2->GetFlowProperties()->SetPressure(0.0_Pa);


    auto p_haematocrit_calculator = GardnerSimplifiedHaematocritSolver<2>::Create();
    // auto p_haematocrit_calculator = PriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    segments[0]->GetFlowProperties()->SetHaematocrit(0.45);
    segments[1]->GetFlowProperties()->SetHaematocrit(0.44440140856573);
    segments[2]->GetFlowProperties()->SetHaematocrit(0.451370374619631);
    segments[3]->GetFlowProperties()->SetHaematocrit(0.44440140856573);
    segments[4]->GetFlowProperties()->SetHaematocrit(0.451370374619631);
    std::cout << "TestRegularToForkedBifurcationUnit" << std::endl;
    std::cout << "segment " << 0 << " radius = " << segments[0]->GetRadius() << std::endl;
    std::cout << "segment " << 1 << " radius = " << segments[1]->GetRadius() << std::endl;
    std::cout << "segment " << 2 << " radius = " << segments[2]->GetRadius() << std::endl;
    std::cout << "segment " << 3 << " radius = " << segments[3]->GetRadius() << std::endl;
    std::cout << "segment " << 4 << " radius = " << segments[4]->GetRadius() << std::endl;
    std::cout << "segment " << 0 << " length = " << segments[0]->GetLength() << std::endl;
    std::cout << "segment " << 1 << " length = " << segments[1]->GetLength() << std::endl;
    std::cout << "segment " << 2 << " length = " << segments[2]->GetLength() << std::endl;
    std::cout << "segment " << 3 << " length = " << segments[3]->GetLength() << std::endl;
    std::cout << "segment " << 4 << " length = " << segments[4]->GetLength() << std::endl;

    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->Calculate();
    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUseDirectSolver(false);
    flow_solver.SetUp();
    flow_solver.Update();
    flow_solver.Solve();

    std::vector<std::shared_ptr<VesselNode<2> > > nodes = p_network->GetNodes();

    double initial_pressure_0 = nodes[0]->GetFlowProperties()->GetPressure();
    double initial_pressure_1 = nodes[1]->GetFlowProperties()->GetPressure();
    double initial_pressure_2 = nodes[2]->GetFlowProperties()->GetPressure();
    double initial_pressure_3 = nodes[3]->GetFlowProperties()->GetPressure();
    std::cout << "node " << 0 << " pressure = " << initial_pressure_0 << std::endl;
    std::cout << "node " << 1 << " pressure = " << initial_pressure_1 << std::endl;
    std::cout << "node " << 2 << " pressure = " << initial_pressure_2 << std::endl;
    std::cout << "node " << 3 << " pressure = " << initial_pressure_3 << std::endl;

    double initial_flow_12 = segments[0]->GetFlowProperties()->GetFlowRate();
    double initial_flow_241 = segments[1]->GetFlowProperties()->GetFlowRate();
    double initial_flow_231 = segments[2]->GetFlowProperties()->GetFlowRate();
    double initial_flow_242 = segments[3]->GetFlowProperties()->GetFlowRate();
    double initial_flow_232 = segments[4]->GetFlowProperties()->GetFlowRate();
    std::cout << "vessel (1,2) flow = " << initial_flow_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 flow = " << initial_flow_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 flow = " << initial_flow_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 flow = " << initial_flow_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 flow = " << initial_flow_232 << std::endl;

    double initial_impedance_12 = segments[0]->GetFlowProperties()->GetImpedance();
    double initial_impedance_241 = segments[1]->GetFlowProperties()->GetImpedance();
    double initial_impedance_231 = segments[2]->GetFlowProperties()->GetImpedance();
    double initial_impedance_242 = segments[3]->GetFlowProperties()->GetImpedance();
    double initial_impedance_232 = segments[4]->GetFlowProperties()->GetImpedance();
    std::cout << "vessel (1,2) impedance reciprocal = " << 1.0/initial_impedance_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 impedance reciprocal = " << 1.0/initial_flow_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 impedance reciprocal = " << 1.0/initial_impedance_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 impedance reciprocal = " << 1.0/initial_impedance_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 impedance reciprocal = " << 1.0/initial_impedance_232 << std::endl;

    double initial_viscosity_12 = segments[0]->GetFlowProperties()->GetViscosity();
    double initial_viscosity_241 = segments[1]->GetFlowProperties()->GetViscosity();
    double initial_viscosity_231 = segments[2]->GetFlowProperties()->GetViscosity();
    double initial_viscosity_242 = segments[3]->GetFlowProperties()->GetViscosity();
    double initial_viscosity_232 = segments[4]->GetFlowProperties()->GetViscosity();
    std::cout << "vessel (1,2) viscosity = " << initial_viscosity_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 viscosity = " << initial_viscosity_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 viscosity = " << initial_viscosity_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 viscosity = " << initial_viscosity_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 viscosity = " << initial_viscosity_232 << std::endl;

    double initial_Haematocrit_12 = segments[0]->GetFlowProperties()->GetHaematocrit();
    double initial_Haematocrit_241 = segments[1]->GetFlowProperties()->GetHaematocrit();
    double initial_Haematocrit_231 = segments[2]->GetFlowProperties()->GetHaematocrit();
    double initial_Haematocrit_242 = segments[3]->GetFlowProperties()->GetHaematocrit();
    double initial_Haematocrit_232 = segments[4]->GetFlowProperties()->GetHaematocrit();
    std::cout << "vessel (1,2) Haematocrit = " << initial_Haematocrit_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 Haematocrit = " << initial_Haematocrit_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 Haematocrit = " << initial_Haematocrit_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 Haematocrit = " << initial_Haematocrit_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 Haematocrit = " << initial_Haematocrit_232 << std::endl;

    double initial_flow_conservation = flow_solver.CheckSolution();
    double initial_haematocrit_conservation = p_haematocrit_calculator->CheckSolution();
    std::cout << "Sup flow =" << initial_flow_conservation << std::endl;
    std::cout << "Sup RBC = " << initial_haematocrit_conservation << std::endl;



    unsigned max_iter = 1000;
    double tolerance2 = 1.e-5;
    std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("InitialHaematocrit.vtp");
    p_network->Write(output_file);
    std::vector<QDimensionless> previous_haematocrit(segments.size(), QDimensionless(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        p_impedance_calculator->Calculate();
        flow_solver.Update();
        flow_solver.Solve();
        std::cout << "node " << 0 << " pressure = " << nodes[0]->GetFlowProperties()->GetPressure() << std::endl;
        std::cout << "node " << 1 << " pressure = " << nodes[1]->GetFlowProperties()->GetPressure() << std::endl;
        std::cout << "node " << 2 << " pressure = " << nodes[2]->GetFlowProperties()->GetPressure() << std::endl;
        std::cout << "node " << 3 << " pressure = " << nodes[3]->GetFlowProperties()->GetPressure() << std::endl;
        std::cout << "vessel (1,2) flow = " << segments[0]->GetFlowProperties()->GetFlowRate() << std::endl;
        std::cout << "vessel (2,4) segment 1 flow = " << segments[1]->GetFlowProperties()->GetFlowRate() << std::endl;
        std::cout << "vessel (2,3) segment 1 flow = " << segments[2]->GetFlowProperties()->GetFlowRate() << std::endl;
        std::cout << "vessel (2,4) segment 2 flow = " << segments[3]->GetFlowProperties()->GetFlowRate() << std::endl;
        std::cout << "vessel (2,3) segment 2 flow = " << segments[4]->GetFlowProperties()->GetFlowRate() << std::endl;
        std::cout << "vessel (1,2) impedance reciprocal = " << 1.0/segments[0]->GetFlowProperties()->GetImpedance() << std::endl;
        std::cout << "vessel (2,4) segment 1 impedance reciprocal = " << 1.0/segments[1]->GetFlowProperties()->GetImpedance() << std::endl;
        std::cout << "vessel (2,3) segment 1 impedance reciprocal = " << 1.0/segments[2]->GetFlowProperties()->GetImpedance() << std::endl;
        std::cout << "vessel (2,4) segment 2 impedance reciprocal = " << 1.0/segments[3]->GetFlowProperties()->GetImpedance() << std::endl;
        std::cout << "vessel (2,3) segment 2 impedance reciprocal = " << 1.0/segments[4]->GetFlowProperties()->GetImpedance() << std::endl;
        std::cout << "vessel (1,2) viscosity = " << segments[0]->GetFlowProperties()->GetViscosity() << std::endl;
        std::cout << "vessel (2,4) segment 1 viscosity = " << segments[1]->GetFlowProperties()->GetViscosity() << std::endl;
        std::cout << "vessel (2,3) segment 1 viscosity = " << segments[2]->GetFlowProperties()->GetViscosity() << std::endl;
        std::cout << "vessel (2,4) segment 2 viscosity = " << segments[3]->GetFlowProperties()->GetViscosity() << std::endl;
        std::cout << "vessel (2,3) segment 2 viscosity = " << segments[4]->GetFlowProperties()->GetViscosity() << std::endl;
        std::cout << "vessel (1,2) Haematocrit = " << segments[0]->GetFlowProperties()->GetHaematocrit() << std::endl;
        std::cout << "vessel (2,4) segment 1 Haematocrit = " << segments[1]->GetFlowProperties()->GetHaematocrit() << std::endl;
        std::cout << "vessel (2,3) segment 1 Haematocrit = " << segments[2]->GetFlowProperties()->GetHaematocrit() << std::endl;
        std::cout << "vessel (2,4) segment 2 Haematocrit = " << segments[3]->GetFlowProperties()->GetHaematocrit() << std::endl;
        std::cout << "vessel (2,3) segment 2 Haematocrit = " << segments[4]->GetFlowProperties()->GetHaematocrit() << std::endl;
        std::cout << "Sup flow =" << flow_solver.CheckSolution() << std::endl;
        std::cout << "Sup RBC = " << p_haematocrit_calculator->CheckSolution() << std::endl;
        p_haematocrit_calculator->Calculate();
        p_viscosity_calculator->Calculate();
        // Get the residual
        unsigned max_difference_index = 0;
        double max_difference = 0.0;
        double h_for_max = 0.0;
        double prev_for_max = 0.0;
        for(unsigned jdx=0;jdx<segments.size();jdx++)
        {
            QDimensionless current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
            QDimensionless difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
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
        if(max_difference<=tolerance2 && idx > 1)
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

    double end_pressure_0 = nodes[0]->GetFlowProperties()->GetPressure();
    double end_pressure_1 = nodes[1]->GetFlowProperties()->GetPressure();
    double end_pressure_2 = nodes[2]->GetFlowProperties()->GetPressure();
    double end_pressure_3 = nodes[3]->GetFlowProperties()->GetPressure();
    std::cout << "node " << 0 << " pressure = " << end_pressure_0 << std::endl;
    std::cout << "node " << 1 << " pressure = " << end_pressure_1 << std::endl;
    std::cout << "node " << 2 << " pressure = " << end_pressure_2 << std::endl;
    std::cout << "node " << 3 << " pressure = " << end_pressure_3 << std::endl;

    double end_flow_12 = segments[0]->GetFlowProperties()->GetFlowRate();
    double end_flow_241 = segments[1]->GetFlowProperties()->GetFlowRate();
    double end_flow_231 = segments[2]->GetFlowProperties()->GetFlowRate();
    double end_flow_242 = segments[3]->GetFlowProperties()->GetFlowRate();
    double end_flow_232 = segments[4]->GetFlowProperties()->GetFlowRate();
    std::cout << "vessel (1,2) flow = " << end_flow_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 flow = " << end_flow_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 flow = " << end_flow_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 flow = " << end_flow_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 flow = " << end_flow_232 << std::endl;

    double end_impedance_12 = segments[0]->GetFlowProperties()->GetImpedance();
    double end_impedance_241 = segments[1]->GetFlowProperties()->GetImpedance();
    double end_impedance_231 = segments[2]->GetFlowProperties()->GetImpedance();
    double end_impedance_242 = segments[3]->GetFlowProperties()->GetImpedance();
    double end_impedance_232 = segments[4]->GetFlowProperties()->GetImpedance();
    std::cout << "vessel (1,2) impedance reciprocal = " << 1.0/end_impedance_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 impedance reciprocal = " << 1.0/end_flow_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 impedance reciprocal = " << 1.0/end_impedance_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 impedance reciprocal = " << 1.0/end_impedance_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 impedance reciprocal = " << 1.0/end_impedance_232 << std::endl;

    double end_viscosity_12 = segments[0]->GetFlowProperties()->GetViscosity();
    double end_viscosity_241 = segments[1]->GetFlowProperties()->GetViscosity();
    double end_viscosity_231 = segments[2]->GetFlowProperties()->GetViscosity();
    double end_viscosity_242 = segments[3]->GetFlowProperties()->GetViscosity();
    double end_viscosity_232 = segments[4]->GetFlowProperties()->GetViscosity();
    std::cout << "vessel (1,2) viscosity = " << end_viscosity_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 viscosity = " << end_viscosity_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 viscosity = " << end_viscosity_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 viscosity = " << end_viscosity_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 viscosity = " << end_viscosity_232 << std::endl;

    double end_Haematocrit_12 = segments[0]->GetFlowProperties()->GetHaematocrit();
    double end_Haematocrit_241 = segments[1]->GetFlowProperties()->GetHaematocrit();
    double end_Haematocrit_231 = segments[2]->GetFlowProperties()->GetHaematocrit();
    double end_Haematocrit_242 = segments[3]->GetFlowProperties()->GetHaematocrit();
    double end_Haematocrit_232 = segments[4]->GetFlowProperties()->GetHaematocrit();
    std::cout << "vessel (1,2) Haematocrit = " << end_Haematocrit_12 << std::endl;
    std::cout << "vessel (2,4) segment 1 Haematocrit = " << end_Haematocrit_241 << std::endl;
    std::cout << "vessel (2,3) segment 1 Haematocrit = " << end_Haematocrit_231 << std::endl;
    std::cout << "vessel (2,4) segment 2 Haematocrit = " << end_Haematocrit_242 << std::endl;
    std::cout << "vessel (2,3) segment 2 Haematocrit = " << end_Haematocrit_232 << std::endl;

    double end_flow_conservation = flow_solver.CheckSolution();
    double end_haematocrit_conservation = p_haematocrit_calculator->CheckSolution();
    std::cout << "Sup flow =" << end_flow_conservation << std::endl;
    std::cout << "Sup RBC = " << end_haematocrit_conservation << std::endl;

    std::cout << "node " << 0 << " normalised pressure difference = " << abs(initial_pressure_0 - end_pressure_0)/(abs(initial_pressure_0) + abs(end_pressure_0)) << std::endl;
    std::cout << "node " << 1 << " normalised pressure difference = " << abs(initial_pressure_1 - end_pressure_1)/(abs(initial_pressure_1) + abs(end_pressure_1)) << std::endl;
    std::cout << "node " << 2 << " normalised pressure difference = " << abs(initial_pressure_2 - end_pressure_2)/(abs(initial_pressure_2) + abs(end_pressure_2)) << std::endl;
    std::cout << "node " << 3 << " normalised pressure difference = " << abs(initial_pressure_3 - end_pressure_3)/(abs(initial_pressure_3) + abs(end_pressure_3)) << std::endl;
    std::cout << "vessel (1,2) normalised flow difference = " << abs(initial_flow_12 - end_flow_12)/(abs(initial_flow_12) + abs(end_flow_12)) << std::endl;
    std::cout << "vessel (2,4) segment 1 normalised flow difference = " << abs(initial_flow_241 - end_flow_241)/(abs(initial_flow_241) + abs(end_flow_241)) << std::endl;
    std::cout << "vessel (2,3) segment 1 normalised flow difference = " << abs(initial_flow_231 - end_flow_231)/(abs(initial_flow_231) + abs(end_flow_231)) << std::endl;
    std::cout << "vessel (2,4) segment 2 normalised flow difference = " << abs(initial_flow_242 - end_flow_242)/(abs(initial_flow_242) + abs(end_flow_242)) << std::endl;
    std::cout << "vessel (2,3) segment 2 normalised flow difference = " << abs(initial_flow_232 - end_flow_232)/(abs(initial_flow_232) + abs(end_flow_232)) << std::endl;
    std::cout << "vessel (1,2) normalised impedance reciprocal difference = " << abs(initial_impedance_12 - end_impedance_12)/(abs(initial_impedance_12) + abs(end_impedance_12)) << std::endl;
    std::cout << "vessel (2,4) segment 1 normalised impedance reciprocal difference = " << abs(initial_impedance_241 - end_impedance_241)/(abs(initial_impedance_241) + abs(end_impedance_241)) << std::endl;
    std::cout << "vessel (2,3) segment 1 normalised impedance reciprocal difference = " << abs(initial_impedance_231 - end_impedance_231)/(abs(initial_impedance_231) + abs(end_impedance_231)) << std::endl;
    std::cout << "vessel (2,4) segment 2 normalised impedance reciprocal difference = " << abs(initial_impedance_242 - end_impedance_242)/(abs(initial_impedance_242) + abs(end_impedance_242)) << std::endl;
    std::cout << "vessel (2,3) segment 2 normalised impedance reciprocal difference = " << abs(initial_impedance_232 - end_impedance_232)/(abs(initial_impedance_232) + abs(end_impedance_232)) << std::endl;
    std::cout << "vessel (1,2) normalised viscosity difference = " << abs(initial_viscosity_12 - end_viscosity_12)/(abs(initial_viscosity_12) + abs(end_viscosity_12)) << std::endl;
    std::cout << "vessel (2,4) segment 1 normalised viscosity difference = " << abs(initial_viscosity_241 - end_viscosity_241)/(abs(initial_viscosity_241) + abs(end_viscosity_241)) << std::endl;
    std::cout << "vessel (2,3) segment 1 normalised viscosity difference = " << abs(initial_viscosity_231 - end_viscosity_231)/(abs(initial_viscosity_231) + abs(end_viscosity_231)) << std::endl;
    std::cout << "vessel (2,4) segment 2 normalised viscosity difference = " << abs(initial_viscosity_242 - end_viscosity_242)/(abs(initial_viscosity_242) + abs(end_viscosity_242)) << std::endl;
    std::cout << "vessel (2,3) segment 2 normalised viscosity difference = " << abs(initial_viscosity_232 - end_viscosity_232)/(abs(initial_viscosity_232) + abs(end_viscosity_232)) << std::endl;
    std::cout << "vessel (1,2) normalised Haematocrit difference = " << abs(initial_Haematocrit_12 - end_Haematocrit_12)/(abs(initial_Haematocrit_12) + abs(end_Haematocrit_12)) << std::endl;
    std::cout << "vessel (2,4) segment 1 normalised Haematocrit difference = " << abs(initial_Haematocrit_241 - end_Haematocrit_241)/(abs(initial_Haematocrit_241) + abs(end_Haematocrit_241)) << std::endl;
    std::cout << "vessel (2,3) segment 1 normalised Haematocrit difference = " << abs(initial_Haematocrit_231 - end_Haematocrit_231)/(abs(initial_Haematocrit_231) + abs(end_Haematocrit_231)) << std::endl;
    std::cout << "vessel (2,4) segment 2 normalised Haematocrit difference = " << abs(initial_Haematocrit_242 - end_Haematocrit_242)/(abs(initial_Haematocrit_242) + abs(end_Haematocrit_242)) << std::endl;
    std::cout << "vessel (2,3) segment 2 normalised Haematocrit difference = " << abs(initial_Haematocrit_232 - end_Haematocrit_232)/(abs(initial_Haematocrit_232) + abs(end_Haematocrit_232)) << std::endl;
    std::cout << "Sup flow normalised difference = " << abs(initial_flow_conservation - end_flow_conservation)/(abs(initial_flow_conservation) + abs(end_flow_conservation)) << std::endl;
    std::cout << "Sup RBC normalised difference= " << abs(initial_haematocrit_conservation - end_haematocrit_conservation)/(abs(initial_haematocrit_conservation) + abs(end_haematocrit_conservation)) << std::endl;

    TS_ASSERT_DELTA(0.44440140856573, segments[1]->GetFlowProperties()->GetHaematocrit(), 1e-5);
    TS_ASSERT_DELTA(0.451370374619631, segments[2]->GetFlowProperties()->GetHaematocrit(), 1e-5);
    TS_ASSERT_DELTA(0.44440140856573, segments[3]->GetFlowProperties()->GetHaematocrit(), 1e-5);
    TS_ASSERT_DELTA(0.451370374619631, segments[4]->GetFlowProperties()->GetHaematocrit(), 1e-5);
    output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
    p_network->Write(output_file);

}

// void TestForkedToForkedBifurcationUnit()
// {
//     QLength input_radius = 50_um;
//     QLength vessel_length = 1000_um;
//     QDimensionless alpha = 0.75;
//
//     QDynamicViscosity viscosity = 1.0*unit::poiseuille;
//
//     double inlet_haematocrit = 0.45;
//     double initial_haematocrit = 0.45;
//
//     // Generate the network
//
//     VesselNetworkGenerator<2> network_generator;
//
//
//     // Length of the vertical projection of first-order vessels
//     QLength domain_side_length_x = vessel_length*(2.0 + 1.0/sqrt(2.0));
//     // vertical size of the domain
//     // QLength domain_side_length_y = (1.0+1.0/sqrt(2.0)) * vessel_length;
//
//     // horizontal size of the domain
//     auto p_file_handler = std::make_shared<OutputFileHandler>("TestForkedToForkedBifurcationUnit", true);
//
//     std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateTrueForkedBifurcationUnit(vessel_length, input_radius, alpha);
//
//     // identify input and output nodes and assign them properties
//
//     VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(0.0_um,vessel_length));
//     VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, (1.0-1.0/sqrt(2.0)) * vessel_length));
//     VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, (1.0+1.0/sqrt(2.0)) * vessel_length));
//     p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
//     p_inlet_node->GetFlowProperties()->SetPressure(1.5_Pa);
//     p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_1->GetFlowProperties()->SetPressure(0.0_Pa);
//     p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_2->GetFlowProperties()->SetPressure(0.0_Pa);
//
//
//     auto p_haematocrit_calculator = GardnerSimplifiedHaematocritSolver<2>::Create();
//     // auto p_haematocrit_calculator = PriesHaematocritSolver<2>::Create();
//     auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
//     auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
//     p_viscosity_calculator->SetPlasmaViscosity(viscosity);
//
//     p_haematocrit_calculator->SetVesselNetwork(p_network);
//     p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
//
//     std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
//     segments[1]->GetFlowProperties()->SetHaematocrit(0.1814643485);
//     segments[2]->GetFlowProperties()->SetHaematocrit(0.184473768602);
//     segments[3]->GetFlowProperties()->SetHaematocrit(0.1814643485);
//     segments[4]->GetFlowProperties()->SetHaematocrit(0.184473768602);
//
//     p_impedance_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->Calculate();
//     p_impedance_calculator->Calculate();
//
//     FlowSolver<2> flow_solver;
//     flow_solver.SetVesselNetwork(p_network);
//     flow_solver.SetUseDirectSolver(false);
//     flow_solver.SetUp();
//
//     unsigned max_iter = 1000;
//     double tolerance2 = 1.e-5;
//     std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("InitialHaematocrit.vtp");
//     p_network->Write(output_file);
//     std::vector<QDimensionless> previous_haematocrit(segments.size(), QDimensionless(initial_haematocrit));
//     for(unsigned idx=0;idx<max_iter;idx++)
//     {
//         p_impedance_calculator->Calculate();
//         flow_solver.Update();
//         flow_solver.Solve();
//         p_haematocrit_calculator->Calculate();
//         p_viscosity_calculator->Calculate();
//         // Get the residual
//         unsigned max_difference_index = 0;
//         double max_difference = 0.0;
//         double h_for_max = 0.0;
//         double prev_for_max = 0.0;
//         for(unsigned jdx=0;jdx<segments.size();jdx++)
//         {
//             QDimensionless current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
//             QDimensionless difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
//             if(difference>max_difference)
//             {
//                 max_difference = difference;
//                 h_for_max = current_haematocrit;
//                 prev_for_max = previous_haematocrit[jdx];
//                 max_difference_index = jdx;
//             }
//             previous_haematocrit[jdx] = current_haematocrit;
//         }
//         std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << " at index " << max_difference_index << std::endl;
//         if(max_difference<=tolerance2 && idx > 1)
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
//
//     std::cout << "Sup flow =" << flow_solver.CheckSolution() << std::endl;
//     std::cout << "Sup RBC = " << p_haematocrit_calculator->CheckSolution() << std::endl;
//
//     std::vector<std::shared_ptr<VesselNode<2> > > network_nodes = p_network->GetNodes();
//
//     TS_ASSERT_DELTA(0.1814643485, segments[1]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//     TS_ASSERT_DELTA(0.184473768602, segments[2]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//     TS_ASSERT_DELTA(0.1814643485, segments[3]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//     TS_ASSERT_DELTA(0.184473768602, segments[4]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//
//     TS_ASSERT_DELTA(0.817352172881, network_nodes[1]->GetFlowProperties()->GetPressure(), 1e-6);
//     TS_ASSERT_DELTA(network_nodes[2]->GetFlowProperties()->GetPressure(), network_nodes[1]->GetFlowProperties()->GetPressure()/2.0, 1e-6);
//     TS_ASSERT_DELTA(network_nodes[3]->GetFlowProperties()->GetPressure(), network_nodes[2]->GetFlowProperties()->GetPressure(), 1e-6);
//     output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
//     p_network->Write(output_file);
//
// }
//
// void TestRegularToRegularBifurcationUnit()
// {
//     QLength input_radius = 50_um;
//     QLength vessel_length = 1000_um;
//     QDimensionless alpha = 0.75;
//
//     QDynamicViscosity viscosity = 1.0*unit::poiseuille;
//
//     double inlet_haematocrit = 0.45;
//     double initial_haematocrit = 0.45;
//
//     // Generate the network
//
//     VesselNetworkGenerator<2> network_generator;
//
//
//     // Length of the vertical projection of first-order vessels
//     QLength domain_side_length_x = vessel_length*(1.0 + sqrt(3.0)/2.0);
//     // vertical size of the domain
//     // QLength domain_side_length_y = (1.0+1.0/sqrt(2.0)) * vessel_length;
//
//     // horizontal size of the domain
//     auto p_file_handler = std::make_shared<OutputFileHandler>("TestRegularToRegularBifurcationUnit", true);
//
//     std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateTrueRegularBifurcationUnit(vessel_length, input_radius, alpha);
//
//     // identify input and output nodes and assign them properties
//
//     VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(0.0_um,vessel_length));
//     VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, 2.0*vessel_length));
//     VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, 0.0_um));
//     p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
//     p_inlet_node->GetFlowProperties()->SetPressure(1.5_Pa);
//     p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_1->GetFlowProperties()->SetPressure(0.0_Pa);
//     p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_2->GetFlowProperties()->SetPressure(0.0_Pa);
//
//
//     auto p_haematocrit_calculator = GardnerSimplifiedHaematocritSolver<2>::Create();
//     // auto p_haematocrit_calculator = PriesHaematocritSolver<2>::Create();
//     auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
//     auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
//     p_viscosity_calculator->SetPlasmaViscosity(viscosity);
//
//     p_haematocrit_calculator->SetVesselNetwork(p_network);
//     p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
//
//     std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
//     segments[1]->GetFlowProperties()->SetHaematocrit(0.451378);
//     segments[2]->GetFlowProperties()->SetHaematocrit(0.444114);
//
//     p_impedance_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->Calculate();
//     p_impedance_calculator->Calculate();
//
//     FlowSolver<2> flow_solver;
//     flow_solver.SetVesselNetwork(p_network);
//     flow_solver.SetUseDirectSolver(false);
//     flow_solver.SetUp();
//
//     unsigned max_iter = 1000;
//     double tolerance2 = 1.e-5;
//     std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("InitialHaematocrit.vtp");
//     p_network->Write(output_file);
//     std::vector<QDimensionless> previous_haematocrit(segments.size(), QDimensionless(initial_haematocrit));
//     for(unsigned idx=0;idx<max_iter;idx++)
//     {
//         p_impedance_calculator->Calculate();
//         flow_solver.Update();
//         flow_solver.Solve();
//         p_haematocrit_calculator->Calculate();
//         p_viscosity_calculator->Calculate();
//         // Get the residual
//         unsigned max_difference_index = 0;
//         double max_difference = 0.0;
//         double h_for_max = 0.0;
//         double prev_for_max = 0.0;
//         for(unsigned jdx=0;jdx<segments.size();jdx++)
//         {
//             QDimensionless current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
//             QDimensionless difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
//             if(difference>max_difference)
//             {
//                 max_difference = difference;
//                 h_for_max = current_haematocrit;
//                 prev_for_max = previous_haematocrit[jdx];
//                 max_difference_index = jdx;
//             }
//             previous_haematocrit[jdx] = current_haematocrit;
//         }
//         std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << " at index " << max_difference_index << std::endl;
//         if(max_difference<=tolerance2 && idx > 1)
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
//
//     std::cout << "Sup flow =" << flow_solver.CheckSolution() << std::endl;
//     std::cout << "Sup RBC = " << p_haematocrit_calculator->CheckSolution() << std::endl;
//
//     std::vector<std::shared_ptr<VesselNode<2> > > network_nodes = p_network->GetNodes();
//
//     TS_ASSERT_DELTA(0.4856553292270605, segments[1]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//     TS_ASSERT_DELTA(0.4935660741847571, segments[2]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//
//     TS_ASSERT_DELTA(1.093439402730676, network_nodes[1]->GetFlowProperties()->GetPressure(), 1e-6);
//     output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
//     p_network->Write(output_file);
//
// }
//
// void TestForkedToRegularBifurcationUnit()
// {
//     QLength input_radius = 50_um;
//     QLength vessel_length = 1000_um;
//     QDimensionless alpha = 0.75;
//
//     QDynamicViscosity viscosity = 1.0*unit::poiseuille;
//
//     double inlet_haematocrit = 0.45;
//     double initial_haematocrit = 0.45;
//
//     // Generate the network
//
//     VesselNetworkGenerator<2> network_generator;
//
//
//     // Length of the vertical projection of first-order vessels
//     QLength domain_side_length_x = vessel_length*(1.0 + sqrt(3.0)/2.0);
//     // vertical size of the domain
//     // QLength domain_side_length_y = (1.0+1.0/sqrt(2.0)) * vessel_length;
//
//     // horizontal size of the domain
//     auto p_file_handler = std::make_shared<OutputFileHandler>("TestForkedToRegularBifurcationUnit", true);
//
//     std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateTrueRegularBifurcationUnit(vessel_length, input_radius, alpha);
//
//     // identify input and output nodes and assign them properties
//
//     VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(0.0_um,vessel_length));
//     VesselNodePtr<2> p_outlet_node_1 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, 2.0*vessel_length));
//     VesselNodePtr<2> p_outlet_node_2 = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
//             Vertex<2>(domain_side_length_x, 0.0_um));
//     p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
//     p_inlet_node->GetFlowProperties()->SetPressure(1.5_Pa);
//     p_outlet_node_1->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_1->GetFlowProperties()->SetPressure(0.0_Pa);
//     p_outlet_node_2->GetFlowProperties()->SetIsOutputNode(true);
//     p_outlet_node_2->GetFlowProperties()->SetPressure(0.0_Pa);
//
//
//     auto p_haematocrit_calculator = GardnerSimplifiedHaematocritSolver<2>::Create();
//     // auto p_haematocrit_calculator = PriesHaematocritSolver<2>::Create();
//     auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
//     auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
//     p_viscosity_calculator->SetPlasmaViscosity(viscosity);
//
//     p_haematocrit_calculator->SetVesselNetwork(p_network);
//     p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
//
//     std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
//     segments[1]->GetFlowProperties()->SetHaematocrit(0.1814643485);
//     segments[2]->GetFlowProperties()->SetHaematocrit(0.184473768602);
//
//     p_impedance_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->SetVesselNetwork(p_network);
//     p_viscosity_calculator->Calculate();
//     p_impedance_calculator->Calculate();
//
//     FlowSolver<2> flow_solver;
//     flow_solver.SetVesselNetwork(p_network);
//     flow_solver.SetUseDirectSolver(false);
//     flow_solver.SetUp();
//
//     unsigned max_iter = 1000;
//     double tolerance2 = 1.e-5;
//     std::string output_file = p_file_handler->GetOutputDirectoryFullPath().append("InitialHaematocrit.vtp");
//     p_network->Write(output_file);
//     std::vector<QDimensionless> previous_haematocrit(segments.size(), QDimensionless(initial_haematocrit));
//     for(unsigned idx=0;idx<max_iter;idx++)
//     {
//         p_impedance_calculator->Calculate();
//         flow_solver.Update();
//         flow_solver.Solve();
//         p_haematocrit_calculator->Calculate();
//         p_viscosity_calculator->Calculate();
//         // Get the residual
//         unsigned max_difference_index = 0;
//         double max_difference = 0.0;
//         double h_for_max = 0.0;
//         double prev_for_max = 0.0;
//         for(unsigned jdx=0;jdx<segments.size();jdx++)
//         {
//             QDimensionless current_haematocrit = segments[jdx]->GetFlowProperties()->GetHaematocrit();
//             QDimensionless difference = std::abs(current_haematocrit - previous_haematocrit[jdx]);
//             if(difference>max_difference)
//             {
//                 max_difference = difference;
//                 h_for_max = current_haematocrit;
//                 prev_for_max = previous_haematocrit[jdx];
//                 max_difference_index = jdx;
//             }
//             previous_haematocrit[jdx] = current_haematocrit;
//         }
//         std::cout << "H at max difference: " << h_for_max << ", Prev H at max difference:" << prev_for_max << " at index " << max_difference_index << std::endl;
//         if(max_difference<=tolerance2 && idx > 1)
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
//
//     std::cout << "Sup flow =" << flow_solver.CheckSolution() << std::endl;
//     std::cout << "Sup RBC = " << p_haematocrit_calculator->CheckSolution() << std::endl;
//
//     std::vector<std::shared_ptr<VesselNode<2> > > network_nodes = p_network->GetNodes();
//
//     TS_ASSERT_DELTA(0.1814643485, segments[1]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//     TS_ASSERT_DELTA(0.184473768602, segments[2]->GetFlowProperties()->GetHaematocrit(), 1e-6);
//
//     TS_ASSERT_DELTA(0.817352172881, network_nodes[1]->GetFlowProperties()->GetPressure(), 1e-6);
//     output_file = p_file_handler->GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
//     p_network->Write(output_file);
//
// }


};
