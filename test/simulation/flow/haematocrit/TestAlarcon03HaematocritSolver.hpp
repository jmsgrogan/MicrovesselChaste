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

#ifndef TESTALARCONHAEMATOCRITSOLVER_HPP
#define TESTALARCONHAEMATOCRITSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "VesselImpedanceCalculator.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "SimulationTime.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "ViscosityCalculator.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestAlarconHaematocritSolver : public CxxTest::TestSuite
{

public:

    void TestTwoVesselNetwork()
    {
        auto p_node1 = VesselNode<2>::Create(0.0_um);
        auto p_node2 = VesselNode<2>::Create(80_um);
        auto p_node3 = VesselNode<2>::Create(160_um);
        p_node1->GetFlowProperties()->SetIsInputNode(true);

        auto p_segment1(VesselSegment<2>::Create(p_node1, p_node2));
        auto p_segment2(VesselSegment<2>::Create(p_node2, p_node3));

        auto p_vessel1(Vessel<2>::Create(p_segment1));
        auto p_vessel2(Vessel<2>::Create(p_segment2));
        p_segment1->GetFlowProperties()->SetFlowRate(1.0*unit::metre_cubed_per_second);
        p_segment2->GetFlowProperties()->SetFlowRate(2.0*unit::metre_cubed_per_second);

        auto p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);

        auto p_haematocrit_calculator = AlarconHaematocritSolver<2>::Create();
        p_haematocrit_calculator->SetVesselNetwork(p_network);
        p_haematocrit_calculator->Calculate();

        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetHaematocrit(),0.45, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetFlowProperties()->GetHaematocrit(),0.45, 1e-6);
    }

    void TestBifurcationInflowNetwork()
    {
        auto p_node1 = VesselNode<2>::Create(0.0_um);
        auto p_node2 = VesselNode<2>::Create(80_um);
        auto p_node3 = VesselNode<2>::Create(160_um);
        auto p_node4 = VesselNode<2>::Create(200_um);
        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node2->GetFlowProperties()->SetIsInputNode(true);

        auto p_vessel1(Vessel<2>::Create(p_node1, p_node3));
        auto p_vessel2(Vessel<2>::Create(p_node2, p_node3));
        auto p_vessel3(Vessel<2>::Create(p_node3, p_node4));
        p_vessel1->GetFlowProperties()->SetFlowRate(1.0*unit::metre_cubed_per_second);
        p_vessel2->GetFlowProperties()->SetFlowRate(1.0*unit::metre_cubed_per_second);
        p_vessel3->GetFlowProperties()->SetFlowRate(1.0*unit::metre_cubed_per_second);

        auto p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);

        auto p_haematocrit_calculator = AlarconHaematocritSolver<2>::Create();
        p_haematocrit_calculator->SetVesselNetwork(p_network);
        p_haematocrit_calculator->Calculate();

        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetHaematocrit(), 0.45, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetFlowProperties()->GetHaematocrit(), 0.45, 1e-6);
        TS_ASSERT_DELTA(p_vessel3->GetFlowProperties()->GetHaematocrit(), 0.9, 1e-6);
    }

    void TestBifurcationOutflowNetwork()
    {
        auto p_node1 = VesselNode<2>::Create(0.0_um);
        auto p_node2 = VesselNode<2>::Create(80_um);
        auto p_node3 = VesselNode<2>::Create(160_um);
        auto p_node4 = VesselNode<2>::Create(200_um);
        p_node4->GetFlowProperties()->SetIsInputNode(true);

        auto p_segment1(VesselSegment<2>::Create(p_node1, p_node3));
        auto p_segment2(VesselSegment<2>::Create(p_node2, p_node3));
        auto p_segment3(VesselSegment<2>::Create(p_node3, p_node4));

        auto p_vessel1(Vessel<2>::Create(p_segment1));
        auto p_vessel2(Vessel<2>::Create(p_segment2));
        auto p_vessel3(Vessel<2>::Create(p_segment3));
        p_vessel1->GetFlowProperties()->SetFlowRate(-1.0*unit::metre_cubed_per_second);
        p_vessel2->GetFlowProperties()->SetFlowRate(-1.0*unit::metre_cubed_per_second);
        p_vessel3->GetFlowProperties()->SetFlowRate(-1.0*unit::metre_cubed_per_second);

        auto p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);

        auto p_haematocrit_calculator = AlarconHaematocritSolver<2>::Create();
        p_haematocrit_calculator->SetVesselNetwork(p_network);
        p_haematocrit_calculator->Calculate();

        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetHaematocrit(),0.15, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetFlowProperties()->GetHaematocrit(),0.3, 1e-6);
        TS_ASSERT_DELTA(p_vessel3->GetFlowProperties()->GetHaematocrit(),0.45, 1e-6);
    }

    void TestBifurcationOutflowNetworkBiasedFlow()
    {
        auto p_node1 = VesselNode<2>::Create(0.0_um);
        auto p_node2 = VesselNode<2>::Create(80_um);
        auto p_node3 = VesselNode<2>::Create(160_um);
        auto p_node4 = VesselNode<2>::Create(200_um);
        p_node4->GetFlowProperties()->SetIsInputNode(true);

        auto p_segment1(VesselSegment<2>::Create(p_node1, p_node3));
        auto p_segment2(VesselSegment<2>::Create(p_node2, p_node3));
        auto p_segment3(VesselSegment<2>::Create(p_node3, p_node4));

        auto p_vessel1(Vessel<2>::Create(p_segment1));
        auto p_vessel2(Vessel<2>::Create(p_segment2));
        auto p_vessel3(Vessel<2>::Create(p_segment3));
        p_vessel1->GetFlowProperties()->SetFlowRate(-1.0*unit::metre_cubed_per_second);
        p_vessel2->GetFlowProperties()->SetFlowRate(-3.0*unit::metre_cubed_per_second);
        p_vessel3->GetFlowProperties()->SetFlowRate(-1.0*unit::metre_cubed_per_second);

        auto p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);

        auto p_haematocrit_calculator = AlarconHaematocritSolver<2>::Create();
        p_haematocrit_calculator->SetVesselNetwork(p_network);
        p_haematocrit_calculator->Calculate();

        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetHaematocrit(), 0.0, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetFlowProperties()->GetHaematocrit(), 0.45, 1e-6);
        TS_ASSERT_DELTA(p_vessel3->GetFlowProperties()->GetHaematocrit(), 0.45, 1e-6);
    }

    void TestHexagonalNetwork()
    {
        // Specify the network dimensions
        QLength vessel_length = 80.0_um;
        QLength width = 800.0_um;
        QLength height = 1000.0_um;

        // Generate the network
        VesselNetworkGenerator<2> network_generator;
        auto p_network = network_generator.GenerateHexagonalNetwork(width, height, vessel_length);

        // Assign flow properties and boundary conditions
        QLength radius = 10_um;
        QDimensionless haematocrit = 0.45;
        QPressure inflow_pressure = 3320.0_Pa;
        QPressure outflow_pressure = 2090.0_Pa;
        QDynamicViscosity reference_viscosity = 1.e-3*unit::poiseuille;

        auto p_segment = p_network->GetVesselSegments()[0];
        p_segment->SetRadius(radius);
        p_segment->GetFlowProperties()->SetHaematocrit(haematocrit);
        p_segment->GetFlowProperties()->SetViscosity(reference_viscosity);
        VesselNetworkPropertyManager<2>::AssignInflows(p_network, Vertex<2>(0.0, 0.0), vessel_length/2.0);

        // Top right corner
        VesselNetworkPropertyManager<2>::AssignOutflows(p_network, Vertex<2>(640.0_um, 960.0_um), vessel_length/2.0);
        VesselNetworkPropertyManager<2>::SetInflowPressures(p_network, inflow_pressure);
        VesselNetworkPropertyManager<2>::SetOutflowPressures(p_network, outflow_pressure);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

        // Set up solvers and calculators
        VesselImpedanceCalculator<2> impedance_calculator;
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();

        FlowSolver<2> flow_solver;
        flow_solver.SetVesselNetwork(p_network);
        flow_solver.SetUp();

        AlarconHaematocritSolver<2> haematocrit_solver;
        haematocrit_solver.SetVesselNetwork(p_network);

        ViscosityCalculator<2> viscosity_calculator;
        viscosity_calculator.SetVesselNetwork(p_network);
        viscosity_calculator.SetPlasmaViscosity(reference_viscosity);

        // Iterate until the haematocrit value converges
        OutputFileHandler output_file_handler("TestAlarconHaematocritSolver");
        unsigned max_iter = 1000;
        double tolerance = 1.e-3;

        std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
        std::vector<double> previous_haematocrit(segments.size(), double(haematocrit));
        for(unsigned idx=0;idx<max_iter;idx++)
        {
            impedance_calculator.Calculate();
            flow_solver.SetUp();
            flow_solver.Solve();
            haematocrit_solver.Calculate();
            viscosity_calculator.Calculate();

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
            if(max_difference<=tolerance)
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
                    std::string output_file = output_file_handler.GetOutputDirectoryFullPath().append(file_suffix);
                    p_network->Write(output_file);
                }
            }

            if(idx==max_iter-1)
            {
                EXCEPTION("Did not converge after " + std::to_string(idx) + " iterations.");
            }
        }

        // Write the final result
        std::string output_file = output_file_handler.GetOutputDirectoryFullPath().append("FinalHaematocrit.vtp");
        p_network->Write(output_file);
    }
};

#endif // TESTALARCONHAEMATOCRITSOLVER_HPP
