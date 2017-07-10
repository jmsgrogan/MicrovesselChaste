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

#ifndef TESTSTRUCTURALADAPTATIONSOLVER_HPP
#define TESTSTRUCTURALADAPTATIONSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "BaseUnits.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "UnitCollection.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestStructuralAdaptationSolver : public CxxTest::TestSuite
{

public:

    void TestMultiVesselNetwork() throw(Exception)
    {
        std::vector<std::shared_ptr<VesselNode<2> > > nodes;
        for(unsigned idx=0; idx<8; idx++)
        {
            nodes.push_back(VesselNode<2>::Create(double(idx*10.0), 0.0));
        }

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(3322.0 * unit::pascals);
        nodes[7]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[7]->GetFlowProperties()->SetPressure(1993.0 * unit::pascals);

        double haematocrit = 0.45;
        std::vector<std::shared_ptr<VesselSegment<2> > > segments;
        for(unsigned idx=0; idx<7; idx++)
        {
            segments.push_back(VesselSegment<2>::Create(nodes[idx], nodes[idx+1]));
            segments[idx]->GetFlowProperties()->SetHaematocrit(haematocrit);
            segments[idx]->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
            segments[idx]->SetRadius(10.0 * 1_um);
        }

        std::vector<std::shared_ptr<Vessel<2> > > vessels;
        for(unsigned idx=0; idx<7; idx++)
        {
            vessels.push_back(Vessel<2>::Create(segments[idx]));
        }

        std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessels(vessels);

        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);

        // Write the network to file
        OutputFileHandler output_file_handler("TestStructuralAdaptationSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("MultiVesselNetwork.vtp");
        std::string progress_output_filename = output_file_handler.GetOutputDirectoryFullPath().append("MultiVesselNetwork_SAAProgress.dat");

        std::shared_ptr<VesselImpedanceCalculator<2> > p_impedance_calculator = VesselImpedanceCalculator<2>::Create();

        StructuralAdaptationSolver<2> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetWriteOutput(true);
        solver.SetOutputFileName(progress_output_filename);
        solver.SetTolerance(0.0001);
        solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        solver.SetTimeIncrement(0.0001 * unit::seconds);

        solver.Solve();

        // Write the network to file
        p_network->Write(output_filename);

        TS_ASSERT_DELTA((nodes[3]->GetFlowProperties()->GetPressure() + nodes[4]->GetFlowProperties()->GetPressure())/(2.0*unit::pascals),((3322.0 + 1993.0) / 2.0), 1e-6);
        TS_ASSERT_DELTA(Qabs(segments[0]->GetFlowProperties()->GetFlowRate())/unit::metre_cubed_per_second,
                        Qabs(segments[1]->GetFlowProperties()->GetFlowRate())/unit::metre_cubed_per_second, 1e-6);

        SimulationTime::Destroy();
    }

    void TestOneVesselNetwork() throw(Exception)
    {
        std::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0, 0.0);
        std::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(80.0e-6, 0.0);
        std::shared_ptr<VesselSegment<2> > p_segment1(VesselSegment<2>::Create(p_node1, p_node2));

        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node1->GetFlowProperties()->SetPressure(3322.0 * unit::pascals);
        p_node2->GetFlowProperties()->SetIsOutputNode(true);
        p_node2->GetFlowProperties()->SetPressure(1993.0 * unit::pascals);

        std::shared_ptr<Vessel<2> > p_vessel1(Vessel<2>::Create(p_segment1));

        std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
        p_network->AddVessel(p_vessel1);

        double radius = 10.0e-6;
        p_segment1->SetRadius(radius*1_um);
        double haematocrit = 0.45;
        p_segment1->GetFlowProperties()->SetHaematocrit(haematocrit);
        p_segment1->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment1);

        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);

        // Write the network to file
        OutputFileHandler output_file_handler("TestStructuralAdaptationSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("OneVesselNetwork.vtp");
        std::string progress_output_filename = output_file_handler.GetOutputDirectoryFullPath().append("OneVesselNetwork_SAAProgress.dat");

        std::shared_ptr<VesselImpedanceCalculator<2> > p_impedance_calculator = VesselImpedanceCalculator<2>::Create();

        StructuralAdaptationSolver<2> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetWriteOutput(true);
        solver.SetOutputFileName(progress_output_filename);
        solver.SetTolerance(0.0001);
        solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        solver.SetTimeIncrement(0.0001*unit::seconds);
        solver.Solve();

        // Write the network to file
        p_network->Write(output_filename);

        SimulationTime::Destroy();
    }

    void TestHexagonalNetwork() throw(Exception)
	{
        // Specify the network dimensions
        QLength vessel_length = 83.0 * 1_um;

        // Generate the network
        VesselNetworkGenerator<2> vascular_network_generator;
        std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalNetwork(800.0* 1_um,
                                                                                                                        1000.0* 1_um,
                                                                                                                        vessel_length);

        std::vector<DimensionalChastePoint<2> > points;
        points.push_back(DimensionalChastePoint<2>(0, 0, 0.0, 1_um));
        points.push_back(DimensionalChastePoint<2>(5, 0, 0.0, 1_um));

        std::vector<std::shared_ptr<VesselNode<2> > > nodes;
        for(unsigned i=0; i < points.size(); i++)
        {
            nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(points[i])));
        }

        std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));

        double radius = 10.0;
        p_segment->SetRadius(radius*1_um);
        double haematocrit = 0.45;
        p_segment->GetFlowProperties()->SetHaematocrit(haematocrit);
        p_segment->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment);

        std::pair<DimensionalChastePoint<2>, DimensionalChastePoint<2> > network_extents = VesselNetworkGeometryCalculator<2>::GetExtents(vascular_network);
        double y_middle = (network_extents.first.GetLocation(1_um)[1]) / 2.0;
        double x_middle = (network_extents.first.GetLocation(1_um)[0]) / 2.0;

        std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

        std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();

        for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[1] >  y_middle)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[0] >  x_middle)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetEndNode()->rGetLocation().GetLocation(1_um)[1] >  y_middle)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[0] >  x_middle)
                    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3320.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[1] <=  y_middle)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[0] <  x_middle)
                    {
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
                }
            }
            if((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if((*vessel_iterator)->GetEndNode()->rGetLocation().GetLocation(1_um)[1] <=  y_middle)
                {
                    if((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[0] <  x_middle)
                    {
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                        (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(2090.0*unit::pascals);
                    }
                }
            }

        }

        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);

        // Write the network to file
        OutputFileHandler output_file_handler("TestStructuralAdaptationSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexagonalVesselNetwork.vtp");
        std::string progress_output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexagonalVesselNetwork_SAAProgress.dat");

        std::shared_ptr<VesselImpedanceCalculator<2> > p_impedance_calculator = VesselImpedanceCalculator<2>::Create();

        StructuralAdaptationSolver<2> solver;
        solver.SetVesselNetwork(vascular_network);
        solver.SetWriteOutput(true);
        solver.SetOutputFileName(progress_output_filename);
        solver.SetTolerance(0.0001);
        solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        solver.SetTimeIncrement(0.001*unit::seconds);
        solver.SetMaxIterations(10000);
        solver.Solve();

        // Write the network to file
        vascular_network->Write(output_filename);

        SimulationTime::Destroy();
	}
};

#endif /*TESTFLOWSOLVER_HPP_*/
