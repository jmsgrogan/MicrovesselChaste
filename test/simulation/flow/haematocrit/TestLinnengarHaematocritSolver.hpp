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

#ifndef TESTLINNENGARHAEMATOCRITSOLVER_HPP
#define TESTLINNENGARHAEMATOCRITSOLVER_HPP

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
#include "LinnengarHaematocritSolver.hpp"
#include "YangHaematocritSolver.hpp"
#include "UnitCollection.hpp"
#include "RegularGrid.hpp"
#include "SimulationTime.hpp"
#include "MicrovesselSolver.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "BetteridgeHaematocritSolver.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestLinnengarHaematocritSolver : public CxxTest::TestSuite
{

public:


  void TestDivergingBifurcationLinnengarHaematocrit() throw(Exception)
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
    double competitor_flow_rate = 1.0;
    double my_flow_rate = 2.0;

    p_vessel1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(parent_flow_rate * unit::metre_cubed_per_second);
    p_vessel2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(my_flow_rate * unit::metre_cubed_per_second);
    p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(competitor_flow_rate * unit::metre_cubed_per_second);

p_vessel2->SetRadius(5.e-6*unit::metres);
    //p_vessel3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(competitor_flow_rate * unit::metre_cubed_per_second);

    std::shared_ptr<VesselNetwork<2> > p_network = std::shared_ptr<VesselNetwork<2> >(new VesselNetwork<2>);
    p_network->AddVessel(p_vessel1);
    p_network->AddVessel(p_vessel2);
    p_network->AddVessel(p_vessel3);

    std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator(new LinnengarHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->Calculate();

    //double parent_haematocrit = 0.45;

    OutputFileHandler output_file_handler("TestLinnengarHaematocritSolver_BifurcOutflowBiased", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("BifurcOutflowBiased_Hemo.vtp");
    p_network->Write(output_filename);

}


void TestInvertedPiLinnengarHaematocrit() throw(Exception)
{ 
    
    
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
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(3.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
 
    p_vessel_4->SetRadius(2.e-5*unit::metres);
         /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);

    //QLength vessel_radius(1.0*GenericParameters::mpCapillaryRadius->GetValue());
    //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);


    std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator(new LinnengarHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    p_haematocrit_calculator->Calculate();

    OutputFileHandler output_file_handler("TestLinningerHaematocritSolver_InvPi", false);

    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi_Hemo.vtp");
    vascular_network->Write(output_filename);	

}



void TestHexagonalNetworkLinnengarHaematocrit() throw(Exception)
{
    // Iterate over vessel lengths
    std::vector<double> lengths;
    std::vector<double> average_oxygen_concentration;
    QLength length_increment = 20_um;
    QLength domain_side_length = 2000_um;
    QLength reference_length = 1_um;
    QLength vessel_radius = 10_um;
    QDynamicViscosity visocity = 1.e-3*unit::poiseuille;
    auto p_file_handler =
                    std::make_shared<OutputFileHandler>("TestLinnengarHaematocritSolver_depl", true);

    double inlet_haematocrit = 0.8;
    unsigned num_samples = 0;

    for(unsigned idx=0; idx<num_samples; idx++)
    {
        std::string extension = boost::lexical_cast<std::string>(idx);
        auto p_internal_file_handler =
                        std::make_shared<OutputFileHandler>("TestLinnengarHaematocritSolver_depl/Length_"+extension, true);

        // Specify the network dimensions
        QLength vessel_length = double(idx+1)*length_increment;

        // Generate the network
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateHexagonalNetwork(domain_side_length,
                domain_side_length, vessel_length);

        // Assign flow properties
        std::vector<VesselNodePtr<2> > nodes;
        nodes.push_back(VesselNodePtr<2> (VesselNode<2>::Create(0_um,5_um)));
        nodes.push_back(VesselNodePtr<2> (VesselNode<2>::Create(5_um,0_um)));
        std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));
        p_segment->SetRadius(vessel_radius);
        p_segment->GetFlowProperties()->SetHaematocrit(inlet_haematocrit);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);
        std::vector<std::shared_ptr<VesselSegment<2> > > segments = p_network->GetVesselSegments();
        for(unsigned jdx=0; jdx<segments.size(); jdx++)
        {
            segments[jdx]->GetFlowProperties()->SetViscosity(visocity);
        }

        VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
                Vertex<2>(0.0_um));
        VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
                Vertex<2>(domain_side_length, domain_side_length));
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(8000.0*unit::pascals);
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(2000.0*unit::pascals);

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength grid_spacing = 5_um;
        p_grid->SetSpacing(grid_spacing);
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = unsigned(domain_side_length/(grid_spacing)) + 1; // num x
        dimensions[1] = unsigned(domain_side_length/(grid_spacing)) + 1; // num_y
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        /**
         * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
         */
        auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
        p_oxygen_pde->SetContinuumLinearInUTerm(-10.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

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
                p_oxygen_solver->SetWriteSolution(true);

        //std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator = LinnengarHaematocritSolver<2>::Create();

        auto p_haematocrit_calculator = ConstantHaematocritSolver<2>::Create();
        auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
        auto p_structural_adaptation_solver = StructuralAdaptationSolver<2>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(1000);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);

        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
        std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFileHandler(p_internal_file_handler);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);

        p_microvessel_solver->Run();

        std::vector<double> solution = p_oxygen_solver->GetSolution();
        double average = 0.0;
        for(unsigned jdx=0;jdx<solution.size();jdx++)
        {
            average += solution[jdx];
        }
        average /=double(solution.size());
        average_oxygen_concentration.push_back(average);
        lengths.push_back(double(idx+1)*length_increment/reference_length);


    std::shared_ptr<std::ofstream> p_out_file = std::shared_ptr<std::ofstream>(new std::ofstream);
    p_out_file->open((p_file_handler->GetOutputDirectoryFullPath() + "/av_oxygen.dat").c_str());
    (*p_out_file) << "Length (micron), Av Oxygen (micro M)"<< std::endl;
    for(unsigned idx=0;idx<average_oxygen_concentration.size();idx++)
    {
        (*p_out_file) << lengths[idx] << "," << average_oxygen_concentration[idx] << std::endl;
    }

    p_out_file->close();



        SimulationTime::Instance()->Destroy();
    }


}


void TestHexagonalNetworkYangHaematocrit() throw(Exception)
{
    // Iterate over vessel lengths
    std::vector<double> lengths;
    std::vector<double> average_oxygen_concentration;
    QLength length_increment = 20_um;
    QLength domain_side_length = 2000_um;
    QLength reference_length = 1_um;
    QLength vessel_radius = 10_um;
    QDynamicViscosity visocity = 1.e-3*unit::poiseuille;
    auto p_file_handler =
                    std::make_shared<OutputFileHandler>("TestYangHaematocritSolver_depl", true);

    double inlet_haematocrit = 0.8;
    unsigned num_samples = 0;

    for(unsigned idx=0; idx<num_samples; idx++)
    {
        std::string extension = boost::lexical_cast<std::string>(idx);
        auto p_internal_file_handler =
                        std::make_shared<OutputFileHandler>("TestYangHaematocritSolver_depl/Length_"+extension, true);

        // Specify the network dimensions
        QLength vessel_length = double(idx+1)*length_increment;

        // Generate the network
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateHexagonalNetwork(domain_side_length,
                domain_side_length, vessel_length);

        // Assign flow properties
        std::vector<VesselNodePtr<2> > nodes;
        nodes.push_back(VesselNodePtr<2> (VesselNode<2>::Create(0_um,5_um)));
        nodes.push_back(VesselNodePtr<2> (VesselNode<2>::Create(5_um,0_um)));
        std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));
        p_segment->SetRadius(vessel_radius);
        p_segment->GetFlowProperties()->SetHaematocrit(inlet_haematocrit);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);
        std::vector<std::shared_ptr<VesselSegment<2> > > segments = p_network->GetVesselSegments();
        for(unsigned jdx=0; jdx<segments.size(); jdx++)
        {
            segments[jdx]->GetFlowProperties()->SetViscosity(visocity);
        }

        VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
                Vertex<2>(0.0_um));
        VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
                Vertex<2>(domain_side_length, domain_side_length));
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(8000.0*unit::pascals);
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(2000.0*unit::pascals);

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength grid_spacing = 5_um;
        p_grid->SetSpacing(grid_spacing);
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = unsigned(domain_side_length/(grid_spacing)) + 1; // num x
        dimensions[1] = unsigned(domain_side_length/(grid_spacing)) + 1; // num_y
        dimensions[2] = 1;
        p_grid->SetDimensions(dimensions);

        /**
         * Next set up the PDEs for oxygen and VEGF. Cells will act as discrete oxygen sinks and discrete vegf sources.
         */
        auto p_oxygen_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        p_oxygen_pde->SetIsotropicDiffusionConstant(Owen11Parameters::mpOxygenDiffusivity->GetValue("User"));
        p_oxygen_pde->SetContinuumLinearInUTerm(-10.0*Owen11Parameters::mpCellOxygenConsumptionRate->GetValue("User"));

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
        p_oxygen_solver->SetWriteSolution(true);

        //std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator = LinnengarHaematocritSolver<2>::Create();

        auto p_haematocrit_calculator = YangHaematocritSolver<2>::Create();
        auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
        auto p_structural_adaptation_solver = StructuralAdaptationSolver<2>::Create();
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(1000);
        p_structural_adaptation_solver->SetTimeIncrement(Owen11Parameters::mpVesselRadiusUpdateTimestep->GetValue("User"));
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);

        SimulationTime::Instance()->SetStartTime(0.0);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);
        std::shared_ptr<MicrovesselSolver<2> > p_microvessel_solver = MicrovesselSolver<2>::Create();
        p_microvessel_solver->SetVesselNetwork(p_network);
        p_microvessel_solver->SetOutputFileHandler(p_internal_file_handler);
        p_microvessel_solver->AddDiscreteContinuumSolver(p_oxygen_solver);
        p_microvessel_solver->SetStructuralAdaptationSolver(p_structural_adaptation_solver);


        p_microvessel_solver->Run();

        std::vector<double> solution = p_oxygen_solver->GetSolution();
        double average = 0.0;
        for(unsigned jdx=0;jdx<solution.size();jdx++)
        {
            average += solution[jdx];
        }
        average /=double(solution.size());
        average_oxygen_concentration.push_back(average);
        lengths.push_back(double(idx+1)*length_increment/reference_length);


    std::shared_ptr<std::ofstream> p_out_file = std::shared_ptr<std::ofstream>(new std::ofstream);
    p_out_file->open((p_file_handler->GetOutputDirectoryFullPath() + "/av_oxygen.dat").c_str());
    (*p_out_file) << "Length (micron), Av Oxygen (micro M)"<< std::endl;
    for(unsigned idx=0;idx<average_oxygen_concentration.size();idx++)
    {
        (*p_out_file) << lengths[idx] << "," << average_oxygen_concentration[idx] << std::endl;
    }

    p_out_file->close();

    OutputFileHandler output_file_handler_network("TestYangHaematocritSolverHexagonal", true);

    std::string output_filename_network = output_file_handler_network.GetOutputDirectoryFullPath().append("HexagonalYangHemo.vtp");
    p_network->Write(output_filename_network);

    SimulationTime::Instance()->Destroy();
    }


}

void TestHexagonalNetworkYangHaematocrit_better() throw(Exception)
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

    OutputFileHandler output_file_handler("TestYangHaematocritSolver_better", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexNet.vtp");
    vascular_network->Write(output_filename);

    std::shared_ptr<YangHaematocritSolver<2> > p_haematocrit_calculator(new YangHaematocritSolver<2>());
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
    const std::string& rDirectory = "TestYangHaematocritSolver_oxygen";
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


};

#endif // TESTLINNENGARHAEMATOCRITSOLVER_HPP
