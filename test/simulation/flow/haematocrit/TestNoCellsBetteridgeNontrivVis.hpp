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

class TestNoCellsBetteridgeNontrivVis : public CxxTest::TestSuite
{

public:

void xTestNoCellsNontriv()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    QLength domain_side_length = 1280_um;
    //QLength vessel_radius = 10_um;
QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivLinn", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(domain_side_length,
            domain_side_length, vessel_length);

    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->SetRadius(vessel_radius);
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length, domain_side_length));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length-vessel_length)/(grid_spacing))+1; // num x
    dimensions[1] = unsigned((domain_side_length)/(grid_spacing))+1; // num_y
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
    auto p_haematocrit_calculator = LinnengarHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance = 1.e-3;

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
}




void xTestNoCellsNontrivEquilateralBett()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    int units_in_y = 4;
    QLength domain_side_length_y = sqrt(2.0)*units_in_y*vessel_length;
    QLength domain_side_length_x = (2+sqrt(2.0))*units_in_y*vessel_length;
    //QLength vessel_radius = 10_um;
QLength vessel_radius = 20_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivHexagonalEquilateralBett", true);

    double inlet_haematocrit = 0.2;
    double initial_haematocrit = 0.2;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetworkEquilateral(domain_side_length_x,
            domain_side_length_y, vessel_length);

    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->SetRadius(vessel_radius);
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, domain_side_length_y));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x-vessel_length)/(grid_spacing))+2; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+2; // num_y
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
    auto p_haematocrit_calculator = BetteridgeHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-8;

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
}

void xTestNoCellsNontrivEquilateralModPries()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    int units_in_y = 4;
    QLength domain_side_length_y = sqrt(2.0)*units_in_y*vessel_length;
    QLength domain_side_length_x = (2+sqrt(2.0))*units_in_y*vessel_length;
    //QLength vessel_radius = 10_um;
QLength vessel_radius = 20_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivHexagonalEquilateralModPries", true);

    double inlet_haematocrit = 0.2;
    double initial_haematocrit = 0.2;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetworkEquilateral(domain_side_length_x,
            domain_side_length_y, vessel_length);

    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->SetRadius(vessel_radius);
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, domain_side_length_y));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x-vessel_length)/(grid_spacing))+2; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+2; // num_y
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
    auto p_haematocrit_calculator = ModifiedPriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-8;

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
}




void xTestNoCellsNontrivEquilateralConst()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    int units_in_y = 4;
    QLength domain_side_length_y = sqrt(2.0)*units_in_y*vessel_length;
    QLength domain_side_length_x = (2+sqrt(2.0))*units_in_y*vessel_length;
    //QLength vessel_radius = 10_um;
QLength vessel_radius = 20_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivHexagonalEquilateralConst", true);

    double inlet_haematocrit = 0.2;
    double initial_haematocrit = 0.2;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetworkEquilateral(domain_side_length_x,
            domain_side_length_y, vessel_length);

    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->SetRadius(vessel_radius);
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, domain_side_length_y));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x-vessel_length)/(grid_spacing))+2; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+2; // num_y
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
    auto p_haematocrit_calculator = ConstantHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-8;

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
}

void xTestNoCellsNontrivVaryingRadii()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    int units_in_y = 6;
    QLength domain_side_length_y = sqrt(2.0)*units_in_y*vessel_length;
    QLength domain_side_length_x = (2+sqrt(2.0))*units_in_y*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivBettVaryingRadii", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetworkRadius(domain_side_length_x,
            domain_side_length_y, vessel_length, max_radius);

    // Assign flow properties
    //auto p_segment = p_network->GetVesselSegments()[0];
    //p_segment->SetRadius(vessel_radius);
    //p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    //VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    VesselNodePtr<2> p_inlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(0.0_um));
    VesselNodePtr<2> p_outlet_node = VesselNetworkGeometryCalculator<2>::GetNearestNode(p_network,
            Vertex<2>(domain_side_length_x, domain_side_length_y));
    p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
    p_inlet_node->GetFlowProperties()->SetPressure(3320.0_Pa);
    p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
    p_outlet_node->GetFlowProperties()->SetPressure(2090.0_Pa);

    auto p_grid = RegularGrid<2>::Create();
    QLength grid_spacing = 5_um;
    p_grid->SetSpacing(grid_spacing);
    c_vector<unsigned, 3> dimensions;
    dimensions[0] = unsigned((domain_side_length_x-vessel_length)/(grid_spacing))+2; // num x
    dimensions[1] = unsigned((domain_side_length_y)/(grid_spacing))+2; // num_y
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
    auto p_haematocrit_calculator = LinnengarHaematocritSolver<2>::Create();
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
    double tolerance = 1.e-3;

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
}



void xTestNoCellsDichotomous()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;
    unsigned order=7;
    QLength domain_side_length_x = (double(order)+1.0)*2.0*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivBettDichotomous", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetwork(order, vessel_length, max_radius);

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
    auto p_haematocrit_calculator = YangHaematocritSolver<2>::Create();
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
    double tolerance = 1.e-3;

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
}


void xTestNoCellsDichotomousUneven()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;
    unsigned order=4;
    QLength domain_side_length_x = (double(order)+1.0)*2.0*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivDichotomousUnevenConst", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;

    double alpha = 1.5;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUneven(order, vessel_length, max_radius, alpha);

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
    auto p_haematocrit_calculator = ConstantHaematocritSolver<2>::Create();
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
    double tolerance = 1.e-3;

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
}


void xTestNoCellsDichotomousUnevenModPries()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;
    unsigned order=4;
    QLength domain_side_length_x = (double(order)+1.0)*2.0*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivDichotomousUnevenModPries", true);

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.2;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUneven(order, vessel_length, max_radius, alpha);

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
    auto p_haematocrit_calculator = ModifiedPriesHaematocritSolver<2>::Create();
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
    double tolerance2 = 1.e-13;

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
}


void xTestNoCellsDichotomousUnevenModPriesNoCorners()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;
    unsigned order=4;
    QLength domain_side_length_x = (double(order)+1.0)*2.0*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivDichotomousUnevenNoCornersYangHigherAlpha", true);

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.3;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCorners(order, vessel_length, max_radius, alpha);

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
    auto p_haematocrit_calculator = YangHaematocritSolver<2>::Create();
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
    double tolerance2 = 1.e-12;

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
}



void xTestNoCellsDichotomousUnevenModPriesNoCornersVaryDistance()
{
    // Specify the domain
    QLength vessel_length = 80_um;
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;
    unsigned order=5;
    double theta = 0.6;
    QLength domain_side_length_x = (1.0-pow(theta,double(order)+1.0))/(1.0-theta)*2.0*vessel_length;
    QLength max_radius = 40_um;
//QLength vessel_radius = 5_um;
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestDichotomousVaryDistanceEdinburgh_Asymmetry_WithX0_extremes_taken_care", true);

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;

    double alpha = 1.8;
    // Generate the network
    VesselNetworkGenerator<2> network_generator;
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCornersVaryDistance(order, vessel_length, max_radius, alpha, theta);

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
    QLength grid_spacing = 2_um;
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
}



void xTestNoCellsDichotomousWithCFL_LengthsFollowRadii()
{
    unsigned order=5;
    double theta = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux=1; i_aux<order+1; i_aux++)
    	{
	dimless_length += sqrt(pow(2.0,-2.0*double(i_aux)/3.0)-pow(2.0, -2.0*double(i_aux)));
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

   for (unsigned k_aux=0; k_aux<5; k_aux++)
   {
    lambda = 4.0+double(k_aux)*2.0;


// Specify the domain
    QLength vessel_length = 0.5*lambda*max_radius;//*pow(2.0,-1/3);
    //int units_in_y = 6;
    QLength domain_side_length_y = 4.0*vessel_length;

    std::ostringstream strs;
    strs << lambda;
    std::string str_lambda = strs.str();
    // this lambda is quotient between the length and radius...in contrary to lambda from previous work
    QLength domain_side_length_x = dimless_length*2.0*lambda*max_radius;
    std::cout << "Here, Domain length is:" << domain_side_length_x << "\n";
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestDichotomousVaryDistanceWithX0_extremes_taken_care_LengthsFollowRadii_NewModel_lambdaEquals"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii(order, vessel_length, max_radius, alpha,theta, lambda);

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
    }
}



void TestNoCellsDichotomousWithCFL_LengthsFollowRadii()
{
    unsigned order=7;
    //UNUSED IN NEW INTERFACE double theta = 1.0;

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

    //UNUSED IN NEW INTERFACE  double alpha = 1.0;
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
    auto p_file_handler = std::make_shared<OutputFileHandler>("Swarmplots_MoreSpreaded_WithX0_LengthsFollowRadii_NewModel_lambdaEquals"+str_lambda, true);
    std::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateForkingNetworkNoCorners(order, vessel_length, max_radius, /*alpha,theta,*/ 2.0*lambda);

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



void xTestNoCellsNontrivInvertedPiLinn()
{

 // Set up the vessel network
    QLength vessel_length(50.0_um);
    QLength reference_length(1.0_um);

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
         /*
         * Now add the vessels to a vessel network.
         */



    std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->AddVessel(p_vessel_5);


        //QLength vessel_radius(1.0e-5*unit::metres);
        //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

        //vascular_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);


std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = p_network->GetVessels();

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

// Specify the domain
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivInvPiLinn", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;
    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);
p_vessel_1->SetRadius(2.e-5*unit::metres);
    p_vessel_2->SetRadius(1.5e-5*unit::metres);
    p_vessel_3->SetRadius(7.e-6*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.2e-5*unit::metres);



std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(4.0*vessel_length, 2.0*vessel_length);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(1.0*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);





    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = 5_um;
    //p_grid->SetSpacing(grid_spacing);
    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = unsigned((domain_side_length-vessel_length)/(grid_spacing))+1; // num x
    //dimensions[1] = unsigned((domain_side_length)/(grid_spacing))+1; // num_y
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
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = LinnengarHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetLinnM(10.0);
    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUseDirectSolver(false);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance = 1.e-5;

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
}


void xTestNoCellsNontrivInvertedPiYang()
{

 // Set up the vessel network
    QLength vessel_length(50.0_um);
    QLength reference_length(1.0_um);

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
         /*
         * Now add the vessels to a vessel network.
         */



    std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->AddVessel(p_vessel_5);


        //QLength vessel_radius(1.0e-5*unit::metres);
        //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

        //vascular_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);


std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = p_network->GetVessels();

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

// Specify the domain
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivInvPiYang", true);

    double inlet_haematocrit = 0.25;
    double initial_haematocrit = 0.25;
    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);
p_vessel_1->SetRadius(2.e-5*unit::metres);
    p_vessel_2->SetRadius(1.5e-5*unit::metres);
    p_vessel_3->SetRadius(7.e-6*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.2e-5*unit::metres);



std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(4.0*vessel_length, 2.0*vessel_length);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(1.0*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);





    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = 5_um;
    //p_grid->SetSpacing(grid_spacing);
    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = unsigned((domain_side_length-vessel_length)/(grid_spacing))+1; // num x
    //dimensions[1] = unsigned((domain_side_length)/(grid_spacing))+1; // num_y
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
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = YangHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUseDirectSolver(false);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance = 1.e-5;

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
}



void xTestNoCellsNontrivInvertedPiModPries()
{

 // Set up the vessel network
    QLength vessel_length(50.0_um);
    QLength reference_length(1.0_um);

    std::shared_ptr<VesselNode<2> > p_node_1 = VesselNode<2>::Create(0.0*vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_2 = VesselNode<2>::Create(vessel_length, 30.0 * 1_um);
    std::shared_ptr<VesselNode<2> > p_node_3 = VesselNode<2>::Create(2.0*vessel_length, 30.0 * 1_um);
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
    //QLength dist_to_prev_bif = p_vessel_2->GetLength();
    //std::cout << dist_to_prev_bif << "\n";
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
         /*
         * Now add the vessels to a vessel network.
         */



    std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->AddVessel(p_vessel_5);


        //QLength vessel_radius(1.0e-5*unit::metres);
        //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

        //vascular_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);


std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;

    std::vector<std::shared_ptr<Vessel<2> > > vessels = p_network->GetVessels();

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

// Specify the domain
QDynamicViscosity viscosity = 1.e-3*unit::poiseuille;
    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivInvPiModPries_VaryingRadii", true);

    double inlet_haematocrit = 0.45;
    double initial_haematocrit = 0.45;
    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);

    p_vessel_1->SetRadius(2.e-5*unit::metres);
    p_vessel_2->SetRadius(1.5e-5*unit::metres);
    p_vessel_3->SetRadius(7.e-6*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.2e-5*unit::metres);

/*
    p_vessel_1->SetRadius(1.e-5*unit::metres);
    p_vessel_2->SetRadius(1.e-5*unit::metres);
    p_vessel_3->SetRadius(1.e-5*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.e-5*unit::metres);
*/

std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(4.0*vessel_length, 2.0*vessel_length);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(1.0*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);





    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = 5_um;
    //p_grid->SetSpacing(grid_spacing);
    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = unsigned((domain_side_length-vessel_length)/(grid_spacing))+1; // num x
    //dimensions[1] = unsigned((domain_side_length)/(grid_spacing))+1; // num_y
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
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = CFL_HardCoded_ModifiedPriesHaematocritSolver<2>::Create();
    auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    p_impedance_calculator->SetVesselNetwork(p_network);
    p_viscosity_calculator->SetVesselNetwork(p_network);

    p_impedance_calculator->Calculate();

    FlowSolver<2> flow_solver;
    flow_solver.SetVesselNetwork(p_network);
    flow_solver.SetUseDirectSolver(false);
    flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-9;

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
}


void xTestNoCellsNontrivInvertedPiModPriesImposedFluxes_CFL()
{

 // Set up the vessel network
    QLength vessel_length(50.0_um);
    QLength reference_length(1.0_um);

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
    //QLength dist_to_prev_bif = p_vessel_2->GetLength();
    //std::cout << dist_to_prev_bif << "\n";
    std::shared_ptr<VesselSegment<2> > p_segment_3 = VesselSegment<2>::Create(p_node_3, p_node_4);
    std::shared_ptr<Vessel<2> > p_vessel_3 = Vessel<2>::Create(p_segment_3);
    std::shared_ptr<VesselSegment<2> > p_segment_4 = VesselSegment<2>::Create(p_node_2, p_node_5);
    std::shared_ptr<Vessel<2> > p_vessel_4 = Vessel<2>::Create(p_segment_4);
    std::shared_ptr<VesselSegment<2> > p_segment_5 = VesselSegment<2>::Create(p_node_3, p_node_6);
    std::shared_ptr<Vessel<2> > p_vessel_5 = Vessel<2>::Create(p_segment_5);
         /*
         * Now add the vessels to a vessel network.
         */


    p_vessel_1->SetRadius(1.e-5*unit::metres);
    p_vessel_2->SetRadius(1.e-5*unit::metres);
    p_vessel_3->SetRadius(1.e-5*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.e-5*unit::metres);

    std::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->AddVessel(p_vessel_5);


        //QLength vessel_radius(1.0e-5*unit::metres);
        //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

        //vascular_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);






    auto p_file_handler = std::make_shared<OutputFileHandler>("TestNoCellsNontrivInvPiModPriesSameRadiiImposedFluxes_CFL_simple_30", true);

    double inlet_haematocrit = 0.55;
    double initial_haematocrit = 0.55;
    // Assign flow properties
    auto p_segment = p_network->GetVesselSegments()[0];
    p_segment->GetFlowProperties()->SetHaematocrit(initial_haematocrit);
    //p_segment->GetFlowProperties()->SetViscosity(viscosity);
    VesselNetworkPropertyManager<2>::SetSegmentProperties(p_network, p_segment);


    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);


/*
    p_vessel_1->SetRadius(1.e-5*unit::metres);
    p_vessel_2->SetRadius(1.e-5*unit::metres);
    p_vessel_3->SetRadius(1.e-5*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.e-5*unit::metres);
*/

std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(4.0*vessel_length, 2.0*vessel_length);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(1.0*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);





    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = 5_um;
    //p_grid->SetSpacing(grid_spacing);
    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = unsigned((domain_side_length-vessel_length)/(grid_spacing))+1; // num x
    //dimensions[1] = unsigned((domain_side_length)/(grid_spacing))+1; // num_y
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
    * Set up a finite difference solver and pass it the pde and grid.
    */
    auto p_oxygen_solver = SimpleLinearEllipticFiniteDifferenceSolver<2>::Create();
    p_oxygen_solver->SetPde(p_oxygen_pde);
    p_oxygen_solver->SetLabel("oxygen");
    p_oxygen_solver->SetGrid(p_grid);

    // auto = YangHaematocritSolver<2>::Create(); // etc etc
    auto p_haematocrit_calculator = CFL_HardCoded_ModifiedPriesHaematocritSolver<2>::Create();
    //auto p_impedance_calculator = VesselImpedanceCalculator<2>::Create();
    //auto p_viscosity_calculator = ViscosityCalculator<2>::Create();
    //p_viscosity_calculator->SetPlasmaViscosity(viscosity);

    p_haematocrit_calculator->SetVesselNetwork(p_network);
    p_haematocrit_calculator->SetHaematocrit(inlet_haematocrit);
    //p_impedance_calculator->SetVesselNetwork(p_network);
    //p_viscosity_calculator->SetVesselNetwork(p_network);

    //p_impedance_calculator->Calculate();

    //FlowSolver<2> flow_solver;
    //flow_solver.SetVesselNetwork(p_network);
    //flow_solver.SetUseDirectSolver(false);
    //flow_solver.SetUp();

    unsigned max_iter = 1000;
    double tolerance2 = 1.e-9;

    std::vector<VesselSegmentPtr<2> > segments = p_network->GetVesselSegments();
    std::vector<double> previous_haematocrit(segments.size(), double(initial_haematocrit));
    for(unsigned idx=0;idx<max_iter;idx++)
    {
        //p_impedance_calculator->Calculate();
        //flow_solver.SetUp();
        //flow_solver.Solve();
        p_haematocrit_calculator->Calculate();
        //p_viscosity_calculator->Calculate();
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
}


};

#endif // TESTNOCELLSBETTERIDGENONTRIVVIS_HPP
