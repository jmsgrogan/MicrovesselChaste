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

#ifndef TESTVESSELBASEDDISCRETESOURCE_HPP_
#define TESTVESSELBASEDDISCRETESOURCE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "FunctionMap.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DensityMap.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "Owen11Parameters.hpp"

#include "BetteridgeHaematocritSolver.hpp"

#include "LinnengarHaematocritSolver.hpp"

//#include "MinimalLinnengarHaematocritSolver.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "FlowSolver.hpp"
#include "VesselImpedanceCalculator.hpp"

#include "VesselNetworkPropertyManager.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselBasedDiscreteSource : public CxxTest::TestSuite
{

public:

    void TestGridFunction()
    {
        auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestGridFunction");

        QLength vessel_length(100.0_um);
        QLength reference_length(1.0_um);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network =
                generator.GenerateSingleVessel(vessel_length, Vertex<2>());
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.4);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length);
        Vertex<2> translation_vector(-1.0*vessel_length/2.0);
        p_domain->Translate(translation_vector);

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength spacing(10.0_um);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Set up a density map
        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_grid);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_lin = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_lin->SetLinearInUValue(1.0*unit::per_second);
        p_vessel_source_lin->SetDensityMap(p_density_map);

        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source_const = VesselBasedDiscreteSource<2>::Create();
        p_vessel_source_const->SetConstantInUValue(2.0* unit::mole_per_metre_cubed_per_second);
        p_vessel_source_const->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_grid);

        // Get the source values at each point on the grid
        std::vector<QRate > point_rates = p_vessel_source_lin->GetLinearInUValues();
        std::vector<QConcentrationFlowRate > point_conc_rates = p_vessel_source_const->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_density_map->GetGridCalculator()->GetGrid()->GetNumberOfPoints(); idx++)
        {
            solution.push_back(double(point_rates[idx]/(1.0 * unit::per_second) + point_conc_rates[idx]/(2.0 * unit::mole_per_metre_cubed_per_second)));
        }

        solver.UpdateSolution(solution);
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();

        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
    }

    void TestMeshFunction()
    {
        auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestMeshFunction");

        QLength vessel_length(100.0_um);
        QLength reference_length(1.0_um);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length, Vertex<2>());
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.4);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length);
        Vertex<2> translation_vector(-1.0*vessel_length/2.0);
        p_domain->Translate(translation_vector);

        // Set up the grid
        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.02*vessel_length));
        p_mesh_generator->Update();

        // Set up a density map
        std::shared_ptr<DensityMap<2> > p_density_map = DensityMap<2>::Create();
        p_density_map->SetVesselNetwork(p_network);
        p_density_map->SetGrid(p_mesh_generator->GetMesh());

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_linear_point_source = VesselBasedDiscreteSource<2>::Create();
        p_linear_point_source->SetLinearInUValue(1.0 * unit::per_second);
        p_linear_point_source->SetDensityMap(p_density_map);

        std::shared_ptr<VesselBasedDiscreteSource<2> > p_const_point_source = VesselBasedDiscreteSource<2>::Create();
        QConcentrationFlowRate consumption_rate(2.0 * unit::mole_per_metre_cubed_per_second);
        p_const_point_source->SetConstantInUValue(consumption_rate);
        p_const_point_source->SetDensityMap(p_density_map);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());

        // Get the source values at each point on the grid
        std::vector<QRate > point_rates = p_linear_point_source->GetLinearInUValues();
        std::vector<QConcentrationFlowRate > point_conc_rates = p_const_point_source->GetConstantInUValues();
        std::vector<double> solution;
        for(unsigned idx=0; idx<point_conc_rates.size(); idx++)
        {
            solution.push_back(double(point_rates[idx]/(1.0 * unit::per_second) + point_conc_rates[idx]/(2.0 * unit::mole_per_metre_cubed_per_second)));
        }
        solver.UpdateElementSolution(solution);
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();

        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
    }

    void TestSimpleLinearEllipticFiniteDifferenceSolver()
    {
        auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver");

        // Set up the vessel network
        QLength vessel_length(100.0_um);
        QLength reference_length(1.0_um);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(vessel_length,
               Vertex<2>());

        QLength vessel_radius(1.0 *GenericParameters::mpCapillaryRadius->GetValue());
        VesselNetworkPropertyManager<2>::SetSegmentRadii(p_network, vessel_radius);

        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length);
        Vertex<2> translation_vector(-1.0*vessel_length/2.0);
        p_domain->Translate(translation_vector);
        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        //QLength spacing(10.0*unit::microns);
        QLength spacing(10.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, spacing);

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        //QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        
        //QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
               GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
               Owen11Parameters::mpReferencePartialPressure->GetValue("User");
std::cout << vessel_oxygen_concentration;
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_vessel_source->SetVesselPermeability(1.0*Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        //solver.SetUseDirectSolver(false);

        solver.SetFileHandler(p_output_file_handler);
        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
                solver.SetWriteSolution(true);
        solver.Solve();
    }


    void TestSimpleLinearEllipticFiniteDifferenceSolver_InvPI() throw(Exception)
    {
    auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver_InvPi_Bett");

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
      
    p_vessel_1->GetSegments()[0]->GetFlowProperties()->SetFlowRate(4.0 * unit::metre_cubed_per_second);
    p_vessel_2->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_3->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0* unit::metre_cubed_per_second);
    p_vessel_4->GetSegments()[0]->GetFlowProperties()->SetFlowRate(2.0 * unit::metre_cubed_per_second);
    p_vessel_5->GetSegments()[0]->GetFlowProperties()->SetFlowRate(1.0 * unit::metre_cubed_per_second);
 
 
         /*
         * Now add the vessels to a vessel network.
         */
    std::shared_ptr<VesselNetwork<2> > vascular_network = VesselNetwork<2>::Create();
    vascular_network->AddVessel(p_vessel_1);
    vascular_network->AddVessel(p_vessel_2);
    vascular_network->AddVessel(p_vessel_3);
    vascular_network->AddVessel(p_vessel_4);
    vascular_network->AddVessel(p_vessel_5);


        //QLength vessel_radius(1.0e-5*unit::metres);
        //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);

        //vascular_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);

    p_vessel_1->SetRadius(2.e-5*unit::metres);
    p_vessel_2->SetRadius(1.5e-5*unit::metres);
    p_vessel_3->SetRadius(7.e-6*unit::metres);
    p_vessel_4->SetRadius(1.e-5*unit::metres);
    p_vessel_5->SetRadius(1.2e-5*unit::metres);
    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    //std::shared_ptr<AlarconHaematocritSolver<2> > p_haematocrit_calculator(new AlarconHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    //p_haematocrit_calculator->SetLinnM(4.0);
    p_haematocrit_calculator->Calculate();

OutputFileHandler output_file_handler("TestSLEFDS_InvPi_Bett", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("InvPi_Bett.vtp");
    vascular_network->Write(output_filename);


std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(4.0*vessel_length, 2.0*vessel_length);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(1.0*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);
     
        //std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator =
        //        DiscreteContinuumMeshGenerator<2>::Create();
        //p_mesh_generator->SetDomain(p_domain);
        //QLength spacing(1.0*unit::microns);
        //p_mesh_generator->SetMaxElementArea(Qpow3(0.02*vessel_length));
        //p_mesh_generator->Update();




       
   

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        //QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        
        //QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
               GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
               Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_vessel_source->SetVesselPermeability(1.0*Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        //SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        //solver.SetGrid(p_grid);
        //solver.SetPde(p_pde);
        //solver.SetVesselNetwork(vascular_network);
        //solver.SetUseDirectSolver(false);

        //solver.SetFileHandler(p_output_file_handler);
        //vascular_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
         //       solver.SetWriteSolution(true);
        //solver.Solve();


        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        //solver.SetGrid(p_mesh_generator->GetMesh());
solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(vascular_network);
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }


   void TestSimpleLinearEllipticFiniteDifferenceSolver_Hexagonal_Betteridge() throw(Exception)
    {
    auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver_Hexagonal_Betteridge");

    QLength reference_length(1.0_um);
    //QLength vessel_radius(1.0 *GenericParameters::mpCapillaryRadius->GetValue());
    //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    //p_vessel_2->SetRadius(1.e-5*unit::metres);
// Set up the grid alternative way
        //std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        //p_domain->AddRectangle(1000.0 * 1_um, 1000.0 * 1_um);
        //std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        //QLength spacing(10.0*unit::microns);
        //QLength spacing(50.0*unit::microns);
        //p_grid->GenerateFromPart(p_domain, spacing);



std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
p_domain->AddRectangle(1000.0 * 1_um, 1000.0 * 1_um);

std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
QLength spacing(0.5*unit::microns);
p_grid->GenerateFromPart(p_domain, spacing);
     

    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = Owen11Parameters::mpLatticeSpacing->GetValue("User");
    //QLength grid_spacing = 10.0 * 1_um;
    //p_grid->SetSpacing(grid_spacing);

    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = 89; // num x
    //dimensions[1] = 97; // num_y
    //dimensions[2] = 1;
    //p_grid->SetDimensions(dimensions);

    // Specify the network dimensions
    QLength vessel_length = 80.0 * 1_um;

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
    p_segment->SetRadius(radius*1.e-6*unit::metres);
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
    FlowSolver<2> flowsolver;
    flowsolver.SetVesselNetwork(vascular_network);
    flowsolver.SetUseDirectSolver(false);
    flowsolver.SetUp();
    flowsolver.Solve();

    OutputFileHandler output_file_handler("TestSLEFDS_Hex_Bet", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("Hexagonal_Betteridge.vtp");
    vascular_network->Write(output_filename);

    //std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator(new LinnengarHaematocritSolver<2>());
    std::shared_ptr<BetteridgeHaematocritSolver<2> > p_haematocrit_calculator(new BetteridgeHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    //p_haematocrit_calculator->SetLinnM(3.0);
    p_haematocrit_calculator->Calculate();

       

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        //QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        
        //QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
               GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
               Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_vessel_source->SetVesselPermeability(1.0*Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(vascular_network);
        //solver.SetUseDirectSolver(false);

        solver.SetFileHandler(p_output_file_handler);
        vascular_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"network.vtp");
                solver.SetWriteSolution(true);
        solver.Solve();
    }


void TestSimpleLinearEllipticFiniteDifferenceSolver_Hexagonal_Linninger_FE() throw(Exception)
    {
    auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteDifferenceSolver_Hexagonal_Linninger");

    QLength reference_length(1.0_um);
    //QLength vessel_radius(1.0 *GenericParameters::mpCapillaryRadius->GetValue());
    //VesselNetworkPropertyManager<2>::SetSegmentRadii(vascular_network, vessel_radius);
    //p_vessel_2->SetRadius(1.e-5*unit::metres);
// Set up the grid alternative way
        //std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        //p_domain->AddRectangle(1000.0 * 1_um, 1000.0 * 1_um);
        //std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        //QLength spacing(10.0*unit::microns);
        //QLength spacing(50.0*unit::microns);
        //p_grid->GenerateFromPart(p_domain, spacing);



    //auto p_grid = RegularGrid<2>::Create();
    //QLength grid_spacing = Owen11Parameters::mpLatticeSpacing->GetValue("User");
    //QLength grid_spacing = 40.0 * 1_um;
    //p_grid->SetSpacing(grid_spacing);

    //c_vector<unsigned, 3> dimensions;
    //dimensions[0] = 23; // num x
    //dimensions[1] = 25; // num_y
    //dimensions[2] = 1;
    //p_grid->SetDimensions(dimensions);

    // Specify the network dimensions
    QLength vessel_length = 80.0 * 1_um;

    // Generate the network
    VesselNetworkGenerator<2> vascular_network_generator;
    std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalNetwork(1400.0 * 1_um,
                                                                                                                    1000.0 * 1_um,
                                                                                                                    vessel_length);

    std::vector<std::shared_ptr<VesselNode<2> > > nodes;
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(0_um,5_um)));
    nodes.push_back(std::shared_ptr<VesselNode<2> > (VesselNode<2>::Create(5_um,0_um)));
    std::shared_ptr<VesselSegment<2> > p_segment(VesselSegment<2>::Create(nodes[0], nodes[1]));

    double radius = 10.0;
    p_segment->SetRadius(radius*1.e-6*unit::metres);
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
    FlowSolver<2> flowsolver;
    flowsolver.SetVesselNetwork(vascular_network);
    //flowsolver.SetUseDirectSolver(false);
    flowsolver.SetUp();
    flowsolver.Solve();

    //std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator(new LinnengarHaematocritSolver<2>());
    std::shared_ptr<LinnengarHaematocritSolver<2> > p_haematocrit_calculator(new LinnengarHaematocritSolver<2>());
    p_haematocrit_calculator->SetVesselNetwork(vascular_network);
    //p_haematocrit_calculator->SetLinnM(3.0);
    p_haematocrit_calculator->Calculate();

       OutputFileHandler output_file_handler("TestSLEFDS_Hex_Lin", false);
    std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("Hexagonal_Linninger.vtp");
    vascular_network->Write(output_filename);


    std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
    p_domain->AddRectangle(1200.0 * 1_um, 960.5 * 1_um);
     
        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator =
               DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.5*vessel_length));
        p_mesh_generator->Update();

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde = DiscreteContinuumLinearEllipticPde<2>::Create();
        //QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        
        //QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        QConcentrationFlowRate consumption_rate(-2.e0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
               GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
               Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_vessel_source->SetReferenceHaematocrit(Owen11Parameters::mpInflowHaematocrit->GetValue("User"));
        p_vessel_source->SetVesselPermeability(1.0*Owen11Parameters::mpVesselOxygenPermeability->GetValue("User"));
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(vascular_network);
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }

    void TestSimpleLinearEllipticFiniteElementSolver()
    {
        auto p_output_file_handler =
                std::make_shared<OutputFileHandler>("TestVesselBasedDiscreteSource/TestSimpleLinearEllipticFiniteElementSolver");

        // Set up the vessel network
        QLength vessel_length(100.0_um);
        QLength reference_length(1.0_um);
        VesselNetworkGenerator<2> generator;
        std::shared_ptr<VesselNetwork<2> > p_network =
                generator.GenerateSingleVessel(vessel_length);
        p_network->GetVessels()[0]->GetFlowProperties()->SetHaematocrit(0.45);

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(vessel_length, vessel_length, Vertex<2>());
        Vertex<2> translation_vector(-1.0*vessel_length/2.0);
        p_domain->Translate(translation_vector);

        std::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator =
                DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        QLength spacing(10.0*unit::microns);
        p_mesh_generator->SetMaxElementArea(Qpow3(0.02*vessel_length));
        p_mesh_generator->Update();

        // Choose the PDE
        std::shared_ptr<DiscreteContinuumLinearEllipticPde<2> > p_pde =
                DiscreteContinuumLinearEllipticPde<2>::Create();
        QDiffusivity diffusivity(0.0033 * unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-2.0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetContinuumConstantInUTerm(consumption_rate);

        // Set up the discrete source
        // Set up the discrete source
        std::shared_ptr<VesselBasedDiscreteSource<2> > p_vessel_source = VesselBasedDiscreteSource<2>::Create();
        QSolubility oxygen_solubility_at_stp = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        QConcentration vessel_oxygen_concentration = oxygen_solubility_at_stp *
                Owen11Parameters::mpReferencePartialPressure->GetValue("User");
        p_vessel_source->SetReferenceConcentration(vessel_oxygen_concentration);
        p_pde->AddDiscreteSource(p_vessel_source);

        // Set up and run the solver
        SimpleLinearEllipticFiniteElementSolver<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_pde);
        solver.SetVesselNetwork(p_network);
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTVESSELBASEDDISCRETESOURCE_HPP_*/
