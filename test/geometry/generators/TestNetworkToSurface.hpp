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

#ifndef TESTNETWORKTOSURFACE_HPP_
#define TESTNETWORKTOSURFACE_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <math.h>
#include "FileFinder.hpp"
#include "VesselNetwork.hpp"
#include "OutputFileHandler.hpp"
#include "NetworkToSurface.hpp"
#include "GeometryWriter.hpp"
#include "PetscTools.hpp"
#include "VesselNetworkGenerator.hpp"
#include "RandomNumberGenerator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestNetworkToSurface : public CxxTest::TestSuite
{
public:

    void TestSingleVessel2d() throw(Exception)
    {
        std::string output_path = "TestNetworkToSurface";
        if(PetscTools::IsParallel())
        {
            output_path += "Parallel";
        }

        // Set up the network
        double length = 100.0;
        double radius = 20.0;
        boost::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0, length/2.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(length, length/2.0, 0.0);
        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node2->GetFlowProperties()->SetIsOutputNode(true);
        boost::shared_ptr<Vessel<2> > p_vessel = Vessel<2>::Create(p_node1, p_node2);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel);
        p_network->SetSegmentRadii(radius* 1.e-6 * unit::metres);
        p_network->SetNodeRadiiFromSegments();

        // Convert it to a surface
        boost::shared_ptr<NetworkToSurface<2> > p_converter = NetworkToSurface<2>::Create();
        p_converter->SetVesselNetwork(p_network);
        p_converter->SetResamplingSplineSize(10.0 * 1.e-6 * unit::metres);
        p_converter->GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        p_converter->SetRemeshingTargetEdgeLength(1.0);
        p_converter->Update();

        // Write out the image
        OutputFileHandler file_handler1(output_path);
        GeometryWriter writer;
        writer.AddInput(p_converter->GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"single_vessel_2d.vtp");
        writer.Write();
    }

    void TestSingleVessel() throw(Exception)
    {
        // Set up the network
        double length = 100.0;
        double radius = 20.0;
        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0, length/2.0, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(length, length/2.0, 0.0);
        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node2->GetFlowProperties()->SetIsOutputNode(true);
        boost::shared_ptr<Vessel<3> > p_vessel = Vessel<3>::Create(p_node1, p_node2);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel);
        p_network->SetSegmentRadii(radius* 1.e-6 * unit::metres);
        p_network->SetNodeRadiiFromSegments();

        // Convert it to a surface
        boost::shared_ptr<NetworkToSurface<3> > p_converter = NetworkToSurface<3>::Create();
        p_converter->SetVesselNetwork(p_network);
        p_converter->GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        p_converter->GetNetworkToImageTool()->SetPaddingFactors(0.1, 0.1, 0.1);
        p_converter->SetDoSmoothing(true);
        p_converter->SetNumSmoothingIterations(30);
        p_converter->SetSmoothingFeatureAngle(180);
        p_converter->SetBandPassFrequency(0.1);
        p_converter->SetRemeshingTargetEdgeLength(10.0);
        p_converter->Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.AddInput(p_converter->GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"single_vessel.vtp");
        writer.Write();
    }

    void TestBifurcationVessel2d() throw(Exception)
    {
        // Set up the network
        double length = 100.0;
        double radius = 20.0;

        boost::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0, length, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(length, length, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node3 = VesselNode<2>::Create(2.0 * length, 2.0*length, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node4 = VesselNode<2>::Create(2.0 * length, 0.0, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node5 = VesselNode<2>::Create(3.0 * length, length, 0.0);
        boost::shared_ptr<VesselNode<2> > p_node6 = VesselNode<2>::Create(4.0 * length, length, 0.0);
        boost::shared_ptr<Vessel<2> > p_vessel1 = Vessel<2>::Create(p_node1, p_node2);
        boost::shared_ptr<Vessel<2> > p_vessel2 = Vessel<2>::Create(p_node2, p_node3);
        boost::shared_ptr<Vessel<2> > p_vessel3 = Vessel<2>::Create(p_node2, p_node4);
        boost::shared_ptr<Vessel<2> > p_vessel4 = Vessel<2>::Create(p_node3, p_node5);
        boost::shared_ptr<Vessel<2> > p_vessel5 = Vessel<2>::Create(p_node4, p_node5);
        boost::shared_ptr<Vessel<2> > p_vessel6 = Vessel<2>::Create(p_node5, p_node6);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);
        p_network->AddVessel(p_vessel4);
        p_network->AddVessel(p_vessel5);
        p_network->AddVessel(p_vessel6);
        p_network->SetSegmentRadii(radius* 1.e-6 * unit::metres);
        p_network->SetNodeRadiiFromSegments();
        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node6->GetFlowProperties()->SetIsOutputNode(true);

        // Convert it to a surface
        NetworkToSurface<2> converter;
        converter.SetVesselNetwork(p_network);
        converter.GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        converter.GetNetworkToImageTool()->SetPaddingFactors(0.1, 0.1, 0.0);
        converter.SetDoSmoothing(true);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.AddInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"bifrucation_vessel_2d.vtp");
        writer.Write();
    }

    void TestBifurcationVessel() throw(Exception)
    {
        // Set up the network
        double length = 100.0;
        double radius = 20.0;

        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0, length, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(length, length, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node3 = VesselNode<3>::Create(2.0 * length, 2.0*length, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node4 = VesselNode<3>::Create(2.0 * length, 0.0, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node5 = VesselNode<3>::Create(3.0 * length, length, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node6 = VesselNode<3>::Create(4.0 * length, length, 0.0);
        boost::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(p_node1, p_node2);
        boost::shared_ptr<Vessel<3> > p_vessel2 = Vessel<3>::Create(p_node2, p_node3);
        boost::shared_ptr<Vessel<3> > p_vessel3 = Vessel<3>::Create(p_node2, p_node4);
        boost::shared_ptr<Vessel<3> > p_vessel4 = Vessel<3>::Create(p_node3, p_node5);
        boost::shared_ptr<Vessel<3> > p_vessel5 = Vessel<3>::Create(p_node4, p_node5);
        boost::shared_ptr<Vessel<3> > p_vessel6 = Vessel<3>::Create(p_node5, p_node6);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);
        p_network->AddVessel(p_vessel4);
        p_network->AddVessel(p_vessel5);
        p_network->AddVessel(p_vessel6);
        p_network->SetSegmentRadii(radius* 1.e-6 * unit::metres);
        p_network->SetNodeRadiiFromSegments();
        p_node1->GetFlowProperties()->SetIsInputNode(true);
        p_node6->GetFlowProperties()->SetIsOutputNode(true);

        // Convert it to a surface
        NetworkToSurface<3> converter;
        converter.SetVesselNetwork(p_network);
        converter.GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        converter.GetNetworkToImageTool()->SetPaddingFactors(0.1, 0.1, 0.1);
        converter.SetDoSmoothing(true);
        converter.SetNumSmoothingIterations(30);
        converter.SetSmoothingFeatureAngle(180);
        converter.SetBandPassFrequency(0.1);
        converter.SetRemeshingTargetEdgeLength(10.0);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.AddInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"bifrucation_vessel.vtp");
        writer.Write();
    }

    void TestHexNetworkVessel2d() throw(Exception)
    {
        VesselNetworkGenerator<2> network_generator = VesselNetworkGenerator<2>();
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateHexagonalNetwork(500*1e-6*unit::metres,
                500*1e-6*unit::metres, 100*1e-6*unit::metres, true);

        boost::shared_ptr<VesselNode<2> > p_bottom_left_node = p_network->GetNearestNode(
                DimensionalChastePoint<2>(0.0, 0.0, 0.0, 1e-6*unit::metres));
        boost::shared_ptr<VesselNode<2> > p_top_right_node = p_network->GetNearestNode(
                DimensionalChastePoint<2>(600, 600, 0.0, 1e-6*unit::metres));
        p_bottom_left_node->GetFlowProperties()->SetIsInputNode(true);
        p_top_right_node->GetFlowProperties()->SetIsOutputNode(true);
        for(unsigned idx=0;idx<p_network->GetVesselSegments().size();idx++)
        {
            double rand = RandomNumberGenerator::Instance()->ranf();
            p_network->GetVesselSegments()[idx]->SetRadius((10.0+20.0*rand)*1.e-6*unit::metres);
        }
        p_network->SetNodeRadiiFromSegments();

        // Convert it to a surface
        NetworkToSurface<2> converter;
        converter.SetVesselNetwork(p_network);
        converter.GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        converter.GetNetworkToImageTool()->SetPaddingFactors(0.1, 0.1, 0.0);
        converter.SetDoSmoothing(true);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.AddInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"ex_network_2d.vtp");
        writer.Write();
    }

    void TestHexNetworkVessel3d() throw(Exception)
    {
        VesselNetworkGenerator<3> network_generator = VesselNetworkGenerator<3>();
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(500*1e-6*unit::metres,
                500*1e-6*unit::metres, 100*1e-6*unit::metres, true);

        boost::shared_ptr<VesselNode<3> > p_bottom_left_node = p_network->GetNearestNode(
                DimensionalChastePoint<3>(0.0, 0.0, 0.0, 1e-6*unit::metres));
        boost::shared_ptr<VesselNode<3> > p_top_right_node = p_network->GetNearestNode(
                DimensionalChastePoint<3>(600, 600, 0.0, 1e-6*unit::metres));
        p_bottom_left_node->GetFlowProperties()->SetIsInputNode(true);
        p_top_right_node->GetFlowProperties()->SetIsOutputNode(true);
        for(unsigned idx=0;idx<p_network->GetVesselSegments().size();idx++)
        {
            double rand = RandomNumberGenerator::Instance()->ranf();
            p_network->GetVesselSegments()[idx]->SetRadius((10.0+20.0*rand)*1.e-6*unit::metres);
        }
        p_network->SetNodeRadiiFromSegments();

        // Convert it to a surface
        NetworkToSurface<3> converter;
        converter.SetVesselNetwork(p_network);
        converter.GetNetworkToImageTool()->SetGridSpacing(2.0 * 1.e-6 * unit::metres);
        converter.GetNetworkToImageTool()->SetPaddingFactors(0.1, 0.1, 0.1);
        converter.SetDoSmoothing(true);
        converter.SetNumSmoothingIterations(30);
        converter.SetSmoothingFeatureAngle(180);
        converter.SetBandPassFrequency(0.1);
        converter.SetRemeshingTargetEdgeLength(10.0);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.AddInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"ex_network_3d.vtp");
        writer.Write();
    }
};

#endif /*TESTNETWORKTOSURFACE_HPP_*/
