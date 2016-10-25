/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is part of Chaste.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

class TestNetworkToSurface : public CxxTest::TestSuite
{
public:

    void TestSingleVessel()
    {
        // Set up the network
        double length = 100.0;
        double radius = 20.0;
        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0, length/2.0, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(length, length/2.0, 0.0);
        boost::shared_ptr<Vessel<3> > p_vessel = Vessel<3>::Create(p_node1, p_node2);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel);
        p_network->SetSegmentRadii(radius* 1.e-6 * unit::metres);
        p_network->SetNodeRadiiFromSegments();

        // Convert it to a surface
        NetworkToSurface<3> converter;
        converter.SetVesselNetwork(p_network);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/");
        GeometryWriter writer;
        writer.SetInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"single_vessel.vti");
        writer.Write();
    }

    void TestBifurcationVessel()
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

        // Convert it to a surface
        NetworkToSurface<3> converter;
        converter.SetVesselNetwork(p_network);
        converter.Update();

        // Write out the image
        OutputFileHandler file_handler1("TestNetworkToSurface/", false);
        GeometryWriter writer;
        writer.SetInput(converter.GetSurface());
        writer.SetFileName(file_handler1.GetOutputDirectoryFullPath()+"bifrucation_vessel.vti");
        writer.Write();
    }
};

#endif /*TESTNETWORKTOSURFACE_HPP_*/