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

#ifndef TESTMicrovesselVtkScene_HPP_
#define TESTMicrovesselVtkScene_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "MicrovesselVtkScene.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "Part.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNetwork.hpp"

class TestMicrovesselVtkScene : public CxxTest::TestSuite
{
public:

    void TestGrid2d()
    {
        OutputFileHandler file_handler1 = OutputFileHandler("TestMicrovesselVtkScene/TestGrid2d");

        // Generate a grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        std::vector<unsigned> extents(3, 1);
        extents[0] = 20;
        extents[1] = 20;
        p_grid->SetExtents(extents);
        p_grid->SetSpacing(20.e-6*unit::metres);

        std::vector<double> grid_values;
        for(unsigned idx=0; idx < p_grid->GetNumberOfPoints(); idx++)
        {
            grid_values.push_back(double(idx)*10.0 + 100.0);
        }
        p_grid->SetPointValues(grid_values);

        MicrovesselVtkScene<2> scene1;
        scene1.SetRegularGrid(p_grid);
        scene1.GetRegularGridActorGenerator()->SetShowPoints(false);
        scene1.GetRegularGridActorGenerator()->SetShowVolume(true);
        scene1.GetRegularGridActorGenerator()->SetDataLabel("Point Values");
        scene1.GetRegularGridActorGenerator()->SetEdgeSize(1.0);
        scene1.GetRegularGridActorGenerator()->SetVolumeOpacity(1.0);

        scene1.SetIsInteractive(false);
        scene1.SetSaveAsAnimation(false);
        scene1.SetSaveAsImages(true);
        scene1.SetOutputFilePath(file_handler1.GetOutputDirectoryFullPath()+"render");
        scene1.Start();
    }

    void TestGrid3d()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestMicrovesselVtkScene/TestGrid3d/");

        // Generate a grid
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        std::vector<unsigned> extents(3, 5);
        extents[0] = 20;
        extents[1] = 20;
        p_grid->SetExtents(extents);
        p_grid->SetSpacing(20.e-6*unit::metres);

        std::vector<double> grid_values;
        for(unsigned idx=0; idx < p_grid->GetNumberOfPoints(); idx++)
        {
            grid_values.push_back(double(idx)*10.0 + 100.0);
        }
        p_grid->SetPointValues(grid_values);

        MicrovesselVtkScene<3> scene1;
        scene1.SetRegularGrid(p_grid);
        scene1.GetRegularGridActorGenerator()->SetShowPoints(false);
        scene1.GetRegularGridActorGenerator()->SetShowVolume(true);
        scene1.GetRegularGridActorGenerator()->SetDataLabel("Point Values");
        scene1.GetRegularGridActorGenerator()->SetEdgeSize(1.0);

        scene1.SetIsInteractive(false);
        scene1.SetSaveAsAnimation(false);
        scene1.SetSaveAsImages(true);
        scene1.SetOutputFilePath(file_handler1.GetOutputDirectoryFullPath()+"render");
        scene1.Start();
    }

    void TestVesselNetwork()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestMicrovesselVtkScene/TestVesselNetwork");


        // Specify the network dimensions
        units::quantity<unit::length> vessel_length = 40.0* 1.e-6 * unit::metres;

        // Generate the network
        VesselNetworkGenerator<3> vascular_network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = vascular_network_generator.GenerateHexagonalUnit(vessel_length);
        p_network->UpdateVesselIds();

        MicrovesselVtkScene<3> scene1;
        scene1.SetVesselNetwork(p_network);

        scene1.GetVesselNetworkActorGenerator()->SetShowPoints(false);
        scene1.GetVesselNetworkActorGenerator()->SetShowVolume(true);
        scene1.GetVesselNetworkActorGenerator()->SetShowEdges(false);
        scene1.GetVesselNetworkActorGenerator()->SetEdgeSize(10.0);
        scene1.GetVesselNetworkActorGenerator()->SetDataLabel("Node Id");

        scene1.SetIsInteractive(false);
        scene1.SetSaveAsAnimation(false);
        scene1.SetSaveAsImages(true);
        scene1.SetOutputFilePath(file_handler1.GetOutputDirectoryFullPath()+"render");
        scene1.Start();
    }

    void TestPart()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestMicrovesselVtkScene/TestPart");
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(100.e-6*unit::metres, 100.e-6*unit::metres, 100.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0, 1.e-6*unit::metres));

        MicrovesselVtkScene<3> scene1;
        c_vector<double, 3> red;
        red[0] = 255.0;
        red[1] = 0.0;
        red[2] = 0.0;
        c_vector<double, 3> green;
        green[0] = 0.0;
        green[1] = 255.0;
        green[2] = 0.0;
        c_vector<double, 3> blue;
        blue[0] = 0.0;
        blue[1] = 0.0;
        blue[2] = 255.0;
        c_vector<double, 3> black;
        black[0] = 0.0;
        black[1] = 0.0;
        black[2] = 0.0;
        c_vector<double, 3> white;
        white[0] = 255.0;
        white[1] = 255.0;
        white[2] = 255.0;

        scene1.SetPart(p_part);
        scene1.GetPartActorGenerator()->SetShowPoints(true);
        scene1.GetPartActorGenerator()->SetShowVolume(true);
        scene1.GetPartActorGenerator()->SetPointSize(5.0);
        scene1.GetPartActorGenerator()->SetEdgeSize(0.1);
        scene1.GetPartActorGenerator()->SetVolumeColor(blue);
        scene1.GetPartActorGenerator()->SetEdgeColor(red);
        scene1.GetPartActorGenerator()->SetPointColor(red);

        scene1.SetIsInteractive(false);
        scene1.SetSaveAsAnimation(false);
        scene1.SetSaveAsImages(true);
        scene1.SetOutputFilePath(file_handler1.GetOutputDirectoryFullPath()+"render");
        scene1.Start();
    }
};
#endif
