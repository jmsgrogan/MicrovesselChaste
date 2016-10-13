/*

 Copyright (c) 2005-2015, University of Oxford.
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

#include "NetworkToSurface.hpp"
#include "NetworkToImage.hpp"
#include <vtkMarchingSquares.h>
#include <vtkMarchingCubes.h>
#include <vtkCleanPolyData.h>
#include <vtkStripper.h>
#include <vtkSplineFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkTransform.h>
#include <vtkClipPolyData.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkBox.h>
#include <vtkProbeFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkvmtkSimpleCapPolyData.h>
#include <vtkvmtkPolyDataSizingFunction.h>
#include <vtkvmtkPolyDataToUnstructuredGridFilter.h>
#include "UblasCustomFunctions.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
NetworkToSurface<DIM>::NetworkToSurface() :
    mpNetwork(),
    mSamplingGridSpacing(2.0),
    mSplineResamplingLength(10.0),
    mpSurface(),
    mpImage()
{

}

template<unsigned DIM>
boost::shared_ptr<NetworkToSurface<DIM> > NetworkToSurface<DIM>::Create()
{
    MAKE_PTR(NetworkToSurface<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
NetworkToSurface<DIM>::~NetworkToSurface()
{

}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> NetworkToSurface<DIM>::GetSurface()
{
    return mpSurface;
}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> NetworkToSurface<DIM>::GetSamplingImage()
{
    return mpImage;
}

template<unsigned DIM>
void NetworkToSurface<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void NetworkToSurface<DIM>::SetResamplingSplineSize(double splineResampleSize)
{
    mSplineResamplingLength = splineResampleSize;
}

template<unsigned DIM>
void NetworkToSurface<DIM>::SetResamplingGridSize(double sampleGridSize)
{
    mSamplingGridSpacing = sampleGridSize;
}

template<unsigned DIM>
void NetworkToSurface<DIM>::Update()
{
    if(DIM==2)
    {
        // Interpolate the network onto a regular grid
        boost::shared_ptr<NetworkToImage<DIM> > p_net_to_image = NetworkToImage<DIM>::Create();
        p_net_to_image->SetNetwork(mpNetwork);
        p_net_to_image->SetGridSpacing(mSamplingGridSpacing);
        p_net_to_image->SetPaddingFactors(0.1, 0.1, 0.0);
        p_net_to_image->SetImageDimension(2);
        p_net_to_image->Update();
        mpImage = p_net_to_image->GetOutput();

        // Get the outer boundaries of the network
        vtkSmartPointer<vtkMarchingSquares> p_squares = vtkSmartPointer<vtkMarchingSquares>::New();
        p_squares->SetInputData(mpImage);
        p_squares->SetValue(0, 1);

        // Remove duplicate points and join up lines to form polylines
        vtkSmartPointer<vtkCleanPolyData> p_cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        p_cleaner->SetInputConnection(p_squares->GetOutputPort());

        vtkSmartPointer<vtkStripper> p_stripper = vtkSmartPointer<vtkStripper>::New();
        p_stripper->SetInputConnection(p_cleaner->GetOutputPort());

        // Downsample and smooth the polyline
        vtkSmartPointer<vtkSplineFilter> p_spline= vtkSmartPointer<vtkSplineFilter>::New();
        p_spline->SetLength(mSplineResamplingLength);
        p_spline->SetSubdivideToLength();
        p_spline->SetInputConnection(p_stripper->GetOutputPort());

        vtkSmartPointer<vtkTriangleFilter> p_triangle = vtkSmartPointer<vtkTriangleFilter>::New();
        p_triangle->SetInputConnection(p_spline->GetOutputPort());
        p_triangle->Update();

        vtkSmartPointer<vtkPolyData> p_cleaned = p_triangle->GetOutput();

        // Want flat ends on input and output network nodes. It is important to do this after smoothing
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
        c_vector<double, 3> box_axis = unit_vector<double>(3,0);

        for(unsigned idx=0; idx< nodes.size(); idx++)
        {
            if(nodes[idx]->GetFlowProperties()->IsInputNode() or nodes[idx]->GetFlowProperties()->IsOutputNode())
            {
                double radius = nodes[idx]->GetRadius()/nodes[idx]->GetReferenceLengthScale();
                vtkSmartPointer<vtkBox> p_box = vtkSmartPointer<vtkBox>::New();
                p_box->SetBounds(-1.1*radius, 0.0, -1.1*radius, 1.1*radius, - 1.1*radius, 1.1*radius);

                c_vector<double, 3> loc;
                loc[0]= nodes[idx]->rGetLocation()[0];
                loc[1]= nodes[idx]->rGetLocation()[1];
                loc[2]=0.0;

                c_vector<double, 3> tangent;
                tangent[0]= nodes[idx]->GetSegment(0)->GetOppositeNode(nodes[idx])->rGetLocation()[0] - loc[0];
                tangent[1]= nodes[idx]->GetSegment(0)->GetOppositeNode(nodes[idx])->rGetLocation()[1] - loc[1];
                tangent[2] = 0.0;
                tangent /=norm_2(tangent);

                double rotation_angle = std::acos(inner_prod(box_axis, tangent))*(180.0/M_PI);
                c_vector<double, 3> rotation_axis = VectorProduct(box_axis, tangent);

                vtkSmartPointer<vtkTransform> p_tranform = vtkSmartPointer<vtkTransform>::New();
                if (std::abs(inner_prod(box_axis, tangent)) < 1.0 - 1.e-6)
                {
                    p_tranform->RotateWXYZ(-rotation_angle, rotation_axis[0], rotation_axis[1], rotation_axis[2]);
                }
                else
                {
                    p_tranform->RotateWXYZ(-rotation_angle, 0.0, 0.0, 1.0);
                }
                p_tranform->Translate(-loc[0],-loc[1], -loc[2]);
                p_box->SetTransform(p_tranform);

                vtkSmartPointer<vtkClipPolyData> p_clipper = vtkSmartPointer<vtkClipPolyData>::New();
                p_clipper->SetInputData(p_cleaned);
                p_clipper->SetClipFunction(p_box);
                p_clipper->Update();

                // Assuming we have two points with connectivity 1, join them
                std::vector<unsigned> edge_ids;
                vtkSmartPointer<vtkIdList> p_cell_list = vtkSmartPointer<vtkIdList>::New();
                for (unsigned idx =0; idx< p_clipper->GetOutput()->GetNumberOfPoints(); idx++)
                {
                    p_clipper->GetOutput()->GetPointCells(idx, p_cell_list);
                    if(p_cell_list->GetNumberOfIds()==1)
                    {
                        edge_ids.push_back(idx);
                    }
                }

                vtkSmartPointer<vtkLine> p_line = vtkSmartPointer<vtkLine>::New();
                p_line->GetPointIds()->SetId(0, edge_ids[0]);
                p_line->GetPointIds()->SetId(1, edge_ids[1]);
                p_clipper->GetOutput()->GetLines()->InsertNextCell(p_line);

                p_cleaned->DeepCopy(p_clipper->GetOutput());
            }
        }
        mpSurface = p_cleaned;
    }
    else
    {
        // Interpolate the network onto a regular grid
        boost::shared_ptr<NetworkToImage<DIM> > p_net_to_image = NetworkToImage<DIM>::Create();
        p_net_to_image->SetNetwork(mpNetwork);
        p_net_to_image->SetGridSpacing(mSamplingGridSpacing);
        p_net_to_image->SetPaddingFactors(0.1, 0.1, 0.1);
        p_net_to_image->SetImageDimension(3);
        p_net_to_image->Update();
        mpImage = p_net_to_image->GetOutput();

        // Get the outer boundaries of the network
        vtkSmartPointer<vtkMarchingCubes> p_cubes = vtkSmartPointer<vtkMarchingCubes>::New();
        p_cubes->SetInputData(mpImage);
        p_cubes->SetValue(0, 1);

        vtkSmartPointer<vtkCleanPolyData> p_cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        p_cleaner->SetInputConnection(p_cubes->GetOutputPort());

        vtkSmartPointer<vtkWindowedSincPolyDataFilter> p_smoother = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
        p_smoother->SetInputConnection(p_cleaner->GetOutputPort());
        p_smoother->SetNumberOfIterations(500.0);
        p_smoother->BoundarySmoothingOn();
        p_smoother->FeatureEdgeSmoothingOn();
        p_smoother->SetFeatureAngle(30.0);
        p_smoother->SetPassBand(0.03);
        p_smoother->NonManifoldSmoothingOn();
        p_smoother->NormalizeCoordinatesOn();
        p_smoother->Update();

        vtkSmartPointer<vtkPolyData> p_cleaned = p_smoother->GetOutput();

        // Open any ends marked as inlet or outlet nodes
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
        c_vector<double, 3> box_axis = unit_vector<double>(3,0);
        for(unsigned idx=0; idx< nodes.size(); idx++)
        {
            if(nodes[idx]->GetFlowProperties()->IsInputNode() or nodes[idx]->GetFlowProperties()->IsOutputNode())
            {
                double radius = nodes[idx]->GetRadius()/nodes[idx]->GetReferenceLengthScale();
                vtkSmartPointer<vtkBox> p_box = vtkSmartPointer<vtkBox>::New();

                p_box->SetBounds(-1.1*radius, 0.0, -1.1*radius, 1.1*radius, - 1.1*radius, 1.1*radius);

                c_vector<double, 3> loc = nodes[idx]->rGetLocation();
                c_vector<double, 3> tangent;
                tangent = nodes[idx]->GetSegment(0)->GetOppositeNode(nodes[idx])->rGetLocation() - loc;
                tangent /=norm_2(tangent);

                double rotation_angle = std::acos(inner_prod(box_axis, tangent))*(180.0/M_PI);
                c_vector<double, 3> rotation_axis = VectorProduct(box_axis, tangent);

                vtkSmartPointer<vtkTransform> p_tranform = vtkSmartPointer<vtkTransform>::New();
                if (std::abs(inner_prod(box_axis, tangent)) < 1.0 - 1.e-6)
                {
                    p_tranform->RotateWXYZ(-rotation_angle, rotation_axis[0], rotation_axis[1], rotation_axis[2]);
                }
                else
                {
                    p_tranform->RotateWXYZ(-rotation_angle, 0.0, 0.0, 1.0);
                }
                p_tranform->Translate(-loc[0],-loc[1], -loc[2]);
                p_box->SetTransform(p_tranform);

                vtkSmartPointer<vtkClipPolyData> p_clipper = vtkSmartPointer<vtkClipPolyData>::New();
                p_clipper->SetInputData(p_cleaned);
                p_clipper->SetClipFunction(p_box);
                p_clipper->Update();
                p_cleaned->DeepCopy(p_clipper->GetOutput());
            }
        }

        // Use vmtk to cap the surface
        vtkSmartPointer<vtkvmtkSimpleCapPolyData> p_capper = vtkSmartPointer<vtkvmtkSimpleCapPolyData>::New();
        p_capper->SetInputData(p_cleaned);
        p_capper->SetCellEntityIdsArrayName("CellEntityIds");
        p_capper->SetCellEntityIdOffset(1);
        p_capper->Update();

        vtkSmartPointer<vtkPolyDataNormals> p_normals = vtkSmartPointer<vtkPolyDataNormals>::New();
        p_normals->SetInputConnection(p_capper->GetOutputPort());
        p_normals->AutoOrientNormalsOn();
        p_normals->SplittingOff();
        p_normals->ConsistencyOn();
        p_normals->Update();

        mpSurface = p_normals->GetOutput();
    }
}

// Explicit instantiation
template class NetworkToSurface<2>;
template class NetworkToSurface<3>;
