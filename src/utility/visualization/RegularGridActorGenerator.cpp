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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkGlyph2D.h>
#include <vtkCubeAxesActor2D.h>
#include <vtkImageData.h>
#include <vtkGeometryFilter.h>
#include <vtkTubeFilter.h>
#include <vtkExtractEdges.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkPolygon.h>
#include <vtkIdList.h>
#include <vtkFeatureEdges.h>
#include <vtkColorTransferFunction.h>
#include <vtkScalarBarActor.h>
#include <vtkTextProperty.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "BaseUnits.hpp"
#include "VesselNetworkWriter.hpp"

#include "RegularGridActorGenerator.hpp"

template<unsigned DIM>
RegularGridActorGenerator<DIM>::RegularGridActorGenerator()
    : AbstractActorGenerator<DIM>(),
      mpRegularGrid(),
      mEdgeOpacity(1.0),
      mUseTubesForEdges(false)
{

}

template<unsigned DIM>
RegularGridActorGenerator<DIM>::~RegularGridActorGenerator()
{

}

template<unsigned DIM>
void RegularGridActorGenerator<DIM>::AddActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    if(mpRegularGrid)
    {
        vtkSmartPointer<vtkImageData> p_grid = vtkImageData::SafeDownCast(mpRegularGrid->GetGlobalVtkGrid());
        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_geom_filter->SetInput(p_grid);
        #else
            p_geom_filter->SetInputData(p_grid);
        #endif
        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        bool has_point_data = (!this->mDataLabel.empty() and p_grid->GetPointData()->HasArray(this->mDataLabel.c_str()));
        if(has_point_data)
        {
            double range[2];
            p_grid->GetPointData()->GetArray(this->mDataLabel.c_str())->GetRange(range);
            for(unsigned idx=0; idx<256; idx++)
            {
                double color[3];
                this->mpColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }

        // Add the points
        if(this->mShowPoints)
        {
            vtkSmartPointer<vtkSphereSource> p_spheres = vtkSmartPointer<vtkSphereSource>::New();
            p_spheres->SetRadius(this->mPointSize);
            p_spheres->SetPhiResolution(16);
            p_spheres->SetThetaResolution(16);

            vtkSmartPointer<vtkGlyph3D> p_glyph = vtkSmartPointer<vtkGlyph3D>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_glyph->SetInputConnection(p_geom_filter->GetOutputPort());
                p_glyph->SetSource(p_spheres->GetOutput());
            #else
                p_glyph->SetInputConnection(p_geom_filter->GetOutputPort());
                p_glyph->SetSourceConnection(p_spheres->GetOutputPort());
            #endif
            p_glyph->ClampingOff();
            p_glyph->SetScaleModeToScaleByScalar();
            p_glyph->SetScaleFactor(1.0);
            p_glyph->Update();
            vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_mapper->SetInput(p_glyph->GetOutput());
            #else
                p_mapper->SetInputData(p_glyph->GetOutput());
            #endif
            p_mapper->ScalarVisibilityOn();

            vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
            p_actor->SetMapper(p_mapper);
            p_actor->GetProperty()->SetColor(this->mPointColor[0]/255.0, this->mPointColor[1]/255.0, this->mPointColor[2]/255.0);
            pRenderer->AddActor(p_actor);
        }

        // Add the edges
        if(this->mShowEdges)
        {
            vtkSmartPointer<vtkFeatureEdges> p_edges = vtkSmartPointer<vtkFeatureEdges>::New();
            p_edges->SetInputConnection(p_geom_filter->GetOutputPort());
            p_edges->SetFeatureEdges(false);
            p_edges->SetBoundaryEdges(true);
            p_edges->SetManifoldEdges(true);
            p_edges->SetNonManifoldEdges(false);

            if(mUseTubesForEdges)
            {
                vtkSmartPointer<vtkTubeFilter> p_voronoi_tubes = vtkSmartPointer<vtkTubeFilter>::New();
                p_voronoi_tubes->SetInputConnection(p_edges->GetOutputPort());
                p_voronoi_tubes->SetRadius(this->mEdgeSize);
                p_voronoi_tubes->SetNumberOfSides(12);

                vtkSmartPointer<vtkPolyDataMapper> p_voronoi_tube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                p_voronoi_tube_mapper->SetInputConnection(p_voronoi_tubes->GetOutputPort());

                vtkSmartPointer<vtkActor> p_voronoi_tube_actor = vtkSmartPointer<vtkActor>::New();
                p_voronoi_tube_actor->SetMapper(p_voronoi_tube_mapper);
                p_voronoi_tube_actor->GetProperty()->SetColor(this->mVolumeColor[0]/255.0, this->mVolumeColor[1]/255.0, this->mVolumeColor[2]/255.0);
                p_voronoi_tube_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
                pRenderer->AddActor(p_voronoi_tube_actor);
            }
            else
            {
                vtkSmartPointer<vtkPolyDataMapper> p_edge_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                p_edge_mapper->SetInputConnection(p_edges->GetOutputPort());
                p_edge_mapper->ScalarVisibilityOff();

                vtkSmartPointer<vtkActor> p_edge_actor = vtkSmartPointer<vtkActor>::New();
                p_edge_actor->SetMapper(p_edge_mapper);
                p_edge_actor->GetProperty()->SetColor(this->mEdgeColor[0]/255.0, this->mEdgeColor[1]/255.0, this->mEdgeColor[2]/255.0);
                p_edge_actor->GetProperty()->SetOpacity(mEdgeOpacity);
                pRenderer->AddActor(p_edge_actor);
            }
        }
        // Add the volume
        if(this->mShowVolume)
        {
            vtkSmartPointer<vtkPolyDataMapper> p_grid_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            p_grid_mapper->SetInputConnection(p_geom_filter->GetOutputPort());
            if(has_point_data)
            {
                p_grid_mapper->SetLookupTable(p_scaled_ctf);
                p_grid_mapper->ScalarVisibilityOn();
                //p_grid_mapper->InterpolateScalarsBeforeMappingOn();
                p_grid_mapper->SelectColorArray(this->mDataLabel.c_str());
                p_grid_mapper->SetScalarModeToUsePointFieldData();
                p_grid_mapper->SetColorModeToMapScalars();
            }

            // Show edges
            vtkSmartPointer<vtkActor> p_volume_actor = vtkSmartPointer<vtkActor>::New();
            p_volume_actor->SetMapper(p_grid_mapper);
            p_volume_actor->GetProperty()->SetEdgeVisibility(1);
            p_volume_actor->GetProperty()->SetLineWidth(this->mEdgeSize);
            if(this->mDataLabel.empty())
            {
                p_volume_actor->GetProperty()->SetColor(this->mEdgeColor[0]/255.0, this->mEdgeColor[1]/255.0, this->mEdgeColor[2]/255.0);
            }
            p_volume_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
            pRenderer->AddActor(p_volume_actor);

            if(has_point_data)
            {
                vtkSmartPointer<vtkScalarBarActor> p_scale_bar = vtkSmartPointer<vtkScalarBarActor>::New();
                p_scale_bar->SetLookupTable(p_scaled_ctf);
                p_scale_bar->SetTitle(this->mDataLabel.c_str());
                p_scale_bar->SetOrientationToHorizontal();
                p_scale_bar->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
                p_scale_bar->GetPositionCoordinate()->SetValue(0.25, 0.84);
                p_scale_bar->SetWidth(0.5);
                p_scale_bar->SetHeight(0.1);
                p_scale_bar->GetTitleTextProperty()->ItalicOff();
                p_scale_bar->GetLabelTextProperty()->ItalicOff();
                p_scale_bar->GetTitleTextProperty()->BoldOff();
                p_scale_bar->GetLabelTextProperty()->BoldOff();
                p_scale_bar->SetLabelFormat("%.2g");
                p_scale_bar->GetTitleTextProperty()->SetFontSize(5.0);
                p_scale_bar->GetLabelTextProperty()->SetFontSize(5.0);
                p_scale_bar->GetTitleTextProperty()->SetColor(0.0, 0.0, 0.0);
                p_scale_bar->GetLabelTextProperty()->SetColor(0.0, 0.0, 0.0);
                pRenderer->AddActor(p_scale_bar);
            }
        }
    }
}

template<unsigned DIM>
void RegularGridActorGenerator<DIM>::SetRegularGrid(std::shared_ptr<RegularGrid<DIM> > pRegularGrid)
{
    mpRegularGrid = pRegularGrid;
}

template<unsigned DIM>
void RegularGridActorGenerator<DIM>::SetEdgeOpacity(double opacity)
{
    mEdgeOpacity = opacity;
}

template class RegularGridActorGenerator<2>;
template class RegularGridActorGenerator<3>;
