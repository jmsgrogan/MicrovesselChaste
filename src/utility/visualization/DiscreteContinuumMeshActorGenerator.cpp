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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#if VTK_MAJOR_VERSION > 5
    #include <vtkNamedColors.h>
#endif
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
#include <vtkScalarBarActor.h>
#include <vtkColorTransferFunction.h>
#include <vtkTextProperty.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "BaseUnits.hpp"

#include "DiscreteContinuumMeshActorGenerator.hpp"


template<unsigned DIM>
DiscreteContinuumMeshActorGenerator<DIM>::DiscreteContinuumMeshActorGenerator()
    : AbstractActorGenerator<DIM>(),
      mpDiscreteContinuumMesh()
{

}

template<unsigned DIM>
DiscreteContinuumMeshActorGenerator<DIM>::~DiscreteContinuumMeshActorGenerator()
{

}

template<unsigned DIM>
void DiscreteContinuumMeshActorGenerator<DIM>::AddActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    if(mpDiscreteContinuumMesh)
    {
        vtkSmartPointer<vtkUnstructuredGrid> p_grid = vtkUnstructuredGrid::SafeDownCast(mpDiscreteContinuumMesh->GetGlobalVtkGrid());

        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_geom_filter->SetInput(p_grid);
        #else
            p_geom_filter->SetInputData(p_grid);
        #endif

        p_geom_filter->Update();

        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!this->mDataLabel.empty() and p_grid->GetPointData()->HasArray(this->mDataLabel.c_str()))
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

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_mapper->SetInput(p_geom_filter->GetOutput());
        #else
            p_mapper->SetInputData(p_geom_filter->GetOutput());
        #endif
        if(!this->mDataLabel.empty() and p_grid->GetPointData()->HasArray(this->mDataLabel.c_str()))
        {
            p_mapper->SetLookupTable(p_scaled_ctf);
            p_mapper->ScalarVisibilityOn();
            p_mapper->SelectColorArray(this->mDataLabel.c_str());
            p_mapper->SetScalarModeToUsePointFieldData();
            p_mapper->SetColorModeToMapScalars();
        }

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        if(this->mShowEdges)
        {
            p_actor->GetProperty()->SetEdgeVisibility(1);
            p_actor->GetProperty()->SetLineWidth(this->mEdgeSize);
            p_actor->GetProperty()->SetColor(this->mEdgeColor[0]/255.0, this->mEdgeColor[1]/255.0, this->mEdgeColor[2]/255.0);
        }

        if(this->mDataLabel.empty())
        {
            p_actor->GetProperty()->SetColor(this->mVolumeColor[0],this->mVolumeColor[1], this->mVolumeColor[2]);
        }
        p_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
        pRenderer->AddActor(p_actor);

        if(!this->mDataLabel.empty() and p_grid->GetPointData()->HasArray(this->mDataLabel.c_str()))
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

template<unsigned DIM>
void DiscreteContinuumMeshActorGenerator<DIM>::SetDiscreteContinuumMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > pDiscreteContinuumMesh)
{
    mpDiscreteContinuumMesh = pDiscreteContinuumMesh;
}

template class DiscreteContinuumMeshActorGenerator<2>;
template class DiscreteContinuumMeshActorGenerator<3>;
