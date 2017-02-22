/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is VesselNetwork of Chaste.

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
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A VesselNetworkICULAR PURPOSE
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
#include <vtkCellData.h>
#include <vtkFeatureEdges.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkTextProperty.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "BaseUnits.hpp"
#include "VesselNetworkWriter.hpp"

#include "VesselNetworkActorGenerator.hpp"


template<unsigned DIM>
VesselNetworkActorGenerator<DIM>::VesselNetworkActorGenerator()
    : AbstractActorGenerator<DIM>(),
      mpVesselNetwork()
{

}

template<unsigned DIM>
VesselNetworkActorGenerator<DIM>::~VesselNetworkActorGenerator()
{

}

template<unsigned DIM>
void VesselNetworkActorGenerator<DIM>::AddActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    if(mpVesselNetwork)
    {
        VesselNetworkWriter<DIM> network_writer;
        network_writer.SetVesselNetwork(mpVesselNetwork);
        vtkSmartPointer<vtkPolyData> p_polydata = network_writer.GetOutput();
        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!this->mDataLabel.empty())
        {
            double range[2];
            p_polydata->GetPointData()->GetArray(this->mDataLabel.c_str())->GetRange(range);
            for(unsigned idx=0; idx<256; idx++)
            {
                double color[3];
                this->mpColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        if(this->mShowPoints)
        {
            vtkSmartPointer<vtkSphereSource> p_spheres = vtkSmartPointer<vtkSphereSource>::New();
            p_spheres->SetRadius(this->mPointSize);
            p_spheres->SetPhiResolution(16);
            p_spheres->SetThetaResolution(16);

            vtkSmartPointer<vtkGlyph3D> p_glyph = vtkSmartPointer<vtkGlyph3D>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_glyph->SetInput(p_polydata);
                p_glyph->SetSource(p_spheres->GetOutput());
            #else
                p_glyph->SetInputData(p_polydata);
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

            vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
            p_actor->SetMapper(p_mapper);
            p_actor->GetProperty()->SetColor(this->mPointColor[0]/255.0, this->mPointColor[1]/255.0, this->mPointColor[2]/255.0);
            pRenderer->AddActor(p_actor);
        }

        if(this->mShowEdges)
        {
            vtkSmartPointer<vtkPolyDataMapper> p_polydata_mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_polydata_mapper2->SetInput(p_polydata);
            #else
                p_polydata_mapper2->SetInputData(p_polydata);
            #endif

            vtkSmartPointer<vtkActor> p_actor2 = vtkSmartPointer<vtkActor>::New();
            p_actor2->SetMapper(p_polydata_mapper2);
            p_actor2->GetProperty()->SetColor(this->mEdgeColor[0]/255.0, this->mEdgeColor[1]/255.0, this->mEdgeColor[2]/255.0);
            pRenderer->AddActor(p_actor2);
        }

        if(this->mShowVolume)
        {
            vtkSmartPointer<vtkTubeFilter> p_tubes = vtkSmartPointer<vtkTubeFilter>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_tubes->SetInput(p_polydata);
            #else
                p_tubes->SetInputData(p_polydata);
            #endif
            p_tubes->SetRadius(this->mEdgeSize);
            p_tubes->SetNumberOfSides(16);

            vtkSmartPointer<vtkPolyDataMapper> p_tube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            p_tube_mapper->SetInputConnection(p_tubes->GetOutputPort());
            if(!this->mDataLabel.empty())
            {
                p_tube_mapper->SetLookupTable(p_scaled_ctf);
                p_tube_mapper->ScalarVisibilityOn();
                p_tube_mapper->InterpolateScalarsBeforeMappingOn();
                p_tube_mapper->SelectColorArray(this->mDataLabel.c_str());
                p_tube_mapper->SetScalarModeToUsePointFieldData();
                p_tube_mapper->SetColorModeToMapScalars();
            }

            vtkSmartPointer<vtkActor> p_tube_actor = vtkSmartPointer<vtkActor>::New();
            p_tube_actor->SetMapper(p_tube_mapper);
            if(this->mDataLabel.empty())
            {
                p_tube_actor->GetProperty()->SetColor(this->mVolumeColor[0]/255.0, this->mVolumeColor[1]/255.0, this->mVolumeColor[2]/255.0);
            }
            p_tube_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
            pRenderer->AddActor(p_tube_actor);

            if(!this->mDataLabel.empty())
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
void VesselNetworkActorGenerator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    this->mpVesselNetwork = pVesselNetwork;
}

template class VesselNetworkActorGenerator<2>;
template class VesselNetworkActorGenerator<3>;
