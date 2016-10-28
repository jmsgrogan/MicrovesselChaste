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

#include "Exception.hpp"
#include <boost/filesystem.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
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
#include <vtkCubeAxesActor2D.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkActorCollection.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkTubeFilter.h>
#include <vtkExtractEdges.h>
#include "MicrovesselVtkScene.hpp"
#include "BaseUnits.hpp"
#include "VesselNetworkWriter.hpp"
#include "Debug.hpp"

// For some reason an explicit interactor style is needed capture mouse events
class customMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static customMouseInteractorStyle* New();
    vtkTypeMacro(customMouseInteractorStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnLeftButtonDown()
    {
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    virtual void OnMiddleButtonDown()
    {
      // Forward events
      vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
    }

    virtual void OnRightButtonDown()
    {
      // Forward events
      vtkInteractorStyleTrackballCamera::OnRightButtonDown();
    }

};

vtkStandardNewMacro(customMouseInteractorStyle);

template<unsigned DIM>
MicrovesselVtkScene<DIM>::MicrovesselVtkScene()
    : mpRenderer(vtkSmartPointer<vtkRenderer>::New()),
      mpRenderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
      mpRenderWindowInteractor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
      mpPart(),
      mpNetwork(),
      mpMesh(),
      mpGrid(),
      mpCellPopulation(),
      mOutputFilePath(),
      mLengthScale(BaseUnits::Instance()->GetReferenceLengthScale()),
      mpColorLookUpTable(vtkSmartPointer<vtkLookupTable>::New())
{
    mpRenderer->SetBackground(1.0, 1.0, 1.0);

    // Create a default LUT
    mpColorLookUpTable->SetNumberOfTableValues(10);
    mpColorLookUpTable->Build();

    #if VTK_MAJOR_VERSION > 5
        vtkSmartPointer<vtkNamedColors> p_named_colors = vtkSmartPointer<vtkNamedColors>::New();
        mpColorLookUpTable->SetTableValue(0, p_named_colors->GetColor4d("Black").GetData());
        mpColorLookUpTable->SetTableValue(1, p_named_colors->GetColor4d("Banana").GetData());
        mpColorLookUpTable->SetTableValue(2, p_named_colors->GetColor4d("Tomato").GetData());
        mpColorLookUpTable->SetTableValue(3, p_named_colors->GetColor4d("Wheat").GetData());
        mpColorLookUpTable->SetTableValue(4, p_named_colors->GetColor4d("Lavender").GetData());
        mpColorLookUpTable->SetTableValue(5, p_named_colors->GetColor4d("Flesh").GetData());
        mpColorLookUpTable->SetTableValue(6, p_named_colors->GetColor4d("Raspberry").GetData());
        mpColorLookUpTable->SetTableValue(7, p_named_colors->GetColor4d("Salmon").GetData());
        mpColorLookUpTable->SetTableValue(8, p_named_colors->GetColor4d("Mint").GetData());
        mpColorLookUpTable->SetTableValue(9, p_named_colors->GetColor4d("Peacock").GetData());
    #endif

    mpRenderWindow->AddRenderer(mpRenderer);
    mpRenderWindow->SetSize(800.0, 600.0);

    mpRenderWindowInteractor->SetRenderWindow(mpRenderWindow);

    vtkSmartPointer<customMouseInteractorStyle> style = vtkSmartPointer<customMouseInteractorStyle>::New();
    mpRenderWindowInteractor->SetInteractorStyle( style );
}

template<unsigned DIM>
MicrovesselVtkScene<DIM>::~MicrovesselVtkScene()
{

}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::ResetRenderer()
{
    vtkSmartPointer<vtkActorCollection> p_actors = mpRenderer->GetActors();
    vtkActor *p_actor;
    for( p_actors->InitTraversal(); (p_actor = p_actors->GetNextItem())!=NULL; )
    {
        mpRenderer->RemoveActor(p_actor);
    }
    mpRenderer->Clear();

    if(mpPart)
    {
        UpdatePartActor();
    }
    if(mpNetwork)
    {
        UpdateVesselNetworkActor();
    }
    if(mpMesh)
    {
        UpdateMeshActor();
    }
    if(mpGrid)
    {
        UpdateRegularGridActor();
    }
    if(mpCellPopulation)
    {
        UpdateCellPopulationActor();
    }

    mpRenderer->ResetCamera();
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetOutputFilePath(const std::string& rPath)
{
    mOutputFilePath = rPath;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdatePartActor()
{
    vtkSmartPointer<vtkPolyData> p_poly = mpPart->GetVtk();
    vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_mapper->SetInput(p_poly);
    #else
        p_mapper->SetInputData(p_poly);
    #endif

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->GetProperty()->SetColor(0,0,1);
    p_actor->SetMapper(p_mapper);
    mpRenderer->AddActor(p_actor);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateVesselNetworkActor()
{
    VesselNetworkWriter<DIM> network_writer;
    network_writer.SetVesselNetwork(mpNetwork);
    vtkSmartPointer<vtkPolyData> p_poly = network_writer.GetOutput();

    vtkSmartPointer<vtkUnsignedCharArray> p_node_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    p_node_colors->SetNumberOfComponents(3);
    p_node_colors->SetName("PointColors");
    double rgb[3];
    unsigned char ucrgb[3];
    mpColorLookUpTable->GetColor(2, rgb);
    for (size_t j = 0; j < 3; ++j)
    {
        ucrgb[j] = static_cast<unsigned char>(rgb[j] * 255);
    }
    for(unsigned idx=0; idx<p_poly->GetNumberOfPoints(); idx++)
    {
        p_node_colors->InsertNextTuple3(ucrgb[0], ucrgb[1], ucrgb[2]);
    }

    vtkSmartPointer<vtkSphereSource> p_sheres = vtkSmartPointer<vtkSphereSource>::New();
    p_sheres->SetRadius(0.5);
    p_sheres->SetPhiResolution(16);
    p_sheres->SetThetaResolution(16);

    vtkSmartPointer<vtkGlyph3D> p_shere_glyphs = vtkSmartPointer<vtkGlyph3D>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_shere_glyphs->SetInput(p_poly);
        p_shere_glyphs->SetSource(p_sheres->GetOutput());
    #else
        p_shere_glyphs->SetInputData(p_poly);
        p_shere_glyphs->SetSourceData(p_sheres->GetOutput());
    #endif

    p_shere_glyphs->ClampingOff();
    p_shere_glyphs->SetScaleModeToScaleByScalar();
    p_shere_glyphs->SetScaleFactor(1.0);

    vtkSmartPointer<vtkPolyDataMapper> p_polydata_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_polydata_mapper->SetInput(p_shere_glyphs->GetOutput());
    #else
        p_polydata_mapper->SetInputData(p_shere_glyphs->GetOutput());
    #endif
    p_polydata_mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->SetMapper(p_polydata_mapper);

    vtkSmartPointer<vtkUnsignedCharArray> p_vessel_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    p_vessel_colors->SetNumberOfComponents(3);
    p_vessel_colors->SetName("VesselColors");
    double rgb2[3];
    unsigned char ucrgb2[3];
    mpColorLookUpTable->GetColor(7, rgb2);
    for (size_t j = 0; j < 3; ++j)
    {
        ucrgb[j] = static_cast<unsigned char>(rgb2[j] * 255);
    }
    for(unsigned idx=0; idx<p_poly->GetNumberOfCells(); idx++)
    {
        p_vessel_colors->InsertNextTuple3(ucrgb2[0], ucrgb2[1], ucrgb2[2]);
    }
    p_poly->GetCellData()->SetScalars(p_vessel_colors);

    vtkSmartPointer<vtkPolyDataMapper> p_polydata_mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_polydata_mapper2->SetInput(p_poly);
    #else
        p_polydata_mapper2->SetInputData(p_poly);
    #endif
    p_polydata_mapper2->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> p_actor2 = vtkSmartPointer<vtkActor>::New();
    p_actor2->SetMapper(p_polydata_mapper2);

    mpRenderer->AddActor(p_actor);
    mpRenderer->AddActor(p_actor2);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateCellPopulationActor()
{
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();

    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
         cell_iter != mpCellPopulation->End();
         ++cell_iter)
    {
        c_vector<double, DIM> centre = mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        if(DIM==3)
        {
            p_points->InsertNextPoint(centre[0], centre[1], centre[2]);
        }
        else
        {
            p_points->InsertNextPoint(centre[0], centre[1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkSphereSource> p_spheres = vtkSmartPointer<vtkSphereSource>::New();
    p_spheres->SetRadius(1.0);
    p_spheres->SetPhiResolution(16);
    p_spheres->SetThetaResolution(16);

    vtkSmartPointer<vtkGlyph3D> p_glyph = vtkSmartPointer<vtkGlyph3D>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_glyph->SetInput(p_polydata);
        p_glyph->SetSource(p_spheres->GetOutput());
    #else
        p_glyph->SetInputData(p_polydata);
        p_glyph->SetSourceData(p_spheres->GetOutput());
    #endif

    p_glyph->ClampingOff();
    p_glyph->SetScaleModeToScaleByScalar();
    p_glyph->SetScaleFactor(1.0);

    vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_mapper->SetInput(p_glyph->GetOutput());
    #else
        p_mapper->SetInputData(p_glyph->GetOutput());
    #endif
    p_mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->SetMapper(p_mapper);
    p_actor->GetProperty()->SetColor(0,1,0);
    mpRenderer->AddActor(p_actor);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateRegularGridActor()
{
    vtkSmartPointer<vtkImageData> p_grid = mpGrid->GetVtkGrid();

    vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_geom_filter->SetInput(p_grid);
    #else
        p_geom_filter->SetInputData(p_grid);
    #endif

    vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_mapper->SetInput(p_geom_filter->GetOutput());
    #else
        p_mapper->SetInputData(p_geom_filter->GetOutput());
    #endif

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->SetMapper(p_mapper);
    p_actor->GetProperty()->SetColor(0,0,1);
    mpRenderer->AddActor(p_actor);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateMeshActor()
{
    vtkSmartPointer<vtkUnstructuredGrid> p_grid = mpMesh->GetAsVtkUnstructuredGrid();

    vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_geom_filter->SetInput(p_grid);
    #else
        p_geom_filter->SetInputData(p_grid);
    #endif

//    vtkSmartPointer<vtkExtractEdges> p_edges = vtkSmartPointer<vtkExtractEdges>::New();
//    p_edges->SetInput(p_geom_filter);

    vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_mapper->SetInput(p_geom_filter->GetOutput());
    #else
        p_mapper->SetInputData(p_geom_filter->GetOutput());
    #endif

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->SetMapper(p_mapper);
    p_actor->GetProperty()->SetColor(0,0,1);
    mpRenderer->AddActor(p_actor);
//    mapper2 = vtk.vtkPolyDataMapper()
//    mapper2.SetInput(tube_filter.GetOutput())
//    #mapper2.ScalarVisibilityOn()
//    actor2 = vtk.vtkActor()
//    actor2.SetMapper(mapper2)
//    actor.GetProperty().SetColor(1,1,1) # (R,G,B
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetPart(boost::shared_ptr<Part<DIM> > pPart)
{
    mpPart = pPart;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation)
{
    mpCellPopulation = pCellPopulation;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpGrid = pGrid;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    mpMesh = pMesh;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::Show(bool interactive)
{
    // Set up axes and camera
//    vtkSmartPointer<vtkCubeAxesActor2D> p_axis = vtkSmartPointer<vtkCubeAxesActor2D>::New();
//    p_axis->SetBounds(0.0, 1000.0, 0.0, 1000.0, 0.0, 1000.0);
//    p_axis->SetCamera(mpRenderer->GetActiveCamera());
//    p_axis->SetLabelFormat("%6.4g");
//    p_axis->SetFlyModeToOuterEdges();
//    p_axis->SetFontFactor(2.0);
////    tprop = vtk.vtkTextProperty()
////    tprop.SetColor(1, 1, 1)
////    axes.SetAxisTitleTextProperty(tprop)
////    axes.SetAxisLabelTextProperty(tprop)
//    mpRenderer->AddViewProp(p_axis);
    ResetRenderer();
    mpRenderer->ResetCamera();

    if(interactive)
    {
        mpRenderWindowInteractor->Initialize();
        mpRenderWindowInteractor->Start();
    }
    else
    {
        mpRenderWindow->SetOffScreenRendering(1);
        mpRenderWindow->Render();

        vtkSmartPointer<vtkWindowToImageFilter> p_window_to_image = vtkSmartPointer<vtkWindowToImageFilter>::New();
        p_window_to_image->SetInput(mpRenderWindow);
        p_window_to_image->Update();

        vtkSmartPointer<vtkPNGWriter> p_writer = vtkSmartPointer<vtkPNGWriter>::New();
        p_writer->SetWriteToMemory(1);
        p_writer->SetInputConnection(p_window_to_image->GetOutputPort());

        if(!mOutputFilePath.empty())
        {
            p_writer->SetWriteToMemory(0);
            p_writer->SetFileName(mOutputFilePath.c_str());
            p_writer->Write();
        }
    }
}

template class MicrovesselVtkScene<2>;
template class MicrovesselVtkScene<3>;
