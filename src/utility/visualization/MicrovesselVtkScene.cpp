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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
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
#include <vtkGlyph2D.h>
#include <vtkCubeAxesActor2D.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkActorCollection.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkTubeFilter.h>
#include <vtkExtractEdges.h>
#include <vtkCamera.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkPolygon.h>
#include <vtkConvexPointSet.h>
#include <vtkIdList.h>
#include <vtkGeometryFilter.h>
#include <vtkTetra.h>
#include <vtkTriangle.h>
#include <vtkLine.h>
#include <vtkFeatureEdges.h>
#include <vtkTextProperty.h>
#include <vtkCubeAxesActor.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "MicrovesselVtkScene.hpp"
#include "BaseUnits.hpp"
#include "VesselNetworkWriter.hpp"

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
      mpCellPopulation(),
      mOutputFilePath(),
      mpColorLookUpTable(vtkSmartPointer<vtkLookupTable>::New()),
    #if VTK_MAJOR_VERSION > 5
      mAnimationWriter(vtkSmartPointer<vtkOggTheoraWriter>::New()),
    #endif
      mWindowToImageFilter(vtkSmartPointer<vtkWindowToImageFilter>::New()),
      mIsInteractive(true),
      mSaveAsAnimation(false),
      mSaveAsImages(false),
      mHasStarted(false),
      mAddAnnotations(false),
      mOutputFrequency(1),
      mpPartGenerator(boost::shared_ptr<PartActorGenerator<DIM> >(new PartActorGenerator<DIM>())),
      mpNetworkGenerator(boost::shared_ptr<VesselNetworkActorGenerator<DIM> >(new VesselNetworkActorGenerator<DIM>())),
      mpDiscreteContinuumMeshGenerator(boost::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> >(new DiscreteContinuumMeshActorGenerator<DIM>())),
      mpGridGenerator(boost::shared_ptr<RegularGridActorGenerator<DIM> >(new RegularGridActorGenerator<DIM>())),
      mLengthScale(BaseUnits::Instance()->GetReferenceLengthScale())
{
    mpRenderer->SetBackground(1.0, 1.0, 1.0);

//    // Create a default LUT
//    mpColorLookUpTable->SetNumberOfTableValues(10);
//    mpColorLookUpTable->Build();
//
//    #if VTK_MAJOR_VERSION > 5
//        vtkSmartPointer<vtkNamedColors> p_named_colors = vtkSmartPointer<vtkNamedColors>::New();
//        mpColorLookUpTable->SetTableValue(0, p_named_colors->GetColor4d("Black").GetData());
//        mpColorLookUpTable->SetTableValue(1, p_named_colors->GetColor4d("Banana").GetData());
//        mpColorLookUpTable->SetTableValue(2, p_named_colors->GetColor4d("Tomato").GetData());
//        mpColorLookUpTable->SetTableValue(3, p_named_colors->GetColor4d("Wheat").GetData());
//        mpColorLookUpTable->SetTableValue(4, p_named_colors->GetColor4d("Lavender").GetData());
//        mpColorLookUpTable->SetTableValue(5, p_named_colors->GetColor4d("Flesh").GetData());
//        mpColorLookUpTable->SetTableValue(6, p_named_colors->GetColor4d("Raspberry").GetData());
//        mpColorLookUpTable->SetTableValue(7, p_named_colors->GetColor4d("Salmon").GetData());
//        mpColorLookUpTable->SetTableValue(8, p_named_colors->GetColor4d("Mint").GetData());
//        mpColorLookUpTable->SetTableValue(9, p_named_colors->GetColor4d("Peacock").GetData());
//    #endif

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
boost::shared_ptr<PartActorGenerator<DIM> > MicrovesselVtkScene<DIM>::GetPartActorGenerator()
{
    return mpPartGenerator;
}

template<unsigned DIM>
boost::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> > MicrovesselVtkScene<DIM>::GetDiscreteContinuumMeshActorGenerator()
{
    return mpDiscreteContinuumMeshGenerator;
}

template<unsigned DIM>
boost::shared_ptr<RegularGridActorGenerator<DIM> > MicrovesselVtkScene<DIM>::GetRegularGridActorGenerator()
{
    return mpGridGenerator;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetworkActorGenerator<DIM> > MicrovesselVtkScene<DIM>::GetVesselNetworkActorGenerator()
{
    return mpNetworkGenerator;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::ResetRenderer(unsigned time_step)
{
    if(!mHasStarted)
    {
        Start();
    }

    vtkSmartPointer<vtkActorCollection> p_actors = mpRenderer->GetActors();
    vtkActor *p_actor;
    for( p_actors->InitTraversal(); (p_actor = p_actors->GetNextItem())!=NULL; )
    {
        mpRenderer->RemoveActor(p_actor);
    }
    mpRenderer->Clear();

    if(mpCellPopulation)
    {
        UpdateCellPopulationActor();
    }
    if(mpPartGenerator)
    {
        mpPartGenerator->AddActor(mpRenderer);
    }
    if(mpNetworkGenerator)
    {
        mpNetworkGenerator->AddActor(mpRenderer);
    }
    if(mpDiscreteContinuumMeshGenerator)
    {
        mpDiscreteContinuumMeshGenerator->AddActor(mpRenderer);
    }
    if(mpGridGenerator)
    {
        mpGridGenerator->AddActor(mpRenderer);
    }

    mpRenderer->ResetCamera();

    vtkSmartPointer<vtkCubeAxesActor> p_cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
    p_cubeAxesActor->SetBounds(mpRenderer->ComputeVisiblePropBounds());
    p_cubeAxesActor->SetCamera(mpRenderer->GetActiveCamera());
    p_cubeAxesActor->GetTitleTextProperty(0)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetLabelTextProperty(0)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetTitleTextProperty(1)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetLabelTextProperty(1)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetTitleTextProperty(2)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetLabelTextProperty(2)->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetXAxesLinesProperty()->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetYAxesLinesProperty()->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->GetZAxesLinesProperty()->SetColor(0.0, 0.0, 0.0);
    p_cubeAxesActor->DrawXGridlinesOff();
    p_cubeAxesActor->DrawYGridlinesOff();
    p_cubeAxesActor->DrawZGridlinesOff();
    p_cubeAxesActor->XAxisMinorTickVisibilityOff();
    p_cubeAxesActor->YAxisMinorTickVisibilityOff();
    p_cubeAxesActor->ZAxisMinorTickVisibilityOff();

    mpRenderer->AddActor(p_cubeAxesActor);

    if(mSaveAsImages)
    {
        mpRenderWindow->SetOffScreenRendering(1);
        mpRenderWindow->Render();
        mWindowToImageFilter->Modified();
        vtkSmartPointer<vtkPNGWriter> p_writer = vtkSmartPointer<vtkPNGWriter>::New();
        p_writer->SetWriteToMemory(1);
        p_writer->SetInputConnection(mWindowToImageFilter->GetOutputPort());
        if(!mOutputFilePath.empty())
        {
            p_writer->SetWriteToMemory(0);
            p_writer->SetFileName((mOutputFilePath+"_"+boost::lexical_cast<std::string>(time_step)+".png").c_str());
            p_writer->Write();
        }
    }
    #if VTK_MAJOR_VERSION > 5
    if(mSaveAsAnimation)
    {
        if(!mSaveAsImages)
        {
            mpRenderWindow->SetOffScreenRendering(1);
            mpRenderWindow->Render();
            mWindowToImageFilter->Modified();
        }
        mAnimationWriter->Write();
    }
    #endif
    if(mIsInteractive)
    {
        mpRenderWindow->SetOffScreenRendering(0);
        mpRenderWindow->Render();
    }
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetOutputFilePath(const std::string& rPath)
{
    mOutputFilePath = rPath;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetSaveAsAnimation(bool saveAsAnimation)
{
    mSaveAsAnimation = saveAsAnimation;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetSaveAsImages(bool saveAsImages)
{
    mSaveAsImages = saveAsImages;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetIsInteractive(bool isInteractive)
{
    mIsInteractive = isInteractive;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateCellPopulationActor()
{
    // Check the cell population type
    if(boost::dynamic_pointer_cast<MeshBasedCellPopulation<DIM> >(mpCellPopulation))
    {
        UpdateMeshBasedCellPopulationActor();
    }
    else
    {
        // fall back to a centre based default
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();

        for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
             cell_iter != mpCellPopulation->End(); ++cell_iter)
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
        p_spheres->SetRadius(0.5);
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
        p_mapper->ScalarVisibilityOn();

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        p_actor->GetProperty()->SetColor(0,1,0);
        mpRenderer->AddActor(p_actor);
    }

}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::UpdateMeshBasedCellPopulationActor()
{
    boost::shared_ptr<MeshBasedCellPopulation<DIM> > p_cell_population = boost::dynamic_pointer_cast<MeshBasedCellPopulation<DIM> >(mpCellPopulation);

    if(!p_cell_population)
    {
        EXCEPTION("Could not cast mesh to MeshBased type.");
    }

    // Add the voronoi mesh
    p_cell_population->CreateVoronoiTessellation();
    vtkSmartPointer<vtkUnstructuredGrid> p_voronoi_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();

    if(p_cell_population->GetVoronoiTessellation() != NULL)
    {
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        p_points->GetData()->SetName("Vertex positions");

        for (unsigned node_num=0; node_num<p_cell_population->GetVoronoiTessellation()->GetNumNodes(); node_num++)
        {
            c_vector<double, DIM> position = p_cell_population->GetVoronoiTessellation()->GetNode(node_num)->rGetLocation();
            if (DIM==2)
            {
                p_points->InsertPoint(node_num, position[0], position[1], 0.0);
            }
            else
            {
                p_points->InsertPoint(node_num, position[0], position[1], position[2]);
            }
        }

        p_voronoi_grid->SetPoints(p_points);

        for (typename VertexMesh<DIM,DIM>::VertexElementIterator iter = p_cell_population->GetVoronoiTessellation()->GetElementIteratorBegin();
             iter != p_cell_population->GetVoronoiTessellation()->GetElementIteratorEnd(); ++iter)
        {
            vtkSmartPointer<vtkCell> p_cell;
            if (DIM == 2)
            {
                p_cell = vtkSmartPointer<vtkPolygon>::New();
            }
            else
            {
                p_cell = vtkSmartPointer<vtkConvexPointSet>::New();
            }
            vtkSmartPointer<vtkIdList> p_cell_id_list = p_cell->GetPointIds();
            p_cell_id_list->SetNumberOfIds(iter->GetNumNodes());
            for (unsigned j=0; j<iter->GetNumNodes(); ++j)
            {
                p_cell_id_list->SetId(j, iter->GetNodeGlobalIndex(j));
            }
            p_voronoi_grid->InsertNextCell(p_cell->GetCellType(), p_cell_id_list);
        }
    }

    vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_geom_filter->SetInput(p_voronoi_grid);
    #else
        p_geom_filter->SetInputData(p_voronoi_grid);
    #endif

    vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    p_mapper->SetInputConnection(p_geom_filter->GetOutputPort());
    p_mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
    p_actor->SetMapper(p_mapper);
    p_actor->GetProperty()->SetColor(1,1,0);
    p_actor->GetProperty()->SetOpacity(0.8);
    mpRenderer->AddActor(p_actor);

    vtkSmartPointer<vtkFeatureEdges> p_voronoi_extract_edges = vtkSmartPointer<vtkFeatureEdges>::New();
    p_voronoi_extract_edges->SetInputConnection(p_geom_filter->GetOutputPort());
    p_voronoi_extract_edges->SetFeatureEdges(false);
    p_voronoi_extract_edges->SetBoundaryEdges(true);
    p_voronoi_extract_edges->SetManifoldEdges(true);
    p_voronoi_extract_edges->SetNonManifoldEdges(false);

    vtkSmartPointer<vtkTubeFilter> p_voronoi_tubes = vtkSmartPointer<vtkTubeFilter>::New();
    p_voronoi_tubes->SetInputConnection(p_voronoi_extract_edges->GetOutputPort());
    p_voronoi_tubes->SetRadius(0.006);
    p_voronoi_tubes->SetNumberOfSides(12);

    vtkSmartPointer<vtkPolyDataMapper> p_voronoi_tube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    p_voronoi_tube_mapper->SetInputConnection(p_voronoi_tubes->GetOutputPort());

    vtkSmartPointer<vtkActor> p_voronoi_tube_actor = vtkSmartPointer<vtkActor>::New();
    p_voronoi_tube_actor->SetMapper(p_voronoi_tube_mapper);
    p_voronoi_tube_actor->GetProperty()->SetColor(1,1,1);
    mpRenderer->AddActor(p_voronoi_tube_actor);

//    // Do the mutable mesh
//    //Make the local mesh into a VtkMesh
//    vtkSmartPointer<vtkUnstructuredGrid> p_mutable_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
//    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
//    p_points->GetData()->SetName("Vertex positions");
//
//    for (typename AbstractMesh<DIM,DIM>::NodeIterator node_iter = p_cell_population->rGetMesh().GetNodeIteratorBegin();
//         node_iter != p_cell_population->rGetMesh().GetNodeIteratorEnd();
//         ++node_iter)
//    {
//        c_vector<double, DIM> current_item = node_iter->rGetLocation();
//        if (DIM == 3)
//        {
//            p_points->InsertNextPoint(current_item[0], current_item[1], current_item[2]);
//        }
//        else if (DIM == 2)
//        {
//            p_points->InsertNextPoint(current_item[0], current_item[1], 0.0);
//        }
//        else // (DIM == 1)
//        {
//            p_points->InsertNextPoint(current_item[0], 0.0, 0.0);
//        }
//    }
//
//    p_mutable_grid->SetPoints(p_points);
//
//    for (typename AbstractTetrahedralMesh<DIM,DIM>::ElementIterator elem_iter = p_cell_population->rGetMesh().GetElementIteratorBegin();
//         elem_iter != p_cell_population->rGetMesh().GetElementIteratorEnd();
//         ++elem_iter)
//    {
//
//        vtkSmartPointer<vtkCell> p_cell;
//        ///\todo This ought to look exactly like the other MakeVtkMesh
//        if (DIM == 3)
//        {
//            p_cell = vtkSmartPointer<vtkTetra>::New();
//        }
//        else if (DIM == 2)
//        {
//            p_cell = vtkSmartPointer<vtkTriangle>::New();
//        }
//        else //(DIM == 1)
//        {
//            p_cell = vtkSmartPointer<vtkLine>::New();
//        }
//        vtkSmartPointer<vtkIdList> p_cell_id_list = p_cell->GetPointIds();
//        for (unsigned j = 0; j < DIM+1; ++j)
//        {
//            unsigned global_node_index = elem_iter->GetNodeGlobalIndex(j);
//            p_cell_id_list->SetId(j, global_node_index);
//        }
//        p_mutable_grid->InsertNextCell(p_cell->GetCellType(), p_cell_id_list);
//    }
//
//    vtkSmartPointer<vtkGeometryFilter> p_mutable_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
//    #if VTK_MAJOR_VERSION <= 5
//    p_mutable_geom_filter->SetInput(p_mutable_grid);
//    #else
//    p_mutable_geom_filter->SetInputData(p_mutable_grid);
//    #endif
//
//    vtkSmartPointer<vtkFeatureEdges> p_extract_edges = vtkSmartPointer<vtkFeatureEdges>::New();
//    p_extract_edges->SetInputConnection(p_mutable_geom_filter->GetOutputPort());
//    p_extract_edges->SetFeatureEdges(false);
//    p_extract_edges->SetBoundaryEdges(true);
//    p_extract_edges->SetManifoldEdges(true);
//    p_extract_edges->SetNonManifoldEdges(false);
//
//    vtkSmartPointer<vtkTubeFilter> p_mutable_tubes = vtkSmartPointer<vtkTubeFilter>::New();
//    p_mutable_tubes->SetInputConnection(p_extract_edges->GetOutputPort());
//    p_mutable_tubes->SetRadius(0.02);
//    p_mutable_tubes->SetNumberOfSides(12);
//
//    vtkSmartPointer<vtkPolyDataMapper> p_mutable_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    p_mutable_mapper->SetInputConnection(p_mutable_tubes->GetOutputPort());
//
//    vtkSmartPointer<vtkActor> p_mutable_actor = vtkSmartPointer<vtkActor>::New();
//    p_mutable_actor->SetMapper(p_mutable_mapper);
//    p_mutable_actor->GetProperty()->SetColor(1,1,1);
//    mpRenderer->AddActor(p_mutable_actor);

    // Render the centres
    vtkSmartPointer<vtkPoints> p_centres_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();

    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
         cell_iter != mpCellPopulation->End(); ++cell_iter)
    {
        c_vector<double, DIM> centre = mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        if(DIM==3)
        {
            p_centres_points->InsertNextPoint(centre[0], centre[1], centre[2]);
        }
        else
        {
            p_centres_points->InsertNextPoint(centre[0], centre[1], 0.0);
        }
    }
    p_polydata->SetPoints(p_centres_points);

    vtkSmartPointer<vtkSphereSource> p_spheres = vtkSmartPointer<vtkSphereSource>::New();
    p_spheres->SetRadius(0.1);
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

    vtkSmartPointer<vtkPolyDataMapper> p_centres_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_centres_mapper->SetInput(p_glyph->GetOutput());
    #else
        p_centres_mapper->SetInputData(p_glyph->GetOutput());
    #endif
    p_centres_mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> p_centres_actor = vtkSmartPointer<vtkActor>::New();
    p_centres_actor->SetMapper(p_centres_mapper);
    p_centres_actor->GetProperty()->SetColor(0.6,0.0,0.0);
    mpRenderer->AddActor(p_centres_actor);

}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetPart(boost::shared_ptr<Part<DIM> > pPart)
{
    mpPartGenerator->SetPart(pPart);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetworkGenerator->SetVesselNetwork(pNetwork);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation)
{
    mpCellPopulation = pCellPopulation;
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpGridGenerator->SetRegularGrid(pGrid);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    mpDiscreteContinuumMeshGenerator->SetDiscreteContinuumMesh(pMesh);
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::End()
{
    #if VTK_MAJOR_VERSION > 5
    if(mSaveAsAnimation and mHasStarted)
    {
        mAnimationWriter->End();
    }
    #endif
}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::Start()
{
    mpRenderer->ResetCamera();
    if(DIM==3)
    {
        mpRenderer->GetActiveCamera()->Azimuth(45.0);
    }

    if(mSaveAsImages or mSaveAsAnimation)
    {
        mpRenderWindow->SetOffScreenRendering(1);
        mWindowToImageFilter->SetInput(mpRenderWindow);
        mWindowToImageFilter->Update();
    }

    if(mSaveAsAnimation)
    {
        #if VTK_MAJOR_VERSION > 5
        mAnimationWriter->SetInputConnection(mWindowToImageFilter->GetOutputPort());
        mAnimationWriter->SetFileName((mOutputFilePath+".ogg").c_str());
        mAnimationWriter->SetRate(1.0);
        mAnimationWriter->Start();
        #endif
    }

    mHasStarted = true;
    ResetRenderer();

    if(mIsInteractive)
    {
        mpRenderWindowInteractor->Initialize();
    }

}

template<unsigned DIM>
void MicrovesselVtkScene<DIM>::StartInteractiveEventHandler()
{
    mpRenderWindowInteractor->Start();
}

template class MicrovesselVtkScene<2>;
template class MicrovesselVtkScene<3>;
