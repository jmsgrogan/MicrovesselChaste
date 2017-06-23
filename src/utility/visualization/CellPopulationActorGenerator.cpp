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

/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is CellPopulation of Chaste.

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
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A CellPopulationICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkDoubleArray.h>
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
#include <vtkTriangle.h>
#include <vtkLine.h>
#include <vtkVoxel.h>
#include <vtkPixel.h>
#include <vtkThreshold.h>
#include <vtkMarchingCubes.h>
#include <vtkMarchingSquares.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkFeatureEdges.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "CellLabel.hpp"
#include "CaBasedCellPopulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "MeshBasedCellPopulationWithGhostNodes.hpp"
#include "CellPopulationActorGenerator.hpp"
#include "Debug.hpp"


template<unsigned DIM>
CellPopulationActorGenerator<DIM>::CellPopulationActorGenerator()
    : AbstractActorGenerator<DIM>(),
      mpCellPopulation(),
      mShowMutableMeshEdges(false),
      mShowVoronoiMeshEdges(true),
      mShowPottsMeshEdges(false),
      mShowPottsMeshOutlines(false),
      mColorByCellType(false),
      mColorByCellData(false),
      mColorByCellMutationState(false),
      mColorByCellLabel(false),
      mShowCellCentres(false),
      mColorCellByUserDefined(false)
{

}

template<unsigned DIM>
CellPopulationActorGenerator<DIM>::~CellPopulationActorGenerator()
{

}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::AddCaBasedCellPopulationActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    vtkSmartPointer<vtkImageData> p_potts_grid = vtkSmartPointer<vtkImageData>::New();
    vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();

    std::shared_ptr<CaBasedCellPopulation<DIM> > p_ca_population =
            std::dynamic_pointer_cast<CaBasedCellPopulation<DIM> >(mpCellPopulation);

    if(p_ca_population and mShowPottsMeshEdges)
    {
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        p_points->GetData()->SetName("Vertex positions");

        unsigned counter = 0;
        c_vector<double, DIM> old_loc = zero_vector<double>(DIM);
        double spacing = 1.0;
        for (typename PottsMesh<DIM>::NodeIterator node_iter = p_ca_population->rGetMesh().GetNodeIteratorBegin();
             node_iter != p_ca_population->rGetMesh().GetNodeIteratorEnd(); ++node_iter)
        {
            c_vector<double, DIM> current_item = node_iter->rGetLocation();
            if (DIM == 3)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], current_item[2]);
            }
            else if (DIM == 2)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], 0.0);
            }
            else // (DIM == 1)
            {
                p_points->InsertNextPoint(current_item[0], 0.0, 0.0);
            }

            if(counter == 0)
            {
                old_loc = current_item;
            }
            if(counter == 1)
            {
                spacing = norm_2(current_item - old_loc);
            }
            counter ++;
        }

        vtkSmartPointer<vtkPolyData> p_temp_polydata = vtkSmartPointer<vtkPolyData>::New();
        p_temp_polydata->SetPoints(p_points);

        double bounds[6];
        p_temp_polydata->GetBounds(bounds);
        vtkSmartPointer<vtkImageData> p_ca_image = vtkSmartPointer<vtkImageData>::New();
        p_ca_image->SetDimensions(std::floor((bounds[1]-bounds[0])/spacing) + 1,
                std::floor((bounds[3]-bounds[2])/spacing) + 1,
                std::floor((bounds[5]-bounds[4])/spacing) + 1);
        p_ca_image->SetOrigin(bounds[0], bounds[2], bounds[4]);
        p_ca_image->SetSpacing(spacing, spacing, spacing);

        #if VTK_MAJOR_VERSION <= 5
            p_geom_filter->SetInput(p_ca_image);
        #else
            p_geom_filter->SetInputData(p_ca_image);
        #endif

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_mapper->SetInputConnection(p_geom_filter->GetOutputPort());

        vtkSmartPointer<vtkActor> p_volume_actor = vtkSmartPointer<vtkActor>::New();
        p_volume_actor->SetMapper(p_mapper);
        p_volume_actor->GetProperty()->SetEdgeVisibility(this->mShowEdges);
        p_volume_actor->GetProperty()->SetLineWidth(this->mEdgeSize);
        p_volume_actor->GetProperty()->SetOpacity(0.6);
        pRenderer->AddActor(p_volume_actor);
    }
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::AddPottsBasedCellPopulationActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    vtkSmartPointer<vtkImageData> p_potts_grid = vtkSmartPointer<vtkImageData>::New();

    std::shared_ptr<PottsBasedCellPopulation<DIM> > p_potts_population =
            std::dynamic_pointer_cast<PottsBasedCellPopulation<DIM> >(mpCellPopulation);

    if(p_potts_population and mShowPottsMeshEdges)
    {
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        p_points->GetData()->SetName("Vertex positions");

        unsigned counter = 0;
        c_vector<double, DIM> old_loc = zero_vector<double>(DIM);
        double spacing = 1.0;

        for (typename PottsMesh<DIM>::NodeIterator node_iter = p_potts_population->rGetMesh().GetNodeIteratorBegin();
             node_iter != p_potts_population->rGetMesh().GetNodeIteratorEnd();
             ++node_iter)
        {
            c_vector<double, DIM> current_item = node_iter->rGetLocation();
            if (DIM == 3)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], current_item[2]);
            }
            else if (DIM == 2)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], 0.0);
            }
            else // (DIM == 1)
            {
                p_points->InsertNextPoint(current_item[0], 0.0, 0.0);
            }
            if(counter == 0)
            {
                old_loc = current_item;
            }
            if(counter == 1)
            {
                spacing = norm_2(current_item - old_loc);
            }
            counter ++;
        }

        vtkSmartPointer<vtkPolyData> p_temp_polydata = vtkSmartPointer<vtkPolyData>::New();
        p_temp_polydata->SetPoints(p_points);

        double bounds[6];
        p_temp_polydata->GetBounds(bounds);
        p_potts_grid = vtkSmartPointer<vtkImageData>::New();

        // Important: We color VTK cells, not points. We add a VTK cell for each Chaste node.
        p_potts_grid->SetDimensions(std::floor((bounds[1]-bounds[0])/spacing) + 2,
                std::floor((bounds[3]-bounds[2])/spacing) + 2,
                std::floor((bounds[5]-bounds[4])/spacing) + 2);
        p_potts_grid->SetOrigin(bounds[0]-spacing/2.0, bounds[2]-spacing/2.0, bounds[4]-spacing/2.0);
        p_potts_grid->SetSpacing(spacing, spacing, spacing);

        vtkSmartPointer<vtkDoubleArray> p_element_ids = vtkSmartPointer<vtkDoubleArray>::New();
        p_element_ids->SetNumberOfTuples(p_potts_grid->GetNumberOfPoints());
        p_element_ids->SetName("Cell Id");

        vtkSmartPointer<vtkDoubleArray> p_element_base_ids = vtkSmartPointer<vtkDoubleArray>::New();
        p_element_base_ids->SetNumberOfTuples(p_potts_grid->GetNumberOfPoints());
        p_element_base_ids->SetName("Cell Base Id");
        for(unsigned idx=0; idx<p_potts_grid->GetNumberOfPoints(); idx++)
        {
            p_element_ids->SetTuple1(idx, -1.0);
        }

        for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
             cell_iter != mpCellPopulation->End(); ++cell_iter)
        {
            PottsElement<DIM>* p_element = p_potts_population->GetElementCorrespondingToCell(*cell_iter);

            for(unsigned idx=0; idx<p_element->GetNumNodes(); idx++)
            {
                unsigned node_index = p_element->GetNode(idx)->GetIndex();

                if(mColorByCellType)
                {
                    p_element_ids->InsertNextTuple1((*cell_iter)->GetCellProliferativeType()->GetColour());
                }
                else if(mColorByCellData and !this->mDataLabel.empty())
                {
                    std::vector<std::string> keys = (*cell_iter)->GetCellData()->GetKeys();
                    if (std::find(keys.begin(), keys.end(), this->mDataLabel) != keys.end())
                    {
                        p_element_ids->InsertNextTuple1((*cell_iter)->GetCellData()->GetItem(this->mDataLabel));
                    }
                    else
                    {
                        p_element_ids->InsertNextTuple1(0.0);
                    }
                }

                else if(mColorByCellMutationState)
                {
                    double mutation_state = (*cell_iter)->GetMutationState()->GetColour();
                    CellPropertyCollection collection = (*cell_iter)->rGetCellPropertyCollection();
                    CellPropertyCollection label_collection = collection.GetProperties<CellLabel>();

                    if (label_collection.GetSize() == 1)
                    {
                        boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(label_collection.GetProperty());
                        mutation_state = p_label->GetColour();
                    }
                    p_element_ids->InsertNextTuple1(mutation_state);
                }
                else if(mColorByCellLabel)
                {
                    double label = 0.0;
                    if ((*cell_iter)->template HasCellProperty<CellLabel>())
                    {
                        CellPropertyCollection collection = (*cell_iter)->rGetCellPropertyCollection().template  GetProperties<CellLabel>();
                        boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
                        label = p_label->GetColour();
                    }
                    p_element_ids->InsertNextTuple1(label);
                }
                else
                {
                    p_element_ids->SetTuple1(node_index, double((*cell_iter)->GetCellId()+1));

                }
                p_element_base_ids->SetTuple1(node_index, double((*cell_iter)->GetCellId()+1));
            }
        }
        p_potts_grid->GetCellData()->SetScalars(p_element_ids);
        p_potts_grid->GetCellData()->AddArray(p_element_ids);
        p_potts_grid->GetCellData()->AddArray(p_element_base_ids);

        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!mColorByCellData)
        {
            double range[2];
            p_potts_grid->GetCellData()->GetArray("Cell Id")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpDiscreteColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(double(idx)*range[1]/255.0, color[0], color[1], color[2]);
            }
        }

        vtkSmartPointer<vtkGeometryFilter> p_geometry_filter_pre =
          vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_geometry_filter_pre->SetInput(p_potts_grid);
        #else
            p_geometry_filter_pre->SetInputData(p_potts_grid);
        #endif

        vtkSmartPointer<vtkThreshold> p_threshold = vtkSmartPointer<vtkThreshold>::New();
        p_threshold->SetInputConnection(p_geometry_filter_pre->GetOutputPort());
        p_threshold->ThresholdByUpper(0.0);
        p_threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "Cell Id");

        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        p_geom_filter->SetInputConnection(p_threshold->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_mapper->SetInputConnection(p_geom_filter->GetOutputPort());
        p_mapper->SetLookupTable(p_scaled_ctf);
        p_mapper->ScalarVisibilityOn();
        p_mapper->SelectColorArray("Cell Id");
        p_mapper->SetScalarModeToUseCellData();
        p_mapper->SetColorModeToMapScalars();

        vtkSmartPointer<vtkActor> p_volume_actor = vtkSmartPointer<vtkActor>::New();
        p_volume_actor->SetMapper(p_mapper);
        //p_volume_actor->GetProperty()->SetEdgeVisibility(this->mShowEdges);
        p_volume_actor->GetProperty()->SetLineWidth(this->mEdgeSize);
        p_volume_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
        if(mColorCellByUserDefined)
        {
            p_volume_actor->GetProperty()->SetColor(this->mPointColor[0], this->mPointColor[1], this->mPointColor[2]);
        }
        pRenderer->AddActor(p_volume_actor);

        if(mShowPottsMeshOutlines)
        {
            vtkSmartPointer<vtkPolyData> p_bounds = vtkSmartPointer<vtkPolyData>::New();

            for(unsigned idx=0; idx<p_potts_grid->GetNumberOfCells(); idx++)
            {
                vtkSmartPointer<vtkThreshold> p_local_threshold = vtkSmartPointer<vtkThreshold>::New();
                #if VTK_MAJOR_VERSION <= 5
                    p_local_threshold->SetInput(p_geom_filter->GetOutput());
                #else
                    p_local_threshold->SetInputData(p_geom_filter->GetOutput());
                #endif
                p_local_threshold->ThresholdBetween(p_element_base_ids->GetTuple1(idx), p_element_base_ids->GetTuple1(idx));
                p_local_threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "Cell Base Id");

                vtkSmartPointer<vtkGeometryFilter> p_local_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
                p_local_geom_filter->SetInputConnection(p_local_threshold->GetOutputPort());

                vtkSmartPointer<vtkFeatureEdges> p_features = vtkSmartPointer<vtkFeatureEdges>::New();
                p_features->SetInputConnection(p_local_geom_filter->GetOutputPort());
                p_features->Update();

                vtkSmartPointer<vtkAppendPolyData> p_append = vtkSmartPointer<vtkAppendPolyData>::New();
                #if VTK_MAJOR_VERSION <= 5
                    p_append->AddInput(p_bounds);
                    p_append->AddInput(p_features->GetOutput());
                #else
                    p_append->AddInputData(p_bounds);
                    p_append->AddInputData(p_features->GetOutput());
                #endif
                p_append->Update();
                p_bounds = p_append->GetOutput();
            }

            vtkSmartPointer<vtkPolyDataMapper> p_mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_mapper2->SetInput(p_bounds);
            #else
                p_mapper2->SetInputData(p_bounds);
            #endif

            vtkSmartPointer<vtkActor> p_volume_actor2 = vtkSmartPointer<vtkActor>::New();
            p_volume_actor2->SetMapper(p_mapper2);
            p_volume_actor2->GetProperty()->SetEdgeVisibility(this->mShowEdges);
            p_volume_actor2->GetProperty()->SetLineWidth(this->mEdgeSize);
            p_volume_actor2->GetProperty()->SetColor(this->mEdgeColor[0], this->mEdgeColor[1], this->mEdgeColor[2]);
            p_volume_actor2->GetProperty()->SetOpacity(this->mVolumeOpacity);
            pRenderer->AddActor(p_volume_actor2);
        }
    }
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::AddActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    if(!mpCellPopulation)
    {
        return;
    }

    // Show cell centres if requested
    if(mShowCellCentres or std::dynamic_pointer_cast<CaBasedCellPopulation<DIM> >(mpCellPopulation) or
            std::dynamic_pointer_cast<NodeBasedCellPopulation<DIM> >(mpCellPopulation))
    {
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkDoubleArray> p_cell_color_reference_data = vtkSmartPointer<vtkDoubleArray>::New();
        p_cell_color_reference_data->SetName("CellColors");
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

            if(mColorByCellType)
            {
                p_cell_color_reference_data->InsertNextTuple1((*cell_iter)->GetCellProliferativeType()->GetColour());
            }
            else if(mColorByCellData and !this->mDataLabel.empty())
            {
                std::vector<std::string> keys = (*cell_iter)->GetCellData()->GetKeys();
                if (std::find(keys.begin(), keys.end(), this->mDataLabel) != keys.end())
                {
                    p_cell_color_reference_data->InsertNextTuple1((*cell_iter)->GetCellData()->GetItem(this->mDataLabel));
                }
                else
                {
                    p_cell_color_reference_data->InsertNextTuple1(0.0);
                }
            }
            else if(mColorByCellMutationState)
            {
                double mutation_state = (*cell_iter)->GetMutationState()->GetColour();

                CellPropertyCollection collection = (*cell_iter)->rGetCellPropertyCollection();
                CellPropertyCollection label_collection = collection.GetProperties<CellLabel>();

                if (label_collection.GetSize() == 1)
                {
                    boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(label_collection.GetProperty());
                    mutation_state = p_label->GetColour();
                }
                p_cell_color_reference_data->InsertNextTuple1(mutation_state);
            }
            else if(mColorByCellLabel)
            {
                double label = 0.0;
                if ((*cell_iter)->template HasCellProperty<CellLabel>())
                {
                    CellPropertyCollection collection = (*cell_iter)->rGetCellPropertyCollection().template GetProperties<CellLabel>();
                    boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
                    label = p_label->GetColour();
                }
                p_cell_color_reference_data->InsertNextTuple1(label);
            }
            else
            {
                p_cell_color_reference_data->InsertNextTuple1((*cell_iter)->GetCellId());
            }
        }
        p_polydata->SetPoints(p_points);
        p_polydata->GetPointData()->AddArray(p_cell_color_reference_data);

        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!mColorByCellData)
        {
            double range[2];
            p_polydata->GetPointData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpDiscreteColorTransferFunction->GetColor((255.0-double(idx))/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        else
        {
            double range[2];
            p_polydata->GetPointData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }

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
        p_mapper->SetLookupTable(p_scaled_ctf);
        p_mapper->ScalarVisibilityOn();
        p_mapper->SelectColorArray("CellColors");
        p_mapper->SetScalarModeToUsePointFieldData();
        p_mapper->SetColorModeToMapScalars();

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        p_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
        if(mColorCellByUserDefined)
        {
            p_actor->GetProperty()->SetColor(this->mPointColor[0], this->mPointColor[1], this->mPointColor[2]);
        }
        pRenderer->AddActor(p_actor);

        if(!this->mDataLabel.empty() and this->mShowScaleBar)
        {
            this->mpScaleBar->SetLookupTable(p_scaled_ctf);
            this->mpScaleBar->SetTitle(this->mDataLabel.c_str());
            pRenderer->AddActor(this->mpScaleBar);
        }
    }

    if(std::dynamic_pointer_cast<MeshBasedCellPopulation<DIM> >(mpCellPopulation) and (mShowMutableMeshEdges or mShowVoronoiMeshEdges))
    {
        AddMeshBasedCellPopulationActor(pRenderer);
    }
    else if (std::dynamic_pointer_cast<VertexBasedCellPopulation<DIM> >(mpCellPopulation) and mShowVoronoiMeshEdges)
    {
        AddVertexBasedCellPopulationActor(pRenderer);
    }
    else if (std::dynamic_pointer_cast<PottsBasedCellPopulation<DIM> >(mpCellPopulation) and (mShowPottsMeshEdges or mShowPottsMeshOutlines))
    {
        AddPottsBasedCellPopulationActor(pRenderer);
    }
    else if (std::dynamic_pointer_cast<CaBasedCellPopulation<DIM> >(mpCellPopulation) and mShowPottsMeshEdges)
    {
        AddCaBasedCellPopulationActor(pRenderer);
    }
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::AddVertexBasedCellPopulationActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    std::shared_ptr<VertexBasedCellPopulation<DIM> > p_cell_population = std::dynamic_pointer_cast<VertexBasedCellPopulation<DIM> >(mpCellPopulation);

    if(!p_cell_population)
    {
        EXCEPTION("Could not cast mesh to Vertex Based type.");
    }

    if(mShowVoronoiMeshEdges)
    {
        vtkSmartPointer<vtkUnstructuredGrid> p_voronoi_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
        vtkSmartPointer<vtkDoubleArray> p_cell_color_reference_data = vtkSmartPointer<vtkDoubleArray>::New();
        p_cell_color_reference_data->SetName("CellColors");

        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        p_points->GetData()->SetName("Vertex positions");
        for (unsigned node_num=0; node_num<p_cell_population->rGetMesh().GetNumNodes(); node_num++)
        {
            c_vector<double, DIM> position = p_cell_population->rGetMesh().GetNode(node_num)->rGetLocation();
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

        for (typename VertexMesh<DIM,DIM>::VertexElementIterator iter = p_cell_population->rGetMesh().GetElementIteratorBegin();
             iter != p_cell_population->rGetMesh().GetElementIteratorEnd(); ++iter)
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

            unsigned element_index = iter->GetIndex();
            CellPtr p_biological_cell = p_cell_population->GetCellUsingLocationIndex(element_index);

            if(mColorByCellType)
            {
                p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellProliferativeType()->GetColour());
            }

            else if(mColorByCellData and !this->mDataLabel.empty())
            {
                std::vector<std::string> keys = p_biological_cell->GetCellData()->GetKeys();
                if (std::find(keys.begin(), keys.end(), this->mDataLabel) != keys.end())
                {
                    p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellData()->GetItem(this->mDataLabel));
                }
                else
                {
                    p_cell_color_reference_data->InsertNextTuple1(0.0);
                }
            }

            else if(mColorByCellMutationState)
            {
                double mutation_state = p_biological_cell->GetMutationState()->GetColour();

                CellPropertyCollection collection = p_biological_cell->rGetCellPropertyCollection();
                CellPropertyCollection label_collection = collection.GetProperties<CellLabel>();

                if (label_collection.GetSize() == 1)
                {
                    boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(label_collection.GetProperty());
                    mutation_state = p_label->GetColour();
                }
                p_cell_color_reference_data->InsertNextTuple1(mutation_state);
            }
            else if(mColorByCellLabel)
            {
                double label = 0.0;
                if (p_biological_cell->HasCellProperty<CellLabel>())
                {
                    CellPropertyCollection collection = p_biological_cell->rGetCellPropertyCollection().GetProperties<CellLabel>();
                    boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
                    label = p_label->GetColour();
                }
                p_cell_color_reference_data->InsertNextTuple1(label);
            }
            else
            {
                p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellId());
            }
        }

        p_voronoi_grid->GetCellData()->AddArray(p_cell_color_reference_data);
        p_voronoi_grid->GetCellData()->SetScalars(p_cell_color_reference_data);

        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!mColorByCellData)
        {
            double range[2];
            p_voronoi_grid->GetCellData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpDiscreteColorTransferFunction->GetColor((255.0-double(idx))/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        else
        {
            double range[2];
            p_voronoi_grid->GetCellData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        p_scaled_ctf->Build();

        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_geom_filter->SetInput(p_voronoi_grid);
        #else
            p_geom_filter->SetInputData(p_voronoi_grid);
        #endif

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_mapper->SetInputConnection(p_geom_filter->GetOutputPort());
        p_mapper->SetLookupTable(p_scaled_ctf);
        p_mapper->ScalarVisibilityOn();
        p_mapper->SelectColorArray("CellColors");
        p_mapper->SetScalarModeToUseCellData();
        p_mapper->SetColorModeToMapScalars();

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        p_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
        if(mColorCellByUserDefined)
        {
            p_actor->GetProperty()->SetColor(this->mPointColor[0], this->mPointColor[1], this->mPointColor[2]);
        }
        pRenderer->AddActor(p_actor);

        vtkSmartPointer<vtkFeatureEdges> p_voronoi_extract_edges = vtkSmartPointer<vtkFeatureEdges>::New();
        p_voronoi_extract_edges->SetInputConnection(p_geom_filter->GetOutputPort());
        p_voronoi_extract_edges->SetFeatureEdges(false);
        p_voronoi_extract_edges->SetBoundaryEdges(true);
        p_voronoi_extract_edges->SetManifoldEdges(true);
        p_voronoi_extract_edges->SetNonManifoldEdges(false);

        vtkSmartPointer<vtkTubeFilter> p_voronoi_tubes = vtkSmartPointer<vtkTubeFilter>::New();
        p_voronoi_tubes->SetInputConnection(p_voronoi_extract_edges->GetOutputPort());
        p_voronoi_tubes->SetRadius(this->mEdgeSize);
        p_voronoi_tubes->SetNumberOfSides(12);

        vtkSmartPointer<vtkPolyDataMapper> p_voronoi_tube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_voronoi_tube_mapper->SetInputConnection(p_voronoi_tubes->GetOutputPort());
        p_voronoi_tube_mapper->ScalarVisibilityOff();

        vtkSmartPointer<vtkActor> p_voronoi_tube_actor = vtkSmartPointer<vtkActor>::New();
        p_voronoi_tube_actor->SetMapper(p_voronoi_tube_mapper);
        p_voronoi_tube_actor->GetProperty()->SetColor(this->mEdgeColor[0], this->mEdgeColor[1], this->mEdgeColor[2]);
        pRenderer->AddActor(p_voronoi_tube_actor);

        if(!this->mDataLabel.empty() and this->mShowScaleBar)
        {
            this->mpScaleBar->SetLookupTable(p_scaled_ctf);
            this->mpScaleBar->SetTitle(this->mDataLabel.c_str());
            pRenderer->AddActor(this->mpScaleBar);
        }
    }
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::AddMeshBasedCellPopulationActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    std::shared_ptr<MeshBasedCellPopulation<DIM> > p_cell_population = std::dynamic_pointer_cast<MeshBasedCellPopulation<DIM> >(mpCellPopulation);
    std::shared_ptr<MeshBasedCellPopulationWithGhostNodes<DIM> > p_cell_population_with_ghost =
            std::dynamic_pointer_cast<MeshBasedCellPopulationWithGhostNodes<DIM> >(mpCellPopulation);


    if(!p_cell_population)
    {
        EXCEPTION("Could not cast mesh to MeshBased type.");
    }

    // Add the voronoi mesh
    if(mShowVoronoiMeshEdges)
    {
        p_cell_population->CreateVoronoiTessellation();
        vtkSmartPointer<vtkUnstructuredGrid> p_voronoi_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
        vtkSmartPointer<vtkDoubleArray> p_cell_color_reference_data = vtkSmartPointer<vtkDoubleArray>::New();
        p_cell_color_reference_data->SetName("CellColors");

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

                unsigned node_index =
                        p_cell_population->GetVoronoiTessellation()->GetDelaunayNodeIndexCorrespondingToVoronoiElementIndex(iter->GetIndex());

                bool is_ghost_node = false;
                if(p_cell_population_with_ghost)
                {
                    is_ghost_node = p_cell_population_with_ghost->IsGhostNode(node_index);
                }

                if(!is_ghost_node)
                {
                    CellPtr p_biological_cell = p_cell_population->GetCellUsingLocationIndex(node_index);

                    if(mColorByCellType)
                    {
                        p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellProliferativeType()->GetColour());
                    }

                    else if(mColorByCellData and !this->mDataLabel.empty())
                    {
                        std::vector<std::string> keys = p_biological_cell->GetCellData()->GetKeys();
                        if (std::find(keys.begin(), keys.end(), this->mDataLabel) != keys.end())
                        {
                            p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellData()->GetItem(this->mDataLabel));
                        }
                        else
                        {
                            p_cell_color_reference_data->InsertNextTuple1(0.0);
                        }
                    }
                    else if(mColorByCellMutationState)
                    {
                        double mutation_state = p_biological_cell->GetMutationState()->GetColour();

                        CellPropertyCollection collection = p_biological_cell->rGetCellPropertyCollection();
                        CellPropertyCollection label_collection = collection.GetProperties<CellLabel>();

                        if (label_collection.GetSize() == 1)
                        {
                            boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(label_collection.GetProperty());
                            mutation_state = p_label->GetColour();
                        }
                        p_cell_color_reference_data->InsertNextTuple1(mutation_state);
                    }
                    else if(mColorByCellLabel)
                    {
                        double label = 0.0;
                        if (p_biological_cell->HasCellProperty<CellLabel>())
                        {
                            CellPropertyCollection collection = p_biological_cell->rGetCellPropertyCollection().GetProperties<CellLabel>();
                            boost::shared_ptr<CellLabel> p_label = boost::static_pointer_cast<CellLabel>(collection.GetProperty());
                            label = p_label->GetColour();
                        }
                        p_cell_color_reference_data->InsertNextTuple1(label);
                    }
                    else
                    {
                        p_cell_color_reference_data->InsertNextTuple1(p_biological_cell->GetCellId());
                    }
                }
                else
                {
                    p_cell_color_reference_data->InsertNextTuple1(-1.0);
                }
            }
        }

        p_voronoi_grid->GetCellData()->AddArray(p_cell_color_reference_data);
        p_voronoi_grid->GetCellData()->SetScalars(p_cell_color_reference_data);

        vtkSmartPointer<vtkColorTransferFunction> p_scaled_ctf = vtkSmartPointer<vtkColorTransferFunction>::New();
        if(!mColorByCellData)
        {
            double range[2];
            p_voronoi_grid->GetCellData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpDiscreteColorTransferFunction->GetColor((255.0-double(idx))/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        else
        {
            double range[2];
            p_voronoi_grid->GetCellData()->GetArray("CellColors")->GetRange(range);
            for(unsigned idx=0; idx<255; idx++)
            {
                double color[3];
                this->mpColorTransferFunction->GetColor(double(idx)/255.0, color);
                p_scaled_ctf->AddRGBPoint(range[0] + double(idx)*(range[1]-range[0])/255.0, color[0], color[1], color[2]);
            }
        }
        p_scaled_ctf->Build();

        vtkSmartPointer<vtkThreshold> p_threshold = vtkSmartPointer<vtkThreshold>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_threshold->SetInput(p_voronoi_grid);
        #else
            p_threshold->SetInputData(p_voronoi_grid);
        #endif
        p_threshold->ThresholdByUpper(0.0);
        p_threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "CellColors");

        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        p_geom_filter->SetInputConnection(p_threshold->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_mapper->SetInputConnection(p_geom_filter->GetOutputPort());
        p_mapper->SetLookupTable(p_scaled_ctf);
        p_mapper->ScalarVisibilityOn();
        //p_grid_mapper->InterpolateScalarsBeforeMappingOn();
        p_mapper->SelectColorArray("CellColors");
        p_mapper->SetScalarModeToUseCellData();
        p_mapper->SetColorModeToMapScalars();

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        p_actor->GetProperty()->SetOpacity(this->mVolumeOpacity);
        if(mColorCellByUserDefined)
        {
            p_actor->GetProperty()->SetColor(this->mPointColor[0], this->mPointColor[1], this->mPointColor[2]);
        }
        pRenderer->AddActor(p_actor);

        vtkSmartPointer<vtkFeatureEdges> p_voronoi_extract_edges = vtkSmartPointer<vtkFeatureEdges>::New();
        p_voronoi_extract_edges->SetInputConnection(p_geom_filter->GetOutputPort());
        p_voronoi_extract_edges->SetFeatureEdges(false);
        p_voronoi_extract_edges->SetBoundaryEdges(true);
        p_voronoi_extract_edges->SetManifoldEdges(true);
        p_voronoi_extract_edges->SetNonManifoldEdges(false);

        vtkSmartPointer<vtkTubeFilter> p_voronoi_tubes = vtkSmartPointer<vtkTubeFilter>::New();
        p_voronoi_tubes->SetInputConnection(p_voronoi_extract_edges->GetOutputPort());
        p_voronoi_tubes->SetRadius(this->mEdgeSize);
        p_voronoi_tubes->SetNumberOfSides(12);

        vtkSmartPointer<vtkPolyDataMapper> p_voronoi_tube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_voronoi_tube_mapper->SetInputConnection(p_voronoi_tubes->GetOutputPort());
        p_voronoi_tube_mapper->ScalarVisibilityOff();

        vtkSmartPointer<vtkActor> p_voronoi_tube_actor = vtkSmartPointer<vtkActor>::New();
        p_voronoi_tube_actor->SetMapper(p_voronoi_tube_mapper);
        p_voronoi_tube_actor->GetProperty()->SetColor(this->mEdgeColor[0], this->mEdgeColor[1], this->mEdgeColor[2]);
        pRenderer->AddActor(p_voronoi_tube_actor);

        if(!this->mDataLabel.empty() and this->mShowScaleBar)
        {
            this->mpScaleBar->SetLookupTable(p_scaled_ctf);
            this->mpScaleBar->SetTitle(this->mDataLabel.c_str());
            pRenderer->AddActor(this->mpScaleBar);
        }
    }

    if(mShowMutableMeshEdges)
    {
        // Do the mutable mesh
        //Make the local mesh into a VtkMesh
        vtkSmartPointer<vtkUnstructuredGrid> p_mutable_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
        vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
        p_points->GetData()->SetName("Vertex positions");

        for (typename AbstractMesh<DIM,DIM>::NodeIterator node_iter = p_cell_population->rGetMesh().GetNodeIteratorBegin();
             node_iter != p_cell_population->rGetMesh().GetNodeIteratorEnd();
             ++node_iter)
        {
            c_vector<double, DIM> current_item = node_iter->rGetLocation();
            if (DIM == 3)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], current_item[2]);
            }
            else if (DIM == 2)
            {
                p_points->InsertNextPoint(current_item[0], current_item[1], 0.0);
            }
            else // (DIM == 1)
            {
                p_points->InsertNextPoint(current_item[0], 0.0, 0.0);
            }
        }

        p_mutable_grid->SetPoints(p_points);

        for (typename AbstractTetrahedralMesh<DIM,DIM>::ElementIterator elem_iter = p_cell_population->rGetMesh().GetElementIteratorBegin();
             elem_iter != p_cell_population->rGetMesh().GetElementIteratorEnd();
             ++elem_iter)
        {

            vtkSmartPointer<vtkCell> p_cell;
            if (DIM == 3)
            {
                p_cell = vtkSmartPointer<vtkTetra>::New();
            }
            else if (DIM == 2)
            {
                p_cell = vtkSmartPointer<vtkTriangle>::New();
            }
            else //(DIM == 1)
            {
                p_cell = vtkSmartPointer<vtkLine>::New();
            }
            vtkSmartPointer<vtkIdList> p_cell_id_list = p_cell->GetPointIds();
            for (unsigned j = 0; j < DIM+1; ++j)
            {
                unsigned global_node_index = elem_iter->GetNodeGlobalIndex(j);
                p_cell_id_list->SetId(j, global_node_index);
            }
            p_mutable_grid->InsertNextCell(p_cell->GetCellType(), p_cell_id_list);
        }

        vtkSmartPointer<vtkGeometryFilter> p_mutable_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
        p_mutable_geom_filter->SetInput(p_mutable_grid);
        #else
        p_mutable_geom_filter->SetInputData(p_mutable_grid);
        #endif

        vtkSmartPointer<vtkFeatureEdges> p_extract_edges = vtkSmartPointer<vtkFeatureEdges>::New();
        p_extract_edges->SetInputConnection(p_mutable_geom_filter->GetOutputPort());
        p_extract_edges->SetFeatureEdges(false);
        p_extract_edges->SetBoundaryEdges(true);
        p_extract_edges->SetManifoldEdges(true);
        p_extract_edges->SetNonManifoldEdges(false);

        vtkSmartPointer<vtkTubeFilter> p_mutable_tubes = vtkSmartPointer<vtkTubeFilter>::New();
        p_mutable_tubes->SetInputConnection(p_extract_edges->GetOutputPort());
        p_mutable_tubes->SetRadius(0.02);
        p_mutable_tubes->SetNumberOfSides(12);

        vtkSmartPointer<vtkPolyDataMapper> p_mutable_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        p_mutable_mapper->SetInputConnection(p_mutable_tubes->GetOutputPort());

        vtkSmartPointer<vtkActor> p_mutable_actor = vtkSmartPointer<vtkActor>::New();
        p_mutable_actor->SetMapper(p_mutable_mapper);
        p_mutable_actor->GetProperty()->SetColor(1,1,1);
        pRenderer->AddActor(p_mutable_actor);
    }
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetCellPopulation(std::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation)
{
    this->mpCellPopulation = pCellPopulation;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetShowMutableMeshEdges(bool showEdges)
{
    mShowMutableMeshEdges = showEdges;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetShowVoronoiMeshEdges(bool showEdges)
{
    mShowVoronoiMeshEdges = showEdges;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetColorByUserDefined(bool colorByCellUserDefined)
{
    mColorCellByUserDefined = colorByCellUserDefined;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetShowPottsMeshEdges(bool showEdges)
{
    mShowPottsMeshEdges = showEdges;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetColorByCellMutationState(bool colorByCellMutationState)
{
    mColorByCellMutationState = colorByCellMutationState;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetColorByCellLabel(bool colorByCellLabel)
{
    mColorByCellLabel = colorByCellLabel;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetShowPottsMeshOutlines(bool showEdges)
{
    mShowPottsMeshOutlines = showEdges;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetColorByCellType(bool colorByCellType)
{
    mColorByCellType = colorByCellType;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetColorByCellData(bool colorByCellData)
{
    mColorByCellData = colorByCellData;
}

template<unsigned DIM>
void CellPopulationActorGenerator<DIM>::SetShowCellCentres(bool showCentres)
{
    mShowCellCentres = showCentres;
}

template class CellPopulationActorGenerator<2>;
template class CellPopulationActorGenerator<3>;
