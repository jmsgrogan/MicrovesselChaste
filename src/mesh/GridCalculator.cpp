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
#include <vtkVersion.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkPoints.h>
#include <vtkCellLocator.h>
#include "PetscTools.hpp"
#include "Exception.hpp"
#include "GridCalculator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
GridCalculator<DIM>::GridCalculator() :
        mpNetwork(),
        mpCellPopulation(NULL),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mCellPopulationReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale()),
        mCellMap(),
        mVesselNodeMap(),
        mSegmentMap(),
        mpGrid(),
        mHasRegularGrid(false),
        mHasUnstructuredGrid(false)
{

}

template<unsigned DIM>
std::shared_ptr<GridCalculator<DIM> > GridCalculator<DIM>::Create()
{
    return std::make_shared<GridCalculator<DIM> >();
}

template<unsigned DIM>
GridCalculator<DIM>::~GridCalculator()
{

}

template<unsigned DIM>
bool GridCalculator<DIM>::CellPopulationIsSet()
{
    return bool(mpCellPopulation);
}

template<unsigned DIM>
std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > GridCalculator<DIM>::GetGrid()
{
    if(!mpGrid)
    {
        EXCEPTION("A grid has been requested, but one has not been set.");
    }
    return mpGrid;
}

template<unsigned DIM>
VesselNetworkPtr<DIM> GridCalculator<DIM>::GetVesselNetwork()
{
    if(!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set in the grid calculator");
    }
    return mpNetwork;
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > GridCalculator<DIM>::GetPointMap(vtkSmartPointer<vtkPoints> pInputPoints)
{
    std::vector<std::vector<unsigned> > point_map(mpGrid->GetNumberOfCells());
    for(unsigned idx=0;idx<pInputPoints->GetNumberOfPoints();idx++)
    {
        int cell_id = mpGrid->GetVtkCellLocator()->FindCell(pInputPoints->GetPoint(idx));
        if(cell_id>=0)
        {
            point_map[cell_id].push_back(idx);
        }
    }
    return point_map;
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > GridCalculator<DIM>::GetPointMap(const std::vector<Vertex<DIM> >& rInputPoints)
{
    QLength grid_length = mpGrid->GetReferenceLengthScale();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0; idx<rInputPoints.size(); idx++)
    {
        c_vector<double, 3> loc = rInputPoints[idx].Convert3(grid_length);
        p_points->InsertNextPoint(&loc[0]);
    }
    return GetPointMap(p_points);
}

template<unsigned DIM>
const std::vector<std::vector<VesselNodePtr<DIM> > >& GridCalculator<DIM>::rGetVesselNodeMap(bool update)
{
    if (!update)
    {
        return mVesselNodeMap;
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a point node map.");
    }

    QLength grid_length = mpGrid->GetReferenceLengthScale();
    mVesselNodeMap.clear();
    mVesselNodeMap = std::vector<std::vector<VesselNodePtr<DIM> > >(mpGrid->GetNumberOfCells());
    for(auto& node:mpNetwork->GetNodes())
    {
        c_vector<double, 3> loc = node->rGetLocation().Convert3(grid_length);
        int cell_id = mpGrid->GetVtkCellLocator()->FindCell(&loc[0]);
        if(cell_id>=0)
        {
            mVesselNodeMap[cell_id].push_back(node);
        }
    }
    return mVesselNodeMap;
}

template<unsigned DIM>
std::vector<std::vector<VesselNodePtr<DIM> > > GridCalculator<DIM>::GetVesselNodeMap(vtkSmartPointer<vtkUnstructuredGrid> pGrid,
            VesselNetworkPtr<DIM> pNetwork, QLength referenceLength)
{
    vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
    p_sampling_locator->SetDataSet(pGrid);
    p_sampling_locator->BuildLocator();

    std::vector<std::vector<VesselNodePtr<DIM> > > vesselNodeMap = std::vector<std::vector<VesselNodePtr<DIM> > >(pGrid->GetNumberOfCells());
    for(auto& node:pNetwork->GetNodes())
    {
        c_vector<double, 3> loc = node->rGetLocation().Convert3(referenceLength);
        int cell_id = p_sampling_locator->FindCell(&loc[0]);
        if(cell_id>=0)
        {
            vesselNodeMap[cell_id].push_back(node);
        }
    }
    return vesselNodeMap;
}

template<unsigned DIM>
const std::vector<std::vector<CellPtr> >& GridCalculator<DIM>::rGetCellMap(bool update)
{
    if (!update)
    {
        return mCellMap;
    }

    if (!mpCellPopulation)
    {
        EXCEPTION("A cell population has not been set. Can not create a cell point map.");
    }

    // Make sure the mesh vtk solution is up to date
    mpGrid->GetGlobalVtkGrid();

    // Loop over all cells and associate cells with the points
    mCellMap.clear();
    mCellMap = std::vector<std::vector<CellPtr> >(mpGrid->GetNumberOfCells());

    QLength grid_length = mpGrid->GetReferenceLengthScale();
    double cell_mesh_length_scaling = mCellPopulationReferenceLength/grid_length;
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
            cell_iter != mpCellPopulation->End(); ++cell_iter)
    {
        c_vector<double, DIM> location = mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        double x_coords[3];
        x_coords[0] = location[0]*cell_mesh_length_scaling;
        x_coords[1] = location[1]*cell_mesh_length_scaling;
        if(DIM == 3)
        {
            x_coords[2] = location[2]*cell_mesh_length_scaling;
        }
        else
        {
            x_coords[2] = 0.0;
        }
        int cell_id = mpGrid->GetVtkCellLocator()->FindCell(&x_coords[0]);
        if(cell_id>=0)
        {
            mCellMap[cell_id].push_back(*cell_iter);
        }
    }
    return mCellMap;
}

template<unsigned DIM>
bool GridCalculator<DIM>::HasStructuredGrid()
{
    return mHasRegularGrid;
}

template<unsigned DIM>
bool GridCalculator<DIM>::HasUnstructuredGrid()
{
    return mHasUnstructuredGrid;
}

template<unsigned DIM>
bool GridCalculator<DIM>::IsSegmentAtLocation(unsigned index, bool update)
{
    if(update or mSegmentMap.size()==0)
    {
        rGetSegmentMap(true);
    }
    if(mSegmentMap.size()==0)
    {
        EXCEPTION("Zero size segment map after update. Something might be wrong with the grid.");
    }

    if(index>=mSegmentMap.size())
    {
        std::string request_index = std::to_string(index);
        std::string grid_size = std::to_string(mSegmentMap.size());
        EXCEPTION("The requested grid index " + request_index + "  is greater than the segment map size " + grid_size);
    }
    return mSegmentMap[index].size()>0;
}

template<unsigned DIM>
std::vector<std::vector<VesselSegmentPtr<DIM> > > GridCalculator<DIM>::GetSegmentMap(vtkSmartPointer<vtkUnstructuredGrid> pGrid,
        VesselNetworkPtr<DIM> pNetwork, QLength referenceLength)
{
    std::vector<std::vector<VesselSegmentPtr<DIM> > > result =
            std::vector<std::vector<VesselSegmentPtr<DIM> > >(pGrid->GetNumberOfCells());
    vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
    p_sampling_locator->SetDataSet(pGrid);
    p_sampling_locator->BuildLocator();

    std::vector<VesselSegmentPtr<DIM> > segments = pNetwork->GetVesselSegments();
    for (auto& segment:segments)
    {
        c_vector<double, 3> loc1 = segment->GetNode(0)->rGetLocation().Convert3(referenceLength);
        c_vector<double, 3> loc2 = segment->GetNode(1)->rGetLocation().Convert3(referenceLength);
        vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();

        p_sampling_locator->FindCellsAlongLine(&loc1[0], &loc2[0], 1.e-8, p_id_list);
        unsigned num_intersections = p_id_list->GetNumberOfIds();
        for(unsigned idx=0; idx<num_intersections; idx++)
        {
            result[p_id_list->GetId(idx)].push_back(segment);
        }
    }
    return result;
}

template<unsigned DIM>
const std::vector<std::vector<VesselSegmentPtr<DIM> > >& GridCalculator<DIM>::rGetSegmentMap(
        bool update, bool useVesselSurface)
{
    if (!update)
    {
        return mSegmentMap;
    }
    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a vessel point map.");
    }

    // Make sure the mesh vtk solution is up to date
    mpGrid->GetGlobalVtkGrid();

    mSegmentMap.clear();
    mSegmentMap = std::vector<std::vector<VesselSegmentPtr<DIM> > >(mpGrid->GetNumberOfCells());
    QLength grid_length = mpGrid->GetReferenceLengthScale();

    std::vector<VesselSegmentPtr<DIM> > segments = mpNetwork->GetVesselSegments();
    for (auto& segment:segments)
    {
        c_vector<double, 3> loc1 = segment->GetNode(0)->rGetLocation().Convert3(grid_length);
        c_vector<double, 3> loc2 = segment->GetNode(1)->rGetLocation().Convert3(grid_length);
        vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();
        if(useVesselSurface)
        {
            double dimensionless_radius = segment->GetRadius()/grid_length;
            vtkSmartPointer<vtkLineSource> p_line_source = vtkSmartPointer<vtkLineSource>::New();
            p_line_source->SetPoint1(&loc1[0]);
            p_line_source->SetPoint2(&loc2[0]);
            vtkSmartPointer<vtkTubeFilter> p_tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
            p_tube_filter->SetInputConnection(p_line_source->GetOutputPort());
            p_tube_filter->SetRadius(dimensionless_radius);
            p_tube_filter->SetCapping(1);
            p_tube_filter->SetNumberOfSides(12);
            vtkSmartPointer<vtkPolyData> p_point_polydata = vtkSmartPointer<vtkPolyData>::New();
            p_point_polydata->SetPoints(mpGrid->GetCellLocations());

            vtkSmartPointer<vtkSelectEnclosedPoints> p_select_encolsed = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
            #if VTK_MAJOR_VERSION <= 5
            p_select_encolsed->SetInput(p_point_polydata);
            #else
            p_select_encolsed->SetInputData(p_point_polydata);
            #endif
            p_select_encolsed->SetSurfaceConnection(p_tube_filter->GetOutputPort());
            p_select_encolsed->Update();
            for(unsigned idx=0;idx<mpGrid->GetNumberOfCells();idx++)
            {
                if(p_select_encolsed->IsInside(idx))
                {
                    mSegmentMap[idx].push_back(segment);
                }
            }
        }
        else
        {
            mpGrid->GetVtkCellLocator()->FindCellsAlongLine(&loc1[0], &loc2[0], 1.e-8, p_id_list);
            unsigned num_intersections = p_id_list->GetNumberOfIds();
            for(unsigned idx=0; idx<num_intersections; idx++)
            {
                mSegmentMap[p_id_list->GetId(idx)].push_back(segment);
            }
        }
    }
    return mSegmentMap;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation,
        QLength cellPopulationReferenceLength,
        QConcentration cellPopulationReferenceConcentration)
{
    mpCellPopulation = &rCellPopulation;
    mCellPopulationReferenceLength = cellPopulationReferenceLength;
    mCellPopulationReferenceConcentration = cellPopulationReferenceConcentration;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetGrid(std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    mpGrid = pGrid;
    mHasRegularGrid = bool(std::dynamic_pointer_cast<RegularGrid<DIM> >(pGrid));
    mHasUnstructuredGrid = !mHasRegularGrid;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetVesselNetwork(VesselNetworkPtr<DIM> pNetwork)
{
    mpNetwork = pNetwork;
}

// Explicit instantiation
template class GridCalculator<2>;
template class GridCalculator<3>;
