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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkThreshold.h>
#include <vtkWedge.h>
#include <vtkVoxel.h>
#include <vtkPoints.h>
#include <vtkUnstructuredGrid.h>
#include "VesselSegment.hpp"
#include "DensityMap.hpp"
#include "GeometryTools.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
DensityMap<DIM>::DensityMap() :
        mVesselSurfaceAreaDensity(),
        mVesselLineDensity(),
        mPerfusedVesselSurfaceAreaDensity(),
        mPerfusedVesselLineDensity(),
        mVesselTipDensity(),
        mVesselBranchDensity(),
        mVesselQuantityDensity(),
        mDimensionlessCellDensity(),
        mDimensionlessCellDensityByMutationType(),
        mpGridCalculator(GridCalculator<DIM>::Create())
{

}

template<unsigned DIM>
std::shared_ptr<DensityMap<DIM> > DensityMap<DIM>::Create()
{
    return std::make_shared<DensityMap<DIM> >();
}

template<unsigned DIM>
DensityMap<DIM>::~DensityMap()
{

}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > DensityMap<DIM>::GetVesselNetwork()
{
    return mpGridCalculator->GetVesselNetwork();
}

template<unsigned DIM>
vtkSmartPointer<vtkUnstructuredGrid> DensityMap<DIM>::GetSamplingGrid(vtkSmartPointer<vtkUnstructuredGrid> pGrid)
{
    vtkSmartPointer<vtkUnstructuredGrid> p_tri_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    p_tri_grid->Allocate(1,1);
    vtkSmartPointer<vtkPoints> p_newpoints = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0; idx<pGrid->GetNumberOfCells(); idx++)
    {
        vtkSmartPointer<vtkWedge> p_wedge = vtkSmartPointer<vtkWedge>::New();
        p_wedge->GetPointIds()->SetNumberOfIds(6);
        vtkSmartPointer<vtkPoints> p_points = pGrid->GetCell(idx)->GetPoints();
        for(unsigned jdx=0;jdx<p_points->GetNumberOfPoints();jdx++)
        {
            vtkIdType id1 = p_newpoints->InsertNextPoint(p_points->GetPoint(jdx)[0], p_points->GetPoint(jdx)[1], -0.1);
            p_wedge->GetPointIds()->SetId(jdx, id1);
            vtkIdType id2 = p_newpoints->InsertNextPoint(p_points->GetPoint(jdx)[0], p_points->GetPoint(jdx)[1], 1.0);
            p_wedge->GetPointIds()->SetId(jdx+3, id2);
        }
        p_tri_grid->InsertNextCell(p_wedge->GetCellType(), p_wedge->GetPointIds());
    }
    p_tri_grid->SetPoints(p_newpoints);
    return p_tri_grid;
}

template<unsigned DIM>
vtkSmartPointer<vtkUnstructuredGrid> DensityMap<DIM>::GetSamplingGrid(std::shared_ptr<RegularGrid<DIM> > pGrid)
{
    vtkSmartPointer<vtkUnstructuredGrid> p_tri_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    p_tri_grid->Allocate(1,1);
    vtkSmartPointer<vtkPoints> p_newpoints = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0; idx<pGrid->GetNumberOfCells(); idx++)
    {
        vtkSmartPointer<vtkVoxel> p_voxel = vtkSmartPointer<vtkVoxel>::New();
        c_vector<double, 6> bbox = pGrid->GetPointBoundingBox(idx, true);
        double z_min = bbox[4];
        double z_max = bbox[5];
        if(DIM==2)
        {
            z_min = -1.e-3;
            z_max = 1.e-3;
        }
        vtkIdType id1 = p_newpoints->InsertNextPoint(bbox[0], bbox[2], z_min);
        p_voxel->GetPointIds()->SetId(0, id1);
        vtkIdType id2 = p_newpoints->InsertNextPoint(bbox[1], bbox[2], z_min);
        p_voxel->GetPointIds()->SetId(1, id2);
        vtkIdType id3 = p_newpoints->InsertNextPoint(bbox[0], bbox[3], z_min);
        p_voxel->GetPointIds()->SetId(2, id3);
        vtkIdType id4 = p_newpoints->InsertNextPoint(bbox[1], bbox[3], z_min);
        p_voxel->GetPointIds()->SetId(3, id4);
        vtkIdType id5 = p_newpoints->InsertNextPoint(bbox[0], bbox[2], z_max);
        p_voxel->GetPointIds()->SetId(4, id5);
        vtkIdType id6 = p_newpoints->InsertNextPoint(bbox[1], bbox[2], z_max);
        p_voxel->GetPointIds()->SetId(5, id6);
        vtkIdType id7 = p_newpoints->InsertNextPoint(bbox[0], bbox[3], z_max);
        p_voxel->GetPointIds()->SetId(6, id7);
        vtkIdType id8 = p_newpoints->InsertNextPoint(bbox[1], bbox[3], z_max);
        p_voxel->GetPointIds()->SetId(7, id8);
        p_tri_grid->InsertNextCell(p_voxel->GetCellType(), p_voxel->GetPointIds());
    }
    p_tri_grid->SetPoints(p_newpoints);
    return p_tri_grid;
}

template<unsigned DIM>
double DensityMap<DIM>::LengthOfLineInCell(vtkSmartPointer<vtkUnstructuredGrid> pSamplingGrid, c_vector<double, DIM> loc1,
        c_vector<double, DIM> loc2, unsigned index, bool loc1InCell, bool loc2InCell)
{
    double length_in_cell = 0.0;
    c_vector<double, 3> point1_loc3d;
    if(DIM==3)
    {
        point1_loc3d = loc1;
    }
    else
    {
        point1_loc3d[0] = loc1[0];
        point1_loc3d[1] = loc1[1];
        point1_loc3d[2] = 0.0;
    }

    c_vector<double, 3> point2_loc3d;
    if(DIM==3)
    {
        point2_loc3d = loc2;
    }
    else
    {
        point2_loc3d[0] = loc2[0];
        point2_loc3d[1] = loc2[1];
        point2_loc3d[2] = 0.0;
    }
    if(loc1InCell and loc2InCell)
    {
        return norm_2(loc2-loc1);
    }
    else
    {
        double t;
        c_vector<double,3> intersection;
        c_vector<double,3> parametric_intersection;
        int subId;
        if(loc1InCell and !loc2InCell)
        {
            int found_intersection1 = pSamplingGrid->GetCell(index)->IntersectWithLine(&point2_loc3d[0], &point1_loc3d[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            if(found_intersection1)
            {
                return norm_2(intersection - point1_loc3d);
            }
        }
        if(loc2InCell and !loc1InCell)
        {
            int found_intersection = pSamplingGrid->GetCell(index)->IntersectWithLine(&point1_loc3d[0], &point2_loc3d[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            if(found_intersection)
            {
                return norm_2(intersection - point2_loc3d);
            }
        }
        // Neither point in cell, does line cross the cell?
        int line_crosses = pSamplingGrid->GetCell(index)->IntersectWithLine(&point1_loc3d[0], &point2_loc3d[0], 1.e-6, t, &intersection[0],
                &parametric_intersection[0], subId);

        if(line_crosses)
        {
            c_vector<double,3> intersection2;
            pSamplingGrid->GetCell(index)->IntersectWithLine(&point2_loc3d[0], &point1_loc3d[0], 1.e-6, t, &intersection2[0], &parametric_intersection[0], subId);
            length_in_cell = norm_2(intersection - intersection2);
        }
        else
        {
            length_in_cell = 0.0;
        }
        return length_in_cell;
    }
}

template<unsigned DIM>
bool DensityMap<DIM>::IsPointInCell(vtkSmartPointer<vtkCellLocator> pCellLocator, c_vector<double, DIM> loc, unsigned index)
{
    c_vector<double, 3> point1_loc3d;
     if(DIM==3)
     {
         point1_loc3d = loc;
     }
     else
     {
         point1_loc3d[0] = loc[0];
         point1_loc3d[1] = loc[1];
         point1_loc3d[2] = 0.0;
     }
     return (pCellLocator->FindCell(&point1_loc3d[0])==index);
}

template<unsigned DIM>
std::shared_ptr<GridCalculator<DIM> > DensityMap<DIM>::GetGridCalculator()
{
    return this->mpGridCalculator;
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselSurfaceAreaDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mVesselSurfaceAreaDensity.size() == num_points)
    {
        return mVesselSurfaceAreaDensity;
    }
    else
    {
        mVesselSurfaceAreaDensity.clear();
        mVesselSurfaceAreaDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<std::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap();
        QLength length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;

        if(this->mpGridCalculator->HasStructuredGrid())
        {
            std::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_regular_grid)
            {
                EXCEPTION("Can't cast to regular grid");
            }
            p_sampling_grid = GetSamplingGrid(p_regular_grid);
        }
        else
        {
            std::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                    std::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_mesh)
            {
                EXCEPTION("Can't cast to mesh");
            }
            if(DIM==3)
            {
                p_sampling_grid = vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid());
            }
            else
            {
                p_sampling_grid = GetSamplingGrid(vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid()));
            }
        }
        vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
        p_sampling_locator->SetDataSet(p_sampling_grid);
        p_sampling_locator->BuildLocator();

        for(unsigned idx=0; idx<segment_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
            {
                c_vector<double, DIM> point1_loc = segment_map[idx][jdx]->GetNode(0)->rGetLocation().GetLocation(length_scale);
                c_vector<double, DIM> point2_loc = segment_map[idx][jdx]->GetNode(1)->rGetLocation().GetLocation(length_scale);
                bool point1_in_cell = IsPointInCell(p_sampling_locator, point1_loc, idx);
                bool point2_in_cell = IsPointInCell(p_sampling_locator, point2_loc, idx);
                double dimless_length_in_cell = LengthOfLineInCell(p_sampling_grid, point1_loc, point2_loc,
                        idx, point1_in_cell, point2_in_cell);
                QLength length_in_cell = dimless_length_in_cell*length_scale;
                QArea surface_area = 2.0*M_PI*segment_map[idx][jdx]->GetRadius()*length_in_cell;
                mVesselSurfaceAreaDensity[idx] += ((surface_area/(length_scale*length_scale))/grid_volumes[idx]);
            }
        }
        return mVesselSurfaceAreaDensity;
    }
}

template<unsigned DIM>
std::vector<double> DensityMap<DIM>::rGetVesselLineDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mVesselLineDensity.size() == num_points)
    {
        return mVesselLineDensity;
    }
    else
    {
        mVesselLineDensity.clear();
        mVesselLineDensity = std::vector<double>(num_points, 0.0);
        std::vector<std::vector<std::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap(update);
        QLength length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);

        vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;
        if(this->mpGridCalculator->HasStructuredGrid())
        {
            std::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_regular_grid)
            {
                EXCEPTION("Can't cast to regular grid");
            }
            p_sampling_grid = GetSamplingGrid(p_regular_grid);
        }
        else
        {
            std::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                    std::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_mesh)
            {
                EXCEPTION("Can't cast to mesh");
            }
            if(DIM==3)
            {
                p_sampling_grid = vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid());
            }
            else
            {
                p_sampling_grid = this->GetSamplingGrid(vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid()));
            }
        }

        vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
        p_sampling_locator->SetDataSet(p_sampling_grid);
        p_sampling_locator->BuildLocator();

        for(unsigned idx=0; idx<segment_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
            {
                c_vector<double, DIM> point1_loc = segment_map[idx][jdx]->GetNode(0)->rGetLocation().GetLocation(length_scale);
                c_vector<double, DIM> point2_loc = segment_map[idx][jdx]->GetNode(1)->rGetLocation().GetLocation(length_scale);
                bool point1_in_cell = IsPointInCell(p_sampling_locator, point1_loc, idx);
                bool point2_in_cell = IsPointInCell(p_sampling_locator, point2_loc, idx);
                double dimless_length_in_cell = LengthOfLineInCell(p_sampling_grid, point1_loc, point2_loc,
                        idx, point1_in_cell, point2_in_cell);
                QLength length_in_cell = dimless_length_in_cell*length_scale;
                double grid_volume = grid_volumes[idx];
                mVesselLineDensity[idx] += (length_in_cell/length_scale)/grid_volume;
            }
        }
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetPerfusedVesselSurfaceAreaDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();

    if(!update and mPerfusedVesselSurfaceAreaDensity.size() == num_points)
    {
        return mPerfusedVesselSurfaceAreaDensity;
    }
    else
    {
        mPerfusedVesselSurfaceAreaDensity.clear();
        mPerfusedVesselSurfaceAreaDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);

        std::vector<std::vector<std::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap();
        QLength length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;

        if(this->mpGridCalculator->HasStructuredGrid())
        {
            std::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_regular_grid)
            {
                EXCEPTION("Can't cast to regular grid");
            }
            p_sampling_grid = GetSamplingGrid(p_regular_grid);
        }
        else
        {
            std::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                    std::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_mesh)
            {
                EXCEPTION("Can't cast to mesh");
            }

            if(DIM==3)
            {
                p_sampling_grid = vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid());
            }
            else
            {
                p_sampling_grid = GetSamplingGrid(vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid()));
            }
        }
        vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
        p_sampling_locator->SetDataSet(p_sampling_grid);
        p_sampling_locator->BuildLocator();

        for(unsigned idx=0; idx<segment_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
            {
                c_vector<double, DIM> point1_loc = segment_map[idx][jdx]->GetNode(0)->rGetLocation().GetLocation(length_scale);
                c_vector<double, DIM> point2_loc = segment_map[idx][jdx]->GetNode(1)->rGetLocation().GetLocation(length_scale);
                bool point1_in_cell = IsPointInCell(p_sampling_locator, point1_loc, idx);
                bool point2_in_cell = IsPointInCell(p_sampling_locator, point2_loc, idx);
                double dimless_length_in_cell = LengthOfLineInCell(p_sampling_grid, point1_loc, point2_loc,
                        idx, point1_in_cell, point2_in_cell);
                QLength length_in_cell = dimless_length_in_cell*length_scale;
                QArea surface_area = 2.0*M_PI*segment_map[idx][jdx]->GetRadius()*length_in_cell;
                if(segment_map[idx][jdx]->GetVessel()->GetFlowProperties()->GetHaematocrit()==0.0)
                {
                    surface_area = 0.0*unit::metres_squared;
                }
                mPerfusedVesselSurfaceAreaDensity[idx] += ((surface_area/(length_scale*length_scale))/grid_volumes[idx]);
            }
        }
        return mPerfusedVesselSurfaceAreaDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetPerfusedVesselLineDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mPerfusedVesselLineDensity.size() == num_points)
    {
        return mPerfusedVesselLineDensity;
    }
    else
    {
        mPerfusedVesselLineDensity.clear();
        mPerfusedVesselLineDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<std::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap();
        QLength length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;

        if(this->mpGridCalculator->HasStructuredGrid())
        {
            std::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_regular_grid)
            {
                EXCEPTION("Can't cast to regular grid");
            }
            p_sampling_grid = GetSamplingGrid(p_regular_grid);
        }
        else
        {
            std::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                    std::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_mesh)
            {
                EXCEPTION("Can't cast to mesh");
            }

            vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;
            if(DIM==3)
            {
                p_sampling_grid = vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid());
            }
            else
            {
                p_sampling_grid = GetSamplingGrid(vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid()));
            }
        }
        vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
        p_sampling_locator->SetDataSet(p_sampling_grid);
        p_sampling_locator->BuildLocator();

        for(unsigned idx=0; idx<segment_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
            {
                c_vector<double, DIM> point1_loc = segment_map[idx][jdx]->GetNode(0)->rGetLocation().GetLocation(length_scale);
                c_vector<double, DIM> point2_loc = segment_map[idx][jdx]->GetNode(1)->rGetLocation().GetLocation(length_scale);
                bool point1_in_cell = IsPointInCell(p_sampling_locator, point1_loc, idx);
                bool point2_in_cell = IsPointInCell(p_sampling_locator, point2_loc, idx);
                double dimless_length_in_cell = LengthOfLineInCell(p_sampling_grid, point1_loc, point2_loc,
                        idx, point1_in_cell, point2_in_cell);
                QLength length_in_cell = dimless_length_in_cell*length_scale;
                if(segment_map[idx][jdx]->GetVessel()->GetFlowProperties()->GetHaematocrit()==0.0)
                {
                    length_in_cell = 0.0*unit::metres;
                }
                mPerfusedVesselLineDensity[idx] += (length_in_cell/length_scale)/grid_volumes[idx];
            }
        }
        return mPerfusedVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselTipDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mVesselTipDensity.size() == num_points)
    {
        return mVesselTipDensity;
    }
    else
    {
        mVesselTipDensity.clear();
        mVesselTipDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<std::shared_ptr<VesselNode<DIM> > > > node_map = this->mpGridCalculator->rGetVesselNodeMap();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        for(unsigned idx=0; idx<node_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < node_map[idx].size(); jdx++)
            {
                if(node_map[idx][jdx]->IsMigrating())
                {
                    mVesselTipDensity[idx] += 1.0/grid_volumes[idx];
                }
            }
        }
        return mVesselTipDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselBranchDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mVesselBranchDensity.size() == num_points)
    {
        return mVesselBranchDensity;
    }
    else
    {
        mVesselBranchDensity.clear();
        mVesselBranchDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<std::shared_ptr<VesselNode<DIM> > > > node_map = this->mpGridCalculator->rGetVesselNodeMap();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        for(unsigned idx=0; idx<node_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < node_map[idx].size(); jdx++)
            {
                if(node_map[idx][jdx]->GetNumberOfSegments()>2)
                {
                    mVesselBranchDensity[idx] += 1.0/grid_volumes[idx];
                }
            }
        }
        return mVesselBranchDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselQuantityDensity(const std::string& rQuantity, bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mVesselQuantityDensity.size() == num_points)
    {
        return mVesselQuantityDensity;
    }
    else
    {
        mVesselQuantityDensity.clear();
        mVesselQuantityDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<std::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap();
        QLength length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;

        if(this->mpGridCalculator->HasStructuredGrid())
        {
            std::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_regular_grid)
            {
                EXCEPTION("Can't cast to regular grid");
            }
            p_sampling_grid = GetSamplingGrid(p_regular_grid);
        }
        else
        {
            std::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                    std::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
            if(!p_mesh)
            {
                EXCEPTION("Can't cast to mesh");
            }

            vtkSmartPointer<vtkUnstructuredGrid> p_sampling_grid;
            if(DIM==3)
            {
                p_sampling_grid = vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid());
            }
            else
            {
                p_sampling_grid = GetSamplingGrid(vtkUnstructuredGrid::SafeDownCast(this->mpGridCalculator->GetGrid()->GetGlobalVtkGrid()));
            }
        }
        vtkSmartPointer<vtkCellLocator> p_sampling_locator = vtkSmartPointer<vtkCellLocator>::New();
        p_sampling_locator->SetDataSet(p_sampling_grid);
        p_sampling_locator->BuildLocator();

        for(unsigned idx=0; idx<segment_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
            {
                c_vector<double, DIM> point1_loc = segment_map[idx][jdx]->GetNode(0)->rGetLocation().GetLocation(length_scale);
                c_vector<double, DIM> point2_loc = segment_map[idx][jdx]->GetNode(1)->rGetLocation().GetLocation(length_scale);
                bool point1_in_cell = IsPointInCell(p_sampling_locator, point1_loc, idx);
                bool point2_in_cell = IsPointInCell(p_sampling_locator, point2_loc, idx);
                double dimless_length_in_cell = LengthOfLineInCell(p_sampling_grid, point1_loc, point2_loc,
                        idx, point1_in_cell, point2_in_cell);
                QLength length_in_cell = dimless_length_in_cell*length_scale;
                double amount = segment_map[idx][jdx]->GetFlowProperties()->GetOutputData()[rQuantity];
                if(segment_map[idx][jdx]->GetFlowProperties()->GetHaematocrit()==0.0)
                {
                    length_in_cell = 0.0*unit::metres;
                }
                mVesselQuantityDensity[idx] += (amount*length_in_cell/length_scale)/grid_volumes[idx];
            }
        }
        return mVesselQuantityDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetCellDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mDimensionlessCellDensity.size() == num_points)
    {
        return mDimensionlessCellDensity;
    }
    else
    {
        mDimensionlessCellDensity.clear();
        mDimensionlessCellDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<CellPtr> > cell_map = this->mpGridCalculator->rGetCellMap();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        for(unsigned idx=0; idx<cell_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < cell_map[idx].size(); jdx++)
            {
                mDimensionlessCellDensity[idx] += 1.0/grid_volumes[idx];
            }
        }
        return mDimensionlessCellDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetCellDensity(boost::shared_ptr<AbstractCellMutationState> pMutationState, bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfCells();
    if(!update and mDimensionlessCellDensityByMutationType.size() == num_points)
    {
        return mDimensionlessCellDensityByMutationType;
    }
    else
    {
        mDimensionlessCellDensityByMutationType.clear();
        mDimensionlessCellDensityByMutationType = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfCells(), 0.0);
        std::vector<std::vector<CellPtr > > cell_map = this->mpGridCalculator->rGetCellMap();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetCellVolumes(true, true);
        for(unsigned idx=0; idx<cell_map.size();idx++)
        {
            for (unsigned jdx = 0; jdx < cell_map[idx].size(); jdx++)
            {
                if(cell_map[idx][jdx]->GetMutationState()==pMutationState)
                {
                    mDimensionlessCellDensityByMutationType[idx] += 1.0/grid_volumes[idx];
                }
            }
        }
        return mDimensionlessCellDensityByMutationType;
    }
}

template<unsigned DIM>
void DensityMap<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation,
                                                             QLength cellPopulationReferenceLength,
                                                             QConcentration cellPopulationReferenceConcentration)
{
    mpGridCalculator->SetCellPopulation(rCellPopulation, cellPopulationReferenceLength, cellPopulationReferenceConcentration);
}

template<unsigned DIM>
void DensityMap<DIM>::SetGrid(std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    mpGridCalculator->SetGrid(pGrid);
}

template<unsigned DIM>
void DensityMap<DIM>::SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGridCalculator)
{
    mpGridCalculator = pGridCalculator;
}

template<unsigned DIM>
void DensityMap<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpGridCalculator->SetVesselNetwork(pNetwork);
}

// Explicit instantiation
template class DensityMap<2>;
template class DensityMap<3>;
