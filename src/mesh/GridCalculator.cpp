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

#include <cmath>
#include "PetscTools.hpp"
#include "Exception.hpp"
#include "GridCalculator.hpp"
#include "BaseUnits.hpp"
#include <vtkLine.h>

template<unsigned DIM>
GridCalculator<DIM>::GridCalculator() :
        mpNetwork(),
        mpCellPopulation(),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mCellMap(),
        mVesselNodeMap(),
        mSegmentMap(),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mpRegularGrid(),
        mpMesh(),
        mHasRegularGrid(false),
        mHasUnstructuredGrid(false)
{

}

template<unsigned DIM>
boost::shared_ptr<GridCalculator<DIM> > GridCalculator<DIM>::Create()
{
    typedef GridCalculator<DIM> GridCalculator_Templated;
    MAKE_PTR(GridCalculator_Templated, pSelf);
    return pSelf;
}

template<unsigned DIM>
GridCalculator<DIM>::~GridCalculator()
{

}

template<unsigned DIM>
boost::shared_ptr<RegularGrid<DIM> > GridCalculator<DIM>::GetGrid()
{
    if(!mpRegularGrid)
    {
        EXCEPTION("A regular grid has been requested, but one has not been set.");
    }
    return mpRegularGrid;
}

template<unsigned DIM>
boost::shared_ptr<DiscreteContinuumMesh<DIM> > GridCalculator<DIM>::GetMesh()
{
    if(!mpMesh)
    {
        EXCEPTION("A DiscreteContinuumMesh has been requested, but one has not been set.");
    }
    return mpMesh;
}

template<unsigned DIM>
unsigned GridCalculator<DIM>::GetNumberOfLocations()
{
    if(mHasRegularGrid)
    {
        return mpRegularGrid->GetNumberOfGlobalPoints();
    }
    else if(mHasUnstructuredGrid)
    {
        return mpMesh->GetNumElements();
    }
    else
    {
        EXCEPTION("No grid has been set.");
    }
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > GridCalculator<DIM>::GetPointMap(std::vector<DimensionalChastePoint<DIM> > inputPoints)
{
    if (mpRegularGrid)
    {
        double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();
        c_vector<unsigned, 3> dimensions = mpRegularGrid->GetDimensions();
        std::vector<std::vector<unsigned> > point_map(mpRegularGrid->GetNumberOfGlobalPoints());

        for (unsigned idx = 0; idx < inputPoints.size(); idx++)
        {
            c_vector<double, DIM> offsets = (inputPoints[idx] - mpRegularGrid->GetOrigin()).GetLocation(mReferenceLength);
            unsigned x_index = round(offsets[0]*scale_factor);
            unsigned y_index = round(offsets[1]*scale_factor);
            unsigned z_index = 0;
            if (DIM == 3)
            {
                z_index = round(offsets[2]*scale_factor);
            }

            if (x_index <= dimensions[0] && y_index <= dimensions[1] && z_index <= dimensions[2])
            {
                unsigned grid_index = x_index + y_index * dimensions[0] + z_index * dimensions[0]* dimensions[1];
                point_map[grid_index].push_back(idx);
            }
        }
        return point_map;
    }
    else if(mHasUnstructuredGrid)
    {
        // Make sure the mesh vtk solution is up to date
        mpMesh->GetAsVtkUnstructuredGrid();

        std::vector<std::vector<unsigned> > point_map(mpMesh->GetNumElements());
        std::vector<int> cell_indices(inputPoints.size());
        for(unsigned idx=0; idx<inputPoints.size(); idx++)
        {
            double x_coords[3];
            x_coords[0] = inputPoints[idx].GetLocation(mReferenceLength)[0];
            x_coords[1] = inputPoints[idx].GetLocation(mReferenceLength)[1];
            if(DIM == 3)
            {
                x_coords[2] = inputPoints[idx].GetLocation(mReferenceLength)[2];
            }
            else
            {
                x_coords[2] = 0.0;
            }

            int cell_id=-1;
            cell_id = mpMesh->GetVtkCellLocator()->FindCell(x_coords);

            if(cell_id>=0)
            {
                point_map[cell_id].push_back(idx);
            }
        }
        return point_map;
    }
    else
    {
        EXCEPTION("No grid has been set.");
    }
}

template<unsigned DIM>
const std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >& GridCalculator<DIM>::GetVesselNodeMap(bool update)
{
    if (!update)
    {
        return mVesselNodeMap;
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a point node map.");
    }

    if(mHasRegularGrid)
    {
        // Loop over all nodes and associate cells with the points
        mVesselNodeMap.clear();
        for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfGlobalPoints(); idx++)
        {
            std::vector<boost::shared_ptr<VesselNode<DIM> > > empty_node_pointers;
            mVesselNodeMap.push_back(empty_node_pointers);
        }

        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
        double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();
        c_vector<unsigned, 3> dimensions = mpRegularGrid->GetDimensions();

        for (unsigned idx = 0; idx < nodes.size(); idx++)
        {
            c_vector<double, DIM> offsets = (nodes[idx]->rGetLocation()-mpRegularGrid->GetOrigin()).GetLocation(mReferenceLength);
            unsigned x_index = round(offsets[0]*scale_factor);
            unsigned y_index = round(offsets[1]*scale_factor);
            unsigned z_index = 0;
            if (DIM == 3)
            {
                z_index = round(offsets[2]*scale_factor);
            }

            if (x_index <= dimensions[0] && y_index <= dimensions[1] && z_index <= dimensions[2])
            {
                unsigned grid_index = x_index + y_index * dimensions[0] + z_index * dimensions[0] * dimensions[1];
                if(grid_index<mVesselNodeMap.size())
                {
                    mVesselNodeMap[grid_index].push_back(nodes[idx]);
                }
            }
        }
        return mVesselNodeMap;
    }
    else if(mHasUnstructuredGrid)
    {
        EXCEPTION("Not implemented for elements yet.");
    }
    else
    {
        EXCEPTION("No grid has been set.");
    }
}

template<unsigned DIM>
const std::vector<std::vector<CellPtr> >& GridCalculator<DIM>::GetCellMap(bool update)
{
    if (!update)
    {
        return mCellMap;
    }

    if (!mpCellPopulation)
    {
        EXCEPTION("A cell population has not been set. Can not create a cell point map.");
    }

    if(mHasRegularGrid)
    {
        // Loop over all cells and associate cells with the points
        double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();
        double cell_scale_factor = mCellPopulationReferenceLength / mReferenceLength;

        mCellMap.clear();
        for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfGlobalPoints(); idx++)
        {
            std::vector<CellPtr> empty_cell_pointers;
            mCellMap.push_back(empty_cell_pointers);
        }

        c_vector<unsigned, 3> dimensions = mpRegularGrid->GetDimensions();
        for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
                cell_iter != mpCellPopulation->End(); ++cell_iter)
        {
            c_vector<double, DIM> location = mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
            c_vector<double, DIM> offsets = location*cell_scale_factor - mpRegularGrid->GetOrigin().GetLocation(mReferenceLength);
            unsigned x_index = round(offsets[0]*scale_factor);
            unsigned y_index = round(offsets[1]*scale_factor);
            unsigned z_index = 0;
            if (DIM == 3)
            {
                z_index = round(offsets[2]*scale_factor);
            }

            if (x_index <= dimensions[0] && y_index <= dimensions[1] && z_index <= dimensions[2])
            {
                unsigned grid_index = x_index + y_index * dimensions[0] + z_index * dimensions[0] * dimensions[1];
                if(grid_index<mCellMap.size())
                {
                    mCellMap[grid_index].push_back(*cell_iter);
                }
            }
        }
        return mCellMap;
    }
    else if(mHasUnstructuredGrid)
    {
        // Make sure the mesh vtk solution is up to date
        mpMesh->GetAsVtkUnstructuredGrid();

        // Loop over all cells and associate cells with the points
        mCellMap.clear();
        mCellMap = std::vector<std::vector<CellPtr> >(mpMesh->GetNumElements());

        double cell_mesh_length_scaling = mCellPopulationReferenceLength/mReferenceLength;
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
            int cell_id = mpMesh->GetVtkCellLocator()->FindCell(x_coords);
            if(cell_id>=0)
            {
                mCellMap[cell_id].push_back(*cell_iter);
            }
        }
        return mCellMap;
    }
    else
    {
        EXCEPTION("No grid has been set.");
    }
}

template<unsigned DIM>
std::vector<double> GridCalculator<DIM>::GetLocationVolumes(bool jiggle)
{
    if(mHasRegularGrid)
    {
        return mpRegularGrid->GetPointVolumes(false, jiggle);
    }
    else if(mHasUnstructuredGrid)
    {
        return mpMesh->GetElementVolumes();
    }
    else
    {
        EXCEPTION("Grid not set.");
    }
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
    if(update)
    {
        GetSegmentMap(update);
    }

    if(index>=mSegmentMap.size())
    {
        EXCEPTION("Out of range index requested for point-segment map");
    }
    return mSegmentMap[index].size()>0;
}

template<unsigned DIM>
std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > GridCalculator<DIM>::GetSegmentMap(
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

    if(mHasRegularGrid)
    {
        // Loop over all points and segments and associate segments with the points
        mSegmentMap.clear();
        for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfGlobalPoints(); idx++)
        {
            std::vector<boost::shared_ptr<VesselSegment<DIM> > > empty_seg_pointers;
            mSegmentMap.push_back(empty_seg_pointers);
        }

        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
        unsigned num_points = mpRegularGrid->GetNumberOfGlobalPoints();
        units::quantity<unit::length> cut_off_length = sqrt(1.0 / 2.0) * mpRegularGrid->GetSpacing();
        for (unsigned jdx = 0; jdx < segments.size(); jdx++)
        {
            double start_point[3];
            double end_point[3];
            c_vector<double,DIM> start_location = segments[jdx]->GetNode(0)->rGetLocation().GetLocation(mReferenceLength);
            start_point[0] = start_location[0];
            start_point[1] = start_location[1];
            if(DIM==3)
            {
                start_point[2] = start_location[2];
            }
            else
            {
                start_point[2] = 0.0;
            }
            c_vector<double,DIM> end_location = segments[jdx]->GetNode(1)->rGetLocation().GetLocation(mReferenceLength);
            end_point[0] = end_location[0];
            end_point[1] = end_location[1];
            if(DIM==3)
            {
                end_point[2] = end_location[2];
            }
            else
            {
                end_point[2] = 0.0;
            }

            for (unsigned idx = 0; idx < num_points; idx++)
            {
                if (!useVesselSurface)
                {
                    double parametric_distance = 0.0;
                    double closest_point[3];
                    double grid_point[3];
                    c_vector<double,DIM> grid_location = mpRegularGrid->GetLocationOfGlobal1dIndex(idx).GetLocation(mReferenceLength);
                    grid_point[0] = grid_location[0];
                    grid_point[1] = grid_location[1];
                    if(DIM==3)
                    {
                        grid_point[2] = grid_location[2];
                    }
                    else
                    {
                        grid_point[2] = 0.0;
                    }

                    double distance = vtkLine::DistanceToLine(grid_point,
                                                              start_point,
                                                              end_point,
                                                              parametric_distance, closest_point);
                    if (distance*mReferenceLength < cut_off_length)
                    {
                        mSegmentMap[idx].push_back(segments[jdx]);
                    }
                }
                else
                {
                    if (segments[jdx]->GetDistance(mpRegularGrid->GetLocationOfGlobal1dIndex(idx))< (segments[jdx]->GetRadius() + cut_off_length))
                    {
                        mSegmentMap[idx].push_back(segments[jdx]);
                    }
                }
            }
        }

        return mSegmentMap;
    }
    else if(mHasUnstructuredGrid)
    {
        // Make sure the mesh vtk solution is up to date
        mpMesh->GetAsVtkUnstructuredGrid();

        mSegmentMap.clear();
        unsigned num_elements = mpMesh->GetNumElements();
        mSegmentMap = std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > >(num_elements);

        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
        for (unsigned jdx = 0; jdx < segments.size(); jdx++)
        {
            DimensionalChastePoint<DIM> loc1 = segments[jdx]->GetNode(0)->rGetLocation();
            DimensionalChastePoint<DIM> loc2 = segments[jdx]->GetNode(1)->rGetLocation();
            double x_coords1[3];
            x_coords1[0] = loc1.GetLocation(mReferenceLength)[0];
            x_coords1[1] = loc1.GetLocation(mReferenceLength)[1];
            if(DIM == 3)
            {
                x_coords1[2] = loc1.GetLocation(mReferenceLength)[2];
            }
            else
            {
                x_coords1[2] = 0.0;
            }
            double x_coords2[3];
            x_coords2[0] = loc2.GetLocation(mReferenceLength)[0];
            x_coords2[1] = loc2.GetLocation(mReferenceLength)[1];
            if(DIM == 3)
            {
                x_coords2[2] = loc2.GetLocation(mReferenceLength)[2];
            }
            else
            {
                x_coords2[2] = 0.0;
            }

            vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();
            mpMesh->GetVtkCellLocator()->FindCellsAlongLine(x_coords1, x_coords2, 1.e-8, p_id_list);
            unsigned num_intersections = p_id_list->GetNumberOfIds();
            for(unsigned idx=0; idx<num_intersections; idx++)
            {
                mSegmentMap[p_id_list->GetId(idx)].push_back(segments[jdx]);
            }
        }

        return mSegmentMap;
    }
    else
    {
        EXCEPTION("No grid has been set.");
    }
}

template<unsigned DIM>
void GridCalculator<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation, units::quantity<unit::length> cellPopulationLengthScale)
{
    mpCellPopulation = &rCellPopulation;
    mCellPopulationReferenceLength = cellPopulationLengthScale;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpRegularGrid = pGrid;
    mHasRegularGrid = true;
    mHasUnstructuredGrid = false;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetGrid(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pGrid)
{
    mpMesh = pGrid;
    mHasRegularGrid = false;
    mHasUnstructuredGrid = true;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

// Explicit instantiation
template class GridCalculator<2> ;
template class GridCalculator<3> ;
