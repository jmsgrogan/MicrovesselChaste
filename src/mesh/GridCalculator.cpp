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
#include <vtkCellLocator.h>
#include <vtkLine.h>
#include "PetscTools.hpp"
#include "Exception.hpp"
#include "GridCalculator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
GridCalculator<DIM>::GridCalculator() :
        mpNetwork(),
        mpCellPopulation(),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mCellMap(),
        mVesselNodeMap(),
        mSegmentMap(),
        mpGrid(),
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
boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > GridCalculator<DIM>::GetGrid()
{
    if(!mpGrid)
    {
        EXCEPTION("A grid has been requested, but one has not been set.");
    }
    return mpGrid;
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > GridCalculator<DIM>::GetPointMap(const std::vector<DimensionalChastePoint<DIM> >& rInputPoints)
{
    std::vector<std::vector<unsigned> > point_map(mpGrid->GetNumberOfLocations());
    units::quantity<unit::length> grid_length = mpGrid->GetReferenceLengthScale();

    for(unsigned idx=0;idx<rInputPoints.size();idx++)
    {
        double x_coords[3];
        c_vector<double, DIM> loc = rInputPoints[idx].GetLocation(grid_length);
        x_coords[0] = loc[0];
        x_coords[1] = loc[1];
        if(DIM == 3)
        {
            x_coords[2] = loc[2];
        }
        else
        {
            x_coords[2] = 0.0;
        }

        int cell_id = mpGrid->GetVtkCellLocator()->FindCell(&x_coords[0]);
        if(cell_id>=0)
        {
            point_map[cell_id].push_back(idx);
        }
    }
    return point_map;
}

template<unsigned DIM>
const std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >& GridCalculator<DIM>::rGetVesselNodeMap(bool update)
{
    if (!update)
    {
        return mVesselNodeMap;
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a point node map.");
    }

    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
    units::quantity<unit::length> grid_length = mpGrid->GetReferenceLengthScale();
    mVesselNodeMap.clear();
    mVesselNodeMap = std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >(mpGrid->GetNumberOfLocations());

    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        double x_coords[3];
        c_vector<double, DIM> loc = nodes[idx]->rGetLocation().GetLocation(grid_length);
        x_coords[0] = loc[0];
        x_coords[1] = loc[1];
        if(DIM == 3)
        {
            x_coords[2] = loc[2];
        }
        else
        {
            x_coords[2] = 0.0;
        }

        int cell_id = mpGrid->GetVtkCellLocator()->FindCell(&x_coords[0]);
        if(cell_id>=0)
        {
            mVesselNodeMap[cell_id].push_back(nodes[idx]);
        }
    }
    return mVesselNodeMap;
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
    mCellMap = std::vector<std::vector<CellPtr> >(mpGrid->GetNumberOfLocations());

    units::quantity<unit::length> grid_length = mpGrid->GetReferenceLengthScale();
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
    if(update)
    {
        rGetSegmentMap(update);
    }

    return mSegmentMap[index].size()>0;
}

template<unsigned DIM>
const std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > >& GridCalculator<DIM>::rGetSegmentMap(
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
    mSegmentMap = std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > >(mpGrid->GetNumberOfLocations());
    units::quantity<unit::length> grid_length = mpGrid->GetReferenceLengthScale();

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    for (unsigned jdx = 0; jdx < segments.size(); jdx++)
    {
        c_vector<double, DIM> loc1 = segments[jdx]->GetNode(0)->rGetLocation().GetLocation(grid_length);
        c_vector<double, DIM> loc2 = segments[jdx]->GetNode(1)->rGetLocation().GetLocation(grid_length);
        double x_coords1[3];
        x_coords1[0] = loc1[0];
        x_coords1[1] = loc1[1];
        if(DIM == 3)
        {
            x_coords1[2] = loc1[2];
        }
        else
        {
            x_coords1[2] = 0.0;
        }
        double x_coords2[3];
        x_coords2[0] = loc2[0];
        x_coords2[1] = loc2[1];
        if(DIM == 3)
        {
            x_coords2[2] = loc2[2];
        }
        else
        {
            x_coords2[2] = 0.0;
        }

        vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();

        if(useVesselSurface)
        {
            double dimensionless_radius = segments[jdx]->GetRadius()/grid_length;
            mpGrid->GetVtkCellLocator()->FindCellsAlongLine(&x_coords1[0], &x_coords2[0], dimensionless_radius, p_id_list);
        }
        else
        {
            mpGrid->GetVtkCellLocator()->FindCellsAlongLine(&x_coords1[0], &x_coords2[0], 1.e-8, p_id_list);
        }

        unsigned num_intersections = p_id_list->GetNumberOfIds();
        for(unsigned idx=0; idx<num_intersections; idx++)
        {
            mSegmentMap[p_id_list->GetId(idx)].push_back(segments[jdx]);
        }
    }
    return mSegmentMap;
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
    mpGrid = pGrid;
    mHasRegularGrid = true;
    mHasUnstructuredGrid = false;
}

template<unsigned DIM>
void GridCalculator<DIM>::SetGrid(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pGrid)
{
    mpGrid = pGrid;
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
