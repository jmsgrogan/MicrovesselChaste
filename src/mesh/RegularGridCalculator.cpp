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
#include "RegularGridCalculator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
RegularGridCalculator<DIM>::RegularGridCalculator() :
        mpNetwork(),
        mpCellPopulation(),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mPointCellMap(),
        mPointNodeMap(),
        mPointSegmentMap(),
        mHasCellPopulation(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mpRegularGrid()
{

}

template<unsigned DIM>
boost::shared_ptr<RegularGridCalculator<DIM> > RegularGridCalculator<DIM>::Create()
{
    typedef RegularGridCalculator<DIM> Reg_Grid_Templated;
    MAKE_PTR(Reg_Grid_Templated, pSelf);
    return pSelf;
}

template<unsigned DIM>
RegularGridCalculator<DIM>::~RegularGridCalculator()
{

}

template<unsigned DIM>
boost::shared_ptr<RegularGrid<DIM> > RegularGridCalculator<DIM>::GetGrid()
{
    return mpRegularGrid;
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > RegularGridCalculator<DIM>::GetPointPointMap(
        std::vector<DimensionalChastePoint<DIM> > inputPoints)
{
    if (!mpRegularGrid)
    {
        EXCEPTION("A regular grid has not been set.");
    }

    double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();
    std::vector<std::vector<unsigned> > point_point_map(mpRegularGrid->GetNumberOfPoints());
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

        if (x_index <= mpRegularGrid->GetExtents()[0] && y_index <= mpRegularGrid->GetExtents()[1] && z_index <= mpRegularGrid->GetExtents()[2])
        {
            unsigned grid_index = x_index + y_index * mpRegularGrid->GetExtents()[0] +
                    z_index * mpRegularGrid->GetExtents()[0] * mpRegularGrid->GetExtents()[1];
            point_point_map[grid_index].push_back(idx);
        }
    }
    return point_point_map;
}

template<unsigned DIM>
const std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >& RegularGridCalculator<DIM>::GetPointNodeMap(bool update)
{
    if (!update)
    {
        return mPointNodeMap;
    }

    if (!mpRegularGrid)
    {
        EXCEPTION("A regular grid has not been set.");
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a point node map.");
    }

    // Loop over all nodes and associate cells with the points
    mPointNodeMap.clear();
    for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfPoints(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > empty_node_pointers;
        mPointNodeMap.push_back(empty_node_pointers);
    }

    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
    double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();

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

        if (x_index <= mpRegularGrid->GetExtents()[0] && y_index <= mpRegularGrid->GetExtents()[1] && z_index <= mpRegularGrid->GetExtents()[2])
        {
            unsigned grid_index = x_index + y_index * mpRegularGrid->GetExtents()[0] +
                    z_index * mpRegularGrid->GetExtents()[0] * mpRegularGrid->GetExtents()[1];
            if(grid_index<mPointNodeMap.size())
            {
                mPointNodeMap[grid_index].push_back(nodes[idx]);
            }
        }
    }
    return mPointNodeMap;
}

template<unsigned DIM>
const std::vector<std::vector<CellPtr> >& RegularGridCalculator<DIM>::GetPointCellMap(bool update)
{
    if (!update)
    {
        return mPointCellMap;
    }

    if (!mpRegularGrid)
    {
        EXCEPTION("A regular grid has not been set.");
    }

    if (!mpCellPopulation)
    {
        EXCEPTION("A cell population has not been set. Can not create a cell point map.");
    }

    // Loop over all cells and associate cells with the points
    double scale_factor = mReferenceLength / mpRegularGrid->GetSpacing();
    double cell_scale_factor = mCellPopulationReferenceLength / mReferenceLength;

    mPointCellMap.clear();
    for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfPoints(); idx++)
    {
        std::vector<CellPtr> empty_cell_pointers;
        mPointCellMap.push_back(empty_cell_pointers);
    }

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

        if (x_index <= mpRegularGrid->GetExtents()[0] && y_index <= mpRegularGrid->GetExtents()[1] && z_index <= mpRegularGrid->GetExtents()[2])
        {
            unsigned grid_index = x_index + y_index * mpRegularGrid->GetExtents()[0] +
                    z_index * mpRegularGrid->GetExtents()[0] * mpRegularGrid->GetExtents()[1];
            if(grid_index<mPointCellMap.size())
            {
                mPointCellMap[grid_index].push_back(*cell_iter);
            }
        }
    }

    return mPointCellMap;
}

template<unsigned DIM>
bool RegularGridCalculator<DIM>::IsSegmentAtLatticeSite(unsigned index, bool update)
{
    if(update)
    {
        GetPointSegmentMap(update);
    }

    if(index>=mPointSegmentMap.size())
    {
        EXCEPTION("Out of range index requested for point-segment map");
    }
    return mPointSegmentMap[index].size()>0;
}

template<unsigned DIM>
std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > RegularGridCalculator<DIM>::GetPointSegmentMap(
        bool update, bool useVesselSurface)
{
    if (!update)
    {
        return mPointSegmentMap;
    }

    if (!mpRegularGrid)
    {
        EXCEPTION("A regular grid has not been set.");
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a vessel point map.");
    }

    // Loop over all points and segments and associate segments with the points
    mPointSegmentMap.clear();
    for (unsigned idx = 0; idx < mpRegularGrid->GetNumberOfPoints(); idx++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > empty_seg_pointers;
        mPointSegmentMap.push_back(empty_seg_pointers);
    }

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    unsigned num_points = mpRegularGrid->GetNumberOfPoints();
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
                c_vector<double,DIM> grid_location = mpRegularGrid->GetLocationOf1dIndex(idx).GetLocation(mReferenceLength);
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
                    mPointSegmentMap[idx].push_back(segments[jdx]);
                }
            }
            else
            {
                if (segments[jdx]->GetDistance(mpRegularGrid->GetLocationOf1dIndex(idx))< (segments[jdx]->GetRadius() + cut_off_length))
                {
                    mPointSegmentMap[idx].push_back(segments[jdx]);
                }
            }
        }
    }

    return mPointSegmentMap;
}

template<unsigned DIM>
void RegularGridCalculator<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation, units::quantity<unit::length> cellPopulationLengthScale)
{
    mpCellPopulation = &rCellPopulation;
    mCellPopulationReferenceLength = cellPopulationLengthScale;
}

template<unsigned DIM>
void RegularGridCalculator<DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpRegularGrid = pGrid;
}

template<unsigned DIM>
void RegularGridCalculator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

// Explicit instantiation
template class RegularGridCalculator<2> ;
template class RegularGridCalculator<3> ;
