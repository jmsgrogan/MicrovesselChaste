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
#include "VesselSegment.hpp"
#include "DensityMap.hpp"
#include "GeometryTools.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
DensityMap<DIM>::DensityMap()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>(),
        mVesselSurfaceAreaDensity(),
        mVesselLineDensity(),
        mPerfusedVesselSurfaceAreaDensity(),
        mPerfusedVesselLineDensity(),
        mVesselTipDensity(),
        mVesselBranchDensity(),
        mVesselQuantityDensity(),
        mDimensionlessCellDensity(),
        mDimensionlessCellDensityByMutationType()
{

}

template<unsigned DIM>
boost::shared_ptr<DensityMap<DIM> > DensityMap<DIM>::Create()
{
    MAKE_PTR(DensityMap<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
DensityMap<DIM>::~DensityMap()
{

}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselSurfaceAreaDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mVesselSurfaceAreaDensity.size() == num_points)
    {
        return mVesselSurfaceAreaDensity;
    }
    else
    {
        if(!this->mpVtkSolution)
        {
            this->Setup();
        }

        mVesselSurfaceAreaDensity.clear();
        mVesselSurfaceAreaDensity = std::vector<double>(this->mpGridCalculator->GetGrid()->GetNumberOfLocations(), 0.0);
        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > segment_map = this->mpGridCalculator->rGetSegmentMap();
        units::quantity<unit::length> length_scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
        std::vector<double> grid_volumes = this->mpGridCalculator->GetGrid()->rGetLocationVolumes(true, true);

        // Get the local grid as VTK unstructured

        if (this->mpNetwork)
        {
            if(this->mpGridCalculator->HasStructuredGrid())
            {
                boost::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                        boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
                if(!p_regular_grid)
                {
                    EXCEPTION("Can't cast to regular grid");
                }

                c_vector<unsigned, 6> extents = p_regular_grid->GetExtents();
                for(unsigned idx=0; idx<segment_map.size();idx++)
                {
                    c_vector<double, 6> bbox = p_regular_grid->GetPointBoundingBox(idx, true);
                    for(unsigned jdx=0;jdx<segment_map[idx].size();jdx++)
                    {
                        units::quantity<unit::length> length = LengthOfLineInBox(segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
                                segment_map[idx][jdx]->GetNode(1)->rGetLocation(), bbox, length_scale);
                        units::quantity<unit::area> surface_area = length*2.0*M_PI*segment_map[idx][jdx]->GetVessel()->GetRadius();
                        mVesselSurfaceAreaDensity[idx] += (surface_area/(length_scale*length_scale));
                    }
                    mVesselSurfaceAreaDensity[idx] /= grid_volumes[idx];
                }
            }
            else
            {
                boost::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
                        boost::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
                if(!p_mesh)
                {
                    EXCEPTION("Can't cast to mesh");
                }

                for(unsigned idx=0; idx<segment_map.size();idx++)
                {
                    // Get the element nodal locations and element volume
                    Element<DIM, DIM>* p_element = p_mesh->GetElement(idx);
                    std::vector<DimensionalChastePoint<DIM> > element_vertices(4);
                    if(p_element->GetNumNodes() != 4)
                    {
                        EXCEPTION("Vessel mesh mapping only supported for linear tetrahedral elements.");
                    }
                    for (unsigned jdx = 0; jdx < 4; jdx++)
                    {
                        element_vertices[jdx] = DimensionalChastePoint<DIM>(p_element->GetNodeLocation(jdx), length_scale);
                    }
                    for (unsigned jdx = 0; jdx < segment_map[idx].size(); jdx++)
                    {
                        units::quantity<unit::length> length_in_box = LengthOfLineInTetra<DIM>(segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
                                segment_map[idx][jdx]->GetNode(1)->rGetLocation(), element_vertices);
                        units::quantity<unit::area> surface_area = 2.0*M_PI*segment_map[idx][jdx]->GetRadius()*length_in_box;
                        mVesselSurfaceAreaDensity[idx] += ((surface_area/(length_scale*length_scale))/grid_volumes[idx]);
                    }
                }
            }
        }
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselLineDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mVesselLineDensity.size() == num_points)
    {
        return mVesselLineDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetPerfusedVesselSurfaceAreaDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mPerfusedVesselSurfaceAreaDensity.size() == num_points)
    {
        return mPerfusedVesselSurfaceAreaDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetPerfusedVesselLineDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mPerfusedVesselLineDensity.size() == num_points)
    {
        return mPerfusedVesselLineDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselTipDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mVesselTipDensity.size() == num_points)
    {
        return mVesselTipDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselBranchDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mVesselBranchDensity.size() == num_points)
    {
        return mVesselBranchDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetVesselQuantityDensity(const std::string& rQuantity, bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mVesselQuantityDensity.size() == num_points)
    {
        return mVesselQuantityDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetCellDensity(bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mDimensionlessCellDensity.size() == num_points)
    {
        return mDimensionlessCellDensity;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
const std::vector<double>& DensityMap<DIM>::rGetCellDensity(boost::shared_ptr<AbstractCellMutationState> pMutationState, bool update)
{
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    if(!update and mDimensionlessCellDensityByMutationType.size() == num_points)
    {
        return mDimensionlessCellDensityByMutationType;
    }
    else
    {
        Solve();
        return mVesselLineDensity;
    }
}

template<unsigned DIM>
void DensityMap<DIM>::Solve()
{

}

// Explicit instantiation
template class DensityMap<2> ;
template class DensityMap<3> ;
