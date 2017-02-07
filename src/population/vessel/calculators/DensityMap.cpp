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

#include "VesselSegment.hpp"
#include "DensityMap.hpp"
#include "GeometryTools.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
DensityMap<DIM>::DensityMap()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>(),
        mUseSurfaceBasedVesselDensity(false),
        mUseLineBasedVesselDensity(true),
        mUsePointBasedVesselDensity(false),
        mUseCellDensity(false),
        m2dProjectedDensity(),
        m1dProjectedDensity(),
        mUseBranchDensity(false),
        mUseTipDensity(false),
        mUsePerfusedDensity(false),
        mRegressingDensity(false)
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
std::vector<double> DensityMap<DIM>::Get2dProjectedDensity()
{
    return m2dProjectedDensity;
}

template<unsigned DIM>
std::vector<double> DensityMap<DIM>::Get1dProjectedDensity()
{
    return m1dProjectedDensity;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseLineBasedVesselDensity(bool useLineBased)
{
    mUseLineBasedVesselDensity = useLineBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseSurfaceBasedVesselDensity(bool useSurfaceBased)
{
    mUseSurfaceBasedVesselDensity = useSurfaceBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUsePointBasedVesselDensity(bool usePointBased)
{
    mUsePointBasedVesselDensity = usePointBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseCellDensity(bool useCellBased)
{
    mUseCellDensity = useCellBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseBranchDensity(bool useBranchBased)
{
    mUseBranchDensity = useBranchBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseTipDensity(bool useTipBased)
{
    mUseTipDensity = useTipBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUsePerfusedDensity(bool usePerfusedBased)
{
    mUsePerfusedDensity = usePerfusedBased;
}

template<unsigned DIM>
void DensityMap<DIM>::SetUseRegressingDensity(bool useRegressingBased)
{
    mRegressingDensity = useRegressingBased;
}

template<unsigned DIM>
void DensityMap<DIM>::Solve()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    unsigned number_of_points = this->mpRegularGrid->GetNumberOfPoints();
    unsigned extents_x = this->mpRegularGrid->GetExtents()[0];
    unsigned extents_y = this->mpRegularGrid->GetExtents()[1];
    unsigned extents_z = this->mpRegularGrid->GetExtents()[2];
    units::quantity<unit::length> spacing = this->mpRegularGrid->GetSpacing();
    double dimensionless_spacing = spacing/this->mpRegularGrid->GetReferenceLengthScale();

    std::vector<double> vessel_solution(number_of_points, 0.0);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments;
    if (this->mpNetwork)
    {
        segments = this->mpNetwork->GetVesselSegments();
        for (unsigned i = 0; i < extents_z; i++) // Z
        {
            for (unsigned j = 0; j < extents_y; j++) // Y
            {
                for (unsigned k = 0; k < extents_x; k++) // X
                {
                    unsigned grid_index = this->mpRegularGrid->Get1dGridIndex(k, j, i);
                    for (unsigned idx = 0; idx <  segments.size(); idx++)
                    {
                        vessel_solution[grid_index] += LengthOfLineInBox(segments[idx]->GetNode(0)->rGetLocation(),
                                                                         segments[idx]->GetNode(1)->rGetLocation(),
                                                                         this->mpRegularGrid->GetPointBoundingBox(k ,j, i),
                                                                         this->mpRegularGrid->GetReferenceLengthScale())/this->mpRegularGrid->GetReferenceLengthScale();
                    }
                    vessel_solution[grid_index] /= std::pow(dimensionless_spacing, 3);
                }
            }
        }
    }

    this->UpdateSolution(vessel_solution);

    if (this->mWriteSolution)
    {
        this->Write();
    }
}

// Explicit instantiation
template class DensityMap<2> ;
template class DensityMap<3> ;
