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

#include "ArtificialImageGenerator.hpp"
#include "VesselSegment.hpp"
#include "GeometryTools.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
DensityMap<DIM>::DensityMap()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>()
//        mUseSurfaceBasedVesselDensity(false),
//        mUseLineBasedVesselDensity(true),
//        mUsePointBasedVesselDensity(false),
//        mUseCellDensity(false),
//        m2dProjectedDensity(),
//        m1dProjectedDensity(),
//        mUseBranchDensity(false),
//        mUseTipDensity(false),
//        mUsePerfusedDensity(false),
//        mRegressingDensity(false)
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
//
//template<unsigned DIM>
//std::vector<double> DensityMap<DIM>::Get2dProjectedDensity()
//{
//    return m2dProjectedDensity;
//}
//
//template<unsigned DIM>
//std::vector<double> DensityMap<DIM>::Get1dProjectedDensity()
//{
//    return m1dProjectedDensity;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseLineBasedVesselDensity(bool useLineBased)
//{
//    mUseLineBasedVesselDensity = useLineBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseSurfaceBasedVesselDensity(bool useSurfaceBased)
//{
//    mUseSurfaceBasedVesselDensity = useSurfaceBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUsePointBasedVesselDensity(bool usePointBased)
//{
//    mUsePointBasedVesselDensity = usePointBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseCellDensity(bool useCellBased)
//{
//    mUseCellDensity = useCellBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseBranchDensity(bool useBranchBased)
//{
//    mUseBranchDensity = useBranchBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseTipDensity(bool useTipBased)
//{
//    mUseTipDensity = useTipBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUsePerfusedDensity(bool usePerfusedBased)
//{
//    mUsePerfusedDensity = usePerfusedBased;
//}
//
//template<unsigned DIM>
//void DensityMap<DIM>::SetUseRegressingDensity(bool useRegressingBased)
//{
//    mRegressingDensity = useRegressingBased;
//}

//template<unsigned DIM>
//void DensityMap<DIM>::Solve()
//{
//    if(!this->mpVtkSolution)
//    {
//        this->Setup();
//    }
//
//    c_vector<unsigned, 6> extents = this->mpGridCalculator->GetGrid()->GetExtents();
//    std::vector<double> vessel_solution(this->mpGridCalculator->GetGrid()->GetNumberOfLocalPoints(), 0.0);
//    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
//    units::quantity<unit::length> length_Scale = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
//
//    if (this->mpNetwork)
//    {
//        for (unsigned i = extents[4]; i < extents[5] + 1; i++) // Z
//        {
//            for (unsigned j = extents[2]; j < extents[3] + 1; j++) // Y
//            {
//                for (unsigned k = extents[0]; k < extents[1] + 1; k++) // X
//                {
//                    unsigned grid_index = this->mpGridCalculator->GetGrid()->GetLocal1dGridIndex(k, j, i);
//                    c_vector<double, 6> bbox = this->mpGridCalculator->GetGrid()->GetPointBoundingBox(k ,j, i, true);
//
//                    for (unsigned idx = 0; idx <  segments.size(); idx++)
//                    {
//                        vessel_solution[grid_index] += LengthOfLineInBox(segments[idx]->GetNode(0)->rGetLocation(),
//                                                                         segments[idx]->GetNode(1)->rGetLocation(),
//                                                                         bbox,length_Scale)/length_Scale;
//                    }
//                    double box_volume = (bbox[1]-bbox[0])*(bbox[3]-bbox[2]);
//                    if(DIM==3)
//                    {
//                        box_volume*=(bbox[5]-bbox[4]);
//                    }
//                    vessel_solution[grid_index] /= box_volume;
//                }
//            }
//        }
//    }
//
//    this->UpdateSolution(vessel_solution);
//
//    if (this->mWriteSolution)
//    {
//        this->Write();
//    }
//}

// Explicit instantiation
template class DensityMap<2> ;
template class DensityMap<3> ;
