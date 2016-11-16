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

#include "VesselNetworkReader.hpp"
#include "NodeFlowProperties.hpp"
#include "SegmentFlowProperties.hpp"
#include "VesselFlowProperties.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "AbstractVesselNetworkComponent.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"
#include "VesselNetworkWriter.hpp"
#include "DensityMap.hpp"
#include "DistanceMap.hpp"
//#include "LacunarityCalculator.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "VesselNetworkGraphCalculator.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"
#include "AbstractVesselNetworkComponentFlowProperties.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties.hpp"

template class NodeFlowProperties<3>;
template class SegmentFlowProperties<3>;
template class VesselFlowProperties<3>;
template class VesselNode<3>;
template class VesselSegment<3>;
template class Vessel<3>;
template class VesselNetwork<3>;
template class VesselNetworkWriter<3>;
template class AbstractVesselNetworkComponent<3>;
template class VesselNetworkGenerator<3>;
template class VesselNetworkReader<3>;
template class VesselNetworkCellPopulationInteractor<3>;
template class DensityMap<3>;
template class DistanceMap<3>;
//template class LacunarityCalculator<3>;
template class VesselNetworkGeometryCalculator<3>;
template class VesselNetworkGraphCalculator<3>;
template class NodeFlowProperties<2>;
template class SegmentFlowProperties<2>;
template class VesselFlowProperties<2>;
template class VesselNode<2>;
template class VesselSegment<2>;
template class Vessel<2>;
template class VesselNetwork<2>;
template class VesselNetworkWriter<2>;
template class AbstractVesselNetworkComponent<2>;
template class VesselNetworkGenerator<2>;
template class VesselNetworkReader<2>;
template class VesselNetworkCellPopulationInteractor<2>;
template class DensityMap<2>;
template class DistanceMap<2>;
//template class LacunarityCalculator<2>;
template class VesselNetworkGeometryCalculator<2>;
template class VesselNetworkGraphCalculator<2>;
template class AbstractVesselNetworkComponentProperties<2>;
template class AbstractVesselNetworkComponentProperties<3>;
template class AbstractVesselNetworkComponentFlowProperties<2>;
template class AbstractVesselNetworkComponentFlowProperties<3>;
template class AbstractVesselNetworkComponentChemicalProperties<2>;
template class AbstractVesselNetworkComponentChemicalProperties<3>;

namespace pyplusplus{
namespace aliases{
typedef std::vector<DimensionalChastePoint<2> > VecDimendionsalChastePoint2;
typedef std::vector<DimensionalChastePoint<3> > VecDimendionsalChastePoint3;
typedef std::pair<DimensionalChastePoint<3>, DimensionalChastePoint<3> > PairDimendionsalChastePoint3DimendionsalChastePoint3;
typedef std::pair<DimensionalChastePoint<2>, DimensionalChastePoint<2> > PairDimendionsalChastePoint2DimendionsalChastePoint2;
}
}//pyplusplus::aliases


