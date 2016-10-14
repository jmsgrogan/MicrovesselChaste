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
#include "LacunarityCalculator.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "VesselNetworkGraphCalculator.hpp"

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
template class LacunarityCalculator<3>;
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
template class LacunarityCalculator<2>;
template class VesselNetworkGeometryCalculator<2>;
template class VesselNetworkGraphCalculator<2>;
