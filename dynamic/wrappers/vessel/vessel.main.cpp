#include <pybind11/pybind11.h>
#include "VesselNetworkReader2.cppwg.hpp"
#include "VesselNetworkReader3.cppwg.hpp"
#include "NodeFlowProperties2.cppwg.hpp"
#include "NodeFlowProperties3.cppwg.hpp"
#include "SegmentFlowProperties2.cppwg.hpp"
#include "SegmentFlowProperties3.cppwg.hpp"
#include "VesselFlowProperties2.cppwg.hpp"
#include "VesselFlowProperties3.cppwg.hpp"
#include "VesselNode2.cppwg.hpp"
#include "VesselNode3.cppwg.hpp"
#include "VesselSegment2.cppwg.hpp"
#include "VesselSegment3.cppwg.hpp"
#include "Vessel2.cppwg.hpp"
#include "Vessel3.cppwg.hpp"
#include "VesselNetworkVtkConverter2.cppwg.hpp"
#include "VesselNetworkVtkConverter3.cppwg.hpp"
#include "VesselNetworkPropertyManager2.cppwg.hpp"
#include "VesselNetworkPropertyManager3.cppwg.hpp"
#include "VesselNetworkPartitioner2.cppwg.hpp"
#include "VesselNetworkPartitioner3.cppwg.hpp"
#include "VesselNetworkGenerator2.cppwg.hpp"
#include "VesselNetworkGenerator3.cppwg.hpp"
#include "AbstractVesselNetworkComponent2.cppwg.hpp"
#include "AbstractVesselNetworkComponent3.cppwg.hpp"
#include "VesselNetworkCellPopulationInteractor2.cppwg.hpp"
#include "VesselNetworkCellPopulationInteractor3.cppwg.hpp"
#include "VesselNetworkWriter2.cppwg.hpp"
#include "VesselNetworkWriter3.cppwg.hpp"
#include "DistanceMap2.cppwg.hpp"
#include "DistanceMap3.cppwg.hpp"
#include "VesselNetworkGeometryCalculator2.cppwg.hpp"
#include "VesselNetworkGeometryCalculator3.cppwg.hpp"
#include "VesselNetworkGraphCalculator2.cppwg.hpp"
#include "VesselNetworkGraphCalculator3.cppwg.hpp"
#include "AbstractVesselNetworkComponentProperties2.cppwg.hpp"
#include "AbstractVesselNetworkComponentProperties3.cppwg.hpp"
#include "AbstractVesselNetworkComponentFlowProperties2.cppwg.hpp"
#include "AbstractVesselNetworkComponentFlowProperties3.cppwg.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties2.cppwg.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_vessel, m)
{
    register_VesselNetworkReader2_class(m);
    register_VesselNetworkReader3_class(m);
    register_NodeFlowProperties2_class(m);
    register_NodeFlowProperties3_class(m);
    register_SegmentFlowProperties2_class(m);
    register_SegmentFlowProperties3_class(m);
    register_VesselFlowProperties2_class(m);
    register_VesselFlowProperties3_class(m);
    register_VesselNode2_class(m);
    register_VesselNode3_class(m);
    register_VesselSegment2_class(m);
    register_VesselSegment3_class(m);
    register_Vessel2_class(m);
    register_Vessel3_class(m);
    register_VesselNetworkVtkConverter2_class(m);
    register_VesselNetworkVtkConverter3_class(m);
    register_VesselNetworkPropertyManager2_class(m);
    register_VesselNetworkPropertyManager3_class(m);
    register_VesselNetworkPartitioner2_class(m);
    register_VesselNetworkPartitioner3_class(m);
    register_VesselNetworkGenerator2_class(m);
    register_VesselNetworkGenerator3_class(m);
    register_AbstractVesselNetworkComponent2_class(m);
    register_AbstractVesselNetworkComponent3_class(m);
    register_VesselNetworkCellPopulationInteractor2_class(m);
    register_VesselNetworkCellPopulationInteractor3_class(m);
    register_VesselNetworkWriter2_class(m);
    register_VesselNetworkWriter3_class(m);
    register_DistanceMap2_class(m);
    register_DistanceMap3_class(m);
    register_VesselNetworkGeometryCalculator2_class(m);
    register_VesselNetworkGeometryCalculator3_class(m);
    register_VesselNetworkGraphCalculator2_class(m);
    register_VesselNetworkGraphCalculator3_class(m);
    register_AbstractVesselNetworkComponentProperties2_class(m);
    register_AbstractVesselNetworkComponentProperties3_class(m);
    register_AbstractVesselNetworkComponentFlowProperties2_class(m);
    register_AbstractVesselNetworkComponentFlowProperties3_class(m);
    register_AbstractVesselNetworkComponentChemicalProperties2_class(m);
    register_AbstractVesselNetworkComponentChemicalProperties3_class(m);
}
