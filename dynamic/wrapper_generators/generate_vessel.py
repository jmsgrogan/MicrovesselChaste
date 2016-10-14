#!/usr/bin/env python

"""
This scipt automatically generates Python bindings using a rule based approach
"""
import sys
from pyplusplus import module_builder
from pyplusplus.module_builder import call_policies
from pygccxml import parser
import generate_bindings

def update_builder(builder):

    include_classes = ["NodeFlowProperties<3>", 
                       "SegmentFlowProperties<3>", 
                       "VesselFlowProperties<3>", 
                       "VesselNode<3>",
                       "VesselSegment<3>",
                       "Vessel<3>", 
                       "VesselNetwork<3>",
                       "VesselNetworkGenerator<3>",
                       "AbstractVesselNetworkComponent<3>",
                       "VesselNetworkWriter<3>",
                       "VesselNetworkReader<3>",
                       "VesselDistribution",
                       "SegmentLocation",
                       "VesselNetworkCellPopulationInteractor<3>",
                       "DensityMap<3>",
                       "DistanceMap<3>",
                       "LacunarityCalculator<3>",
                       "VesselNetworkGeometryCalculator<3>",
                       "VesselNetworkGraphCalculator<3>",
                       "NodeFlowProperties<2>",
                       "SegmentFlowProperties<2>",
                       "VesselFlowProperties<2>",
                       "VesselNode<2>",
                       "VesselSegment<2>",
                       "Vessel<2>",
                       "VesselNetwork<2>",
                       "VesselNetworkWriter<2>",
                       "AbstractVesselNetworkComponent<2>",
                       "VesselNetworkGenerator<2>",
                       "VesselNetworkReader<2>",
                       "VesselNetworkCellPopulationInteractor<2>",
                       "DensityMap<2>",
                       "DistanceMap<2>",
                       "LacunarityCalculator<2>",
                       "VesselNetworkGeometryCalculator<2>",
                       "VesselNetworkGraphCalculator<2>"]
    
    # Include each class and do template renaming
    for eachClass in include_classes:
        builder.class_(eachClass).include()  
        new_name = generate_bindings.template_replace(eachClass)
        if(new_name != eachClass):
            builder.class_(eachClass).rename(new_name) 
            
    # Default template args problem
#    builder.class_('VesselNetworkGenerator<3> ').calldefs().use_default_arguments=False    
    
    # The VesselSegment and Vessel classes need factory constructors as they have private constructor methods
    builder.add_declaration_code('boost::shared_ptr<VesselSegment<3> > (*VS3_Nodes)(boost::shared_ptr<VesselNode<3> >, boost::shared_ptr<VesselNode<3> >) = &VesselSegment<3>::Create;')
    builder.add_declaration_code('boost::shared_ptr<VesselSegment<3> > (*VS3_Copy)(boost::shared_ptr<VesselSegment<3> >) = &VesselSegment<3>::Create;')
    builder.add_declaration_code('boost::shared_ptr<Vessel<3> > (*V3_SingleSegment)(boost::shared_ptr<VesselSegment<3> >) = &Vessel<3>::Create;')
    builder.add_declaration_code('boost::shared_ptr<Vessel<3> > (*V3_MultiSegment)(std::vector<boost::shared_ptr<VesselSegment<3> > >) = &Vessel<3>::Create;')
    builder.add_declaration_code('boost::shared_ptr<Vessel<3> > (*V3_Nodes)(std::vector<boost::shared_ptr<VesselNode<3> > >) = &Vessel<3>::Create;')
    builder.class_('VesselSegment<3>').add_registration_code('def("__init__", bp::make_constructor(VS3_Nodes))')
    builder.class_('VesselSegment<3>').add_registration_code('def("__init__", bp::make_constructor(VS3_Copy))')
    builder.class_('Vessel<3>').add_registration_code('def("__init__", bp::make_constructor(V3_SingleSegment))')
    builder.class_('Vessel<3>').add_registration_code('def("__init__", bp::make_constructor(V3_MultiSegment))')
    builder.class_('Vessel<3>').add_registration_code('def("__init__", bp::make_constructor(V3_Nodes))')

    return builder