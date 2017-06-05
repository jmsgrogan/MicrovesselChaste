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

#include "SmartPointers.hpp"
#include "SegmentFlowProperties.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

template <unsigned DIM>
VesselNetworkPropertyManager<DIM>::VesselNetworkPropertyManager()
{

}

template <unsigned DIM>
VesselNetworkPropertyManager<DIM>::~VesselNetworkPropertyManager()
{

}

template <unsigned DIM>
boost::shared_ptr<VesselNetworkPropertyManager<DIM> > VesselNetworkPropertyManager<DIM>::Create()
{
    MAKE_PTR(VesselNetworkPropertyManager<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::AssignInflows(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        DimensionalChastePoint<DIM> location, units::quantity<unit::length> searchRadius)
{
    if(pNetwork->GetNodes().size()>0)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > inside_nodes = VesselNetworkGeometryCalculator<DIM>::GetNodesInSphere(
                pNetwork, location, searchRadius);
        for(unsigned idx=0;idx<inside_nodes.size();idx++)
        {
            inside_nodes[idx]->GetFlowProperties()->SetIsInputNode(true);
        }
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::AssignOutflows(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        DimensionalChastePoint<DIM> location, units::quantity<unit::length> searchRadius)
{
    if(pNetwork->GetNodes().size()>0)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > outside_nodes = VesselNetworkGeometryCalculator<DIM>::GetNodesInSphere(
                pNetwork, location, searchRadius);
        for(unsigned idx=0;idx<outside_nodes.size();idx++)
        {
            outside_nodes[idx]->GetFlowProperties()->SetIsOutputNode(true);
        }
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetInflowPressures(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        units::quantity<unit::pressure> pressure)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    for(unsigned idx=0; idx<nodes.size(); idx++)
    {
        if(nodes[idx]->GetFlowProperties()->IsInputNode())
        {
            nodes[idx]->GetFlowProperties()->SetPressure(pressure);
        }
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetOutflowPressures(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::pressure> pressure)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    for(unsigned idx=0; idx<nodes.size(); idx++)
    {
        if(nodes[idx]->GetFlowProperties()->IsOutputNode())
        {
            nodes[idx]->GetFlowProperties()->SetPressure(pressure);
        }
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::CopySegmentFlowProperties(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, unsigned index)
{
    boost::shared_ptr<SegmentFlowProperties<DIM> > properties = pNetwork->GetVesselSegments()[index]->GetFlowProperties();
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it;
    for(it = segments.begin(); it != segments.end(); it++)
    {
        (*it)->SetFlowProperties(*properties);
    }
}


template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetNodeRadiiFromSegments(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    for(unsigned idx=0; idx<nodes.size(); idx++)
    {
        units::quantity<unit::length> av_radius = 0.0 * unit::metres;
        for(unsigned jdx=0; jdx<nodes[idx]->GetNumberOfSegments(); jdx++)
        {
            av_radius += nodes[idx]->GetSegment(jdx)->GetRadius();
        }
        av_radius /= double(nodes[idx]->GetNumberOfSegments());
        nodes[idx]->SetRadius(av_radius);
    }
    pNetwork->Modified(false, false, false);
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetworkPropertyManager<DIM>::GetInflowNodes(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselNode<DIM> > > inflow_nodes;
    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        if(nodes[idx]->GetFlowProperties()->IsInputNode())
        {
            inflow_nodes.push_back(nodes[idx]);
        }
    }
    return inflow_nodes;
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetworkPropertyManager<DIM>::GetOutflowNodes(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselNode<DIM> > > outflow_nodes;
    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        if(nodes[idx]->GetFlowProperties()->IsOutputNode())
        {
            outflow_nodes.push_back(nodes[idx]);
        }
    }
    return outflow_nodes;
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetSegmentProperties(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        boost::shared_ptr<VesselSegment<DIM> >  prototype)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it;
    for(it = segments.begin(); it != segments.end(); it++)
    {
        (*it)->SetRadius(prototype->GetRadius());
        (*it)->GetFlowProperties()->SetImpedance(prototype->GetFlowProperties()->GetImpedance());
        (*it)->GetFlowProperties()->SetHaematocrit(prototype->GetFlowProperties()->GetHaematocrit());
        (*it)->GetFlowProperties()->SetFlowRate(prototype->GetFlowProperties()->GetFlowRate());
        (*it)->GetFlowProperties()->SetViscosity(prototype->GetFlowProperties()->GetViscosity());
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetNodeRadii(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        units::quantity<unit::length> radius)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();

    for(unsigned idx=0; idx<nodes.size();idx++)
    {
        nodes[idx]->SetRadius(radius);
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetSegmentRadii(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        units::quantity<unit::length> radius)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();

    for(unsigned idx=0; idx<segments.size();idx++)
    {
        segments[idx]->SetRadius(radius);
    }
}

template <unsigned DIM>
void VesselNetworkPropertyManager<DIM>::SetSegmentViscosity(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        units::quantity<unit::dynamic_viscosity> viscosity)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx]->GetFlowProperties()->SetViscosity(viscosity);
    }
}

// Explicit instantiation
template class VesselNetworkPropertyManager<2>;
template class VesselNetworkPropertyManager<3>;

