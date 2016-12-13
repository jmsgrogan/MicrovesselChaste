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

#include <iostream>
#include <math.h>
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "SegmentFlowProperties.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkWriter.hpp"

template <unsigned DIM>
VesselNetwork<DIM>::VesselNetwork() : AbstractVesselNetworkComponent<DIM>(),
  mVessels(),
  mSegments(),
  mSegmentsUpToDate(false),
  mNodes(),
  mNodesUpToDate(false),
  mVesselNodes(),
  mVesselNodesUpToDate(false)
{

}

template <unsigned DIM>
VesselNetwork<DIM>::~VesselNetwork()
{

}

template <unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetwork<DIM>::Create()
{
    MAKE_PTR(VesselNetwork<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
void VesselNetwork<DIM>::AddVessel(boost::shared_ptr<Vessel<DIM> > pVessel)
{
    mVessels.push_back(pVessel);
    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
}

template <unsigned DIM>
void VesselNetwork<DIM>::AddVessels(std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    mVessels.insert(mVessels.end(), vessels.begin(), vessels.end());
    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
}

template <unsigned DIM>
void VesselNetwork<DIM>::CopySegmentFlowProperties(unsigned index)
{
    boost::shared_ptr<SegmentFlowProperties<DIM> > properties = GetVesselSegments()[index]->GetFlowProperties();
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it;
    for(it = segments.begin(); it != segments.end(); it++)
    {
        (*it)->SetFlowProperties(*properties);
    }
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::CopyVessels()
{
    return CopyVessels(mVessels);
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::CopyVessels(std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator vessel_iter;
    std::vector<boost::shared_ptr<Vessel<DIM> > > new_vessels;
    for(vessel_iter = vessels.begin(); vessel_iter != vessels.end(); vessel_iter++)
    {
        typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > new_segments;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = (*vessel_iter)->GetSegments();
        for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
        {
            new_segments.push_back(VesselSegment<DIM>::Create(VesselNode<DIM>::Create((*segment_iter)->GetNode(0)->rGetLocation()),
                                                              VesselNode<DIM>::Create((*segment_iter)->GetNode(1)->rGetLocation())));
        }
        new_vessels.push_back(Vessel<DIM>::Create(new_segments));
    }

    MergeCoincidentNodes(new_vessels);
    AddVessels(new_vessels);
    return new_vessels;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::DivideVessel(boost::shared_ptr<Vessel<DIM> > pVessel,
                                                                     const DimensionalChastePoint<DIM>& location)
{
    boost::shared_ptr<VesselSegment<DIM> > p_segment;

    // If the divide location coincides with one of the end nodes don't divide and return that node
    if (pVessel->GetStartNode()->IsCoincident(location) || pVessel->GetEndNode()->IsCoincident(location))
    {

        if (pVessel->GetStartNode()->IsCoincident(location))
        {
            return pVessel->GetStartNode();
        }
        else
        {
            return pVessel->GetEndNode();
        }
    }
    else
    {
        bool locatedInsideVessel = false;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pVessel->GetSegments();
        for (unsigned idx = 0; idx < segments.size(); idx++)
        {
            if (segments[idx]->GetDistance(location)/segments[idx]->GetNode(0)->GetReferenceLengthScale() <= 1e-6)
            {
                locatedInsideVessel = true;
                p_segment = segments[idx];
                break;
            }
        }
        if(!locatedInsideVessel)
        {
            EXCEPTION("There is no segment at the requested division location.");
        }
    }

    boost::shared_ptr<VesselNode<DIM> > p_new_node = pVessel->DivideSegment(location); // network segments and nodes out of date

    // create two new vessels and assign them the old vessel's properties
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > start_segments;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > end_segments;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pVessel->GetSegments();
    unsigned segment_index = segments.size()+1;
    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        start_segments.push_back(segments[idx]);
        if (segments[idx]->GetNode(1)->IsCoincident(location))
        {
            segment_index = idx;
            break;
        }
    }

    if (segment_index == segments.size()-1)
    {
        EXCEPTION("Vessel segment not found.");
    }
    for (unsigned idx = segment_index+1; idx < segments.size(); idx++)
    {
        end_segments.push_back(segments[idx]);
    }

    boost::shared_ptr<Vessel<DIM> > p_new_vessel1 = Vessel<DIM>::Create(start_segments);
    boost::shared_ptr<Vessel<DIM> > p_new_vessel2 = Vessel<DIM>::Create(end_segments);
    p_new_vessel1->CopyDataFromExistingVessel(pVessel);
    p_new_vessel2->CopyDataFromExistingVessel(pVessel);
    p_new_vessel1->GetFlowProperties()->SetRegressionTime(pVessel->GetFlowProperties()->GetRegressionTime());
    p_new_vessel2->GetFlowProperties()->SetRegressionTime(pVessel->GetFlowProperties()->GetRegressionTime());

    AddVessel(p_new_vessel1);
    AddVessel(p_new_vessel2);
    RemoveVessel(pVessel, false);
    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;

    return p_new_node;
}

template <unsigned DIM>
void VesselNetwork<DIM>::ExtendVessel(boost::shared_ptr<Vessel<DIM> > pVessel, boost::shared_ptr<VesselNode<DIM> > pEndNode,
                                        boost::shared_ptr<VesselNode<DIM> > pNewNode)
{
    if(pVessel->GetStartNode() == pEndNode)
    {
        boost::shared_ptr<VesselSegment<DIM> > p_segment = VesselSegment<DIM>::Create(pNewNode, pEndNode);
        p_segment->SetFlowProperties(*(pEndNode->GetSegments()[0]->GetFlowProperties()));
        p_segment->SetRadius(pEndNode->GetSegments()[0]->GetRadius());
        p_segment->SetMaturity(0.0);
        pVessel->AddSegment(p_segment);
    }
    else
    {
        boost::shared_ptr<VesselSegment<DIM> > p_segment = VesselSegment<DIM>::Create(pEndNode, pNewNode);
        p_segment->SetFlowProperties(*(pEndNode->GetSegments()[0]->GetFlowProperties()));
        p_segment->SetRadius(pEndNode->GetSegments()[0]->GetRadius());
        p_segment->SetMaturity(0.0);
        pVessel->AddSegment(p_segment);
    }

    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetwork<DIM>::FormSprout(const DimensionalChastePoint<DIM>& sproutBaseLocation,
                                                                 const DimensionalChastePoint<DIM>& sproutTipLocation)
{
    // locate vessel at which the location of the sprout base exists
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > nearest_segment = GetNearestSegment(sproutBaseLocation);
    if (nearest_segment.second / nearest_segment.first->GetNode(0)->GetReferenceLengthScale()  > 1e-6)
    {
        EXCEPTION("No vessel located at sprout base.");
    }

    // divide vessel at location of sprout base
    boost::shared_ptr<VesselNode<DIM> > p_new_node = DivideVessel(nearest_segment.first->GetVessel(), sproutBaseLocation);

    // create new vessel
    boost::shared_ptr<VesselNode<DIM> > p_new_node_at_tip = VesselNode<DIM>::Create(p_new_node);
    p_new_node_at_tip->SetLocation(sproutTipLocation);
    p_new_node_at_tip->SetIsMigrating(true);
    p_new_node_at_tip->GetFlowProperties()->SetIsInputNode(false);
    p_new_node_at_tip->GetFlowProperties()->SetIsOutputNode(false);
    boost::shared_ptr<VesselSegment<DIM> > p_new_segment = VesselSegment<DIM>::Create(p_new_node, p_new_node_at_tip);
    p_new_segment->CopyDataFromExistingSegment(nearest_segment.first);
    p_new_segment->GetFlowProperties()->SetFlowRate(0.0*unit::metre_cubed_per_second);
    p_new_segment->GetFlowProperties()->SetImpedance(0.0*unit::pascal_second_per_metre_cubed);
    p_new_segment->GetFlowProperties()->SetHaematocrit(0.0);
    p_new_segment->GetFlowProperties()->SetGrowthStimulus(0.0*unit::per_second);
    p_new_segment->SetMaturity(0.0);

    boost::shared_ptr<Vessel<DIM> > p_new_vessel = Vessel<DIM>::Create(p_new_segment);
    // Sprouting won't save you.
    p_new_vessel->GetFlowProperties()->SetRegressionTime(nearest_segment.first->GetVessel()->GetFlowProperties()->GetRegressionTime());
    AddVessel(p_new_vessel);
    return p_new_vessel;
}

template <unsigned DIM>
void VesselNetwork<DIM>::SetNodeRadiiFromSegments()
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
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
}

template <unsigned DIM>
std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > VesselNetwork<DIM>::GetExtents(bool useRadii)
{
    units::quantity<unit::length> x_max = -DBL_MAX*unit::metres;
    units::quantity<unit::length> y_max = -DBL_MAX*unit::metres;
    units::quantity<unit::length> z_max = -DBL_MAX*unit::metres;

    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it;
    for(it = nodes.begin(); it != nodes.end(); it++)
    {
        units::quantity<unit::length> length_scale = (*it)->rGetLocation().GetReferenceLengthScale();
        c_vector<double, DIM> location = (*it)->rGetLocation().GetLocation(length_scale);
        if(location[0]*length_scale > x_max)
        {
            x_max = location[0]*length_scale;
            if(useRadii)
            {
                x_max += (*it)->GetRadius();
            }
        }
        if(location[1]*length_scale > y_max)
        {
            y_max = location[1]*length_scale;
            if(useRadii)
            {
                y_max += (*it)->GetRadius();
            }
        }
        if(DIM > 2)
        {
            if(location[2]*length_scale > z_max)
            {
                z_max = location[2]*length_scale;
                if(useRadii)
                {
                    z_max += (*it)->GetRadius();
                }
            }
        }
    }

    units::quantity<unit::length> x_min = x_max;
    units::quantity<unit::length> y_min = y_max;
    units::quantity<unit::length> z_min = z_max;
    for(it = nodes.begin(); it != nodes.end(); it++)
    {
        units::quantity<unit::length> length_scale = (*it)->rGetLocation().GetReferenceLengthScale();
        c_vector<double, DIM> location = (*it)->rGetLocation().GetLocation(length_scale);
        if(location[0]*length_scale < x_min)
        {
            x_min = location[0]*length_scale;
            if(useRadii)
            {
                x_min -= (*it)->GetRadius();
            }
        }
        if(location[1]*length_scale < y_min)
        {
            y_min = location[1]*length_scale;
            if(useRadii)
            {
                y_min -= (*it)->GetRadius();
            }
        }
        if(DIM > 2)
        {
            if(location[2]*length_scale < z_min)
            {
                z_min = location[2]*length_scale;
                if(useRadii)
                {
                    z_min -= (*it)->GetRadius();
                }
            }
        }
    }

    units::quantity<unit::length> base_length = BaseUnits::Instance()->GetReferenceLengthScale();
    std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > bbox(DimensionalChastePoint<DIM>(x_min/base_length, y_min/base_length, z_min/base_length, base_length),
                                                                              DimensionalChastePoint<DIM>(x_max/base_length, y_max/base_length, z_max/base_length, base_length));
    return bbox;
}

template<unsigned DIM>
std::map<std::string, double> VesselNetwork<DIM>::GetOutputData()
{
    this->mOutputData.clear();
    return this->mOutputData;
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetwork<DIM>::GetDistanceToNearestNode(const DimensionalChastePoint<DIM>& rLocation)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(rLocation);
        if (node_distance < min_distance)
        {
            min_distance = node_distance;
            nearest_node = (*node_iter) ;
        }
    }
    return min_distance;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::GetNearestNode(boost::shared_ptr<VesselNode<DIM> > pInputNode)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        if((*node_iter) != pInputNode)
        {
            units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(pInputNode->rGetLocation());
            if (node_distance < min_distance)
            {
                min_distance = node_distance;
                nearest_node = (*node_iter) ;
            }
        }
    }
    return nearest_node;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::GetNearestNode(const DimensionalChastePoint<DIM>& location)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(location);
        if (node_distance < min_distance)
        {
            min_distance = node_distance;
            nearest_node = (*node_iter) ;
        }
    }

    return nearest_node;
}

template <unsigned DIM>
std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > VesselNetwork<DIM>::GetNearestSegment(boost::shared_ptr<VesselSegment<DIM> > pSegment)
{
    boost::shared_ptr<VesselSegment<DIM> > nearest_segment;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();
    units::quantity<unit::length> length_scale = segments[0]->GetNode(0)->GetReferenceLengthScale();

    double min_distance = DBL_MAX;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
    for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
    {
        if(!pSegment->IsConnectedTo((*segment_iter)))
        {
            // Get the segment to segment distance (http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment())
            c_vector<double, DIM> u = (*segment_iter)->GetNode(1)->rGetLocation().GetLocation(length_scale) -
                    (*segment_iter)->GetNode(0)->rGetLocation().GetLocation(length_scale);
            c_vector<double, DIM> v = pSegment->GetNode(1)->rGetLocation().GetLocation(length_scale) -
                    pSegment->GetNode(0)->rGetLocation().GetLocation(length_scale);
            c_vector<double, DIM> w = (*segment_iter)->GetNode(0)->rGetLocation().GetLocation(length_scale) -
                    pSegment->GetNode(0)->rGetLocation().GetLocation(length_scale);

            double a = inner_prod(u,u);
            double b = inner_prod(u,v);
            double c = inner_prod(v,v);
            double d = inner_prod(u,w);
            double e = inner_prod(v,w);

            double dv = a * c - b * b;
            double sc, sn, sd = dv;
            double tc, tn ,td = dv;

            if(dv < 1.e-12) // almost parallel segments
            {
                sn = 0.0;
                sd = 1.0;
                tn = e;
                td = c;
            }
            else // get the closest point on the equivalent infinite lines
            {
                sn = (b*e - c*d);
                tn = (a*e - b*d);
                if ( sn < 0.0)
                {
                    sn = 0.0;
                    tn = e;
                    td = c;
                }
                else if(sn > sd)
                {
                    sn =sd;
                    tn = e+ b;
                    td = c;
                }
            }

            if(tn < 0.0)
            {
                tn = 0.0;
                if(-d < 0.0)
                {
                    sn = 0.0;
                }
                else if(-d > a)
                {
                    sn = sd;
                }
                else
                {
                    sn = -d;
                    sd = a;
                }
            }
            else if(tn > td)
            {
                tn = td;
                if((-d + b) < 0.0)
                {
                    sn = 0.0;
                }
                else if((-d + b) > a)
                {
                    sn = sd;
                }
                else
                {
                    sn = (-d + b);
                    sd = a;
                }
            }

            sc = (std::abs(sn) < 1.e-12 ? 0.0 : sn/sd);
            tc = (std::abs(tn) < 1.e-12 ? 0.0 : tn/td);
            c_vector<double, DIM> dp = w + (sc * u) - (tc * v);

            double segment_distance = norm_2(dp);
            if (segment_distance < min_distance)
            {
                min_distance = segment_distance;
                nearest_segment = (*segment_iter) ;
            }
        }

    }
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > return_pair =
            std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >(nearest_segment, min_distance * length_scale);
    return return_pair;
}

template <unsigned DIM>
std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > VesselNetwork<DIM>::GetNearestSegment(boost::shared_ptr<VesselNode<DIM> > pNode,
                                                                                                                          bool sameVessel)
{
    boost::shared_ptr<VesselSegment<DIM> > nearest_segment;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();

    units::quantity<unit::length>  min_distance = DBL_MAX * unit::metres;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
    for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
    {
        units::quantity<unit::length> segment_distance = (*segment_iter)->GetDistance(pNode->rGetLocation());

        if (segment_distance < min_distance && (*segment_iter)->GetNode(0) != pNode && (*segment_iter)->GetNode(1) != pNode)
        {
            if(sameVessel)
            {
                min_distance = segment_distance;
                nearest_segment = (*segment_iter) ;
            }
            else
            {
                bool same_vessel = false;
                std::vector<boost::shared_ptr<VesselSegment<DIM> > > node_segs = pNode->GetSegments();
                for(unsigned idx=0;idx<node_segs.size();idx++)
                {
                    if(node_segs[idx]->GetVessel() == (*segment_iter)->GetVessel())
                    {
                        same_vessel = true;
                    }
                }
                if(!same_vessel)
                {
                    min_distance = segment_distance;
                    nearest_segment = (*segment_iter);
                }
            }
        }
    }
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > return_pair =
            std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >(nearest_segment, min_distance);
    return return_pair;
}

template <unsigned DIM>
std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >  VesselNetwork<DIM>::GetNearestSegment(const DimensionalChastePoint<DIM>& location)
{
    boost::shared_ptr<VesselSegment<DIM> > nearest_segment;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();

    units::quantity<unit::length>  min_distance = DBL_MAX * unit::metres;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
    for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
    {
        units::quantity<unit::length>  segment_distance = (*segment_iter)->GetDistance(location);
        if (segment_distance < min_distance)
        {
            min_distance = segment_distance;
            nearest_segment = (*segment_iter) ;
        }
    }
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > return_pair =
            std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >(nearest_segment, min_distance);
    return return_pair;
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetwork<DIM>::GetNearestVessel(const DimensionalChastePoint<DIM>& location)
{
    return GetNearestSegment(location).first->GetVessel();
}

template <unsigned DIM>
void VesselNetwork<DIM>::RemoveShortVessels(units::quantity<unit::length> cutoff, bool endsOnly)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels_to_remove;

    for(unsigned idx=0; idx<mVessels.size(); idx++)
    {
        if(mVessels[idx]->GetLength() < cutoff)
        {
            if(endsOnly && (mVessels[idx]->GetStartNode()->GetNumberOfSegments() == 1 || mVessels[idx]->GetEndNode()->GetNumberOfSegments() == 1 ))
            {
                vessels_to_remove.push_back(mVessels[idx]);
            }
            else if(!endsOnly)
            {
                vessels_to_remove.push_back(mVessels[idx]);
            }
        }
    }

    for(unsigned idx=0; idx<vessels_to_remove.size(); idx++)
    {
        RemoveVessel(vessels_to_remove[idx]);
    }
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeShortVessels(units::quantity<unit::length> cutoff)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels_to_merge;
    for(unsigned idx=0; idx<mVessels.size(); idx++)
    {
        if(mVessels[idx]->GetLength() < cutoff)
        {
            vessels_to_merge.push_back(mVessels[idx]);
        }
    }

    // Get the nodes, remove the vessel, move the nodes together
    for(unsigned idx=0; idx<vessels_to_merge.size(); idx++)
    {
        vessels_to_merge[idx]->GetEndNode()->SetLocation(vessels_to_merge[idx]->GetStartNode()->rGetLocation());
        RemoveVessel(vessels_to_merge[idx], true);
    }

    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
    MergeCoincidentNodes();
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::NumberOfNodesNearLocation(const DimensionalChastePoint<DIM>& rLocation, double tolerance)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    unsigned num_nodes = 0;

    for(unsigned idx = 0; idx < nodes.size(); idx++)
    {
        if(nodes[idx]->GetDistance(rLocation)/nodes[idx]->GetReferenceLengthScale() <= tolerance + 1.e-6)
        {
            num_nodes++;
        }
    }
    return num_nodes;
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetwork<DIM>::GetNodes()
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }

    return mNodes;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::GetNode(unsigned index)
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }

    return mNodes[index];
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfNodes()
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }

    return mNodes.size();
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfVesselNodes()
{
    if(!mVesselNodesUpToDate)
    {
        UpdateVesselNodes();
    }

    return mVesselNodes.size();
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNodeIndex(boost::shared_ptr<VesselNode<DIM> > node)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > vec_nodes = GetNodes();

    unsigned index = 0;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it;
    for(it = vec_nodes.begin(); it != vec_nodes.end(); it++)
    {
        if(*it == node)
        {
            return index;
        }
        index ++;
    }
    EXCEPTION("Node is not in the network.");

}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfVessels()
{
    return mVessels.size();
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetwork<DIM>::GetVesselEndNodes()
{
    if(!mVesselNodesUpToDate)
    {
        UpdateVesselNodes();
    }
    return mVesselNodes;
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetwork<DIM>::GetVessel(unsigned index)
{
    if(index  >= mVessels.size())
    {
        EXCEPTION("Requested vessel index out of range");
    }

    return mVessels[index];
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::GetVessels()
{
    return mVessels;
}


template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetMaxBranchesOnNode()
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetVesselEndNodes();
    unsigned num_nodes = nodes.size();

    // Get maximum number of segments attached to a node in the whole network.
    unsigned max_num_branches = 0;
    for(unsigned node_index = 0; node_index < num_nodes; node_index++)
    {
        boost::shared_ptr<VesselNode<DIM> > p_each_node = nodes[node_index];
        unsigned num_segments_on_node = nodes[node_index]->GetNumberOfSegments();

        if (num_segments_on_node > max_num_branches)
        {
            max_num_branches = num_segments_on_node;
        }
    }
    return max_num_branches;
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetVesselIndex(boost::shared_ptr<Vessel<DIM> > pVessel)
{
    unsigned index = 0;
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        if(*it == pVessel)
        {
            return index;
        }
        index ++;
    }
    EXCEPTION("Input vessel is not in the network.");
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetVesselSegmentIndex(boost::shared_ptr<VesselSegment<DIM> > pVesselSegment)
{
    unsigned index = 0;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it;
    for(it = segments.begin(); it != segments.end(); it++)
    {
        if(*it == pVesselSegment)
        {
            return index;
        }
        index++;
    }
    EXCEPTION("Input vessel is not in the network.");
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselSegment<DIM> > > VesselNetwork<DIM>::GetVesselSegments()
{
    if(!mSegmentsUpToDate)
    {
        UpdateSegments();
    }

    return mSegments;
}

template <unsigned DIM>
bool VesselNetwork<DIM>::NodeIsInNetwork(boost::shared_ptr<VesselNode<DIM> > pSourceNode)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    return (std::find(nodes.begin(), nodes.end(), pSourceNode) != nodes.end());
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(double tolerance)
{
    MergeCoincidentNodes(GetNodes(), tolerance);
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(std::vector<boost::shared_ptr<Vessel<DIM> > > pVessels, double tolerance)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
    for(unsigned idx = 0; idx <pVessels.size(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > vessel_nodes = pVessels[idx]->GetNodes();
        nodes.insert(nodes.end(), vessel_nodes.begin(), vessel_nodes.end());
    }
    MergeCoincidentNodes(nodes, tolerance);
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes, double tolerance)
{
    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it;
    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it2;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it3;

    for(it = nodes.begin(); it != nodes.end(); it++)
    {
        for(it2 = nodes.begin(); it2 != nodes.end(); it2++)
        {
            // If the nodes are not identical
            if ((*it) != (*it2))
            {
                // If the node locations are the same - according to the ChastePoint definition
                bool is_coincident = false;
                if(tolerance >0.0)
                {
                    is_coincident = (*it)->GetDistance((*it2)->rGetLocation())/(*it)->GetReferenceLengthScale() <= tolerance;
                }
                else
                {
                    is_coincident = (*it)->IsCoincident((*it2)->rGetLocation());
                }

                if(is_coincident)
                {
                    // Replace the node corresponding to 'it2' with the one corresponding to 'it'
                    // in all segments.
                    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = (*it2)->GetSegments();
                    for(it3 = segments.begin(); it3 != segments.end(); it3++)
                    {
                        if ((*it3)->GetNode(0) == (*it2))
                        {
                            (*it3)->ReplaceNode(0, (*it));
                        }
                        else if(((*it3)->GetNode(1) == (*it2)))
                        {
                            (*it3)->ReplaceNode(1, (*it));
                        }
                    }
                }
            }
        }
    }
    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
}

template <unsigned DIM>
void VesselNetwork<DIM>::SetSegmentProperties(boost::shared_ptr<VesselSegment<DIM> >  prototype)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();

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
void VesselNetwork<DIM>::Translate(DimensionalChastePoint<DIM> rTranslationVector)
{
    Translate(rTranslationVector, mVessels);
}

template <unsigned DIM>
void VesselNetwork<DIM>::Translate(DimensionalChastePoint<DIM> rTranslationVector, std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    std::set<boost::shared_ptr<VesselNode<DIM> > > nodes;
    for(unsigned idx = 0; idx <vessels.size(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > vessel_nodes = vessels[idx]->GetNodes();
        std::copy(vessel_nodes.begin(), vessel_nodes.end(), std::inserter(nodes, nodes.begin()));
    }

    typename std::set<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        DimensionalChastePoint<DIM> old_loc = (*node_iter)->rGetLocation();
        old_loc.Translate(rTranslationVector);
        (*node_iter)->SetLocation(old_loc);
    }
}

template <unsigned DIM>
void VesselNetwork<DIM>::RemoveVessel(boost::shared_ptr<Vessel<DIM> > pVessel, bool deleteVessel)
{

    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it = std::find(mVessels.begin(), mVessels.end(), pVessel);
    if(it != mVessels.end())
    {
        if(deleteVessel)
        {
            (*it)->Remove();
        }
        mVessels.erase(it);
    }
    else
    {
        EXCEPTION("Vessel is not contained inside network.");
    }

    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
}

template <unsigned DIM>
void VesselNetwork<DIM>::SetNodeRadii(units::quantity<unit::length> radius)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();

    for(unsigned idx=0; idx<nodes.size();idx++)
    {
        nodes[idx]->SetRadius(radius);
    }

}

template <unsigned DIM>
void VesselNetwork<DIM>::SetSegmentRadii(units::quantity<unit::length> radius)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();

    for(unsigned idx=0; idx<segments.size();idx++)
    {
        segments[idx]->SetRadius(radius);
    }
}

template <unsigned DIM>
void VesselNetwork<DIM>::SetSegmentViscosity(units::quantity<unit::dynamic_viscosity> viscosity)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx]->GetFlowProperties()->SetViscosity(viscosity);
    }
}

template <unsigned DIM>
void VesselNetwork<DIM>::UpdateAll(bool merge)
{
    if(merge)
    {
        MergeCoincidentNodes();
    }
    UpdateSegments();
    UpdateVesselNodes();
    UpdateNodes();
    UpdateVesselIds();
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateNodes()
{
    mNodes.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        for (unsigned idx=0; idx<(*it)->GetNumberOfNodes(); idx++)
        {
            (*it)->GetNode(idx)->SetComparisonId(0);
        }
    }
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        for (unsigned idx=0; idx<(*it)->GetNumberOfNodes(); idx++)
        {
            if((*it)->GetNode(idx)->GetComparisonId()==0)
            {
                mNodes.push_back((*it)->GetNode(idx));
                (*it)->GetNode(idx)->SetComparisonId(1);
            }
        }
    }
    mNodesUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateSegments()
{
    mSegments.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > vessel_segments = (*it)->GetSegments();
        std::copy(vessel_segments.begin(), vessel_segments.end(), std::back_inserter(mSegments));
    }
    mSegmentsUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateVesselNodes()
{
    mVesselNodes.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        (*it)->UpdateNodes();
        (*it)->GetStartNode()->SetComparisonId(0);
        (*it)->GetEndNode()->SetComparisonId(0);
    }
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        if((*it)->GetStartNode()->GetComparisonId()==0)
        {
            mVesselNodes.push_back((*it)->GetStartNode());
            (*it)->GetStartNode()->SetComparisonId(1);
        }
        if((*it)->GetEndNode()->GetComparisonId()==0)
        {
            mVesselNodes.push_back((*it)->GetEndNode());
            (*it)->GetEndNode()->SetComparisonId(1);
        }
    }
    mVesselNodesUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateVesselIds()
{
    for(unsigned idx=0;idx<mVessels.size();idx++)
    {
        mVessels[idx]->SetId(idx);
    }
}

template<unsigned DIM>
bool VesselNetwork<DIM>::VesselCrossesLineSegment(const DimensionalChastePoint<DIM>& coordinate_1,
                                                  const DimensionalChastePoint<DIM>& coordinate_2,
                                                  double tolerance)
{
    boost::shared_ptr<VesselSegment<DIM> > temp_segment = VesselSegment<DIM>::Create(VesselNode<DIM>::Create(coordinate_1), VesselNode<DIM>::Create(coordinate_2));
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > nearest_segment = GetNearestSegment(temp_segment);

    // todo a false here does not necessarily guarantee that a vessel does not cross a line segment since get nearest
    // segment only returns one segment
    double nearest_seg_dist = nearest_segment.second / nearest_segment.first->GetNode(0)->GetReferenceLengthScale();
    double coord1_distance = nearest_segment.first->GetDistance(coordinate_1) / nearest_segment.first->GetNode(0)->GetReferenceLengthScale();
    double coord2_distance = nearest_segment.first->GetDistance(coordinate_2) / nearest_segment.first->GetNode(0)->GetReferenceLengthScale();

    bool crosses_segment = (nearest_seg_dist<= tolerance) && (coord1_distance > tolerance) && (coord2_distance > tolerance);
    return  crosses_segment;
}

template<unsigned DIM>
void VesselNetwork<DIM>::Write(const std::string& rFileName)
{
    boost::shared_ptr<VesselNetworkWriter<DIM> > p_writer = VesselNetworkWriter<DIM>::Create();
    p_writer->SetFileName(rFileName);
    p_writer->SetVesselNetwork(this->shared_from_this());
    p_writer->Write();
}

// Explicit instantiation
template class VesselNetwork<2>;
template class VesselNetwork<3>;

