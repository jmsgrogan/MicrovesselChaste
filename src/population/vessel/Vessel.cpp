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
#include "Exception.hpp"
#include "UblasIncludes.hpp"
#include "SimulationTime.hpp"
#include "Vessel.hpp"

template<unsigned DIM>
Vessel<DIM>::Vessel() : AbstractVesselNetworkComponent<DIM>(),
        mSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > >()),
        mNodes(std::vector<std::shared_ptr<VesselNode<DIM> > >()),
        mNodesUpToDate(false),
        mpFlowProperties(std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM>())),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0)
{


}

template<unsigned DIM>
Vessel<DIM>::Vessel(std::shared_ptr<VesselSegment<DIM> > pSegment) : AbstractVesselNetworkComponent<DIM>(),
        mSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > >()),
        mNodes(std::vector<std::shared_ptr<VesselNode<DIM> > >()),
        mNodesUpToDate(false),
        mpFlowProperties(std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM>())),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0)
{
    mSegments.push_back(pSegment);
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
Vessel<DIM>::Vessel(std::vector<std::shared_ptr<VesselSegment<DIM> > > segments) : AbstractVesselNetworkComponent<DIM>(),
        mSegments(segments),
        mNodes(std::vector<std::shared_ptr<VesselNode<DIM> > >()),
        mNodesUpToDate(false),
        mpFlowProperties(std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM>())),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0)
{
    if (segments.size() > 1)
    {
        for (unsigned i = 1; i < mSegments.size(); i++)
        {
            if (!mSegments[i]->IsConnectedTo(mSegments[i - 1]))
            {
                EXCEPTION("Input vessel segments are not attached in the correct order.");
            }
        }

        for (unsigned i = 0; i < mSegments.size(); i++)
        {
            for (unsigned j = 0; j < mSegments.size(); j++)
            {
                if (i != j && i != j - 1 && i != j + 1)
                {
                    if (mSegments[i]->IsConnectedTo(mSegments[j]))
                    {
                        EXCEPTION("Input vessel segments are not correctly connected.");
                    }
                }
            }
        }
    }

    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
Vessel<DIM>::Vessel(std::vector<std::shared_ptr<VesselNode<DIM> > > nodes) :
        mSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > >()),
        mNodes(std::vector<std::shared_ptr<VesselNode<DIM> > >()),
        mNodesUpToDate(false),
        mpFlowProperties(std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM>())),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0)
{

    if (nodes.size() < 2)
    {
        EXCEPTION("Insufficient number of nodes to define a segment.");
    }
    else
    {
        for (unsigned i = 1; i < nodes.size(); i++)
        {
            mSegments.push_back(VesselSegment<DIM>::Create(nodes[i-1], nodes[i]));
        }
    }
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
Vessel<DIM>::Vessel(std::shared_ptr<VesselNode<DIM> > pStartNode, std::shared_ptr<VesselNode<DIM> > pEndNode)
    :        mSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > >()),
             mNodes(std::vector<std::shared_ptr<VesselNode<DIM> > >()),
             mNodesUpToDate(false),
             mpFlowProperties(std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM>())),
             mGlobalIndex(0),
             mLocalIndex(0),
             mOwnerRank(0),
             mIsHalo(false),
             mHasHalo(false),
             mOtherProcessorRank(0)
{
    mSegments.push_back(VesselSegment<DIM>::Create(pStartNode, pEndNode));
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
Vessel<DIM>::~Vessel()
{
}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > Vessel<DIM>::Create(std::shared_ptr<VesselSegment<DIM> > pSegment)
{
    std::shared_ptr<Vessel<DIM> > p_self = std::shared_ptr<Vessel<DIM> >(new Vessel<DIM>(pSegment));

    // Add the vessel to the segment
    pSegment->AddVessel(p_self->shared_from_this());
    return p_self;
}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > Vessel<DIM>::Create(std::vector<std::shared_ptr<VesselSegment<DIM> > > segments)
{
    std::shared_ptr<Vessel<DIM> > p_self = std::shared_ptr<Vessel<DIM> >(new Vessel<DIM> (segments));

    // Add the vessel to the segments
    for (unsigned i = 0; i < segments.size(); i++)
    {
        segments[i]->AddVessel(p_self->shared_from_this());
    }
    return p_self;
}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > Vessel<DIM>::Create(std::vector<std::shared_ptr<VesselNode<DIM> > > nodes)
{
    std::shared_ptr<Vessel<DIM> > p_self = std::shared_ptr<Vessel<DIM> >(new Vessel<DIM> (nodes));

    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = p_self->GetSegments();

    // Add the vessel to the new segments
    for (unsigned i = 0; i < segments.size(); i++)
    {
        segments[i]->AddVessel(p_self->shared_from_this());
    }
    return p_self;
}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > Vessel<DIM>::Create(std::shared_ptr<VesselNode<DIM> > pStartNode, std::shared_ptr<VesselNode<DIM> > pEndNode)
{
    std::shared_ptr<Vessel<DIM> > p_self = std::shared_ptr<Vessel<DIM> >(new Vessel<DIM>(pStartNode, pEndNode));

    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = p_self->GetSegments();

    // Add the vessel to the new segment
    for (unsigned i = 0; i < segments.size(); i++)
    {
        segments[i]->AddVessel(p_self->shared_from_this());
    }
    return p_self;
}

template<unsigned DIM>
void Vessel<DIM>::AddSegment(std::shared_ptr<VesselSegment<DIM> > pSegment)
{

    if(mSegments.size() == 1)
    {
        pSegment->AddVessel(Shared());
        if(pSegment->GetNode(0) == mSegments[0]->GetNode(0))
        {
            mSegments.insert(mSegments.begin(), pSegment);
        }
        else if(pSegment->GetNode(1) == mSegments[0]->GetNode(0))
        {
            mSegments.insert(mSegments.begin(), pSegment);
        }
        else if(pSegment->GetNode(0) == mSegments[0]->GetNode(1))
        {
            mSegments.push_back(pSegment);
        }
        else if(pSegment->GetNode(1) == mSegments[0]->GetNode(1))
        {
            mSegments.push_back(pSegment);
        }
        else
        {
            EXCEPTION("Input vessel segment does not coincide with any end of the vessel.");
        }
    }
    else
    {
        if (pSegment->IsConnectedTo(mSegments.back()))
        // Append to end of vessel
        {
            pSegment->AddVessel(Shared());
            mSegments.push_back(pSegment);
        }
        else if (pSegment->IsConnectedTo(mSegments.front()))
        // Insert at the start of the vessel
        {
            pSegment->AddVessel(Shared());
            mSegments.insert(mSegments.begin(), pSegment);
        }
        else
        {
            EXCEPTION("Input vessel segment does not coincide with any end of the multi-segment vessel.");
        }
    }

    mNodesUpToDate = false;
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
void Vessel<DIM>::AddSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > > segments)
{

    if (segments.front()->IsConnectedTo(mSegments.back()))
    {
        mSegments.insert(mSegments.end(), segments.begin(), segments.end());
    }
    else if (segments.back()->IsConnectedTo(mSegments.front()))
    {
        segments.insert(segments.end(), mSegments.begin(), mSegments.end());
        mSegments = segments;
    }
    else if (segments.front()->IsConnectedTo(mSegments.front()))
    {
        std::reverse(segments.begin(), segments.end());
        segments.insert(segments.end(), mSegments.begin(), mSegments.end());
        mSegments = segments;
    }
    else if (segments.back()->IsConnectedTo(mSegments.back()))
    {
        std::reverse(segments.begin(), segments.end());
        mSegments.insert(mSegments.end(), segments.begin(), segments.end());
    }
    else
    {
        EXCEPTION("Input vessel segments do not coincide with any end of the vessel.");
    }

    for (unsigned i = 1; i < mSegments.size(); i++)
    {
        if (!mSegments[i]->IsConnectedTo(mSegments[i - 1]))
        {
            EXCEPTION("Input vessel segments are not attached in the correct order.");
        }
    }

    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        for (unsigned j = 0; j < mSegments.size(); j++)
        {
            if (i != j && i != j - 1 && i != j + 1)
            {
                if (mSegments[i]->IsConnectedTo(mSegments[j]))
                {
                    EXCEPTION("Input vessel segments are not correctly connected.");
                }
            }
        }
    }

    // Add the vessel to the segments
    for (unsigned i = 0; i < segments.size(); i++)
    {
        segments[i]->AddVessel(Shared());
    }

    mNodesUpToDate = false;
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
void Vessel<DIM>::CopyDataFromExistingVessel(std::shared_ptr<Vessel<DIM> > pTargetVessel)
{
    this->mOutputData = pTargetVessel->GetOutputData();
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > Vessel<DIM>::DivideSegment(const Vertex<DIM>& location, QLength distanceTolerance)
{
    // Identify segment
    std::shared_ptr<VesselSegment<DIM> > pVesselSegment;
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        if (mSegments[i]->GetDistance(location) <= distanceTolerance)
        {
            pVesselSegment = mSegments[i];
            if (pVesselSegment->GetNode(0)->IsCoincident(location))
            {
                return pVesselSegment->GetNode(0);
            }
            if (pVesselSegment->GetNode(1)->IsCoincident(location))
            {
                return pVesselSegment->GetNode(1);
            }

        }
    }

    if (!pVesselSegment)
    {
        EXCEPTION("Specified location is not on a segment in this vessel.");
    }
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        for (unsigned j = 0; j < mSegments.size(); j++)
        {
            if (i != j && i != j - 1 && i != j + 1)
            {
                if (mSegments[i]->IsConnectedTo(mSegments[j]))
                {
                    EXCEPTION("Input vessel segments are not correctly connected.");
                }
            }
        }
    }

    // The node's data is averaged from the original segments's nodes
    // Get the closest node
    QLength distance0 = pVesselSegment->GetNode(0)->GetDistance(location);
    QLength distance1 = pVesselSegment->GetNode(1)->GetDistance(location);
    unsigned closest_index;

    if (distance0 <= distance1)
    {
        closest_index = 0;
    }
    else
    {
        closest_index = 1;
    }

    // Make a copy of the closest node
    std::shared_ptr<VesselNode<DIM> > p_new_node = VesselNode<DIM>::Create(*pVesselSegment->GetNode(closest_index));
    p_new_node->SetLocation(location);
    p_new_node->GetFlowProperties()->SetIsInputNode(false);
    p_new_node->GetFlowProperties()->SetIsOutputNode(false);

    // Make two new segments
    std::shared_ptr<VesselSegment<DIM> > p_new_segment0 = VesselSegment<DIM>::Create(pVesselSegment->GetNode(0), p_new_node);
    std::shared_ptr<VesselSegment<DIM> > p_new_segment1 = VesselSegment<DIM>::Create(p_new_node, pVesselSegment->GetNode(1));
    p_new_segment0->CopyDataFromExistingSegment(pVesselSegment);
    p_new_segment1->CopyDataFromExistingSegment(pVesselSegment);

    // Add new segments, ensuring they are correctly ordered
    std::vector<std::shared_ptr<VesselSegment<DIM> > > newSegments;
    typename std::vector<std::shared_ptr<VesselSegment<DIM> > >::iterator it = std::find(mSegments.begin(), mSegments.end(), pVesselSegment);

    if (it == mSegments.end())
    {
        EXCEPTION("Vessel segment is not contained inside vessel.");
    }

    if (mSegments.size() == 1)
    {
        newSegments.push_back(p_new_segment0);
        newSegments.push_back(p_new_segment1);
    }
    else if(it == mSegments.begin())
    {
        if((*(it + 1))->IsConnectedTo(p_new_segment1))
        {
            newSegments.push_back(p_new_segment0);
            newSegments.push_back(p_new_segment1);
        }
        else
        {
            newSegments.push_back(p_new_segment1);
            newSegments.push_back(p_new_segment0);
        }
    }
    else if ((*(it - 1))->IsConnectedTo(p_new_segment0))
    {
        newSegments.push_back(p_new_segment0);
        newSegments.push_back(p_new_segment1);
    }
    else
    {
        newSegments.push_back(p_new_segment1);
        newSegments.push_back(p_new_segment0);
    }

    mSegments.insert(it, newSegments.begin(), newSegments.end());

    // Remove old segment
    it = std::find(mSegments.begin(), mSegments.end(), pVesselSegment);

    if (it != mSegments.end())
    {
        mSegments.erase(it);
        pVesselSegment->Remove();
    }
    else
    {
        EXCEPTION("Vessel segment is not contained inside vessel.");
    }

    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        for (unsigned j = 0; j < mSegments.size(); j++)
        {
            if (i != j && i != j - 1 && i != j + 1)
            {
                if (mSegments[i]->IsConnectedTo(mSegments[j]))
                {
                    EXCEPTION("Input vessel segments are not correctly connected.");
                }
            }
        }
    }

    // Add the vessel to the segments
    for (unsigned i = 0; i < newSegments.size(); i++)
    {
        newSegments[i]->AddVessel(Shared());
    }

    mNodesUpToDate = false;
    mpFlowProperties->UpdateSegments(mSegments);
    return p_new_node;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetGlobalIndex()
{
    return mGlobalIndex;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetLocalIndex()
{
    return mLocalIndex;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetOwnerRank()
{
    return mOwnerRank;
}

template<unsigned DIM>
bool Vessel<DIM>::IsHalo()
{
    return mIsHalo;
}

template<unsigned DIM>
bool Vessel<DIM>::HasHalo()
{
    return mHasHalo;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetOtherProcessorRank()
{
    return mOtherProcessorRank;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetOtherProcessorLocalIndex()
{
return 0;
}

template<unsigned DIM>
std::shared_ptr<VesselFlowProperties<DIM> > Vessel<DIM>::GetFlowProperties() const
{
    return this->mpFlowProperties;
}

template<unsigned DIM>
std::map<std::string, double> Vessel<DIM>::GetOutputData()
{
    std::map<std::string, double> flow_data = this->mpFlowProperties->GetOutputData();
    this->mOutputData.clear();
    this->mOutputData.insert(flow_data.begin(), flow_data.end());
    this->mOutputData["Vessel Id"] = double(this->GetId());
    this->mOutputData["Vessel Radius m"] = this->GetRadius() / unit::metres;
    this->mOutputData["Vessel Maturity"] = this->GetMaturity();
    this->mOutputData["Vessel Owner Rank"] = this->GetOwnerRank();
    this->mOutputData["Vessel Is Halo"] = this->IsHalo();
    this->mOutputData["Vessel Has Halo"] = this->HasHalo();
    return this->mOutputData;
}

template<unsigned DIM>
QLength Vessel<DIM>::GetClosestEndNodeDistance(const Vertex<DIM>& rLocation)
{
    QLength distance_1 = this->GetStartNode()->GetDistance(rLocation);
    QLength distance_2 = this->GetEndNode()->GetDistance(rLocation);
    if(distance_1 > distance_2)
    {
        return distance_2;
    }
    else
    {
        return distance_1;
    }
}

template<unsigned DIM>
QLength Vessel<DIM>::GetDistance(const Vertex<DIM>& rLocation) const
{
    // Get the distance to the nearest segment in the vessel
    QLength nearest_distance = DBL_MAX * unit::metres;
    for(unsigned idx=0; idx<mSegments.size(); idx++)
    {
        QLength seg_distance = mSegments[idx]->GetDistance(rLocation);
        if(seg_distance < nearest_distance)
        {
            nearest_distance = seg_distance;
        }
    }
    return nearest_distance;
}

template<unsigned DIM>
std::vector<std::shared_ptr<Vessel<DIM> > > Vessel<DIM>::GetConnectedVessels()
{
    std::vector<std::shared_ptr<VesselSegment<DIM> > > start_segments = GetStartNode()->GetSegments();
    std::vector<std::shared_ptr<VesselSegment<DIM> > > end_segments = GetStartNode()->GetSegments();

    std::vector<std::shared_ptr<Vessel<DIM> > > connected;

    for(unsigned idx=0; idx<start_segments.size();idx++)
    {
        if(start_segments[idx]->GetVessel() != Shared())
        {
            connected.push_back(start_segments[idx]->GetVessel());
        }
    }
    for(unsigned idx=0; idx<end_segments.size();idx++)
    {
        if(end_segments[idx]->GetVessel() != Shared())
        {
            connected.push_back(end_segments[idx]->GetVessel());
        }
    }
    return connected;
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > Vessel<DIM>::GetEndNode()
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }

    return mNodes.back();
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > Vessel<DIM>::GetNodeAtOppositeEnd(
        std::shared_ptr<VesselNode<DIM> > pQueryNode)
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }

    if (pQueryNode == GetStartNode())
    {
        return GetEndNode();
    }
    else if (pQueryNode == GetEndNode())
    {
        return GetStartNode();
    }
    else
    {
        EXCEPTION("Query node is not at either end of the vessel.");
    }
}

template<unsigned DIM>
QLength Vessel<DIM>::GetLength() const
{
    QLength length = 0.0 * unit::metres;
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        length = length + mSegments[i]->GetLength();
    }
    return length;
}

template<unsigned DIM>
QLength Vessel<DIM>::GetRadius() const
{
    QLength radius = 0.0 * unit::metres;
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        radius = radius + mSegments[i]->GetRadius();
    }
    return radius / (double(mSegments.size()));
}

template<unsigned DIM>
double Vessel<DIM>::GetMaturity() const
{
    double maturity = 0.0;
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        maturity = maturity + mSegments[i]->GetMaturity();
    }
    return maturity / (double(mSegments.size()));
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > Vessel<DIM>::GetNode(unsigned index)
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }
    if(index >= mNodes.size())
    {
        EXCEPTION("Out of bounds node index requested");
    }
    return mNodes[index];
}

template<unsigned DIM>
std::vector<std::shared_ptr<VesselNode<DIM> > > Vessel<DIM>::GetNodes()
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes;
}

template<unsigned DIM>
const std::vector<std::shared_ptr<VesselNode<DIM> > >& Vessel<DIM>::rGetNodes()
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetNumberOfNodes()
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return GetNumberOfSegments() + 1;
}

template<unsigned DIM>
unsigned Vessel<DIM>::GetNumberOfSegments()
{
    return mSegments.size();
}

template<unsigned DIM>
std::shared_ptr<VesselSegment<DIM> > Vessel<DIM>::GetSegment(unsigned index)
{
    if(index  >= mSegments.size())
    {
        EXCEPTION("Requested segment index out of range");
    }
    return mSegments[index];
}

template<unsigned DIM>
std::vector<std::shared_ptr<VesselSegment<DIM> > > Vessel<DIM>::GetSegments()
{
    return mSegments;
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > Vessel<DIM>::GetStartNode()
{
    if (!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes.front();
}

template<unsigned DIM>
bool Vessel<DIM>::IsConnectedTo(std::shared_ptr<Vessel<DIM> > pOtherVessel)
{
    if (GetStartNode() == pOtherVessel->GetStartNode() || GetEndNode() == pOtherVessel->GetStartNode()
            || GetStartNode() == pOtherVessel->GetEndNode() || GetEndNode() == pOtherVessel->GetEndNode())
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<unsigned DIM>
void Vessel<DIM>::Remove()
{
    // Detach all segments from their nodes
    for (unsigned idx = 0; idx < mSegments.size(); idx++)
    {
        mSegments[idx]->Remove();
    }
    mNodesUpToDate = false;
    mSegments = std::vector<std::shared_ptr<VesselSegment<DIM> > >();
}

template<unsigned DIM>
void Vessel<DIM>::RemoveSegments(SegmentLocation::Value location)
{
    if (mSegments.size() == 1)
    {
        EXCEPTION("Vessel must have at least one segment.");
    }
    if (location == SegmentLocation::Start)
    {
        mSegments.front()->RemoveVessel();
        mSegments.erase(mSegments.begin());
    }
    else if (location == SegmentLocation::End)
    {
        mSegments.back()->RemoveVessel();
        mSegments.pop_back();
    }
    else
    {
        EXCEPTION("You can only remove segments from the start or end of vessels.");
    }
    mNodesUpToDate = false;
    mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
void Vessel<DIM>::SetFlowProperties(const VesselFlowProperties<DIM> & rFlowProperties)
{
    this->mpFlowProperties = std::shared_ptr<VesselFlowProperties<DIM> >(new VesselFlowProperties<DIM> (rFlowProperties));
    this->mpFlowProperties->UpdateSegments(mSegments);
}

template<unsigned DIM>
void Vessel<DIM>::SetRadius(QLength radius)
{
    for (unsigned i = 0; i < mSegments.size(); i++)
    {
        mSegments[i]->SetRadius(radius);
    }
}

template<unsigned DIM>
void Vessel<DIM>::SetGlobalIndex(unsigned index)
{
    mGlobalIndex =index;
}

template<unsigned DIM>
void Vessel<DIM>::SetLocalIndex(unsigned index)
{
    mLocalIndex = index;
}

template<unsigned DIM>
void Vessel<DIM>::SetOwnerRank(unsigned rank)
{
    mOwnerRank = rank;
}

template<unsigned DIM>
void Vessel<DIM>::SetIsHalo(bool isHalo)
{
    mIsHalo = isHalo;
}

template<unsigned DIM>
void Vessel<DIM>::SetHasHalo(bool hasHalo)
{
    mHasHalo = hasHalo;
}

template<unsigned DIM>
void Vessel<DIM>::SetOtherProcessorRank(unsigned otherRank)
{
    mOtherProcessorRank = otherRank;
}

template<unsigned DIM>
void Vessel<DIM>::SetOtherProcessorLocalIndex(unsigned otherIndex)
{

}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > Vessel<DIM>::Shared()
{
    return this->shared_from_this();
}

template<unsigned DIM>
void Vessel<DIM>::UpdateNodes()
{
    mNodes.clear();

    if (mSegments.size() == 1)
    {
        mNodes.push_back(mSegments[0]->GetNode(0));
        mNodes.push_back(mSegments[0]->GetNode(1));
    }

    // Add the start and end nodes of the first segment and then
    // the end nodes of every other segment
    else
    {
        if (mSegments[1]->HasNode(mSegments[0]->GetNode(1)))
        {
            mNodes.push_back(mSegments[0]->GetNode(0));
            mNodes.push_back(mSegments[0]->GetNode(1));
        }
        else if (mSegments[1]->HasNode(mSegments[0]->GetNode(1)))
        {
            mNodes.push_back(mSegments[0]->GetNode(1));
            mNodes.push_back(mSegments[0]->GetNode(0));
        }

        for (unsigned idx = 1; idx < mSegments.size(); idx++)
        {
            if (mNodes[idx] == mSegments[idx]->GetNode(0))
            {
                mNodes.push_back(mSegments[idx]->GetNode(1));
            }
            else
            {
                mNodes.push_back(mSegments[idx]->GetNode(0));
            }
        }
    }
    mNodesUpToDate = true;
}

// Explicit instantiation
template class Vessel<2>;
template class Vessel<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(Vessel, 2)
EXPORT_TEMPLATE_CLASS1(Vessel, 3)
