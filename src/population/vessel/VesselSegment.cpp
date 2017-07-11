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
#include "UblasIncludes.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "VesselSegment.hpp"
#include "GeometryTools.hpp"

template<unsigned DIM>
VesselSegment<DIM>::VesselSegment() :
        AbstractVesselNetworkComponent<DIM>(),
        mNodes(),
        mVessel(std::weak_ptr<Vessel<DIM> >()),
        mpFlowProperties(std::make_shared<SegmentFlowProperties<DIM> >()),
        mMaturity(1.0),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0),
        mOtherProcessorLocalIndex(0)
{

}

template<unsigned DIM>
VesselSegment<DIM>::VesselSegment(std::shared_ptr<VesselNode<DIM> > pNode1, std::shared_ptr<VesselNode<DIM> > pNode2) :
        AbstractVesselNetworkComponent<DIM>(),
        mNodes(std::pair<std::shared_ptr<VesselNode<DIM> >, std::shared_ptr<VesselNode<DIM> > >(pNode1, pNode2)),
        mVessel(std::weak_ptr<Vessel<DIM> >()),
        mpFlowProperties(std::make_shared<SegmentFlowProperties<DIM> >()),
        mMaturity(1.0),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0),
        mOtherProcessorLocalIndex(0)
{
}

template<unsigned DIM>
VesselSegment<DIM>::VesselSegment(const VesselSegment<DIM>& rSegment) :
    std::enable_shared_from_this<VesselSegment<DIM> >(), AbstractVesselNetworkComponent<DIM>(),
    mNodes(rSegment.GetNodes()),
    mVessel(std::weak_ptr<Vessel<DIM> >()),
    mpFlowProperties(std::make_shared<SegmentFlowProperties<DIM> >()),
    mMaturity(1.0),
    mGlobalIndex(0),
    mLocalIndex(0),
    mOwnerRank(0),
    mIsHalo(false),
    mHasHalo(false),
    mOtherProcessorRank(0),
    mOtherProcessorLocalIndex(0)
{
    this->SetFlowProperties(*(rSegment.GetFlowProperties()));
}

template<unsigned DIM>
std::shared_ptr<VesselSegment<DIM> > VesselSegment<DIM>::Create(std::shared_ptr<VesselNode<DIM> > pNode1,
        std::shared_ptr<VesselNode<DIM> > pNode2)
{
    std::shared_ptr<VesselSegment<DIM> > pSelf(new VesselSegment<DIM>(pNode1, pNode2));

    // Make sure the specified nodes are not the same
    if (pNode1 == pNode2)
    {
        EXCEPTION("Attempted to assign the same node to both ends of a vessel segment.");
    }

    // Add the segment to the nodes
    pNode1->AddSegment(pSelf->shared_from_this());
    pNode2->AddSegment(pSelf->shared_from_this());
    return pSelf;
}

template<unsigned DIM>
std::shared_ptr<VesselSegment<DIM> > VesselSegment<DIM>::Create(std::shared_ptr<VesselSegment<DIM> > pSegment)
{
    if(!pSegment)
    {
        EXCEPTION("A Null pointer cannot be used when copying segments.");
    }
    std::shared_ptr<VesselSegment<DIM> > p_self(new VesselSegment<DIM>(*pSegment));

    // Add the segment to the nodes
    p_self->GetNode(0)->AddSegment(p_self->shared_from_this());
    p_self->GetNode(1)->AddSegment(p_self->shared_from_this());
    return p_self;
}

template<unsigned DIM>
VesselSegment<DIM>::~VesselSegment()
{
}

template<unsigned DIM>
void VesselSegment<DIM>::AddVessel(std::shared_ptr<Vessel<DIM> > pVessel)
{
    mVessel = pVessel;
}

template<unsigned DIM>
void VesselSegment<DIM>::CopyDataFromExistingSegment(const std::shared_ptr<VesselSegment<DIM> > pTargetSegment)
{
    this->mOutputData = pTargetSegment->GetOutputData();
    this->SetRadius(pTargetSegment->GetRadius());
    this->SetFlowProperties(*(pTargetSegment->GetFlowProperties()));
}

template<unsigned DIM>
QLength VesselSegment<DIM>::GetDistance(const Vertex<DIM>& location) const
{
    return GetDistanceToLineSegment(mNodes.first->rGetLocation(), mNodes.second->rGetLocation(), location);
}

template<unsigned DIM>
std::shared_ptr<SegmentFlowProperties<DIM> > VesselSegment<DIM>::GetFlowProperties() const
{
    return this->mpFlowProperties;
}

template<unsigned DIM>
QLength VesselSegment<DIM>::GetLength() const
{
    return mNodes.second->GetDistance(mNodes.first->rGetLocation());
}

template<unsigned DIM>
double VesselSegment<DIM>::GetMaturity() const
{
    return mMaturity;
}

template<unsigned DIM>
unsigned VesselSegment<DIM>::GetGlobalIndex()
{
    return mGlobalIndex;
}

template<unsigned DIM>
unsigned VesselSegment<DIM>::GetLocalIndex()
{
    return mLocalIndex;
}

template<unsigned DIM>
unsigned VesselSegment<DIM>::GetOwnerRank()
{
    return mOwnerRank;
}

template<unsigned DIM>
bool VesselSegment<DIM>::IsHalo()
{
    return mIsHalo;
}

template<unsigned DIM>
bool VesselSegment<DIM>::HasHalo()
{
    return mHasHalo;
}

template<unsigned DIM>
unsigned VesselSegment<DIM>::GetOtherProcessorRank()
{
    return mOtherProcessorRank;
}

template<unsigned DIM>
unsigned VesselSegment<DIM>::GetOtherProcessorLocalIndex()
{
    return mOtherProcessorLocalIndex;
}

template<unsigned DIM>
std::map<std::string, double> VesselSegment<DIM>::GetOutputData()
{
    this->mOutputData.clear();
    std::map<std::string, double> flow_data = this->mpFlowProperties->GetOutputData();
    this->mOutputData.insert(flow_data.begin(), flow_data.end());
    this->mOutputData["Segment Id"] = double(this->GetId());
    this->mOutputData["Segment Radius m: "] = this->GetRadius() / 1_m;
    this->mOutputData["Maturity"] = this->GetMaturity();
    return this->mOutputData;
}

template<unsigned DIM>
Vertex<DIM> VesselSegment<DIM>::GetMidPoint() const
{
    return mNodes.first->rGetLocation().GetMidPoint(mNodes.second->rGetLocation());
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselSegment<DIM>::GetNode(unsigned index) const
{
    if (index == 0u)
    {
        return mNodes.first;
    }
    else if (index == 1u)
    {
        return mNodes.second;
    }
    else
    {
        EXCEPTION("A node index other than 0 or 1 has been requested for a Vessel Segment.");
    }
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselSegment<DIM>::GetOppositeNode(std::shared_ptr<VesselNode<DIM> > pInputNode) const
{
    if(pInputNode == mNodes.first)
    {
        return mNodes.second;
    }
    else if(pInputNode == mNodes.second)
    {
        return mNodes.first;
    }
    else
    {
        EXCEPTION("Input node is not on the segment");
    }
}

template<unsigned DIM>
std::pair<std::shared_ptr<VesselNode<DIM> >, std::shared_ptr<VesselNode<DIM> > > VesselSegment<DIM>::GetNodes() const
{
    return mNodes;
}

template<unsigned DIM>
Vertex<DIM> VesselSegment<DIM>::GetPointProjection(const  Vertex<DIM>& location, bool projectToEnds) const
{
    return GetPointProjectionOnLineSegment(mNodes.first->rGetLocation().rGetLocation(),
            mNodes.second->rGetLocation().rGetLocation(),
            location.rGetLocation(), projectToEnds);
}

template<unsigned DIM>
c_vector<double, DIM> VesselSegment<DIM>::GetUnitTangent() const
{
    return mNodes.first->rGetLocation().GetUnitTangent(mNodes.second->rGetLocation());
}

template<unsigned DIM>
std::shared_ptr<Vessel<DIM> > VesselSegment<DIM>::GetVessel() const
{
    if (mVessel.lock())
    {
        return mVessel.lock();
    }
    else
    {
        EXCEPTION("A vessel has been requested but this segment doesn't have one.");
    }
}

template<unsigned DIM>
bool VesselSegment<DIM>::HasNode(std::shared_ptr<VesselNode<DIM> > pNode) const
{
    return (pNode == GetNode(0) || pNode == GetNode(1));
}

template<unsigned DIM>
bool VesselSegment<DIM>::IsConnectedTo(std::shared_ptr<VesselSegment<DIM> > otherSegment) const
{
    bool isConnectedToSegment = false;
    if (this->GetNode(0) == otherSegment->GetNode(0) || this->GetNode(0) == otherSegment->GetNode(1)
            || this->GetNode(1) == otherSegment->GetNode(0) || this->GetNode(1) == otherSegment->GetNode(1))
    {
        isConnectedToSegment = true;
    }

    return isConnectedToSegment;
}

template<unsigned DIM>
void VesselSegment<DIM>::RemoveVessel()
{
    mVessel = std::weak_ptr<Vessel<DIM> >();
}

template<unsigned DIM>
void VesselSegment<DIM>::Remove()
{
    mNodes.first->RemoveSegment(Shared());
    mNodes.second->RemoveSegment(Shared());
    RemoveVessel();
}

template<unsigned DIM>
void VesselSegment<DIM>::ReplaceNode(unsigned oldNodeIndex, std::shared_ptr<VesselNode<DIM> > pNewNode)
{
    if (oldNodeIndex == 0u)
    {
        mNodes.first->RemoveSegment(Shared());
        mNodes.first = pNewNode;
        mNodes.first->AddSegment(Shared());
    }
    else if (oldNodeIndex == 1u)
    {
        mNodes.second->RemoveSegment(Shared());
        mNodes.second = pNewNode;
        mNodes.second->AddSegment(Shared());
    }
    else
    {
        EXCEPTION("A node index other than 0 or 1 has been requested for a Vessel Segment.");
    }

    if (mVessel.lock() != NULL)
    {
        mVessel.lock()->UpdateNodes();
    }
}

template<unsigned DIM>
void VesselSegment<DIM>::SetMaturity(double maturity)
{
    mMaturity = maturity;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetGlobalIndex(unsigned index)
{
    mGlobalIndex =index;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetLocalIndex(unsigned index)
{
    mLocalIndex = index;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetOwnerRank(unsigned rank)
{
    mOwnerRank = rank;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetIsHalo(bool isHalo)
{
    mIsHalo = isHalo;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetHasHalo(bool hasHalo)
{
    mHasHalo = hasHalo;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetOtherProcessorRank(unsigned otherRank)
{
    mOtherProcessorRank = otherRank;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetOtherProcessorLocalIndex(unsigned otherIndex)
{
    mOtherProcessorLocalIndex = otherIndex;
}

template<unsigned DIM>
void VesselSegment<DIM>::SetFlowProperties(const SegmentFlowProperties<DIM> & rFlowProperties)
{
    this->mpFlowProperties = std::shared_ptr<SegmentFlowProperties<DIM> >(new SegmentFlowProperties<DIM> (rFlowProperties));
}

template<unsigned DIM>
std::shared_ptr<VesselSegment<DIM> > VesselSegment<DIM>::Shared()
{
    std::shared_ptr<VesselSegment<DIM> > pSegment = this->shared_from_this();
    return pSegment;
}

// Explicit instantiation
template class VesselSegment<2>;
template class VesselSegment<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(VesselSegment, 2)
EXPORT_TEMPLATE_CLASS1(VesselSegment, 3)
