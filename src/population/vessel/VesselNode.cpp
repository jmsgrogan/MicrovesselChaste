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

#include "Exception.hpp"
#include "VesselNode.hpp"

template<unsigned DIM>
VesselNode<DIM>::VesselNode(QLength v1, QLength v2, QLength v3) : AbstractVesselNetworkComponent<DIM>(),
        mLocation(Vertex<DIM>(v1 ,v2, v3)),
        mSegments(std::vector<std::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(std::make_shared<NodeFlowProperties<DIM> >()),
        mpChemicalProperties(std::make_shared<NodeChemicalProperties<DIM> >()),
        mPtrComparisonId(0),
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
VesselNode<DIM>::VesselNode(const Vertex<DIM>& location) : AbstractVesselNetworkComponent<DIM>(),
        mLocation(location),
        mSegments(std::vector<std::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(std::make_shared<NodeFlowProperties<DIM> >()),
        mpChemicalProperties(std::make_shared<NodeChemicalProperties<DIM> >()),
        mPtrComparisonId(0),
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
VesselNode<DIM>::VesselNode(const VesselNode<DIM>& rExistingNode) :
        std::enable_shared_from_this<VesselNode<DIM> >(), AbstractVesselNetworkComponent<DIM>(),
        mLocation(rExistingNode.rGetLocation()),
        mSegments(std::vector<std::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(std::make_shared<NodeFlowProperties<DIM> >()),
        mpChemicalProperties(std::make_shared<NodeChemicalProperties<DIM> >()),
        mPtrComparisonId(0),
        mGlobalIndex(0),
        mLocalIndex(0),
        mOwnerRank(0),
        mIsHalo(false),
        mHasHalo(false),
        mOtherProcessorRank(0),
        mOtherProcessorLocalIndex(0)
{
    SetFlowProperties(*(rExistingNode.GetFlowProperties()));
    mIsMigrating = rExistingNode.IsMigrating();
}


template<unsigned DIM>
VesselNode<DIM>::~VesselNode()
{
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(QLength v1, QLength v2, QLength v3)
{
    return std::make_shared<VesselNode<DIM> >(v1, v2, v3);
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(const Vertex<DIM>& location)
{
    return std::make_shared<VesselNode<DIM> >(location);
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(const VesselNode<DIM>& rExistingNode)
{
    return std::make_shared<VesselNode<DIM> >(rExistingNode);
}

template<unsigned DIM>
std::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(std::shared_ptr<VesselNode<DIM> > pExistingNode)
{
    if(!pExistingNode)
    {
        EXCEPTION("A Null pointer cannot be used when copying nodes.");
    }
    return std::make_shared<VesselNode<DIM> >(*pExistingNode);
}

template<unsigned DIM>
void VesselNode<DIM>::AddSegment(std::shared_ptr<VesselSegment<DIM> > pVesselSegment)
{
    // Vessel segments can only be attached to a node once. Note use of lock to get shared_ptr from
    // weak_ptr.
    for(auto& segment:mSegments)
    {
        if (segment.lock() == pVesselSegment)
        {
            EXCEPTION("This segment is already attached to this node.");
        }
    }
    mSegments.push_back(std::weak_ptr<VesselSegment<DIM> >(pVesselSegment));
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetComparisonId()
{
    return mPtrComparisonId;
}

template<unsigned DIM>
QLength VesselNode<DIM>::GetDistance(const Vertex<DIM>& rLocation) const
{
    return mLocation.GetDistance(rLocation);
}

template<unsigned DIM>
std::shared_ptr<NodeFlowProperties<DIM> > VesselNode<DIM>::GetFlowProperties() const
{
    return this->mpFlowProperties;
}

template<unsigned DIM>
std::shared_ptr<NodeChemicalProperties<DIM> > VesselNode<DIM>::GetChemicalProperties() const
{
    return this->mpChemicalProperties;
}

template<unsigned DIM>
const Vertex<DIM>& VesselNode<DIM>::rGetLocation() const
{
    return mLocation;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetNumberOfSegments() const
{
    return mSegments.size();
}

template<unsigned DIM>
std::map<std::string, double> VesselNode<DIM>::GetOutputData()
{
    this->mOutputData.clear();
    std::map<std::string, double> flow_data = this->mpFlowProperties->GetOutputData();
    this->mOutputData.insert(flow_data.begin(), flow_data.end());
    this->mOutputData["Node Id"] = double(this->GetId());
    this->mOutputData["Node Radius m"] = this->GetRadius() / 1_m;
    this->mOutputData["Node Is Migrating"] = double(this->IsMigrating());
    this->mOutputData["Node Owner Rank"] = this->GetOwnerRank();
    this->mOutputData["Node Is Halo"] = this->IsHalo();
    this->mOutputData["Node Has Halo"] = this->HasHalo();

    std::map<std::string, double> chemical_data = this->mpChemicalProperties->GetOutputData();
    this->mOutputData.insert(chemical_data.begin(), chemical_data.end());
    return this->mOutputData;
}

template<unsigned DIM>
std::shared_ptr<VesselSegment<DIM> > VesselNode<DIM>::GetSegment(unsigned index) const
{
    if(index >= mSegments.size())
    {
        EXCEPTION("Requested segment index out of range");
    }
    else
    {
        // Convert to shared ptr from weak ptr
        return mSegments[index].lock();
    }
}

template<unsigned DIM>
std::vector<std::shared_ptr<VesselSegment<DIM> > > VesselNode<DIM>::GetSegments() const
{
    // Need to do it this way because of weak pointers, can't just return mSegments
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments(mSegments.size());
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx] = mSegments[idx].lock();
    }
    return segments;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetGlobalIndex()
{
    return mGlobalIndex;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetLocalIndex()
{
    return mLocalIndex;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetOwnerRank()
{
    return mOwnerRank;
}

template<unsigned DIM>
bool VesselNode<DIM>::IsHalo()
{
    return mIsHalo;
}

template<unsigned DIM>
bool VesselNode<DIM>::HasHalo()
{
    return mHasHalo;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetOtherProcessorRank()
{
    return mOtherProcessorRank;
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetOtherProcessorLocalIndex()
{
    return mOtherProcessorLocalIndex;
}

template<unsigned DIM>
bool VesselNode<DIM>::IsAttachedTo(const std::shared_ptr<VesselSegment<DIM> > pSegment) const
{
    // Need to get shared ptr from current node to allow for comparison
    return (pSegment->GetNode(0) == this->shared_from_this() || pSegment->GetNode(1) == this->shared_from_this());
}

template<unsigned DIM>
bool VesselNode<DIM>::IsCoincident(const Vertex<DIM>& rLocation) const
{
    return this->mLocation.IsCoincident(rLocation);
}

template<unsigned DIM>
bool VesselNode<DIM>::IsMigrating() const
{
    return mIsMigrating;
}

template<unsigned DIM>
void VesselNode<DIM>::RemoveSegment(std::shared_ptr<VesselSegment<DIM> > pVesselSegment)
{
    // Need to do it this way due to weak pointer use
    for (unsigned idx = 0; idx < mSegments.size(); idx++)
    {
        if (mSegments[idx].lock() == pVesselSegment)
        {
            mSegments.erase(mSegments.begin() + idx);
            break;
        }
    }
}

template<unsigned DIM>
void VesselNode<DIM>::SetComparisonId(unsigned id)
{
    mPtrComparisonId = id;
}

template<unsigned DIM>
void VesselNode<DIM>::SetFlowProperties(const NodeFlowProperties<DIM>& rFlowProperties)
{
    this->mpFlowProperties = std::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>(rFlowProperties));
}

template<unsigned DIM>
void VesselNode<DIM>::SetChemicalProperties(const NodeChemicalProperties<DIM>& rChemicalProperties)
{
    this->mpChemicalProperties = std::shared_ptr<NodeChemicalProperties<DIM> >(new NodeChemicalProperties<DIM>(rChemicalProperties));
}

template<unsigned DIM>
void VesselNode<DIM>::SetLocation(const Vertex<DIM>& location)
{
    this->mLocation = Vertex<DIM>(location);
}

template<unsigned DIM>
void VesselNode<DIM>::SetLocation(QLength x, QLength y, QLength z)
{
    this->mLocation = Vertex<DIM>(x,y,z);
}

template<unsigned DIM>
void VesselNode<DIM>::SetIsMigrating(bool isMigrating)
{
    mIsMigrating = isMigrating;
}

template<unsigned DIM>
void VesselNode<DIM>::SetGlobalIndex(unsigned index)
{
    mGlobalIndex =index;
}

template<unsigned DIM>
void VesselNode<DIM>::SetLocalIndex(unsigned index)
{
    mLocalIndex = index;
}

template<unsigned DIM>
void VesselNode<DIM>::SetOwnerRank(unsigned rank)
{
    mOwnerRank = rank;
}

template<unsigned DIM>
void VesselNode<DIM>::SetIsHalo(bool isHalo)
{
    mIsHalo = isHalo;
}

template<unsigned DIM>
void VesselNode<DIM>::SetHasHalo(bool hasHalo)
{
    mHasHalo = hasHalo;
}

template<unsigned DIM>
void VesselNode<DIM>::SetOtherProcessorRank(unsigned otherRank)
{
    mOtherProcessorRank = otherRank;
}

template<unsigned DIM>
void VesselNode<DIM>::SetOtherProcessorLocalIndex(unsigned otherIndex)
{
    mOtherProcessorLocalIndex = otherIndex;
}

// Explicit instantiation
template class VesselNode<2>;
template class VesselNode<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(VesselNode, 2)
EXPORT_TEMPLATE_CLASS1(VesselNode, 3)
