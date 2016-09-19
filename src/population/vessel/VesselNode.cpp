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

#include <boost/weak_ptr.hpp>
#include "Exception.hpp"
#include "VesselNode.hpp"

template<unsigned DIM>
VesselNode<DIM>::VesselNode(double v1, double v2, double v3, units::quantity<unit::length> referenceLength) : AbstractVesselNetworkComponent<DIM>(),
        mLocation(DimensionalChastePoint<DIM>(v1 ,v2, v3, referenceLength)),
        mSegments(std::vector<boost::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>())),
        mPtrComparisonId(0)
{
    this->mpFlowProperties = boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>());
}

template<unsigned DIM>
VesselNode<DIM>::VesselNode(double v1, double v2, double v3) : AbstractVesselNetworkComponent<DIM>(),
        mLocation(DimensionalChastePoint<DIM>(v1 ,v2, v3)),
        mSegments(std::vector<boost::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>())),
        mPtrComparisonId(0)
{
    this->mpFlowProperties = boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>());
}

template<unsigned DIM>
VesselNode<DIM>::VesselNode(const DimensionalChastePoint<DIM>& location) : AbstractVesselNetworkComponent<DIM>(),
        mLocation(location),
        mSegments(std::vector<boost::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>())),
        mPtrComparisonId(0)
{
    this->mpFlowProperties = boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>());
}

template<unsigned DIM>
VesselNode<DIM>::VesselNode(const VesselNode<DIM>& rExistingNode) :
        boost::enable_shared_from_this<VesselNode<DIM> >(), AbstractVesselNetworkComponent<DIM>(),
        mLocation(rExistingNode.rGetLocation()),
        mSegments(std::vector<boost::weak_ptr<VesselSegment<DIM> > >()),
        mIsMigrating(false),
        mpFlowProperties(boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>())),
        mPtrComparisonId(0)
{
    SetFlowProperties(*(rExistingNode.GetFlowProperties()));
    mIsMigrating = rExistingNode.IsMigrating();
}


template<unsigned DIM>
VesselNode<DIM>::~VesselNode()
{
}

template<unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(double v1, double v2, double v3, units::quantity<unit::length> referenceLength)
{
    MAKE_PTR_ARGS(VesselNode<DIM>, pSelf, (v1, v2, v3, referenceLength));
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(double v1, double v2, double v3)
{
    MAKE_PTR_ARGS(VesselNode<DIM>, pSelf, (v1, v2, v3));
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(const DimensionalChastePoint<DIM>& location)
{
    MAKE_PTR_ARGS(VesselNode<DIM>, pSelf, (location));
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(const VesselNode<DIM>& rExistingNode)
{
    MAKE_PTR_ARGS(VesselNode<DIM>, pSelf, (rExistingNode));
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNode<DIM>::Create(boost::shared_ptr<VesselNode<DIM> > pExistingNode)
{
    if(!pExistingNode)
    {
        EXCEPTION("A Null pointer cannot be used when copying nodes.");
    }
    MAKE_PTR_ARGS(VesselNode<DIM>, pSelf, (*pExistingNode));
    return pSelf;
}

template<unsigned DIM>
void VesselNode<DIM>::AddSegment(boost::shared_ptr<VesselSegment<DIM> > pVesselSegment)
{
    // Vessel segments can only be attached to a node once. Note use of lock to get shared_ptr from
    // weak_ptr.
    for (unsigned idx = 0; idx < mSegments.size(); idx++)
    {
        if (mSegments[idx].lock() == pVesselSegment)
        {
            EXCEPTION("This segment is already attached to this node.");
        }
    }
    mSegments.push_back(boost::weak_ptr<VesselSegment<DIM> >(pVesselSegment));
}

template<unsigned DIM>
unsigned VesselNode<DIM>::GetComparisonId()
{
    return mPtrComparisonId;
}

template<unsigned DIM>
units::quantity<unit::length> VesselNode<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation) const
{
    return mLocation.GetDistance(rLocation, true);
}

template<unsigned DIM>
boost::shared_ptr<NodeFlowProperties<DIM> > VesselNode<DIM>::GetFlowProperties() const
{
    return this->mpFlowProperties;
}

template<unsigned DIM>
const DimensionalChastePoint<DIM>& VesselNode<DIM>::rGetLocation() const
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
    this->mOutputData["Node Radius m"] = this->GetRadius() / unit::metres;
    this->mOutputData["Node Is Migrating"] = double(this->IsMigrating());
    return this->mOutputData;
}

template<unsigned DIM>
units::quantity<unit::length> VesselNode<DIM>::GetReferenceLengthScale() const
{
    return mLocation.GetReferenceLengthScale();
}

template<unsigned DIM>
boost::shared_ptr<VesselSegment<DIM> > VesselNode<DIM>::GetSegment(unsigned index) const
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
std::vector<boost::shared_ptr<VesselSegment<DIM> > > VesselNode<DIM>::GetSegments() const
{
    // Need to do it this way because of weak pointers, can't just return mSegments
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments(mSegments.size());
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        segments[idx] = mSegments[idx].lock();
    }
    return segments;
}

template<unsigned DIM>
bool VesselNode<DIM>::IsAttachedTo(const boost::shared_ptr<VesselSegment<DIM> > pSegment) const
{
    // Need to get shared ptr from current node to allow for comparison
    return (pSegment->GetNode(0) == this->shared_from_this() || pSegment->GetNode(1) == this->shared_from_this());
}

template<unsigned DIM>
bool VesselNode<DIM>::IsCoincident(const DimensionalChastePoint<DIM>& rLocation) const
{
    return this->mLocation.IsCoincident(rLocation, true);
}

template<unsigned DIM>
bool VesselNode<DIM>::IsMigrating() const
{
    return mIsMigrating;
}

template<unsigned DIM>
void VesselNode<DIM>::RemoveSegment(boost::shared_ptr<VesselSegment<DIM> > pVesselSegment)
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
    this->mpFlowProperties = boost::shared_ptr<NodeFlowProperties<DIM> >(new NodeFlowProperties<DIM>(rFlowProperties));
}

template<unsigned DIM>
void VesselNode<DIM>::SetLocation(const DimensionalChastePoint<DIM>& location)
{
    this->mLocation = DimensionalChastePoint<DIM>(location);
}

template<unsigned DIM>
void VesselNode<DIM>::SetLocation(double x, double y, double z, units::quantity<unit::length> referenceLength)
{
    this->mLocation = DimensionalChastePoint<DIM>(x,y,z,referenceLength);
}

template<unsigned DIM>
void VesselNode<DIM>::SetIsMigrating(bool isMigrating)
{
    mIsMigrating = isMigrating;
}

template<unsigned DIM>
void VesselNode<DIM>::SetReferenceLengthScale(units::quantity<unit::length> lengthScale)
{
    this->mLocation.SetReferenceLengthScale(lengthScale);
}

// Explicit instantiation
template class VesselNode<2> ;
template class VesselNode<3> ;
