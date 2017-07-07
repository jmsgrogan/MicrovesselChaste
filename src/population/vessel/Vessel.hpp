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

#ifndef VESSEL_HPP_
#define VESSEL_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/vector.hpp>
#include "ChasteSerialization.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "VesselFlowProperties.hpp"

/**
 * Struct to denote segment locations on the vessel
 */
struct SegmentLocation
{
    /**
     * Values are start and end of segment
     */
    enum Value
    {
        Start, End
    };
};

/**
 * This is a class for vessels. A vessel is a component of a vessel network.
 * Vessels are a collection of connected straight-line segments, such as a polyline.
 * Vessel data and properties are derived from averaging or summing over their segments as required.
 */
template<unsigned DIM>
class Vessel : public std::enable_shared_from_this<Vessel<DIM> >, public AbstractVesselNetworkComponent<DIM>
{
private:

    friend class boost::serialization::access;

    /**
     * Do the serialize
     * @param ar the archive
     * @param version the archive version number
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        #if BOOST_VERSION < 105600
            EXCEPTION("Serialization not supported for Boost < 1.56");
        #else
            ar & boost::serialization::base_object<AbstractVesselNetworkComponent<DIM> >(*this);
            ar & mSegments;
            ar & mNodes;
            ar & mpFlowProperties;
        #endif
    }

    /**
     * Vessel segments
     */
    std::vector<std::shared_ptr<VesselSegment<DIM> > > mSegments;

    /**
     * Nodes
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > mNodes;

    /**
     * Is the data in mNodes up to date.
     */
    bool mNodesUpToDate;

    /**
     * A flow property collection for the vessel
     */
    std::shared_ptr<VesselFlowProperties<DIM> > mpFlowProperties;

    /**
     * The global index
     */
    unsigned mGlobalIndex;

    /**
     * The local index
     */
    unsigned mLocalIndex;

    /**
     * Owner rank
     */
    unsigned mOwnerRank;

    /**
     * Is this a Halo vessel
     */
    bool mIsHalo;

    /**
     * Is there are Halo on another processor corresponding to this vessel
     */
    bool mHasHalo;

    /**
     * Who owns the real vessel
     */
    unsigned mOtherProcessorRank;

    /**
     * For serialization only
     */
    Vessel();

    /**
     * Constructor. Kept private as the factory Create methods should be used instead.
     *
     * The vessel should always have at least one segment.
     * @param pSegment the input segment
     */
    Vessel(std::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     *
     * The vessel should always have at least one segment. This is useful for initializing with many segments at once.
     * @param segments a collection of segments, should be joined end to tip
     */
    Vessel(std::vector<std::shared_ptr<VesselSegment<DIM> > > segments);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     *
     * Initialize with a vector of nodes. The nodes are joined by segments in order. The ends are not closed.
     * @param nodes these nodes will be joined to form the vessel
     */
    Vessel(std::vector<std::shared_ptr<VesselNode<DIM> > > nodes);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     * Initialize with two nodes.
     * @param pStartNode the start node
     * @param pEndNode the end node
     */
    Vessel(std::shared_ptr<VesselNode<DIM> > pStartNode, std::shared_ptr<VesselNode<DIM> > pEndNode);

public:

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @param pSegment the input segment
     * @return a pointer to the vessel
     */
    static std::shared_ptr<Vessel<DIM> > Create(std::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @param segments a collection of segments, should be joined end to tip
     * @return a pointer to the vessel
     */
    static std::shared_ptr<Vessel<DIM> > Create(std::vector<std::shared_ptr<VesselSegment<DIM> > > segments);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @param nodes these nodes will be joined to form the vessel
     * @return a pointer to the vessel
     */
    static std::shared_ptr<Vessel<DIM> > Create(std::vector<std::shared_ptr<VesselNode<DIM> > > nodes);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @param pStartNode the start node
     * @param pEndNode the end node
     * @return a pointer to the vessel
     */
    static std::shared_ptr<Vessel<DIM> > Create(std::shared_ptr<VesselNode<DIM> > pStartNode,
                                                  std::shared_ptr<VesselNode<DIM> > pEndNode);

    /**
     * Destructor.
     */
    ~Vessel();

    /**
     * Add a single segment to either end of the vessel
     * @param pSegment the segment
     */
    void AddSegment(std::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Add a collection of segments to either end of the vessel
     * @param pSegments the segments
     */
    void AddSegments(std::vector<std::shared_ptr<VesselSegment<DIM> > > pSegments);

    /**
     * Copy the member data from the input vessel.
     * @param pTargetVessel the vessel to be copied from
     */
    void CopyDataFromExistingVessel(std::shared_ptr<Vessel<DIM> > pTargetVessel);

    /**
     * Divide the vessel at the specified location
     * @param rLocation the location of the division
     * @param distanceTolerance how far from a segment should the probe point be
     * @return the node at the division
     */
    std::shared_ptr<VesselNode<DIM> > DivideSegment(const DimensionalChastePoint<DIM>& rLocation,
                                                      double distanceTolerance = 1.e-6);

    /**
     * Return the dimensionless distance to the vessel end node closest to the input location
     * @param rLocation the location to probe
     * @return the distance to the closest end node
     */
    QLength GetClosestEndNodeDistance(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Return the distance from the vessel to the input location
     * @param rLocation the location to probe
     * @return the distance from the vessel to the input location
     */
    QLength GetDistance(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * @return vector of vessels connected to this one
     */
    std::vector<std::shared_ptr<Vessel<DIM> > > GetConnectedVessels();

    /**
     * @return shared pointer to the second node of the last segment
     */
    std::shared_ptr<VesselNode<DIM> > GetEndNode();

    /**
     * Return the flow properties of the component
     *
     * @return the flow properties of the component
     */
    std::shared_ptr<VesselFlowProperties<DIM> > GetFlowProperties() const;

    /**
     * @param pQueryNode the query node
     * @return shared pointer to the node at the opposite end of the vessel
     * to the supplied one.
     */
    std::shared_ptr<VesselNode<DIM> > GetNodeAtOppositeEnd(std::shared_ptr<VesselNode<DIM> > pQueryNode);

    /**
     * Return the length
     *
     * @return the length
     */
    QLength GetLength() const;

    /**
     * Return the radius
     *
     * @return the radius
     */
    QLength GetRadius() const;

    /**
     * Return the maturity
     *
     * @return the maturity
     */
    double GetMaturity() const;

    /**
     * Return the vessel node
     * @param index the query index
     * @return the vessel node
     */
    std::shared_ptr<VesselNode<DIM> > GetNode(unsigned index);

    /**
     * Return the vessel's nodes
     *
     * @return the vessel nodes
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > GetNodes();

    /**
     * Return a reference to the vessel node vector, avoids a copy
     *
     * @return a reference to the vessel node vector
     */
    const std::vector<std::shared_ptr<VesselNode<DIM> > >& rGetNodes();

    /**
     * Return the number of nodes in the vessel
     * @return unsigned the number of nodes
     */
    unsigned GetNumberOfNodes();

    /**
     * @return the number of segments
     */
    unsigned GetNumberOfSegments();

    /**
     * Return a map of vessel data for use by the vtk writer
     * @return a map of vessel data for use by the vtk writer
     */
    std::map<std::string, double> GetOutputData();

    /**
     * @param index the segment index to return
     * @return the indexed segment
     */
    std::shared_ptr<VesselSegment<DIM> > GetSegment(unsigned index);

    /**
     * @return all the vessel segments
     */
    std::vector<std::shared_ptr<VesselSegment<DIM> > > GetSegments();

    /**
     * @return the vessel stat node
     */
    std::shared_ptr<VesselNode<DIM> > GetStartNode();

    /**
     * Return the global index
     * @return the global index
     */
    unsigned GetGlobalIndex();

    /**
     * Return the local index
     * @return the local index
     */
    unsigned GetLocalIndex();

    /**
     * Return the owner rank
     * @return the owner rank
     */
    unsigned GetOwnerRank();

    /**
     * Is this a halo vessel
     * @return Is this a halo vessel
     */
    bool IsHalo();

    /**
     * Is there a halo on another processor
     * @return Is there a halo on another processor
     */
    bool HasHalo();

    /**
     * The rank of the processor storing the other vessel
     * @return the rank of the processor storing the other vessel
     */
    unsigned GetOtherProcessorRank();

    /**
     * The index of the other vessel on the other processor
     * @return the index of the other vessel on the other processor
     */
    unsigned GetOtherProcessorLocalIndex();

    /**
     * @param pOtherVessel the other vessel to check for connect
     * @return whether the vessel is connected to another vessel.
     */
    bool IsConnectedTo(std::shared_ptr<Vessel<DIM> > pOtherVessel);

    /**
     * Remove the vessel from all its segments
     */
    void Remove();

    /**
     * Remove segments from the ends of a vessel
     * @param location which end to remove from
     */
    void RemoveSegments(SegmentLocation::Value location);

    /**
     * Set the radius
     * @param radius the radius
     */
    void SetRadius(QLength radius);

    /**
     * Set the flow properties of the vessel
     *
     * @param rFlowProperties the flow properties to be set
     */
    void SetFlowProperties(const VesselFlowProperties<DIM>& rFlowProperties);

    /**
     * Set the global index
     * @param index the global index
     */
    void SetGlobalIndex(unsigned index);

    /**
     * Set the local index
     * @param index the local index
     */
    void SetLocalIndex(unsigned index);

    /**
     * Set the owner rank
     * @param rank the owner rank
     */
    void SetOwnerRank(unsigned rank);

    /**
     * Set is this a halo vessel
     * @param isHalo Is this a halo vessel
     */
    void SetIsHalo(bool isHalo);

    /**
     * Set is there a halo on another processor
     * @param hasHalo Is there a halo on another processor
     */
    void SetHasHalo(bool hasHalo);

    /**
     * Set the rank of the processor storing the other vessel
     * @param otherRank the rank of the processor storing the other vessel
     */
    void SetOtherProcessorRank(unsigned otherRank);

    /**
     * Set the index of the other vessel on the other processor
     * @param otherIndex the index of the other vessel on the other processor
     */
    void SetOtherProcessorLocalIndex(unsigned otherIndex);

    /**
     * Update the data in mNodes
     */
    void UpdateNodes();

private:

    /**
     * @return std::shared_ptr<Vessel<DIM> >
     */
    std::shared_ptr<Vessel<DIM> > Shared();
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(Vessel, 2)
EXPORT_TEMPLATE_CLASS1(Vessel, 3)

#endif /* VESSEL_HPP_ */
