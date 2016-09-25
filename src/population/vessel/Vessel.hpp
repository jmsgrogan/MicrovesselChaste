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

#ifndef VESSEL_HPP_
#define VESSEL_HPP_

#include <vector>
#include <string>
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "VesselFlowProperties.hpp"

/**
 *  Struct to denote segment locations on the vessel
 */
struct SegmentLocation
{
    enum Value
    {
        Start, End
    };
};

/**
 * This is a class for vessels. A vessel is a component of a vessel network
 * .
 * Vessel are a collection of connected straight-line segments, i.e. a poly-line.
 * Vessel data and properties are derived from averaging or summing over their
 * segments as required.
 */
template<unsigned DIM>
class Vessel : public boost::enable_shared_from_this<Vessel<DIM> >, public AbstractVesselNetworkComponent<DIM>
{
private:

    /**
     *  Vessel segments
     */
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > mSegments;

    /**
     *  Nodes
     */
    std::vector<boost::shared_ptr<VesselNode<DIM> > > mNodes;

    /**
     *  Is the data in mNodes up to date.
     */
    bool mNodesUpToDate;

    /**
     * A flow property collection for the vessel
     */
    boost::shared_ptr<VesselFlowProperties<DIM> > mpFlowProperties;

    /**
     * Constructor. Kept private as the factory Create methods should be used instead.
     *
     * The vessel should always have at least one segment.
     * @param pSegment the input segment
     */
    Vessel(boost::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     *
     * The vessel should always have at least one segment. This is useful for initializing with many segments at once.
     * @param segments a collection of segments, should be joined end to tip
     */
    Vessel(std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     *
     * Initialize with a vector of nodes. The nodes are joined by segments in order. The ends are not closed.
     * @param nodes these nodes will be joined to form the vessel
     */
    Vessel(std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes);

    /**
     * Alternate Constructor. Kept private as the factory Create methods should be used instead.
     * Initialize with two nodes.
     * @param pStartNode the start node
     * @param pEndNode the end node
     */
    Vessel(boost::shared_ptr<VesselNode<DIM> > pStartNode, boost::shared_ptr<VesselNode<DIM> > pEndNode);

public:

    /*
     * Construct a new instance of the class and return a shared pointer to it.
     * @param pSegment the input segment
     */
    static boost::shared_ptr<Vessel<DIM> > Create(boost::shared_ptr<VesselSegment<DIM> > pSegment);

    /*
     * Construct a new instance of the class and return a shared pointer to it.
     * @param segments a collection of segments, should be joined end to tip
     */
    static boost::shared_ptr<Vessel<DIM> > Create(std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments);

    /*
     * Construct a new instance of the class and return a shared pointer to it.
     * @param nodes these nodes will be joined to form the vessel
     */
    static boost::shared_ptr<Vessel<DIM> > Create(std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes);

    /*
     * Construct a new instance of the class and return a shared pointer to it.
     * @param pStartNode the start node
     * @param pEndNode the end node
     */
    static boost::shared_ptr<Vessel<DIM> > Create(boost::shared_ptr<VesselNode<DIM> > pStartNode,
                                                  boost::shared_ptr<VesselNode<DIM> > pEndNode);

    /**
     * Destructor.
     */
    ~Vessel();

    /**
     * Add a single segment to either end of the vessel
     * @param pSegment the segment
     */
    void AddSegment(boost::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Add a collection of segments to either end of the vessel
     * @param pSegments the segments
     */
    void AddSegments(std::vector<boost::shared_ptr<VesselSegment<DIM> > > pSegments);

    /*
     * Copy the member data from the input vessel.
     * @param pTargetVessel the vessel to be copied from
     */
    void CopyDataFromExistingVessel(boost::shared_ptr<Vessel<DIM> > pTargetVessel);

    /**
     * Divide the vessel at the specified location
     * @param rLocation the location of the division
     * @param distanceTolerance how far from a segment should the probe point be
     */
    boost::shared_ptr<VesselNode<DIM> > DivideSegment(const DimensionalChastePoint<DIM>& rLocation,
                                                      double distanceTolerance = 1.e-6);

    /**
     * Return the dimensionless distance to the vessel end node closest to the input location
     * @param rLocation the location to probe
     */
    units::quantity<unit::length> GetClosestEndNodeDistance(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Return the distance from the vessel to the input location
     * @param rLocation the location to probe
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * @return vector of vessels connected to this one
     */
    std::vector<boost::shared_ptr<Vessel<DIM> > > GetConnectedVessels();

    /**
     * @return shared pointer to the second node of the last segment
     */
    boost::shared_ptr<VesselNode<DIM> > GetEndNode();

    /**
     * Return the flow properties of the component
     *
     * @return the flow properties of the component
     */
    boost::shared_ptr<VesselFlowProperties<DIM> > GetFlowProperties() const;

    /**
     * @return shared pointer to the node at the opposite end of the vessel
     * to the supplied one.
     */
    boost::shared_ptr<VesselNode<DIM> > GetNodeAtOppositeEnd(boost::shared_ptr<VesselNode<DIM> > pQueryNode);

    /**
     * Return the length
     *
     * @return the length
     */
    units::quantity<unit::length> GetLength() const;

    /**
     * Return the radius
     *
     * @return the radius
     */
    units::quantity<unit::length> GetRadius() const;

    /**
     * Return the vessel node
     *
     * @return the vessel node
     */
    boost::shared_ptr<VesselNode<DIM> > GetNode(unsigned index);

    /**
     * Return the vessel's nodes
     *
     * @return the vessel nodes
     */
    std::vector<boost::shared_ptr<VesselNode<DIM> > > GetNodes();

    /**
     * Return a reference to the vessel node vector, avoids a copy
     *
     * @return a reference to the vessel node vector
     */
    const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rGetNodes();

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
    boost::shared_ptr<VesselSegment<DIM> > GetSegment(unsigned index);

    /**
     * @return all the vessel segments
     */
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > GetSegments();

    /**
     * @return the vessel stat node
     */
    boost::shared_ptr<VesselNode<DIM> > GetStartNode();

    /**
     * @param pOtherVessel the other vessel to check for connect
     * @return whether the vessel is connected to another vessel.
     */
    bool IsConnectedTo(boost::shared_ptr<Vessel<DIM> > pOtherVessel);

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
     * Set the  radius
     */
    void SetRadius(units::quantity<unit::length>  radius);

    /**
     * Set the flow properties of the vessel
     *
     * @param rFlowProperties the flow properties to be set
     */
    void SetFlowProperties(const VesselFlowProperties<DIM>& rFlowProperties);

    /**
     * Update the data in mNodes
     */
    void UpdateNodes();


private:

    /**
     * @return boost::shared_ptr<Vessel<DIM> >
     */
    boost::shared_ptr<Vessel<DIM> > Shared();
};

#endif /* VESSEL_HPP_ */
