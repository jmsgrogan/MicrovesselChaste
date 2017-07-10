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

#ifndef VESSELSEGMENT_HPP_
#define VESSELSEGMENT_HPP_

#include <memory>
#include <vector>
#include <string>
#include "ChasteSerialization.hpp"
#include "SegmentFlowProperties.hpp"
#include "UblasVectorInclude.hpp"
#include "ChastePoint.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponent.hpp"
#include "DimensionalChastePoint.hpp"
#include "Exception.hpp"

 // Forward declaration to allow vessels to manage adding and removing themselves from segments and segment management by vessels.
template<unsigned DIM>
class Vessel;

template<unsigned DIM>
class VesselNode;

/**
 * This is a class for vessel segments. They are components of a vessel network. Vessel segments are straight sub-units of vessels, defined by the positions of
 * their end nodes. Nodes cannot be created by the vessel segment class, they are
 * instead managed by the VesselNetwork class. Segments must always have two nodes.
 */
template<unsigned DIM>
class VesselSegment : public std::enable_shared_from_this<VesselSegment<DIM> >, public AbstractVesselNetworkComponent<DIM>
{
    /**
     * Allow vessels to manage adding and removing themselves from segments.
     */
    friend class Vessel<DIM> ;

private:

    /**
     * Archiving.
     */
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
            ar & mMaturity;
            ar & mNodes;
            ar & mpFlowProperties;
        #endif
    }

    /**
     * Container for segment nodes
     */
    std::pair<std::shared_ptr<VesselNode<DIM> >, std::shared_ptr<VesselNode<DIM> > > mNodes;

    /**
     * Weak pointer to the vessel owning this segment
     */
    std::weak_ptr<Vessel<DIM> > mVessel;

    /**
     * A flow property collection for the segment
     */
    std::shared_ptr<SegmentFlowProperties<DIM> > mpFlowProperties;

    /**
     * A measure of vessel maturity
     */
    double mMaturity;

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
     * Is this a Halo segment
     */
    bool mIsHalo;

    /**
     * Is there are Halo on another processor corresponding to this segment
     */
    bool mHasHalo;

    /**
     * Who owns the real segment
     */
    unsigned mOtherProcessorRank;

    /**
     * What is the segment index on the other processor
     */
    unsigned mOtherProcessorLocalIndex;

    /**
     * For serialization only
     */
    VesselSegment();

    /**
     * Constructor - This is private as instances of this class must be created with a corresponding shared pointer. This is
     * implemented using the static Create method.
     *
     * @param pNode1 the first node in the segment
     * @param pNode2 the second node in the segment
     */
    VesselSegment(std::shared_ptr<VesselNode<DIM> > pNode1, std::shared_ptr<VesselNode<DIM> > pNode2);

public:

    /**
     * Copy Constructor - This should not be used directly as instances of this class must be created with a corresponding shared pointer. This is
     * implemented using the static Create method. This class can not be made private.
     *
     * @param rSegment the segment to be copied
     */
    VesselSegment(const VesselSegment<DIM>& rSegment);

    /**
     * Construct a new instance of the class and return a shared pointer to it. Also manage the association of segments to nodes by
     * passing self weak pointers to the nodes.
     *
     * @param pNode1 the first node in the segment
     * @param pNode2 the second node in the segment
     * @return a pointer to the newly created segment
     */
    static std::shared_ptr<VesselSegment<DIM> > Create(std::shared_ptr<VesselNode<DIM> > pNode1,
                                                           std::shared_ptr<VesselNode<DIM> > pNode2);

    /**
     * Construct a new instance of the class and return a shared pointer to it. Also manage the association of segments to nodes by
     * passing self weak pointers to the nodes.
     *
     * @param pSegment the segment to be copied
     * @return a pointer to the newly created segment
     */
    static std::shared_ptr<VesselSegment<DIM> > Create(std::shared_ptr<VesselSegment<DIM> > pSegment);

    /*
     * Destructor
     */
    ~VesselSegment();

    /**
     * Copy a selection of member data and VasculatureData from the input segment. Convenient alternative to the copy constructor as nodes aren't
     * copied.
     * @param pTargetSegment the segment from which data is to be copied
     */
    void CopyDataFromExistingSegment(const std::shared_ptr<VesselSegment<DIM> > pTargetSegment);

    /**
     * Return the segment data.
     * @return the segment data
     */
    std::map<std::string, double> GetOutputData();

    /**
     * Return the distance between the input point and the segment. If the projection of the
     * point is within the segment the distance is the perpendicular distance to the segment.
     * Otherwise it is the distance to the nearest node.
     *
     * @param location the point the get the distance from
     * @return the distance to the segment
     */
    QLength GetDistance(const DimensionalChastePoint<DIM>& location) const;

    /**
     * Return the flow properties of the component
     *
     * @return the flow properties of the component
     */
    std::shared_ptr<SegmentFlowProperties<DIM> > GetFlowProperties() const;

    /**
     * Return the dimensional length
     *
     * @return the segment length
     */
    QLength GetLength() const;

    /**
     * Return a point mid-way along the vessel segment
     *
     * @return a point midway along the segment
     */
    DimensionalChastePoint<DIM> GetMidPoint() const;

    /**
     * Return the maturiy
     *
     * @return the maturiy
     */
    double GetMaturity() const;

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
     * Is this a halo segment
     * @return Is this a halo segment
     */
    bool IsHalo();

    /**
     * Is there a halo on another processor
     * @return Is there a halo on another processor
     */
    bool HasHalo();

    /**
     * The rank of the processor storing the other segment
     * @return the rank of the processor storing the other segment
     */
    unsigned GetOtherProcessorRank();

    /**
     * The index of the other segment on the other processor
     * @return the index of the other segment on the other processor
     */
    unsigned GetOtherProcessorLocalIndex();

    /**
     * Return a pointer to the node specified by the index
     * @param index the node index
     * @return a pointer to the node specified by the index
     */
    std::shared_ptr<VesselNode<DIM> > GetNode(unsigned index) const;

    /**
     * Return a pointer to the node on the other side of the segment
     *
     * @param pInputNode the node to get the opposite one to
     * @return a pointer to the node on the other side of the segment
     */
    std::shared_ptr<VesselNode<DIM> > GetOppositeNode(std::shared_ptr<VesselNode<DIM> > pInputNode) const;

    /**
     * Return the segment nodes as a pair
     *
     * @return the segment nodes as a pair
     */
    std::pair<std::shared_ptr<VesselNode<DIM> >, std::shared_ptr<VesselNode<DIM> > > GetNodes() const;

    /**
     * Return the projection of a point onto the segment. If the projection is outside the segment an
     * Exception is thrown.
     *
     * @param location the location to be projected
     * @param projectToEnds use end projection
     * @return the location of the projected point
     */
    DimensionalChastePoint<DIM> GetPointProjection(const DimensionalChastePoint<DIM>& location, bool projectToEnds = false) const;

    /**
     * Return a unit vector pointing along the segment. The orientation along the segment is from node0 to node 1.
     *
     * @return a unit vector pointing along the segment
     */
    c_vector<double, DIM> GetUnitTangent() const;

    /**
     * Return a pointer to the vessel.
     *
     * @return the vessel attached to the segment
     */
    std::shared_ptr<Vessel<DIM> > GetVessel() const;

    /**
     * Return whether the node is in the segment.
     * @param pNode the query node
     * @return whether the node is in the segment
     */
    bool HasNode(std::shared_ptr<VesselNode<DIM> > pNode) const;

    /**
     * Return whether the segment is connected to another segment.
     *
     * @param pOtherSegment the segment to check connectivity with
     * @return whether the segment is connected to the input segment
     */
    bool IsConnectedTo(std::shared_ptr<VesselSegment<DIM> > pOtherSegment) const;

    /**
     * Replace the node at the specified index with the passed in node.
     *
     * @param oldNodeIndex the index of the node to be replaced
     * @param pNewNode the node to be added to the segment
     */
    void ReplaceNode(unsigned oldNodeIndex, std::shared_ptr<VesselNode<DIM> > pNewNode);

    /**
     * Remove the segment from its nodes.
     */
    void Remove();

    /**
     * Set the flow properties of the segment
     *
     * @param rFlowProperties the flow properties to be set
     */
    void SetFlowProperties(const SegmentFlowProperties<DIM>& rFlowProperties);

    /**
     * Set the maturity
     * @param maturity the maturity
     */
    void SetMaturity(double maturity);

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
     * Set is this a halo segment
     * @param isHalo Is this a halo segment
     */
    void SetIsHalo(bool isHalo);

    /**
     * Set is there a halo on another processor
     * @param hasHalo Is there a halo on another processor
     */
    void SetHasHalo(bool hasHalo);

    /**
     * Set the rank of the processor storing the other segment
     * @param otherRank the rank of the processor storing the other segment
     */
    void SetOtherProcessorRank(unsigned otherRank);

    /**
     * Set the index of the other segment on the other processor
     * @param otherIndex the index of the other segment on the other processor
     */
    void SetOtherProcessorLocalIndex(unsigned otherIndex);


private:

    /**
     * Return a std::shared_ptr to this object
     *
     * @return a shared pointer to the segment
     */
    std::shared_ptr<VesselSegment<DIM> > Shared();

    /**
     * Add an adjoining Vessel to the segment.
     *
     * @param pVessel a vessel to be added to the segment
     */
    void AddVessel(std::shared_ptr<Vessel<DIM> > pVessel);

    /**
     * Remove an adjoining vessel from the segment.
     */
    void RemoveVessel();

};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(VesselSegment, 2)
EXPORT_TEMPLATE_CLASS1(VesselSegment, 3)


#endif /* VESSELSEGMENT_HPP_ */
