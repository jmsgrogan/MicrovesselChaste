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

#ifndef VESSELNODE_HPP_
#define VESSELNODE_HPP_

#include <vector>
#include <string>
#include <map>
#include "ChasteSerialization.hpp"
#include "SmartPointers.hpp"
#include "Vertex.hpp"
#include "AbstractVesselNetworkComponent.hpp"
#include "NodeFlowProperties.hpp"
#include "NodeChemicalProperties.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "VectorUnitCollection.hpp"
#include "VesselSegment.hpp"
#include "Exception.hpp"

/**
 * Forward declaration to allow segments to manage adding and removing themselves from nodes.
 */
template<unsigned DIM>
class VesselSegment;

/**
 * This is a class for vessel nodes, which are vessel network components.
 *
 * Nodes are point locations along a vessel. They are used for describing the end positions of
 * straight line vessel segments.
 */
template<unsigned DIM>
class VesselNode : public std::enable_shared_from_this<VesselNode<DIM> >, public AbstractVesselNetworkComponent<DIM>
{
private:

    /**
     * Archiving. Note that mSegments is not serialized here. Segments should add or remove themselves
     * from nodes when serializing/de-serializing. It is not managed by nodes.
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
            ar & mLocation;
            ar & mIsMigrating;
            ar & mpFlowProperties;
            ar & mPtrComparisonId;
        #endif
    }

    /**
     * Allow segments to manage adding and removing themselves from nodes.
     */
    friend class VesselSegment<DIM> ;

    /**
     * Location of a node in space.
     */
    Vertex<DIM> mLocation;

    /**
     * Collection of pointers to Vessel Segments connected to this node.
     */
    std::vector<std::weak_ptr<VesselSegment<DIM> > > mSegments;

    /**
     * Is the vessel allowed to extend at this node
     */
    bool mIsMigrating;

    /**
     * A flow property collection for the node
     */
    std::shared_ptr<NodeFlowProperties<DIM> > mpFlowProperties;

    /**
     * A chemical property collection for the node
     */
    std::shared_ptr<NodeChemicalProperties<DIM> > mpChemicalProperties;

    /**
     * This is used for comparing nodes in some VesselNetwork methods.
     * It should not be used otherwise.
     */
    unsigned mPtrComparisonId;

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
     * Is this a Halo node
     */
    bool mIsHalo;

    /**
     * Is there are Halo on another processor corresponding to this node
     */
    bool mHasHalo;

    /**
     * Who owns the real node
     */
    unsigned mOtherProcessorRank;

    /**
     * What is the node index on the other processor
     */
    unsigned mOtherProcessorLocalIndex;

public:

    /**
     * Constructor.
     * Create a node using xyz coordinates
     *
     * @param v1  the node's x-coordinate
     * @param v2  the node's y-coordinate
     * @param v3  the node's z-coordinate
     * @param referenceLength the reference length scale, defaults to micron
     */
    VesselNode(QLength v1 = 0_m, QLength v2 = 0_m, QLength v3 = 0_m);

    /**
     * Constructor.
     * Create a node using ublas c_vector
     *
     * @param location the node's location (defaults to 0.0)
     */
    VesselNode(const Vertex<DIM>& rLocation);

    /**
     * Copy constructor.
     * @param rExistingNode the node to copy from
     */
    VesselNode(const VesselNode<DIM>& rExistingNode);

    /**
     * Destructor
     */
    ~VesselNode();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @param v1  the node's x-coordinate (defaults to 0 micron)
     * @param v2  the node's y-coordinate (defaults to 0 micron)
     * @param v3  the node's z-coordinate (defaults to 0 micron)
     * @return a pointer to the newly created node
     */
    static std::shared_ptr<VesselNode<DIM> > Create(QLength v1, QLength v2 = 0_m, QLength v3 = 0_m);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @param location the node's location (defaults to 0.0  micron)
     * @return a pointer to the newly created node
     */
    static std::shared_ptr<VesselNode<DIM> > Create(const Vertex<DIM>& location);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @param rExistingNode the node to copy from
     * @return a pointer to the newly created node
     */
    static std::shared_ptr<VesselNode<DIM> > Create(const VesselNode<DIM>& rExistingNode);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @param pExistingNode the node to copy from
     * @return a pointer to the newly created node
     */
    static std::shared_ptr<VesselNode<DIM> > Create(std::shared_ptr<VesselNode<DIM> > pExistingNode);

    /**
     * Return the Id for comparing pointer contents
     * @return the node id
     */
    unsigned GetComparisonId();

    /**
     * Return the distance between the input location and the node
     *
     * @param rLocation the location to calculate the distance to
     * @return the distance to the location
     */
    QLength GetDistance(const Vertex<DIM>& rLocation) const;

    /**
     * Return the flow properties of the component
     *
     * @return the flow properties of the component
     */
    std::shared_ptr<NodeFlowProperties<DIM> > GetFlowProperties() const;

    /**
     * Return the chemical properties of the component
     *
     * @return the chemical properties of the component
     */
    std::shared_ptr<NodeChemicalProperties<DIM> > GetChemicalProperties() const;

    /**
     * Return a reference to the location of the node
     *
     * @return a vector at the location of the node
     */
    const Vertex<DIM>& rGetLocation() const;

    /**
     * Return the number of attached segments
     *
     * @return the number of segments attached to the node
     */
    unsigned GetNumberOfSegments() const;

    /**
     * Return a map of output data for writers
     *
     * @return a map of component data for use by the vtk writer
     */
    std::map<std::string, double> GetOutputData();

    /**
     * Return a pointer to the indexed vessel segment
     * @param index the segment index
     * @return a vector of pointers to the attached vessel segments
     */
    std::shared_ptr<VesselSegment<DIM> > GetSegment(unsigned index) const;

    /**
     * Return a vector of pointers to the attached vessel segments.
     *
     * @return a vector of pointers to the attached vessel segments
     */
    std::vector<std::shared_ptr<VesselSegment<DIM> > > GetSegments() const;

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
     * Is this a halo node
     * @return Is this a halo node
     */
    bool IsHalo();

    /**
     * Is there a halo on another processor
     * @return Is there a halo on another processor
     */
    bool HasHalo();

    /**
     * The rank of the processor storing the other node
     * @return the rank of the processor storing the other node
     */
    unsigned GetOtherProcessorRank();

    /**
     * The index of the other node on the other processor
     * @return the index of the other node on the other processor
     */
    unsigned GetOtherProcessorLocalIndex();

    /**
     * Return true if the input segment is attached to the node
     *
     * @param pSegment a pointer to the segment to query
     * @return whether the input segment is attached to the node
     */
    bool IsAttachedTo(const std::shared_ptr<VesselSegment<DIM> > pSegment) const;

    /**
     * Return true if the node is coincident with the input location
     *
     * @param rLocation the query location
     * @return whether then node is coincident with the input location
     */
    bool IsCoincident(const Vertex<DIM>& rLocation) const;

    /**
     * Has the node been designated as migrating. This is useful for keeping track of
     * nodes in angiogenesis simulations.
     *
     * @return whether the node is marked as migrating.
     */
    bool IsMigrating() const;

    /**
     * Set the id used for comparing the contents of pointers. This is only used
     * by certain methods in the VesselNetwork class. Use SetId for generic node
     * labelling.
     *
     * @param id for node comparison
     */
    void SetComparisonId(unsigned id);

    /**
     * Set the flow properties of the node
     *
     * @param rFlowProperties the flow properties to be set
     */
    void SetFlowProperties(const NodeFlowProperties<DIM>& rFlowProperties);

    /**
     * Set the flow properties of the node
     *
     * @param rFlowProperties the flow properties to be set
     */
    void SetChemicalProperties(const NodeChemicalProperties<DIM>& rChemicalProperties);

    /**
     * Set that the node is migrating
     *
     * @param isMigrating whether the node is migrating
     */
    void SetIsMigrating(bool isMigrating);

    /**
     * Set the location of the node. It is assumed that this location is consistent
     * with the nodes reference length scale, default is micron.
     *
     * @param rLocation a ublas c_vector specifying the location
     */
    void SetLocation(const Vertex<DIM>& rLocation);

    /**
     * Set the location of the node. It is assumed that this location is consistent
     * with the nodes reference length scale, default is micron.
     *
     * @param x the x location
     * @param y the y location
     * @param z the z location
     * @param referenceLength the reference length scale
     */
    void SetLocation(QLength v1, QLength v2 = 0_m, QLength v3 = 0_m);

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
     * Set is this a halo node
     * @param isHalo Is this a halo node
     */
    void SetIsHalo(bool isHalo);

    /**
     * Set is there a halo on another processor
     * @param hasHalo Is there a halo on another processor
     */
    void SetHasHalo(bool hasHalo);

    /**
     * Set the rank of the processor storing the other node
     * @param otherRank the rank of the processor storing the other node
     */
    void SetOtherProcessorRank(unsigned otherRank);

    /**
     * Set the index of the other node on the other processor
     * @param otherIndex the index of the other node on the other processor
     */
    void SetOtherProcessorLocalIndex(unsigned otherIndex);

private:

    /**
     * Add a vessel segment the node. Private because node-segment connectivity needs to be managed.
     *
     *  @param pVesselSegment the segment to be added
     */
    void AddSegment(std::shared_ptr<VesselSegment<DIM> > pVesselSegment);

    /**
     * Remove a vessel segment from the node. Private because node-segment connectivity needs to be managed.
     *
     *  @param pVesselSegment the segment to be removed
     */
    void RemoveSegment(std::shared_ptr<VesselSegment<DIM> > pVesselSegment);
};

/**
 * Convenience typedef
 */
template<unsigned DIM>
using VesselNodePtr = std::shared_ptr<VesselNode<DIM> >;

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(VesselNode, 2)
EXPORT_TEMPLATE_CLASS1(VesselNode, 3)
#endif /* VESSELNODE_HPP_ */
