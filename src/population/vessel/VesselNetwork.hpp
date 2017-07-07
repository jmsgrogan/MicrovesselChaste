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

#ifndef VESSELNETWORK_HPP_
#define VESSELNETWORK_HPP_

#include <vector>
#include <set>
#include <map>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include "ChasteSerialization.hpp"
#include "Vessel.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DistributedVectorFactory.hpp"
#include "AbstractVesselNetworkComponent.hpp"

/**
 * Forward declare VTK members
 */
class vtkPolyData;
class vtkCellLocator;

/**
 * A vessel network is a collection of vessels. The network can be distributed over processors, in
 * which case only a portion of the network is stored. Vessel Nodes are divided geometrically amongst processors.
 */
template<unsigned DIM>
class VesselNetwork : public std::enable_shared_from_this<VesselNetwork<DIM> >, public AbstractVesselNetworkComponent<DIM>
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
            ar & mVessels;
        #endif
    }

    /**
     * Container for Vessels in the VesselNetwork.
     */
    std::vector<std::shared_ptr<Vessel<DIM> > > mVessels;

    /**
     * Container for Halo vessel segments in the VesselNetwork.
     */
    std::vector<std::shared_ptr<VesselSegment<DIM> > > mSegments;

    /**
     * Is the data in mSegments up to date.
     */
    bool mSegmentsUpToDate;

    /**
     * Container for nodes in the VesselNetwork.
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > mNodes;

    /**
     * Is the data in mNodes up to date.
     */
    bool mNodesUpToDate;

    /**
     * Container for vessel nodes in the VesselNetwork.
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > mVesselNodes;

    /**
     * Is the data in mVesselNodes up to date.
     */
    bool mVesselNodesUpToDate;

    /**
     * A vtk representation of the geometry. Used internally for
     * node and segment location. Indexing is according to
     * the indexing of mNodes and mSegments.
     */
    vtkSmartPointer<vtkPolyData> mpVtkGeometry;

    /**
     * A vtk cell locator for vessel segments. Indexing is according to
     * the indexing of mSegments.
     * node and segment location.
     */
    vtkSmartPointer<vtkCellLocator> mpVtkSegmentCellLocator;

    /**
     * Is the vtk geometry representation up to date.
     */
    bool mVtkGeometryUpToDate;

    /**
     * Used for solving systems on vessel nodes
     */
    std::shared_ptr<DistributedVectorFactory> mDistributedVectorFactory;

public:

    /**
     * Constructor.
     */
    VesselNetwork();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return share pointer to the network
     */
    static std::shared_ptr<VesselNetwork<DIM> > Create();

    /**
     * Destructor
     */
    virtual ~VesselNetwork();

    /**
     * Adds a vessel to the VesselNetwork.
     * @param pVessel the vessel
     */
    void AddVessel(std::shared_ptr<Vessel<DIM> > pVessel);

    /**
     * Adds a collection of vessels to the VesselNetwork
     * @param vessels the vessels
     */
    void AddVessels(std::vector<std::shared_ptr<Vessel<DIM> > > vessels);

    /**
     * Remove all vessels from the network, used for parallel partitioning
     */
    void ClearVessels();

    /**
     * Make a copy of all vessels, but with new nodes and segments in each copy. Return the new vessels.
     * @return the new vessels
     */
    std::vector<std::shared_ptr<Vessel<DIM> > > CopyVessels();

    /**
     * Make a copy of the selected vessels, but with new nodes and segments in each copy. Return the new vessels.
     * @param vessels the vessels to be copied
     * @return the new vessels
     */
    std::vector<std::shared_ptr<Vessel<DIM> > > CopyVessels(std::vector<std::shared_ptr<Vessel<DIM> > > vessels);

    /**
     * Divides a vessel into two at the specified location.
     * @param pVessel the vessel to be divided
     * @param rLocation the division location
     * @return the node at the division location
     */
    virtual std::shared_ptr<VesselNode<DIM> > DivideVessel(std::shared_ptr<Vessel<DIM> > pVessel,
                                                     const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Add a new node to the end of the vessel
     * @param pVessel the vessel to be extended
     * @param pEndNode the node that the new segment will start on, should already be on the end of the vessel
     * @param pNewNode the new node to be added to the end of the vessel
     */
    virtual void ExtendVessel(std::shared_ptr<Vessel<DIM> > pVessel, std::shared_ptr<VesselNode<DIM> > pEndNode,
                      std::shared_ptr<VesselNode<DIM> > pNewNode);

    /**
     * Forms a sprout at the specified locations.
     * @param sproutBaseLocation the sprout base
     * @param sproutTipLocation the sprout tip
     * @return the new sprout
     */
    virtual std::shared_ptr<Vessel<DIM> > FormSprout(std::shared_ptr<VesselNode<DIM> > pSproutBase,
                                               const DimensionalChastePoint<DIM>& sproutTipLocation);

    /**
     * Get index of the node
     * @param pNode the probe node
     * @return index of the node
     */
    unsigned GetNodeIndex(std::shared_ptr<VesselNode<DIM> > pNode);

    void SetDistributedVectorFactory(std::shared_ptr<DistributedVectorFactory>  vectorFactory);

    std::shared_ptr<DistributedVectorFactory> GetDistributedVectorFactory();

    /**
     * Return the indexed node in the network
     * This is dangerous as the node member array can be updated and this index will be out of date, use GetNodes instead.
     * @param index the node index
     * @return the node
     */
    std::shared_ptr<VesselNode<DIM> > GetNode(unsigned index);

    /**
     * Return the nodes in the network
     * @return all the network nodes
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > GetNodes();

    /**
     * Return the VTK representation of the network
     * @return the VTK representation of the network
     */
    vtkSmartPointer<vtkPolyData> GetVtk();

    /**
     * Return the VTK cell locator for segments
     * @return the VTK cell locator for segments
     */
    vtkSmartPointer<vtkCellLocator> GetVtkCellLocator();

    /**
     * Return the number of nodes in the network.
     * @return the number of nodes in the network.
     */
    unsigned GetNumberOfNodes();

    /**
     * Return the number of nodes in the network.
     * @return the number of nodes in the network.
     */
    std::vector<unsigned> GetNumberOfNodesPerProcess();

    /**
     * Return the number of vessel nodes in the network.
     * @return  the number of vessel nodes in the network.
     */
    unsigned GetNumberOfVesselNodes();

    /**
     * Return the number of vessels in the network.
     * @return the number of vessels in the network.
     */
    unsigned GetNumberOfVessels();

    /**
     * Return the number of branches on the most highly connected node
     * @return the number of branches on the most highly connected node
     */
    unsigned GetMaxBranchesOnNode();

    /**
     * Return a map of vessel network data for use by the vtk writer
     * @return a map of vessel network  data for use by the vtk writer
     */
    std::map<std::string, double> GetOutputData();

    /**
     * Return the only the nodes at the ends of vessels in the network
     * @return the nodes at the ends of vessels in the network
     */
    std::vector<std::shared_ptr<VesselNode<DIM> > > GetVesselEndNodes();

    /**
     * Return the Index of the specified vessel
     * @param pVessel the query vessel
     * @return the Index of the specified vessel
     */
    unsigned GetVesselIndex(std::shared_ptr<Vessel<DIM> > pVessel);

    /**
     * Return the Index of the specified vessel segment
     * @param pVesselSegment the query segment
     * @return the Index of the specified vessel segment
     */
    unsigned GetVesselSegmentIndex(std::shared_ptr<VesselSegment<DIM> > pVesselSegment);

    /**
     * Return the vessel segments in the network
     * @return the vessel segments in the network
     */
    std::vector<std::shared_ptr<VesselSegment<DIM> > > GetVesselSegments();

    std::shared_ptr<VesselSegment<DIM> > GetVesselSegment(unsigned index);

    /**
     * Return the indexed vessel
     * @param index the query index
     * @return the indexed vessel
     */
    std::shared_ptr<Vessel<DIM> > GetVessel(unsigned index);

    /**
     * Return the vessels in the network
     * @return the vessels in the network
     */
    std::vector<std::shared_ptr<Vessel<DIM> > > GetVessels();

    /**
     * Return whether node is in network.
     * @param pSourceNode the node
     * @return is the node in the network
     */
    bool NodeIsInNetwork(std::shared_ptr<VesselNode<DIM> > pSourceNode);

    /**
     * Merge short vessels in the network
     * @param cutoff how short is short
     */
    void MergeShortVessels(QLength cutoff = 10.0 * 1_um);

    /**
     * Merge nodes with the same spatial location. Useful for
     * tidying up networks read from file.
     * @param tolerance how close together are nodes for them to be coincident
     */
    void MergeCoincidentNodes(double tolerance = 0.0);

    /**
     * Merge nodes with the same spatial location. Useful for
     * tidying up networks read from file.
     * @param pVessels the vessels for merging
     * @param tolerance how close together are nodes for them to be coincident
     */
    void MergeCoincidentNodes(std::vector<std::shared_ptr<Vessel<DIM> > > pVessels, double tolerance = 0.0);

    /**
     * Merge nodes with the same spatial location. Useful for
     * tidying up networks read from file.
     * @param nodes the nodes to merge
     * @param tolerance how close together are nodes for them to be coincident
     */
    void MergeCoincidentNodes(std::vector<std::shared_ptr<VesselNode<DIM> > > nodes, double tolerance = 0.0);

    /**
     * Convenience method to signify when node, segment and vessel storage have gone out of date.
     */
    void Modified(bool nodesOutOfDate=true, bool segmentsOutOfDate=true, bool vesselsOutOfDate=true);

    /**
     * Removes a vessel from the network
     * @param pVessel the vessel to remove
     * @param deleteVessel also remove the vessel from its child segments and nodes if true.
     */
    void RemoveVessel(std::shared_ptr<Vessel<DIM> > pVessel, bool deleteVessel = false);

    /**
     * Remove short vessels from the network
     * @param cutoff the minumum vessel length
     * @param endsOnly just remove vessels with connectivity 1
     */
    void RemoveShortVessels(QLength cutoff = 10.0* 1_um, bool endsOnly = true);

    /**
     * Translate the network along the provided vector
     * @param rTranslationVector the translation vector
     */
    void Translate(DimensionalChastePoint<DIM> rTranslationVector);

    /**
     * Translate specific vessels along the provided vector
     * @param rTranslationVector the translation vector
     * @param vessels the vessels to translate
     */
    void Translate(DimensionalChastePoint<DIM> rTranslationVector, std::vector<std::shared_ptr<Vessel<DIM> > > vessels);

    /**
     * Update the network node collection
     */
    void UpdateNodes();

    /**
     * Update the network segment collection
     */
    void UpdateSegments();

    /**
     * Update the network vessel collection
     */
    void UpdateVesselNodes();

    /**
     * Update the vessel id tags
     */
    void UpdateVesselIds();

    /**
     * Update the internal vtk geometry
     */
    void UpdateInternalVtkGeometry();

    /**
     * Update all dynamic storage in the vessel network, optionally merge coincident nodes
     * @param merge whether to merge co-incident nodes
     */
    void UpdateAll(bool merge=false);

    /**
     * Write the network to file
     * @param rFileName the filename
     */
    void Write(const std::string& rFileName, bool masterOnly=true);

};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(VesselNetwork, 2)
EXPORT_TEMPLATE_CLASS1(VesselNetwork, 3)

#endif /* VESSELNETWORK_HPP_ */
