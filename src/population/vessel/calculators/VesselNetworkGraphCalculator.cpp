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

#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include "Exception.hpp"
#include "VesselNetworkGraphCalculator.hpp"
#include "PetscTools.hpp"

/**
 * Helper class for "connected" methods
 */
template<typename TimeMap> class bfs_time_visitor : public boost::default_bfs_visitor
{
    /**
     * Value typedef
     */
    typedef typename boost::property_traits<TimeMap>::value_type T;

public:

    /**
     * A time map
     */
    TimeMap m_timemap;

    /**
     * Discover time
     */
    T& m_time;

    /**
     * BFS search visitor
     * @param tmap the time map
     * @param t the discovery time.
     */
    bfs_time_visitor(TimeMap tmap, T& t)
    :m_timemap(tmap),
     m_time(t)
    {
    }

    /**
     * Vertex discovery
     * @param u the vertex
     * @param g the graph
     */
    template<typename Vertex, typename Graph>
    void discover_vertex(Vertex u, const Graph& g) const
    {
        put(m_timemap, u, m_time++);
    }
};

template <unsigned DIM>
VesselNetworkGraphCalculator<DIM>::VesselNetworkGraphCalculator() :
	mpVesselNetwork()
{

}

template <unsigned DIM>
VesselNetworkGraphCalculator<DIM>::~VesselNetworkGraphCalculator()
{

}

template <unsigned DIM>
std::shared_ptr<VesselNetworkGraphCalculator<DIM> > VesselNetworkGraphCalculator<DIM>::Create()
{
    return std::make_shared<VesselNetworkGraphCalculator<DIM> >();

}

template <unsigned DIM>
std::vector<std::vector<unsigned> > VesselNetworkGraphCalculator<DIM>::GetNodeNodeConnectivity()
{
	if(!mpVesselNetwork)
	{
		EXCEPTION("Vessel network not set in graph calculator");
	}

    std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = mpVesselNetwork->GetVesselEndNodes();
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
    std::vector<std::vector<unsigned> > node_vessel_connectivity = GetNodeVesselConnectivity();

    std::vector<std::vector<unsigned> > connectivity;
    for (unsigned node_index = 0; node_index < nodes.size(); node_index++)
    {
        std::vector<unsigned> node_indexes;
        std::shared_ptr<VesselNode<DIM> > p_node = nodes[node_index];
        unsigned num_branches = node_vessel_connectivity[node_index].size();
        for (unsigned vessel_index = 0; vessel_index < num_branches; vessel_index++)
        {
            std::shared_ptr<Vessel<DIM> > p_vessel = vessels[node_vessel_connectivity[node_index][vessel_index]];

            // Get the node at the other end of the vessel
            std::shared_ptr<VesselNode<DIM> > p_other_node = p_vessel->GetNodeAtOppositeEnd(p_node);
            typename std::vector<std::shared_ptr<VesselNode<DIM> > >::iterator node_iter = std::find(nodes.begin(), nodes.end(), p_other_node);
            unsigned other_node_index = std::distance(nodes.begin(), node_iter);
            node_indexes.push_back(other_node_index);
        }
        connectivity.push_back(node_indexes);
    }
    return connectivity;
}

template <unsigned DIM>
std::vector<std::vector<unsigned> > VesselNetworkGraphCalculator<DIM>::GetNodeVesselConnectivity()
{
	if(!mpVesselNetwork)
	{
		EXCEPTION("Vessel network not set in graph calculator");
	}

    std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = mpVesselNetwork->GetVesselEndNodes();
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
    unsigned num_nodes = nodes.size();
    std::vector<std::vector<unsigned> > connectivity;
    for (unsigned node_index = 0; node_index < num_nodes; node_index++)
    {
        std::shared_ptr<VesselNode<DIM> > p_node = nodes[node_index];
        std::vector<unsigned> vessel_indexes;

        unsigned num_segments_on_node = p_node->GetNumberOfSegments();
        for (unsigned segment_index = 0; segment_index < num_segments_on_node; segment_index++)
        {
            std::shared_ptr<Vessel<DIM> > p_vessel = p_node->GetSegments()[segment_index]->GetVessel();

            typename std::vector<std::shared_ptr<Vessel<DIM> > >::iterator vessel_iter =
                    std::find(vessels.begin(), vessels.end(), p_vessel);
            unsigned vessel_index = std::distance(vessels.begin(), vessel_iter);
            vessel_indexes.push_back(vessel_index);
        }
        connectivity.push_back(vessel_indexes);
    }
    return connectivity;
}

template <unsigned DIM>
bool VesselNetworkGraphCalculator<DIM>::IsConnected(std::shared_ptr<VesselNode<DIM> > pSourceNode, std::shared_ptr<VesselNode<DIM> > pQueryNode)
{
	if(!mpVesselNetwork)
	{
		EXCEPTION("Vessel network not set in graph calculator");
	}

    if (!mpVesselNetwork->NodeIsInNetwork(pSourceNode))
    {
        EXCEPTION("Source node is not in network.");
    }
    if (!mpVesselNetwork->NodeIsInNetwork(pQueryNode))
    {
        EXCEPTION("Query node is not in network.");
    }

    if (pSourceNode == pQueryNode)
    {
        return true;
    }

    std::shared_ptr<Vessel<DIM> > p_source_vessel = pSourceNode->GetSegments()[0]->GetVessel();
    std::shared_ptr<Vessel<DIM> > p_query_vessel = pQueryNode->GetSegments()[0]->GetVessel();

    if (p_source_vessel == p_query_vessel || p_source_vessel->IsConnectedTo(p_query_vessel))
    {
        return true;
    }

    // Assign the vessel nodes unique IDs
    std::vector<std::shared_ptr<VesselNode<DIM> > >  vessel_nodes = mpVesselNetwork->GetVesselEndNodes();
    typename std::vector<std::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    unsigned counter = 0;
    for(node_iter = vessel_nodes.begin(); node_iter != vessel_nodes.end(); node_iter++)
    {
        (*node_iter)->SetId(counter);
        counter ++;
    }

    // Get the start nodes of the containing and query vessels
    std::shared_ptr<VesselNode<DIM> > pEquivalentSourceNode = p_source_vessel->GetStartNode();
    std::shared_ptr<VesselNode<DIM> > pEquivalentQueryNode = p_query_vessel->GetStartNode();

    // construct graph representation of vessel network
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

    Graph G;

    std::vector<std::shared_ptr<Vessel<DIM> > >  vessels = mpVesselNetwork->GetVessels();
    for (unsigned i = 0; i < vessels.size(); i++)
    {
        add_edge(vessels[i]->GetStartNode()->GetId(), vessels[i]->GetEndNode()->GetId(), G);
    }

    // typedefs
    typedef boost::graph_traits<Graph>::vertices_size_type Size;

    // a vector to hold the discover time property for each vertex
    std::vector<Size> dtime(num_vertices(G));

    Size time = 0;
    bfs_time_visitor<Size*>vis(&dtime[0], time);

    // use breadth first search to establish discovery time of all nodes from node1
    // this assigns a discovery time to dTime for each node (index relates to nodeID)
    // dTime is zero for node1 and all other nodes that are not connected to node1
    // dTime is nonzero for all nodes that are connected to node1 (except node 1 itself)
    breadth_first_search(G,vertex(pEquivalentSourceNode->GetId(),G), boost::visitor(vis));

    return (dtime[pEquivalentQueryNode->GetId()] > 0);
}

template <unsigned DIM>
std::vector<bool > VesselNetworkGraphCalculator<DIM>::IsConnected(std::vector<std::shared_ptr<VesselNode<DIM> > > sourceNodes,
        std::vector<std::shared_ptr<VesselNode<DIM> > > queryNodes)
{
	if(!mpVesselNetwork)
	{
		EXCEPTION("Vessel network not set in graph calculator");
	}

    // Assign the vessel nodes unique IDs
    std::vector<std::shared_ptr<VesselNode<DIM> > >  vessel_nodes = mpVesselNetwork->GetVesselEndNodes();

    typename std::vector<std::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    unsigned counter = 0;
    for(node_iter = vessel_nodes.begin(); node_iter != vessel_nodes.end(); node_iter++)
    {
        (*node_iter)->SetId(counter);
        counter ++;
    }

    // construct graph representation of vessel network
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
    typedef boost::graph_traits<Graph>::vertices_size_type Size;

    Graph G;

    std::vector<std::shared_ptr<Vessel<DIM> > >  vessels = mpVesselNetwork->GetVessels();
    for (unsigned i = 0; i < vessels.size(); i++)
    {
        add_edge(vessels[i]->GetStartNode()->GetId(), vessels[i]->GetEndNode()->GetId(), G);
    }

    std::vector<bool > connected(queryNodes.size(),false);

    for(unsigned i=0; i<sourceNodes.size(); i++)
    {
        if (!mpVesselNetwork->NodeIsInNetwork(sourceNodes[i]))
        {
            EXCEPTION("Source node is not in network.");
        }

        std::shared_ptr<VesselNode<DIM> > pSourceNode = sourceNodes[i];
        std::shared_ptr<Vessel<DIM> > p_source_vessel = pSourceNode->GetSegments()[0]->GetVessel();
        std::shared_ptr<VesselNode<DIM> > pEquivalentSourceNode = p_source_vessel->GetStartNode();

        // a vector to hold the discover time property for each vertex
        std::vector<Size> dtime(num_vertices(G));

        Size time = 0;
        bfs_time_visitor<Size*>vis(&dtime[0], time);

        breadth_first_search(G,vertex(pEquivalentSourceNode->GetId(),G), boost::visitor(vis));

        for (unsigned j=0; j<queryNodes.size(); j++)
        {
            if (!mpVesselNetwork->NodeIsInNetwork(queryNodes[j]))
            {
                EXCEPTION("Query node is not in network.");
            }

            std::shared_ptr<VesselNode<DIM> > pQueryNode = queryNodes[j];

            if (pSourceNode == pQueryNode)
            {
                connected[j] = true;
                continue;
            }

            std::shared_ptr<Vessel<DIM> > p_query_vessel = pQueryNode->GetSegments()[0]->GetVessel();
            if (p_source_vessel == p_query_vessel || p_source_vessel->IsConnectedTo(p_query_vessel))
            {
                connected[j] = true;
                continue;
            }

            std::shared_ptr<VesselNode<DIM> > pEquivalentQueryNode = p_query_vessel->GetStartNode();

            if (dtime[pEquivalentQueryNode->GetId()] > 0)
            {
                connected[j] = true;
            }
        }
    }

    return connected;
}

template <unsigned DIM>
void VesselNetworkGraphCalculator<DIM>::WriteConnectivity(const std::string& output_filename)
{
	if(!mpVesselNetwork)
	{
		EXCEPTION("Vessel network not set in graph calculator");
	}

    // construct graph representation of vessel network
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> G;
    typename std::vector<std::shared_ptr<VesselNode<DIM> > >::iterator node_iterator;
    std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = mpVesselNetwork->GetVesselEndNodes();

    for (node_iterator = nodes.begin(); node_iterator != nodes.end(); node_iterator++)
    {
        if ((*node_iterator)->GetSegments().size() > 1)
        {
            for (unsigned j = 1; j < (*node_iterator)->GetSegments().size(); j++)
            {
                add_edge(mpVesselNetwork->GetVesselIndex((*node_iterator)->GetSegments()[0]->GetVessel()),
                         mpVesselNetwork->GetVesselIndex((*node_iterator)->GetSegments()[j]->GetVessel()), G);
            }
        }
    }

    std::vector<std::shared_ptr<Vessel<DIM> > >  vessels = mpVesselNetwork->GetVessels();
    typename std::vector<std::shared_ptr<Vessel<DIM> > >::iterator vessel_iterator;
    for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
    {
        if ((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1 && (*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
        {
            add_edge(mpVesselNetwork->GetVesselIndex((*vessel_iterator)),
                     mpVesselNetwork->GetVesselIndex((*vessel_iterator)), G);
        }
    }

    if(PetscTools::AmMaster())
    {
        std::ofstream outf(output_filename.c_str());
        boost::dynamic_properties dp;
        dp.property("node_id", get(boost::vertex_index, G));
        write_graphviz_dp(outf, G, dp);
    }
}

template <unsigned DIM>
void VesselNetworkGraphCalculator<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    mpVesselNetwork = pVesselNetwork;
}

// Explicit instantiation
template class VesselNetworkGraphCalculator<2>;
template class VesselNetworkGraphCalculator<3>;

