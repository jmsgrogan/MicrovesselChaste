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

#include <algorithm>
#include "VesselNetworkPartitioner.hpp"
#include "Exception.hpp"
#include "PetscTools.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

template<unsigned DIM>
VesselNetworkPartitioner<DIM>::VesselNetworkPartitioner() :
    mpNetwork(),
    mUseSimpleGeometricPartition(true),
    mParitionAxis(0)
{

}

template<unsigned DIM>
VesselNetworkPartitioner<DIM>::~VesselNetworkPartitioner()
{
}

template<unsigned DIM>
void VesselNetworkPartitioner<DIM>::SetUseSimpleGeometricPartitioning(bool useSimple)
{
    mUseSimpleGeometricPartition = useSimple;
}

template<unsigned DIM>
void VesselNetworkPartitioner<DIM>::SetPartitionAxis(unsigned partitionAxis)
{
    mParitionAxis = partitionAxis;
}

template<unsigned DIM>
void VesselNetworkPartitioner<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void VesselNetworkPartitioner<DIM>::Update()
{
    if(!mpNetwork)
    {
        EXCEPTION("Vessel network not set in partitioner.");
    }
    if(PetscTools::IsSequential())
    {
        return;
    }

    // Partition extents
    unsigned rank = PetscTools::GetMyRank();
    unsigned num_procs = PetscTools::GetNumProcs();

    std::pair<Vertex<DIM>, Vertex<DIM> > extents = VesselNetworkGeometryCalculator<DIM>::GetExtents(mpNetwork);
    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    double domain_width = 0.0;
    double domain_start = 0.0;
    double delta_x = extents.second.Convert(reference_length)[0] - extents.first.Convert(reference_length)[0];
    double delta_y = extents.second.Convert(reference_length)[1] - extents.first.Convert(reference_length)[1];
    double delta_z = 0.0;
    if(DIM==3)
    {
        delta_z = extents.second.Convert(reference_length)[2] - extents.first.Convert(reference_length)[2];
    }

    if(mUseSimpleGeometricPartition)
    {
        if(mParitionAxis==0)
        {
            domain_width = delta_x/(double(num_procs));
        }
        else if(mParitionAxis==1)
        {
            domain_width = delta_y/(double(num_procs));
        }
        else if(mParitionAxis==2 and DIM==3)
        {
            domain_width = delta_z/(double(num_procs));
        }
        if(DIM==2 and mParitionAxis==2)
        {
            EXCEPTION("Can't partition across Z in 2D.");
        }
        domain_start = extents.first.Convert(reference_length)[mParitionAxis];
    }

    // Assign nodes, segments and vessels to processors
    std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = mpNetwork->GetVessels();

    // All nodes not in the processor's domain are first labelled halos. Any nodes not contained
    // in vessels or halo vessels are allowed to go out of scope later.
    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        double loc = 0.0;
        if(mUseSimpleGeometricPartition)
        {
            loc = nodes[idx]->rGetLocation().Convert(reference_length)[mParitionAxis];
        }
        unsigned processor_loc = std::floor((loc-domain_start)/domain_width);
        if(processor_loc>=num_procs)
        {
            processor_loc = num_procs-1;
        }
        std::cout << "node " << idx << " assigned to rank " << processor_loc << std::endl;
        nodes[idx]->SetOwnerRank(processor_loc);
        if(processor_loc!=rank)
        {
            nodes[idx]->SetIsHalo(true);
        }
    }

    // Vessels with majority nodes on the processor are owned by the processor, vessels with
    // any nodes in the domain are halos.
    std::vector<std::shared_ptr<Vessel<DIM> > > local_vessels;
    std::vector<std::shared_ptr<Vessel<DIM> > > halo_vessels;
    std::cout << "num vessels" << vessels.size() << std::endl;
    for(unsigned idx=0;idx<vessels.size();idx++)
    {
        std::vector<std::shared_ptr<VesselNode<DIM> > > vessel_nodes = vessels[idx]->GetNodes();
        std::vector<unsigned> nodes_per_proc(num_procs, 0);
        unsigned num_nodes = vessel_nodes.size();
        for(unsigned jdx=0;jdx<num_nodes;jdx++)
        {
            nodes_per_proc[vessel_nodes[jdx]->GetOwnerRank()]++;
        }

        // Get lowest index of max value
        std::vector<unsigned>::iterator result = std::max_element(nodes_per_proc.begin(), nodes_per_proc.end());
        unsigned max_index = std::distance(nodes_per_proc.begin(), result);
        unsigned max_value = nodes_per_proc[max_index];

        if(double(max_value)>(num_nodes/2.0))
        {
            vessels[idx]->SetOwnerRank(max_index);
        }
        else if(double(max_value)==double(num_nodes/2.0) and vessels[idx]->GetStartNode()->GetOwnerRank()==max_index)
        {
            vessels[idx]->SetOwnerRank(max_index);
        }

        if(vessels[idx]->GetOwnerRank()!=rank)
        {
            if(nodes_per_proc[rank]>0)
            {
                halo_vessels.push_back(vessels[idx]);
                vessels[idx]->SetIsHalo(true);
                for(unsigned jdx=0;jdx<vessels[idx]->GetNodes().size();jdx++)
                 {
                     if(!vessel_nodes[jdx]->IsHalo())
                     {
                         vessel_nodes[jdx]->SetHasHalo(true);
                     }
                 }
            }
        }
        else
        {
            local_vessels.push_back(vessels[idx]);
            if(nodes_per_proc[rank]!=num_nodes)
            {
                vessels[idx]->SetHasHalo(true);
                for(unsigned jdx=0;jdx<vessels[idx]->GetNodes().size();jdx++)
                {
                    if(!vessel_nodes[jdx]->IsHalo())
                    {
                        vessel_nodes[jdx]->SetHasHalo(true);
                    }
                }
            }
        }
        for(unsigned jdx=0;jdx<vessels[idx]->GetSegments().size(); jdx++)
        {
            vessels[idx]->GetSegments()[jdx]->SetIsHalo(vessels[idx]->IsHalo());
            vessels[idx]->GetSegments()[jdx]->SetOwnerRank(vessels[idx]->GetOwnerRank());
        }
    }

    // Get the number of nodes per rank
    std::vector<unsigned> nodes_per_rank(num_procs, 0);
    unsigned global_index_counter = 0;
    for(unsigned idx=0;idx<num_procs;idx++)
    {
        unsigned local_index_counter = 0;
        for(unsigned jdx=0; jdx<nodes.size();jdx++)
        {
            if(nodes[jdx]->GetOwnerRank()==idx and !nodes[jdx]->IsHalo())
            {
                nodes[jdx]->SetGlobalIndex(global_index_counter);
                nodes[jdx]->SetLocalIndex(local_index_counter);
                nodes_per_rank[idx]++;
                global_index_counter++;
                local_index_counter++;
            }
        }
    }

    // Rebuild the vessel arrays in the network
    std::cout << "rank " << rank << " num v " << local_vessels.size() << std::endl;

    mpNetwork->ClearVessels();
    mpNetwork->AddVessels(local_vessels);
    mpNetwork->AddVessels(halo_vessels);
    unsigned low_index = 0;
    if(rank>0)
    {
        unsigned sum_lower_ranks=0;
        for(unsigned idx=0;idx<rank;idx++)
        {
            sum_lower_ranks+=nodes_per_rank[idx];
        }
        low_index = sum_lower_ranks;
    }
    unsigned high_index = low_index + nodes_per_rank[rank];

    std::shared_ptr<DistributedVectorFactory> p_vector_factory =
            std::shared_ptr<DistributedVectorFactory>(new DistributedVectorFactory(low_index, high_index, nodes.size()));

    mpNetwork->SetDistributedVectorFactory(p_vector_factory);
    mpNetwork->UpdateAll(false);
}

// Explicit instantiation
template class VesselNetworkPartitioner<2>;
template class VesselNetworkPartitioner<3>;
