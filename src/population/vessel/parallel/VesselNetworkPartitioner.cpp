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

#include "VesselNetworkPartitioner.hpp"
#include "Exception.hpp"
#include "PetscTools.hpp"

template<unsigned DIM>
VesselNetworkPartitioner<DIM>::VesselNetworkPartitioner() :
    mpNetwork()
{

}

template<unsigned DIM>
VesselNetworkPartitioner<DIM>::~VesselNetworkPartitioner()
{
}

template<unsigned DIM>
void VesselNetworkPartitioner<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
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
    double domain_width = 100.0;
    unsigned rank = PetscTools::GetMyRank();
    unsigned num_procs = PetscTools::GetNumProcs();
    double width_per_processor = domain_width/double(num_procs);
    double processor_left_bound = double(rank)*width_per_processor;
    double processor_right_bound = DBL_MAX;
    if(rank<num_procs-1)
    {
        processor_right_bound = domain_width;
    }
    else
    {
        processor_right_bound = (rank+1)*width_per_processor;
    }

    // Assign segments and nodes to processors
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = mpNetwork->GetVessels();

    // Label nodes according to their processor
    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        double x_loc = nodes[idx]->rGetLocation().GetLocation(reference_length)[0];
        unsigned processor_loc = std::floor(x_loc/domain_width);
        nodes[idx]->SetOwnerRank(processor_loc);
        if(processor_loc!=rank)
        {

        }
    }
    for(unsigned idx=0;idx<segments.size();idx++)
    {
        boost::shared_ptr<VesselNode<DIM> > p_start_node = segments[idx]->GetNodes().first;
        boost::shared_ptr<VesselNode<DIM> > p_end_node = segments[idx]->GetNodes().second;
        if(p_start_node->GetOwnerRank()!=rank and p_end_node->GetOwnerRank()==rank)
        {
            segments[idx]->SetIsHalo(true);
            segments[idx]->SetOwnerRank(p_start_node->GetOwnerRank());
        }
        else if(p_start_node->GetOwnerRank()==rank and p_end_node->GetOwnerRank()!=rank)
        {
            segments[idx]->SetHasHalo(true);
            segments[idx]->SetOtherProcessorRank(p_end_node->GetOwnerRank());
        }
        else if(p_start_node->GetOwnerRank()!=rank and p_end_node->GetOwnerRank()!=rank)
        {
            segments[idx]->SetIsHalo(true);
            segments[idx]->SetOwnerRank(p_start_node->GetOwnerRank());
        }
    }

    // Identify Halos
    for(unsigned idx=0;idx<nodes.size();idx++)
    {
        bool isHalo=false;
        if(nodes[idx]->GetOwnerRank()!=rank)
        {
            for(unsigned jdx=0;jdx<nodes[idx]->GetSegments().size();jdx++)
            {
                if(nodes[idx]->GetSegments()[jdx]->IsHalo())
                {
                    isHalo = true;
                    break;
                }
            }
        }
        nodes[idx]->SetIsHalo(isHalo);
    }

    // Create halo segments
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels_on_proc;
    for(unsigned idx=0;idx<vessels.size();idx++)
    {
        bool remove_vessel = false;
        // If no nodes or halos are on the proc remove the vessel
        std::vector<boost::shared_ptr<VesselNode<DIM> > > vessel_nodes = vessels[idx]->GetNodes();
        unsigned num_other_procs = 0;
        for(unsigned jdx=0;jdx<vessel_nodes.size(); jdx++)
        {
            if(vessel_nodes[jdx]->GetOwnerRank()!=rank)
            {

            }
        }
    }

    // Rebuild the vessel arrays in the network

}

// Explicit instantiation
template class VesselNetworkPartitioner<2>;
template class VesselNetworkPartitioner<3>;
