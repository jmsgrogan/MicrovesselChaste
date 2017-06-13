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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkMergePoints.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <iostream>
#include <math.h>
#include <cfloat>
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "SegmentFlowProperties.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkWriter.hpp"
#include "Timer.hpp"
#include "PetscTools.hpp"
#include "VesselNetworkVtkConverter.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

template <unsigned DIM>
VesselNetwork<DIM>::VesselNetwork() : AbstractVesselNetworkComponent<DIM>(),
  mVessels(),
  mSegments(),
  mSegmentsUpToDate(false),
  mNodes(),
  mNodesUpToDate(false),
  mVesselNodes(),
  mVesselNodesUpToDate(false),
  mpVtkGeometry(),
  mpVtkSegmentCellLocator(),
  mVtkGeometryUpToDate(false),
  mDistributedVectorFactory()
{

}

template <unsigned DIM>
VesselNetwork<DIM>::~VesselNetwork()
{

}

template <unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetwork<DIM>::Create()
{
    MAKE_PTR(VesselNetwork<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
void VesselNetwork<DIM>::AddVessel(boost::shared_ptr<Vessel<DIM> > pVessel)
{
    mVessels.push_back(pVessel);
    Modified();
}

template <unsigned DIM>
void VesselNetwork<DIM>::AddVessels(std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    mVessels.insert(mVessels.end(), vessels.begin(), vessels.end());
    Modified();
}

template <unsigned DIM>
void VesselNetwork<DIM>::ClearVessels()
{
    mVessels.clear();
    mSegments.clear();
    mNodes.clear();
    mVesselNodes.clear();
    mSegmentsUpToDate = false;
    mNodesUpToDate = false;
    mVesselNodesUpToDate = false;
    mVtkGeometryUpToDate = false;
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::CopyVessels()
{
    return CopyVessels(mVessels);
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::CopyVessels(std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator vessel_iter;
    std::vector<boost::shared_ptr<Vessel<DIM> > > new_vessels;
    for(vessel_iter = vessels.begin(); vessel_iter != vessels.end(); vessel_iter++)
    {
        typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > new_segments;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = (*vessel_iter)->GetSegments();
        for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
        {
            new_segments.push_back(VesselSegment<DIM>::Create(VesselNode<DIM>::Create((*segment_iter)->GetNode(0)->rGetLocation()),
                                                              VesselNode<DIM>::Create((*segment_iter)->GetNode(1)->rGetLocation())));
        }
        new_vessels.push_back(Vessel<DIM>::Create(new_segments));
    }

    MergeCoincidentNodes(new_vessels);
    AddVessels(new_vessels);
    return new_vessels;
}

template <unsigned DIM>
void VesselNetwork<DIM>::SetDistributedVectorFactory(boost::shared_ptr<DistributedVectorFactory>  vectorFactory)
{
    mDistributedVectorFactory = vectorFactory;
}

template <unsigned DIM>
boost::shared_ptr<DistributedVectorFactory> VesselNetwork<DIM>::GetDistributedVectorFactory()
{
    if(PetscTools::IsSequential())
    {
        unsigned low_index = 0;
        unsigned high_index = GetNodes().size();
        return boost::shared_ptr<DistributedVectorFactory>(new DistributedVectorFactory(low_index, high_index, high_index));
    }
    else
    {
        return mDistributedVectorFactory;
    }
}

template <unsigned DIM>
std::vector<unsigned> VesselNetwork<DIM>::GetNumberOfNodesPerProcess()
{
    return std::vector<unsigned>(0, GetNodes().size());
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::DivideVessel(boost::shared_ptr<Vessel<DIM> > pVessel,
                                                                     const DimensionalChastePoint<DIM>& location)
{
    boost::shared_ptr<VesselSegment<DIM> > p_segment;

    // If the divide location coincides with one of the end nodes don't divide and return that node
    if (pVessel->GetStartNode()->IsCoincident(location) || pVessel->GetEndNode()->IsCoincident(location))
    {

        if (pVessel->GetStartNode()->IsCoincident(location))
        {
            return pVessel->GetStartNode();
        }
        else
        {
            return pVessel->GetEndNode();
        }
    }
    else
    {
        bool locatedInsideVessel = false;
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pVessel->GetSegments();
        for (unsigned idx = 0; idx < segments.size(); idx++)
        {
            if (segments[idx]->GetDistance(location)/BaseUnits::Instance()->GetReferenceLengthScale() <= 1e-6)
            {
                locatedInsideVessel = true;
                p_segment = segments[idx];
                break;
            }
        }
        if(!locatedInsideVessel)
        {
            EXCEPTION("There is no segment at the requested division location.");
        }
    }

    boost::shared_ptr<VesselNode<DIM> > p_new_node = pVessel->DivideSegment(location); // network segments and nodes out of date

    // create two new vessels and assign them the old vessel's properties
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > start_segments;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > end_segments;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pVessel->GetSegments();
    unsigned segment_index = segments.size()+1;
    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        start_segments.push_back(segments[idx]);
        if (segments[idx]->GetNode(1)->IsCoincident(location))
        {
            segment_index = idx;
            break;
        }
    }

    if (segment_index == segments.size()-1)
    {
        EXCEPTION("Vessel segment not found.");
    }
    for (unsigned idx = segment_index+1; idx < segments.size(); idx++)
    {
        end_segments.push_back(segments[idx]);
    }

    boost::shared_ptr<Vessel<DIM> > p_new_vessel1 = Vessel<DIM>::Create(start_segments);
    boost::shared_ptr<Vessel<DIM> > p_new_vessel2 = Vessel<DIM>::Create(end_segments);
    p_new_vessel1->CopyDataFromExistingVessel(pVessel);
    p_new_vessel2->CopyDataFromExistingVessel(pVessel);
    p_new_vessel1->GetFlowProperties()->SetRegressionTime(pVessel->GetFlowProperties()->GetRegressionTime());
    p_new_vessel2->GetFlowProperties()->SetRegressionTime(pVessel->GetFlowProperties()->GetRegressionTime());

    AddVessel(p_new_vessel1);
    AddVessel(p_new_vessel2);
    RemoveVessel(pVessel, false);
    Modified();
    return p_new_node;
}

template <unsigned DIM>
void VesselNetwork<DIM>::ExtendVessel(boost::shared_ptr<Vessel<DIM> > pVessel, boost::shared_ptr<VesselNode<DIM> > pEndNode,
                                        boost::shared_ptr<VesselNode<DIM> > pNewNode)
{
    if(pVessel->GetStartNode() == pEndNode)
    {
        boost::shared_ptr<VesselSegment<DIM> > p_segment = VesselSegment<DIM>::Create(pNewNode, pEndNode);
        p_segment->SetFlowProperties(*(pEndNode->GetSegments()[0]->GetFlowProperties()));
        p_segment->SetRadius(pEndNode->GetSegments()[0]->GetRadius());
        p_segment->SetMaturity(0.0);
        pVessel->AddSegment(p_segment);
    }
    else
    {
        boost::shared_ptr<VesselSegment<DIM> > p_segment = VesselSegment<DIM>::Create(pEndNode, pNewNode);
        p_segment->SetFlowProperties(*(pEndNode->GetSegments()[0]->GetFlowProperties()));
        p_segment->SetRadius(pEndNode->GetSegments()[0]->GetRadius());
        p_segment->SetMaturity(0.0);
        pVessel->AddSegment(p_segment);
    }

    Modified();
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetwork<DIM>::FormSprout(const DimensionalChastePoint<DIM>& sproutBaseLocation,
                                                                 const DimensionalChastePoint<DIM>& sproutTipLocation)
{
    // locate vessel at which the location of the sprout base exists
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > nearest_segment = VesselNetworkGeometryCalculator<DIM>::GetNearestSegment(
            this->shared_from_this(), sproutBaseLocation);
    if (nearest_segment.second / BaseUnits::Instance()->GetReferenceLengthScale()  > 1e-6)
    {
        EXCEPTION("No vessel located at sprout base.");
    }

    // divide vessel at location of sprout base
    boost::shared_ptr<VesselNode<DIM> > p_new_node = DivideVessel(nearest_segment.first->GetVessel(), sproutBaseLocation);

    // create new vessel
    boost::shared_ptr<VesselNode<DIM> > p_new_node_at_tip = VesselNode<DIM>::Create(p_new_node);
    p_new_node_at_tip->SetLocation(sproutTipLocation);
    p_new_node_at_tip->SetIsMigrating(true);
    p_new_node_at_tip->GetFlowProperties()->SetIsInputNode(false);
    p_new_node_at_tip->GetFlowProperties()->SetIsOutputNode(false);
    p_new_node_at_tip->GetFlowProperties()->SetPressure(0.0*unit::pascals);
    boost::shared_ptr<VesselSegment<DIM> > p_new_segment = VesselSegment<DIM>::Create(p_new_node, p_new_node_at_tip);
    p_new_segment->CopyDataFromExistingSegment(nearest_segment.first);
    p_new_segment->GetFlowProperties()->SetFlowRate(0.0*unit::metre_cubed_per_second);
    p_new_segment->GetFlowProperties()->SetImpedance(0.0*unit::pascal_second_per_metre_cubed);
    p_new_segment->GetFlowProperties()->SetHaematocrit(0.0);
    p_new_segment->GetFlowProperties()->SetGrowthStimulus(0.0*unit::per_second);
    p_new_segment->SetMaturity(0.0);

    boost::shared_ptr<Vessel<DIM> > p_new_vessel = Vessel<DIM>::Create(p_new_segment);
    // Sprouting won't save you.
    p_new_vessel->GetFlowProperties()->SetRegressionTime(nearest_segment.first->GetVessel()->GetFlowProperties()->GetRegressionTime());
    AddVessel(p_new_vessel);
    return p_new_vessel;
}

template<unsigned DIM>
std::map<std::string, double> VesselNetwork<DIM>::GetOutputData()
{
    this->mOutputData.clear();
    return this->mOutputData;
}

template <unsigned DIM>
void VesselNetwork<DIM>::RemoveShortVessels(units::quantity<unit::length> cutoff, bool endsOnly)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels_to_remove;

    for(unsigned idx=0; idx<mVessels.size(); idx++)
    {
        if(mVessels[idx]->GetLength() < cutoff)
        {
            if(endsOnly && (mVessels[idx]->GetStartNode()->GetNumberOfSegments() == 1 || mVessels[idx]->GetEndNode()->GetNumberOfSegments() == 1 ))
            {
                vessels_to_remove.push_back(mVessels[idx]);
            }
            else if(!endsOnly)
            {
                vessels_to_remove.push_back(mVessels[idx]);
            }
        }
    }

    for(unsigned idx=0; idx<vessels_to_remove.size(); idx++)
    {
        RemoveVessel(vessels_to_remove[idx]);
    }
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeShortVessels(units::quantity<unit::length> cutoff)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels_to_merge;
    for(unsigned idx=0; idx<mVessels.size(); idx++)
    {
        if(mVessels[idx]->GetLength() < cutoff)
        {
            vessels_to_merge.push_back(mVessels[idx]);
        }
    }

    // Get the nodes, remove the vessel, move the nodes together
    for(unsigned idx=0; idx<vessels_to_merge.size(); idx++)
    {
        vessels_to_merge[idx]->GetEndNode()->SetLocation(vessels_to_merge[idx]->GetStartNode()->rGetLocation());
        RemoveVessel(vessels_to_merge[idx], true);
    }

    Modified();
    MergeCoincidentNodes();
}

template <unsigned DIM>
void VesselNetwork<DIM>::Modified(bool nodesOutOfDate, bool segmentsOutOfDate, bool vesselsOutOfDate)
{
    mVtkGeometryUpToDate = false;
    mSegmentsUpToDate = !segmentsOutOfDate;
    mNodesUpToDate = !nodesOutOfDate;
    mVesselNodesUpToDate = !nodesOutOfDate or !vesselsOutOfDate;
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetwork<DIM>::GetNodes()
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes;
}

template <unsigned DIM>
vtkSmartPointer<vtkPolyData> VesselNetwork<DIM>::GetVtk()
{
    if(!mVtkGeometryUpToDate)
    {
        UpdateInternalVtkGeometry();
    }
    return mpVtkGeometry;
}

template <unsigned DIM>
vtkSmartPointer<vtkCellLocator> VesselNetwork<DIM>::GetVtkCellLocator()
{
    if(!mVtkGeometryUpToDate)
    {
        UpdateInternalVtkGeometry();
    }
    return mpVtkSegmentCellLocator;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetwork<DIM>::GetNode(unsigned index)
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes[index];
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfNodes()
{
    if(!mNodesUpToDate)
    {
        UpdateNodes();
    }
    return mNodes.size();
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfVesselNodes()
{
    if(!mVesselNodesUpToDate)
    {
        UpdateVesselNodes();
    }
    return mVesselNodes.size();
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNodeIndex(boost::shared_ptr<VesselNode<DIM> > node)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > vec_nodes = GetNodes();

    unsigned index = 0;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it;
    for(it = vec_nodes.begin(); it != vec_nodes.end(); it++)
    {
        if(*it == node)
        {
            return index;
        }
        index ++;
    }
    EXCEPTION("Node is not in the network.");

}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetNumberOfVessels()
{
    return mVessels.size();
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetwork<DIM>::GetVesselEndNodes()
{
    if(!mVesselNodesUpToDate)
    {
        UpdateVesselNodes();
    }
    return mVesselNodes;
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetwork<DIM>::GetVessel(unsigned index)
{
    if(index  >= mVessels.size())
    {
        EXCEPTION("Requested vessel index out of range");
    }

    return mVessels[index];
}

template <unsigned DIM>
std::vector<boost::shared_ptr<Vessel<DIM> > > VesselNetwork<DIM>::GetVessels()
{
    return mVessels;
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetMaxBranchesOnNode()
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetVesselEndNodes();
    unsigned num_nodes = nodes.size();

    // Get maximum number of segments attached to a node in the whole network.
    unsigned max_num_branches = 0;
    for(unsigned node_index = 0; node_index < num_nodes; node_index++)
    {
        boost::shared_ptr<VesselNode<DIM> > p_each_node = nodes[node_index];
        unsigned num_segments_on_node = nodes[node_index]->GetNumberOfSegments();

        if (num_segments_on_node > max_num_branches)
        {
            max_num_branches = num_segments_on_node;
        }
    }
    return max_num_branches;
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetVesselIndex(boost::shared_ptr<Vessel<DIM> > pVessel)
{
    unsigned index = 0;
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        if(*it == pVessel)
        {
            return index;
        }
        index ++;
    }
    EXCEPTION("Input vessel is not in the network.");
}

template <unsigned DIM>
unsigned VesselNetwork<DIM>::GetVesselSegmentIndex(boost::shared_ptr<VesselSegment<DIM> > pVesselSegment)
{
    unsigned index = 0;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = GetVesselSegments();
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it;
    for(it = segments.begin(); it != segments.end(); it++)
    {
        if(*it == pVesselSegment)
        {
            return index;
        }
        index++;
    }
    EXCEPTION("Input vessel is not in the network.");
}

template <unsigned DIM>
std::vector<boost::shared_ptr<VesselSegment<DIM> > > VesselNetwork<DIM>::GetVesselSegments()
{
    if(!mSegmentsUpToDate)
    {
        UpdateSegments();
    }
    return mSegments;
}

template <unsigned DIM>
bool VesselNetwork<DIM>::NodeIsInNetwork(boost::shared_ptr<VesselNode<DIM> > pSourceNode)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = GetNodes();
    return (std::find(nodes.begin(), nodes.end(), pSourceNode) != nodes.end());
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(double tolerance)
{
    MergeCoincidentNodes(GetNodes(), tolerance);
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(std::vector<boost::shared_ptr<Vessel<DIM> > > pVessels, double tolerance)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
    for(unsigned idx = 0; idx <pVessels.size(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > vessel_nodes = pVessels[idx]->GetNodes();
        nodes.insert(nodes.end(), vessel_nodes.begin(), vessel_nodes.end());
    }
    MergeCoincidentNodes(nodes, tolerance);
}

template <unsigned DIM>
void VesselNetwork<DIM>::MergeCoincidentNodes(std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes, double tolerance)
{
    if(nodes.size()==0)
    {
        return;
    }
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator it3;
    vtkSmartPointer<vtkMergePoints> p_merge = vtkSmartPointer<vtkMergePoints>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    double bounds[6];
    bounds[0] = 0;
    bounds[1] = 3000;
    bounds[2] = 0;
    bounds[3] = 3000;
    bounds[4] = 0;
    bounds[5] = 10;
    p_merge->SetTolerance(tolerance);
    p_merge->InitPointInsertion(p_points, bounds);
    units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();
    std::vector<int> unique_index_map= std::vector<int>(nodes.size(), -1);
    for(unsigned idx=0; idx<nodes.size(); idx++)
    {
        c_vector<double, DIM> loc = nodes[idx]->rGetLocation().GetLocation(length_scale);
        vtkIdType id;
        int new_point;
        if(DIM==3)
        {
            new_point = p_merge->InsertUniquePoint(&loc[0], id);
        }
        else
        {
            double loc2d[3];
            loc2d[0] = loc[0];
            loc2d[1] = loc[1];
            loc2d[2] = 0.0;
            new_point = p_merge->InsertUniquePoint(loc2d, id);
        }
        if(new_point)
        {
            unique_index_map[id] = int(idx);
        }
        else
        {
            if(unique_index_map[id]<0)
            {
                EXCEPTION("Unexpected index found during attempted node merge");
            }
            // Replace the node corresponding to 'it2' with the one corresponding to 'it'
            // in all segments.
            std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = nodes[idx]->GetSegments();
            for(it3 = segments.begin(); it3 != segments.end(); it3++)
            {
                if ((*it3)->GetNode(0) == nodes[idx])
                {
                    (*it3)->ReplaceNode(0, nodes[unique_index_map[id]]);
                }
                else if(((*it3)->GetNode(1) == nodes[idx]))
                {
                    (*it3)->ReplaceNode(1, nodes[unique_index_map[id]]);
                }
            }
        }
    }
    Modified();
}

template <unsigned DIM>
void VesselNetwork<DIM>::Translate(DimensionalChastePoint<DIM> rTranslationVector)
{
    Translate(rTranslationVector, mVessels);
}

template <unsigned DIM>
void VesselNetwork<DIM>::Translate(DimensionalChastePoint<DIM> rTranslationVector, std::vector<boost::shared_ptr<Vessel<DIM> > > vessels)
{
    std::set<boost::shared_ptr<VesselNode<DIM> > > nodes;
    for(unsigned idx = 0; idx <vessels.size(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > vessel_nodes = vessels[idx]->GetNodes();
        std::copy(vessel_nodes.begin(), vessel_nodes.end(), std::inserter(nodes, nodes.begin()));
    }

    typename std::set<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        DimensionalChastePoint<DIM> old_loc = (*node_iter)->rGetLocation();
        old_loc.Translate(rTranslationVector);
        (*node_iter)->SetLocation(old_loc);
    }
    Modified(false, false, false);
}

template <unsigned DIM>
void VesselNetwork<DIM>::RemoveVessel(boost::shared_ptr<Vessel<DIM> > pVessel, bool deleteVessel)
{
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it = std::find(mVessels.begin(), mVessels.end(), pVessel);
    if(it != mVessels.end())
    {
        if(deleteVessel)
        {
            (*it)->Remove();
        }
        mVessels.erase(it);
    }
    else
    {
        EXCEPTION("Vessel is not contained inside network.");
    }

    Modified();
}

template <unsigned DIM>
void VesselNetwork<DIM>::UpdateAll(bool merge)
{
    if(merge)
    {
        MergeCoincidentNodes();
    }
    UpdateSegments();
    UpdateVesselNodes();
    UpdateNodes();
    UpdateVesselIds();
    UpdateInternalVtkGeometry(); // This goes last
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateNodes()
{
    mNodes.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        for (unsigned idx=0; idx<(*it)->GetNumberOfNodes(); idx++)
        {
            (*it)->GetNode(idx)->SetComparisonId(0);
        }
    }
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        for (unsigned idx=0; idx<(*it)->GetNumberOfNodes(); idx++)
        {
            if((*it)->GetNode(idx)->GetComparisonId()==0)
            {
                mNodes.push_back((*it)->GetNode(idx));
                (*it)->GetNode(idx)->SetComparisonId(1);
            }
        }
    }
    mNodesUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateSegments()
{
    mSegments.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > vessel_segments = (*it)->GetSegments();
        std::copy(vessel_segments.begin(), vessel_segments.end(), std::back_inserter(mSegments));
    }
    mSegmentsUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateVesselNodes()
{
    mVesselNodes.clear();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator it;
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        (*it)->UpdateNodes();
        (*it)->GetStartNode()->SetComparisonId(0);
        (*it)->GetEndNode()->SetComparisonId(0);
    }
    for(it = mVessels.begin(); it != mVessels.end(); it++)
    {
        if((*it)->GetStartNode()->GetComparisonId()==0)
        {
            mVesselNodes.push_back((*it)->GetStartNode());
            (*it)->GetStartNode()->SetComparisonId(1);
        }
        if((*it)->GetEndNode()->GetComparisonId()==0)
        {
            mVesselNodes.push_back((*it)->GetEndNode());
            (*it)->GetEndNode()->SetComparisonId(1);
        }
    }
    mVesselNodesUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateVesselIds()
{
    for(unsigned idx=0;idx<mVessels.size();idx++)
    {
        mVessels[idx]->SetId(idx);
    }
}

template<unsigned DIM>
void VesselNetwork<DIM>::UpdateInternalVtkGeometry()
{
    mpVtkGeometry = VesselNetworkVtkConverter<DIM>::GetVtkRepresentation(this->shared_from_this());
    mpVtkSegmentCellLocator = vtkSmartPointer<vtkCellLocator>::New();
    mpVtkSegmentCellLocator->SetDataSet(mpVtkGeometry);
    if(GetNodes().size()>0)
    {
        mpVtkSegmentCellLocator->BuildLocator();
    }
    mVtkGeometryUpToDate = true;
}

template<unsigned DIM>
void VesselNetwork<DIM>::Write(const std::string& rFileName, bool masterOnly)
{
    boost::shared_ptr<VesselNetworkWriter<DIM> > p_writer = VesselNetworkWriter<DIM>::Create();
    p_writer->SetFileName(rFileName);
    p_writer->SetVesselNetwork(this->shared_from_this());
    p_writer->Write(masterOnly);
}

// Explicit instantiation
template class VesselNetwork<2>;
template class VesselNetwork<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(VesselNetwork, 2)
EXPORT_TEMPLATE_CLASS1(VesselNetwork, 3)
