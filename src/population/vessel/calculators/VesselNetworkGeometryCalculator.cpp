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
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkPolyData.h>
#include <vtkCellLocator.h>
#include "Exception.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "GeometryTools.hpp"


template <unsigned DIM>
VesselNetworkGeometryCalculator<DIM>::VesselNetworkGeometryCalculator()
{

}

template <unsigned DIM>
VesselNetworkGeometryCalculator<DIM>::~VesselNetworkGeometryCalculator()
{

}

template <unsigned DIM>
std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > VesselNetworkGeometryCalculator<DIM>::GetExtents(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        bool useRadii)
{
    units::quantity<unit::length> x_max = -DBL_MAX*unit::metres;
    units::quantity<unit::length> y_max = -DBL_MAX*unit::metres;
    units::quantity<unit::length> z_max = -DBL_MAX*unit::metres;

    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator it;
    for(it = nodes.begin(); it != nodes.end(); it++)
    {
        units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();
        c_vector<double, DIM> location = (*it)->rGetLocation().GetLocation(length_scale);
        if(location[0]*length_scale > x_max)
        {
            x_max = location[0]*length_scale;
            if(useRadii)
            {
                x_max += (*it)->GetRadius();
            }
        }
        if(location[1]*length_scale > y_max)
        {
            y_max = location[1]*length_scale;
            if(useRadii)
            {
                y_max += (*it)->GetRadius();
            }
        }
        if(DIM > 2)
        {
            if(location[2]*length_scale > z_max)
            {
                z_max = location[2]*length_scale;
                if(useRadii)
                {
                    z_max += (*it)->GetRadius();
                }
            }
        }
    }

    units::quantity<unit::length> x_min = x_max;
    units::quantity<unit::length> y_min = y_max;
    units::quantity<unit::length> z_min = z_max;
    for(it = nodes.begin(); it != nodes.end(); it++)
    {
        units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();
        c_vector<double, DIM> location = (*it)->rGetLocation().GetLocation(length_scale);
        if(location[0]*length_scale < x_min)
        {
            x_min = location[0]*length_scale;
            if(useRadii)
            {
                x_min -= (*it)->GetRadius();
            }
        }
        if(location[1]*length_scale < y_min)
        {
            y_min = location[1]*length_scale;
            if(useRadii)
            {
                y_min -= (*it)->GetRadius();
            }
        }
        if(DIM > 2)
        {
            if(location[2]*length_scale < z_min)
            {
                z_min = location[2]*length_scale;
                if(useRadii)
                {
                    z_min -= (*it)->GetRadius();
                }
            }
        }
    }

    units::quantity<unit::length> base_length = BaseUnits::Instance()->GetReferenceLengthScale();
    std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > bbox(DimensionalChastePoint<DIM>(x_min/base_length, y_min/base_length, z_min/base_length, base_length),
                                                                              DimensionalChastePoint<DIM>(x_max/base_length, y_max/base_length, z_max/base_length, base_length));
    return bbox;
}

template<unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > VesselNetworkGeometryCalculator<DIM>::GetNodesInSphere(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>&  rCentre, units::quantity<unit::length> radius)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselNode<DIM> > > inside_nodes;
    for(unsigned idx = 0; idx < nodes.size(); idx++)
    {
        if(nodes[idx]->GetDistance(rCentre) <= radius)
        {
            inside_nodes.push_back(nodes[idx]);
        }
    }
    return inside_nodes;
}

template <unsigned DIM>
boost::shared_ptr<VesselNetworkGeometryCalculator<DIM> > VesselNetworkGeometryCalculator<DIM>::Create()
{
    MAKE_PTR(VesselNetworkGeometryCalculator<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetDistanceToNearestNode(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>& rLocation)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(rLocation);
        if (node_distance < min_distance)
        {
            min_distance = node_distance;
            nearest_node = (*node_iter) ;
        }
    }
    return min_distance;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetworkGeometryCalculator<DIM>::GetNearestNode(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        boost::shared_ptr<VesselNode<DIM> > pInputNode)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        if((*node_iter) != pInputNode)
        {
            units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(pInputNode->rGetLocation());
            if (node_distance < min_distance)
            {
                min_distance = node_distance;
                nearest_node = (*node_iter) ;
            }
        }
    }
    return nearest_node;
}

template <unsigned DIM>
boost::shared_ptr<VesselNode<DIM> > VesselNetworkGeometryCalculator<DIM>::GetNearestNode(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>& location)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    boost::shared_ptr<VesselNode<DIM> > nearest_node;
    units::quantity<unit::length> min_distance = DBL_MAX*unit::metres;

    typename std::vector<boost::shared_ptr<VesselNode<DIM> > >::iterator node_iter;
    for(node_iter = nodes.begin(); node_iter != nodes.end(); node_iter++)
    {
        units::quantity<unit::length> node_distance = (*node_iter)->GetDistance(location);
        if (node_distance < min_distance)
        {
            min_distance = node_distance;
            nearest_node = (*node_iter) ;
        }
    }

    return nearest_node;
}

template <unsigned DIM>
std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > VesselNetworkGeometryCalculator<DIM>::GetNearestSegment(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        boost::shared_ptr<VesselSegment<DIM> > pSegment)
{
    boost::shared_ptr<VesselSegment<DIM> > nearest_segment;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();

    double min_distance = DBL_MAX;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
    for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
    {
        if(!pSegment->IsConnectedTo((*segment_iter)))
        {
            // Get the segment to segment distance (http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment())
            c_vector<double, DIM> u = (*segment_iter)->GetNode(1)->rGetLocation().GetLocation(length_scale) -
                    (*segment_iter)->GetNode(0)->rGetLocation().GetLocation(length_scale);
            c_vector<double, DIM> v = pSegment->GetNode(1)->rGetLocation().GetLocation(length_scale) -
                    pSegment->GetNode(0)->rGetLocation().GetLocation(length_scale);
            c_vector<double, DIM> w = (*segment_iter)->GetNode(0)->rGetLocation().GetLocation(length_scale) -
                    pSegment->GetNode(0)->rGetLocation().GetLocation(length_scale);

            double a = inner_prod(u,u);
            double b = inner_prod(u,v);
            double c = inner_prod(v,v);
            double d = inner_prod(u,w);
            double e = inner_prod(v,w);

            double dv = a * c - b * b;
            double sc, sn, sd = dv;
            double tc, tn ,td = dv;

            if(dv < 1.e-12) // almost parallel segments
            {
                sn = 0.0;
                sd = 1.0;
                tn = e;
                td = c;
            }
            else // get the closest point on the equivalent infinite lines
            {
                sn = (b*e - c*d);
                tn = (a*e - b*d);
                if ( sn < 0.0)
                {
                    sn = 0.0;
                    tn = e;
                    td = c;
                }
                else if(sn > sd)
                {
                    sn =sd;
                    tn = e+ b;
                    td = c;
                }
            }

            if(tn < 0.0)
            {
                tn = 0.0;
                if(-d < 0.0)
                {
                    sn = 0.0;
                }
                else if(-d > a)
                {
                    sn = sd;
                }
                else
                {
                    sn = -d;
                    sd = a;
                }
            }
            else if(tn > td)
            {
                tn = td;
                if((-d + b) < 0.0)
                {
                    sn = 0.0;
                }
                else if((-d + b) > a)
                {
                    sn = sd;
                }
                else
                {
                    sn = (-d + b);
                    sd = a;
                }
            }

            sc = (std::abs(sn) < 1.e-12 ? 0.0 : sn/sd);
            tc = (std::abs(tn) < 1.e-12 ? 0.0 : tn/td);
            c_vector<double, DIM> dp = w + (sc * u) - (tc * v);

            double segment_distance = norm_2(dp);
            if (segment_distance < min_distance)
            {
                min_distance = segment_distance;
                nearest_segment = (*segment_iter) ;
            }
        }

    }
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > return_pair =
            std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >(nearest_segment, min_distance * length_scale);
    return return_pair;
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetNearestSegment(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        boost::shared_ptr<VesselNode<DIM> > pNode,
        boost::shared_ptr<VesselSegment<DIM> >& pEmptySegment,
        bool sameVessel, units::quantity<unit::length> radius)
{

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();

    c_vector<double, DIM> loc = pNode->rGetLocation().GetLocation(length_scale);
    vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();
    c_vector<double, 6> bbox;
    double dimensionless_radius = radius/length_scale;

    vtkSmartPointer<vtkPolyData> p_close_network = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkCellLocator> p_close_cell_locator = vtkSmartPointer<vtkCellLocator>::New();
    std::map<unsigned, unsigned> local_global_map;
    if(dimensionless_radius>0.0)
    {
        bbox[0] = loc[0]-dimensionless_radius;
        bbox[1] = loc[0]+dimensionless_radius;
        bbox[2] = loc[1]-dimensionless_radius;
        bbox[3] = loc[1]+dimensionless_radius;
        bbox[4] = -1.e-3;
        bbox[5] = +1.e-3;
        if(DIM==3)
        {
            bbox[4] = loc[2]-dimensionless_radius;
            bbox[5] = loc[2]+dimensionless_radius;
        }
        pNetwork->GetVtkCellLocator()->FindCellsWithinBounds(&bbox[0], p_id_list);
        if(p_id_list->GetNumberOfIds()==0)
        {
            pEmptySegment = boost::shared_ptr<VesselSegment<DIM> >();
            return DBL_MAX*unit::metres;
        }

        // Narrow down the returned cells for the next search
        vtkSmartPointer<vtkPolyData> p_temp_network = vtkPolyData::SafeDownCast(pNetwork->GetVtkCellLocator()->GetDataSet());
        vtkSmartPointer<vtkCellArray> p_close_lines = vtkSmartPointer<vtkCellArray>::New();
        unsigned num_non_self = 0;
        for(unsigned idx=0;idx<p_id_list->GetNumberOfIds();idx++)
        {
           unsigned index = p_id_list->GetId(idx);
           if(pNode->IsAttachedTo(segments[index]) and !sameVessel)
           {
               continue;
           }
           p_close_lines->InsertNextCell(p_temp_network->GetCell(index));
           local_global_map[num_non_self] = index;
           num_non_self++;

        }
        if(num_non_self==0)
        {
            pEmptySegment = boost::shared_ptr<VesselSegment<DIM> >();
            return DBL_MAX*unit::metres;
        }
        p_close_network->SetPoints(p_temp_network->GetPoints());
        p_close_network->SetLines(p_close_lines);
        p_close_cell_locator->SetDataSet(p_close_network);
        p_close_cell_locator->BuildLocator();
    }
    else
    {
        p_close_network = pNetwork->GetVtk();
        p_close_cell_locator = pNetwork->GetVtkCellLocator();
        for(unsigned idx=0;idx<p_close_network->GetNumberOfCells();idx++)
        {
            local_global_map[idx] = idx;
        }
    }

    double closest[3];
    vtkIdType cellId;
    int subId;
    double distance_sq;
    if(DIM==3)
    {
        p_close_cell_locator->FindClosestPoint(&loc[0], closest, cellId, subId, distance_sq);
    }
    else
    {
        c_vector<double, 3> loc_3d;
        loc_3d[0] = loc[0];
        loc_3d[1] = loc[1];
        loc_3d[2] = 0.0;
        p_close_cell_locator->FindClosestPoint(&loc_3d[0], closest, cellId, subId, distance_sq);
    }
    pEmptySegment = segments[local_global_map[cellId]];
    units::quantity<unit::length> distance = std::sqrt(distance_sq)*length_scale;
    return distance;
}

template <unsigned DIM>
std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >  VesselNetworkGeometryCalculator<DIM>::GetNearestSegment(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>& location)
{
    boost::shared_ptr<VesselSegment<DIM> > nearest_segment;
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();

    units::quantity<unit::length>  min_distance = DBL_MAX * unit::metres;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;
    for(segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
    {
        units::quantity<unit::length>  segment_distance = (*segment_iter)->GetDistance(location);
        if (segment_distance < min_distance)
        {
            min_distance = segment_distance;
            nearest_segment = (*segment_iter) ;
        }
    }
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > return_pair =
            std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> >(nearest_segment, min_distance);
    return return_pair;
}

template <unsigned DIM>
boost::shared_ptr<Vessel<DIM> > VesselNetworkGeometryCalculator<DIM>::GetNearestVessel(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>& location)
{
    return GetNearestSegment(pNetwork, location).first->GetVessel();
}

template <unsigned DIM>
std::vector<units::quantity<unit::length> > VesselNetworkGeometryCalculator<DIM>::GetInterCapillaryDistances(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = pNetwork->GetVessels();
    std::vector<units::quantity<unit::length> > distances;
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        units::quantity<unit::length> min_distance = 1.e6 * unit::metres;
        for(unsigned jdx=0; jdx<vessels.size(); jdx++)
        {
            if(vessels[idx]!=vessels[jdx])
            {
                units::quantity<unit::length> distance = vessels[idx]->GetStartNode()->GetDistance(vessels[jdx]->GetStartNode()->rGetLocation());
                if(distance < min_distance)
                {
                    min_distance = distance;
                }
            }
        }
        distances.push_back(min_distance);
    }
    return distances;
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetTotalLength(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = pNetwork->GetVessels();
    units::quantity<unit::length>  length = 0.0* unit::metres;
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        length += vessels[idx]->GetLength();
    }
    return length;
}

template <unsigned DIM>
units::quantity<unit::volume> VesselNetworkGeometryCalculator<DIM>::GetTotalVolume(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    units::quantity<unit::volume> volume = 0.0*units::pow<3>(unit::metres);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    for(unsigned idx=0; idx< segments.size(); idx++)
    {
        volume += segments[idx]->GetLength() * segments[idx]->GetRadius() * segments[idx]->GetRadius() * M_PI;
    }
    return volume;
}

template <unsigned DIM>
units::quantity<unit::area> VesselNetworkGeometryCalculator<DIM>::GetTotalSurfaceArea(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    units::quantity<unit::area> area = 0.0*units::pow<2>(unit::metres);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();
    for(unsigned idx=0; idx< segments.size(); idx++)
    {
        area += segments[idx]->GetLength() * 2.0 * segments[idx]->GetRadius() * M_PI;
    }
    return area;
}

template <unsigned DIM>
units::quantity<unit::length>  VesselNetworkGeometryCalculator<DIM>::GetAverageInterSegmentDistance(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();

    // store segment midpoints
    std::vector<DimensionalChastePoint<DIM> > midpoints(segments.size());
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        midpoints[idx] = segments[idx]->GetMidPoint();
    }

    // get intersegment distances
    units::quantity<unit::length>  av_dist = 0.0 * unit::metres;
    for(unsigned idx=0; idx<segments.size(); idx++)
    {
        units::quantity<unit::length> min_dist = 1.e6 * unit::metres;
        for(unsigned jdx=0; jdx<segments.size(); jdx++)
        {
            if(segments[idx] != segments[jdx] && segments[idx]->GetVessel() != segments[jdx]->GetVessel())
            {
                units::quantity<unit::length> dist = GetDistance(midpoints[idx], midpoints[jdx]);
                if(dist < min_dist)
                {
                    min_dist = dist;
                }
            }
        }
        av_dist += min_dist;
    }
    return av_dist / double(segments.size());
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetAverageVesselLength(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    return GetTotalLength(pNetwork) / double(pNetwork->GetVessels().size());
}

template <unsigned DIM>
std::vector<unsigned> VesselNetworkGeometryCalculator<DIM>::GetVesselLengthDistribution(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        double binSpacing, unsigned numberOfBins)
{
    std::vector<unsigned> bins(numberOfBins, 0);

    // populate the bins
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = pNetwork->GetVessels();
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        unsigned bin_label = std::floor(vessels[idx]->GetLength() / (binSpacing*unit::metres));
        if(bin_label > numberOfBins)
        {
            bin_label = numberOfBins;
        }
        bins[bin_label]++;
    }
    return bins;
}

template <unsigned DIM>
unsigned VesselNetworkGeometryCalculator<DIM>::GetNumberOfNodesNearLocation(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
        const DimensionalChastePoint<DIM>& rLocation, double tolerance)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    unsigned num_nodes = 0;

    for(unsigned idx = 0; idx < nodes.size(); idx++)
    {
        if(nodes[idx]->GetDistance(rLocation)/BaseUnits::Instance()->GetReferenceLengthScale() <= tolerance + 1.e-6)
        {
            num_nodes++;
        }
    }
    return num_nodes;
}

template<unsigned DIM>
bool VesselNetworkGeometryCalculator<DIM>::VesselCrossesLineSegment(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, const DimensionalChastePoint<DIM>& coordinate_1,
                                                  const DimensionalChastePoint<DIM>& coordinate_2,
                                                  double tolerance)
{
    boost::shared_ptr<VesselSegment<DIM> > temp_segment = VesselSegment<DIM>::Create(VesselNode<DIM>::Create(coordinate_1), VesselNode<DIM>::Create(coordinate_2));
    std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > nearest_segment = GetNearestSegment(pNetwork, temp_segment);

    // todo a false here does not necessarily guarantee that a vessel does not cross a line segment since get nearest
    // segment only returns one segment
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    double nearest_seg_dist = nearest_segment.second / reference_length;
    double coord1_distance = nearest_segment.first->GetDistance(coordinate_1) / reference_length;
    double coord2_distance = nearest_segment.first->GetDistance(coordinate_2) / reference_length;
    bool crosses_segment = (nearest_seg_dist<= tolerance) && (coord1_distance > tolerance) && (coord2_distance > tolerance);
    return  crosses_segment;
}

// Explicit instantiation
template class VesselNetworkGeometryCalculator<2>;
template class VesselNetworkGeometryCalculator<3>;

