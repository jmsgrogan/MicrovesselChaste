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

#include "Exception.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

template <unsigned DIM>
VesselNetworkGeometryCalculator<DIM>::VesselNetworkGeometryCalculator() :
	mpVesselNetwork()
{

}

template <unsigned DIM>
VesselNetworkGeometryCalculator<DIM>::~VesselNetworkGeometryCalculator()
{

}

template <unsigned DIM>
boost::shared_ptr<VesselNetworkGeometryCalculator<DIM> > VesselNetworkGeometryCalculator<DIM>::Create()
{
    MAKE_PTR(VesselNetworkGeometryCalculator<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
std::vector<units::quantity<unit::length> > VesselNetworkGeometryCalculator<DIM>::GetInterCapillaryDistances()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
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
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetTotalLength()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
    units::quantity<unit::length>  length = 0.0* unit::metres;
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        length += vessels[idx]->GetLength();
    }
    return length;
}

template <unsigned DIM>
units::quantity<unit::volume> VesselNetworkGeometryCalculator<DIM>::GetTotalVolume()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    units::quantity<unit::volume> volume = 0.0*units::pow<3>(unit::metres);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();
    for(unsigned idx=0; idx< segments.size(); idx++)
    {
        volume += segments[idx]->GetLength() * segments[idx]->GetRadius() * segments[idx]->GetRadius() * M_PI;
    }
    return volume;
}

template <unsigned DIM>
units::quantity<unit::area> VesselNetworkGeometryCalculator<DIM>::GetTotalSurfaceArea()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    units::quantity<unit::area> area = 0.0*units::pow<2>(unit::metres);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();
    for(unsigned idx=0; idx< segments.size(); idx++)
    {
        area += segments[idx]->GetLength() * 2.0 * segments[idx]->GetRadius() * M_PI;
    }
    return area;
}

template <unsigned DIM>
units::quantity<unit::length>  VesselNetworkGeometryCalculator<DIM>::GetAverageInterSegmentDistance()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();

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
        double min_dist = 1.e6;
        for(unsigned jdx=0; jdx<segments.size(); jdx++)
        {
            if(segments[idx] != segments[jdx] && segments[idx]->GetVessel() != segments[jdx]->GetVessel())
            {
                double dist = norm_2(midpoints[idx].rGetLocation() - midpoints[jdx].rGetLocation() );
                if(dist < min_dist)
                {
                    min_dist = dist;
                }
            }
        }
        av_dist += min_dist * unit::metres ;
    }
    return av_dist / double(segments.size());
}

template <unsigned DIM>
units::quantity<unit::length> VesselNetworkGeometryCalculator<DIM>::GetAverageVesselLength()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    return GetTotalLength() / double(mpVesselNetwork->GetVessels().size());
}

template <unsigned DIM>
std::vector<unsigned> VesselNetworkGeometryCalculator<DIM>::GetVesselLengthDistribution(double binSpacing, unsigned numberOfBins)
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("Vessel network not set in geometry calculator");
    }

    std::vector<unsigned> bins(numberOfBins, 0);

    // populate the bins
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
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
void VesselNetworkGeometryCalculator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    mpVesselNetwork = pVesselNetwork;
}

// Explicit instantiation
template class VesselNetworkGeometryCalculator<2>;
template class VesselNetworkGeometryCalculator<3>;

