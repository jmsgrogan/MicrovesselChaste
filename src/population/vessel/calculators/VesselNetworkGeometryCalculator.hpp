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

#ifndef VESSELNETWORKGEOMETRYCALCULATOR_HPP_
#define VESSELNETWORKGEOMETRYCALCULATOR_HPP_

#include <string>
#include <vector>
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"

/**
 * Calculate geometric properties of vessel networks
 */
template<unsigned DIM>
class VesselNetworkGeometryCalculator
{

public:

    /**
     * Constructor
     */
    VesselNetworkGeometryCalculator();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @return a pointer to a class instance
     */
    static std::shared_ptr<VesselNetworkGeometryCalculator<DIM> > Create();

    /**
     * Destructor
     */
    ~VesselNetworkGeometryCalculator();

    /**
     * Get distance to nearest node
     * @param rLocation the probe point
     * @return the distance to the node
     */
    static units::quantity<unit::length> GetDistanceToNearestNode(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Get the node nearest to the specified location
     * @param rLocation the probe point
     * @return the nearest node
     */
    static std::shared_ptr<VesselNode<DIM> > GetNearestNode(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Get the node nearest to the specified node
     * @param pInputNode the probe point
     * @return the nearest node
     */
    static std::shared_ptr<VesselNode<DIM> > GetNearestNode(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            std::shared_ptr<VesselNode<DIM> > pInputNode);

    /**
     * Get the segment nearest to the specified segment and the distance to it
     * @param pSegment the probe segment
     * @return the segment nearest to the specified segment and the distance to it
     */
    static std::pair<std::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > GetNearestSegment(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            std::shared_ptr<VesselSegment<DIM> > pSegment);

    /**
     * Return the distance to the nearest segment. Also over-rides the input segment with the nearest one
     * if a segment is found. The input segment is set to NULL if a segment is not found in the optional
     * bounding region.
     * @param pNode the probe node
     * @param pEmptySegment an empty segment pointer. Becomes the nearest segment if one is found or NULL otherwise.
     * @param sameVessel can the segment be on the same vessel.
     * @param radius an optional search radius, providing one significantly speeds up the search.
     * @return the distance to the nearest segment. Large if none is found, check pEmptySegment instead.
     */
    static units::quantity<unit::length> GetNearestSegment(std::shared_ptr<VesselNetwork<DIM> > pNetwork, std::shared_ptr<VesselNode<DIM> > pNode,
            std::shared_ptr<VesselSegment<DIM> >& pEmptySegment,
            bool sameVessel = true, units::quantity<unit::length> radius = 0.0*unit::metres);

    static units::quantity<unit::length> GetNearestSegmentNonVtk(std::shared_ptr<VesselNetwork<DIM> > pNetwork, std::shared_ptr<VesselNode<DIM> > pNode,
            std::shared_ptr<VesselSegment<DIM> >& pEmptySegment, bool sameVessel = true);

    /**
     * Get the segment nearest to the specified location and the distance to it
     * @param rLocation the probe location
     * @return the segment nearest to the specified segment and the distance to it
     */
    static std::pair<std::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > GetNearestSegment(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Get the vessel nearest to the specified location
     * @param rLocation the probe location
     * @return the vessel nearest to the specified segment and the distance to it
     */
    static std::shared_ptr<Vessel<DIM> > GetNearestVessel(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Get the intercapillary distance using a 2d measure
     * @return
     */
    static std::vector<units::quantity<unit::length> > GetInterCapillaryDistances(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the total length of the network
     * @return the total length of the network
     */
    static units::quantity<unit::length> GetTotalLength(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the total volume of the network
     * @return the total volume of the network
     */
    static units::quantity<unit::volume> GetTotalVolume(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the total surface area of the network
     * @return the total surface area of the network
     */
    static units::quantity<unit::area> GetTotalSurfaceArea(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the average distance between segments
     * @return the average distance between segments
     */
    static units::quantity<unit::length> GetAverageInterSegmentDistance(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the average vessel length
     * @return the average vessel length
     */
    static units::quantity<unit::length> GetAverageVesselLength(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return a histogram of vessel length distributions
     * @param binSpacing the bin spacing
     * @param numberOfBins the number of bins
     * @return a histogram of vessel length distributions
     */
    static std::vector<unsigned> GetVesselLengthDistribution(std::shared_ptr<VesselNetwork<DIM> > pNetwork, double binSpacing = 10.0, unsigned numberOfBins = 10);

    /**
     * Get the number of nodes near to a specified point
     * @param rLocation the probe point
     * @param tolerance the tolerance for proximty calculation
     * @return the number of nodes
     */
    static unsigned GetNumberOfNodesNearLocation(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>&  rLocation, double tolerance = 0.0);

    /**
     * Return the nodes inside a sphere
     * @param rCentre the centre of the sphere
     * @param radius the sphere radius
     * @return the nodes in the sphere
     */
    static std::vector<std::shared_ptr<VesselNode<DIM> > > GetNodesInSphere(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>&  rCentre, units::quantity<unit::length>  radius);

    /**
     * Return the extents of the vessel network in the form ((xmin, xmax), (ymin, ymax), (zmin, zmax))
     * @param useRadii use the vessel radii in calculations
     * @return the extents of the vessel network in the form ((xmin, xmax), (ymin, ymax), (zmin, zmax))
     */
    static std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > GetExtents(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            bool useRadii = false);

    /**
     * Returns whether a vessel crosses a line segment.
     * @param rCoord1 the start of the line segment
     * @param rCoord2 the end of the line segment
     * @param tolerance how close to crossing is considered crossing
     * @return whether a vessel crosses a line segment.
     */
    static bool VesselCrossesLineSegment(std::shared_ptr<VesselNetwork<DIM> > pNetwork,
            const DimensionalChastePoint<DIM>& rCoord1, const DimensionalChastePoint<DIM>& rCoord2, double tolerance = 1e-6);

};

#endif /* VESSELNETWORKGEOMETRYCALCULATOR_HPP_ */
