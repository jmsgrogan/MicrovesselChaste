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

#ifndef VESSELNETWORKPROPERTYMANAGER_HPP_
#define VESSELNETWORKPROPERTYMANAGER_HPP_

#include <vector>
#include <set>
#include <map>
#include "VesselNetwork.hpp"
#include "DimensionalChastePoint.hpp"

/**
 * A convenience class for assigning properties (flow, chemical, phenotypic etc) to vessel networks.
 */
template<unsigned DIM>
class VesselNetworkPropertyManager
{

public:

    /**
     * Constructor.
     */
    VesselNetworkPropertyManager();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return share pointer to the manager
     */
    static boost::shared_ptr<VesselNetworkPropertyManager<DIM> > Create();

    /**
     * Destructor
     */
    virtual ~VesselNetworkPropertyManager();

    /**
     * Any nodes within the specified radius of the input location are assigned as inflows
     * @param location the search location
     * @param searchRadius the search radius
     */
    static void AssignInflows(boost::shared_ptr<VesselNetwork<DIM> > pNetwork,
            DimensionalChastePoint<DIM> location, units::quantity<unit::length> searchRadius);

    /**
     * Any nodes within the specified radius of the input location are assigned as outflows
     * @param location the search location
     * @param searchRadius the search radius
     */
    static void AssignOutflows(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, DimensionalChastePoint<DIM> location, units::quantity<unit::length> searchRadius);

    /**
     * Copy flow properties from the specified segment to all other vessel network segments
     * @param index the segment index to be copied
     */
    static void CopySegmentFlowProperties(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, unsigned index=0);

    /**
     * Return the inflow nodes
     * @return the inflow nodes
     */
    static std::vector<boost::shared_ptr<VesselNode<DIM> > > GetInflowNodes(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the outflow nodes
     * @return the outflow nodes
     */
    static std::vector<boost::shared_ptr<VesselNode<DIM> > > GetOutflowNodes(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the nodal radii to the same value
     * @param radius the node radius value
     */
    static void SetNodeRadii(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::length> radius);

    /**
     * Get the node radius by averaging its segments
     */
    static void SetNodeRadiiFromSegments(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    static void SetInflowPressures(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::pressure> pressure);

    static void SetOutflowPressures(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::pressure> pressure);

    /**
     * Set the properties of the segments in the network based on those of the prototype
     * @param prototype a prototype segment from which to copy properties
     */
    static void SetSegmentProperties(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, boost::shared_ptr<VesselSegment<DIM> > prototype);

    /**
     * Set the segment radii to the same value
     * @param radius the segment radius
     */
    static void SetSegmentRadii(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::length> radius);

    /**
     * Set the segment viscosity to the same value
     * @param viscosity the segment viscosity
     */
    static void SetSegmentViscosity(boost::shared_ptr<VesselNetwork<DIM> > pNetwork, units::quantity<unit::dynamic_viscosity> viscosity);

};


#endif /* VESSELNETWORKPROPERTYMANAGER_HPP_ */
