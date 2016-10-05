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

private:

    /**
     * Container for the VesselNetwork.
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

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
    static boost::shared_ptr<VesselNetworkGeometryCalculator<DIM> > Create();

    /**
     * Destructor
     */
    ~VesselNetworkGeometryCalculator();

    /**
     * Set the vessel network
     * @param pVesselNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Get the intercapillary distance using a 2d measure
     * @return
     */
    std::vector<units::quantity<unit::length> > GetInterCapillaryDistances();

    /**
     * Return the total length of the network
     * @return the total length of the network
     */
    units::quantity<unit::length> GetTotalLength();

    /**
     * Return the total volume of the network
     * @return the total volume of the network
     */
    units::quantity<unit::volume> GetTotalVolume();

    /**
     * Return the total surface area of the network
     * @return the total surface area of the network
     */
    units::quantity<unit::area> GetTotalSurfaceArea();

    /**
     * Return the average distance between segments
     * @return the average distance between segments
     */
    units::quantity<unit::length> GetAverageInterSegmentDistance();

    /**
     * Return the average vessel length
     * @return the average vessel length
     */
    units::quantity<unit::length> GetAverageVesselLength();

    /**
     * Return a histogram of vessel length distributions
     * @param binSpacing the bin spacing
     * @param numberOfBins the number of bins
     * @return a histogram of vessel length distributions
     */
    std::vector<unsigned> GetVesselLengthDistribution(double binSpacing = 10.0, unsigned numberOfBins = 10);

};

#endif /* VESSELNETWORKGEOMETRYCALCULATOR_HPP_ */
