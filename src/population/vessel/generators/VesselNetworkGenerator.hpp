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

#ifndef VESSELNETWORKGENERATOR_HPP_
#define VESSELNETWORKGENERATOR_HPP_

#include <vector>
#include <string>
#include "VesselNetwork.hpp"
#include "Vessel.hpp"
#include "VesselNode.hpp"
#include "Part.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "DimensionalChastePoint.hpp"

/**
 *  Struct to define random vessel distribution properties
 *
 *  Regular: Vessels are distributed in regular patterns
 *  Uniform: Vessels are distributed from seeds of a uniform random distribution
 *  Two Layer: Vessels are distributed from seeds of a two-layer uniform-normal random distribution
 *  Custom: Manually provide seeds for the vessel distribution
 */
struct VesselDistribution
{
    /**
     * Values for different distributions
     */
    enum Value
    {
        REGULAR, UNIFORM, TWO_LAYER, CUSTOM
    };
};


template<unsigned DIM>
class VesselNetworkGenerator
{

    /**
     * The reference length scale for the vessel network, default in microns.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     */
    VesselNetworkGenerator();

    /**
     * Destructor
     */
    ~VesselNetworkGenerator();

    /*
     * Create a vessel network with all vessels parallel. Vessels are aligned in the 'Z' direction in 3D
     * @param domain A part representing the extents of the spatial domain
     * @param targetDensity The desired number of vessel length per unit volume, this will be only satisfied approximately
     * @param distrbutionType The way to disperse initial seeds for the vessel distribution
     * @param useBbox Whether to use the domain bounding box or the exact shape, the former is faster
     * @param seeds User provided seed locations for the vessel locations, used with CUSTOM distribution type
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateParrallelNetwork(boost::shared_ptr<Part<DIM> > domain,
                                                                        double targetDensity,
                                                                        VesselDistribution::Value distrbutionType,
                                                                        double exclusionDistance = 0.0,
                                                                        bool useBbox = false,
                                                                        std::vector<boost::shared_ptr<Vertex> > seeds =
                                                                                std::vector<boost::shared_ptr<Vertex> >());

    /*
     * Create a 3d vessel network
     * @param domain A part representing the extents of the spatial domain
     * @param targetDensity The desired number of vessel length per unit volume, this will be only satisfied approximately
     * @param distrbutionType The way to disperse initial seeds for the vessel distribution
     * @param useBbox Whether to use the domain bounding box or the exact shape, the former is faster
     * @param seeds User provided seed locations for the vessel locations, used with CUSTOM distribution type
     */
    boost::shared_ptr<VesselNetwork<DIM> > Generate3dNetwork(boost::shared_ptr<Part<DIM> > domain,
                                                                        std::vector<double> targetDensity,
                                                                        VesselDistribution::Value distrbutionType,
                                                                        double exclusionDistance = 0.0,
                                                                        bool useBbox = false,
                                                                        std::vector<boost::shared_ptr<Vertex> > seeds =
                                                                                std::vector<boost::shared_ptr<Vertex> >());

    /*
     * Creates a hexagonal network corresponding to that of Alarcon et al. (2006)
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateHexagonalNetwork(units::quantity<unit::length> width,
                                                                    units::quantity<unit::length> height,
                                                                    units::quantity<unit::length> vesselLength);
    /*
     * Creates a hexagonal repeating unit
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateHexagonalUnit(units::quantity<unit::length> vesselLength);

    /*
     * Creates a bifurcation repeating unit
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateBifurcationUnit(units::quantity<unit::length> vesselLength,
                                                                   DimensionalChastePoint<DIM> startPosition);

    /*
     * Creates a single vessel
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateSingleVessel(units::quantity<unit::length> vesselLength,
                                                                DimensionalChastePoint<DIM> startPosition,
                                                                    unsigned divisions = 0, unsigned axis = 2);

    /*
     * Creates an oval shaped network with one inlet and one outlet
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateOvalNetwork(units::quantity<unit::length> scaleFactor,
                                                                     unsigned num_increments = 40,
                                                                     double a_param = 0.5,
                                                                     double b_param = 1.0);
    /*
     * Generate a network on the edges of a Part
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateFromPart(boost::shared_ptr<Part<DIM> > part);

    /*
     * Creates a vessel network based on a voronoi tesselation in the provided cube.
     */
    boost::shared_ptr<VesselNetwork<DIM> > GenerateVoronoiNetwork(units::quantity<unit::length> cubeX,
                                                                  units::quantity<unit::length> cubeY,
                                                                  units::quantity<unit::length> cubeZ, unsigned numPoints = 400);

    /*
     * Pattern Unit. Coincident nodes are automatically merged in this method.
     */
    void PatternUnitByTranslation(boost::shared_ptr<VesselNetwork<DIM> > pInputUnit, std::vector<unsigned> numberOfUnits);

    void MapToSphere(boost::shared_ptr<VesselNetwork<DIM> > pInputUnit,
                     units::quantity<unit::length> radius,
                     units::quantity<unit::length> thickess,
                     double azimuthExtent,
                     double polarExtent);

    /**
     * Set the reference length scale
     *
     * @param rLengthScale the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> rReferenceLength);

};

#endif /* VESSELNETWORKGENERATOR_HPP_ */
