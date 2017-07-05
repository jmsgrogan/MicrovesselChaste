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
    QLength mReferenceLength;

public:

    /**
     * Constructor
     */
    VesselNetworkGenerator();

    /**
     * Destructor
     */
    ~VesselNetworkGenerator();

    /**
     * Create a vessel network with all vessels parallel. Vessels are aligned in the 'Z' direction in 3D
     * @param domain A part representing the extents of the spatial domain
     * @param targetDensity The desired vessel length per unit volume, this will be only satisfied approximately
     * @param distrbutionType The way to disperse initial seeds for the vessel distribution
     * @param useBbox Whether to use the domain bounding box or the exact shape, the former is faster
     * @param seeds User provided seed locations for the vessel locations, used with CUSTOM distribution type
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateParrallelNetwork(PartPtr<DIM> domain,
                                                                        QArea targetDensity,
                                                                        VesselDistribution::Value distrbutionType,
                                                                        QLength exclusionDistance = 0.0*unit::metres,
                                                                        bool useBbox = false,
                                                                        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > seeds =
                                                                                std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > >());
    /**
     * Creates a hexagonal network corresponding to that of Alarcon et al. (2006)
     * @param width the widht
     * @param height the height
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateHexagonalNetwork(QLength width,
                                                                    QLength height,
                                                                    QLength vesselLength,
                                                                    bool fillDomain=false);
    /**
     * Creates a hexagonal repeating unit
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateHexagonalUnit(QLength vesselLength);

    /**
     * Creates a bifurcation repeating unit
     * @param vesselLength the vessel length
     * @param startPosition the start position of the unit
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateBifurcationUnit(QLength vesselLength,
                                                                   DimensionalChastePoint<DIM> startPosition);

    /**
     * Creates a single vessel
     * @param vesselLength the vessel length
     * @param startPosition the start position for the vessel
     * @param divisions the number of divisions
     * @param axis the alignment axis
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateSingleVessel(QLength vesselLength,
                                                                DimensionalChastePoint<DIM> startPosition,
                                                                    unsigned divisions = 0, unsigned axis = 2);

    /**
     * Creates an oval shaped network with one inlet and one outlet
     * @param scaleFactor a multiplication for from the initial unit length
     * @param num_increments the number of increments
     * @param a_param a width parameter
     * @param a_param a length parameter
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateOvalNetwork(QLength scaleFactor,
                                                                     unsigned num_increments = 40,
                                                                     double a_param = 0.5,
                                                                     double b_param = 1.0);
    /**
     * Generate a network on the edges of a Part
     * @param pPart the input part
     * @return a shared pointer to the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > GenerateFromPart(PartPtr<DIM> pPart);

    /**
     * Pattern Unit. Coincident nodes are automatically merged in this method.
     * @param pInputUnit the input unit
     * @param numberOfUnits the number of units in each direction
     */
    void PatternUnitByTranslation(std::shared_ptr<VesselNetwork<DIM> > pInputUnit, std::vector<unsigned> numberOfUnits);

    /**
     * Map the network onto a sphere
     * @param pInputUnit the input unit
     * @param radius the sphere radius
     * @param thickess the sphere thickness
     * @param azimuthExtent the azimuth extents
     * @param polarExtent the polar extents
     */
    void MapToSphere(std::shared_ptr<VesselNetwork<DIM> > pInputUnit,
                     QLength radius,
                     QLength thickess,
                     double azimuthExtent,
                     double polarExtent);

    /**
     * Set the reference length scale
     *
     * @param rLengthScale the reference length scale
     */
    void SetReferenceLengthScale(QLength rReferenceLength);

};

#endif /* VESSELNETWORKGENERATOR_HPP_ */
