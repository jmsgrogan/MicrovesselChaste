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
#include "Vertex.hpp"

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
    VesselNetworkPtr<DIM> GenerateParallelNetwork(PartPtr<DIM> domain,
            QPerArea targetDensity, VesselDistribution::Value distrbutionType,
            QLength exclusionDistance = 0_m, bool useBbox = false,
            std::vector<std::shared_ptr<Vertex<DIM> > > seeds =
                    std::vector<std::shared_ptr<Vertex<DIM> > >());
    /**
     * Creates a hexagonal network corresponding to that of Alarcon et al. (2006)
     * @param width the widht
     * @param height the height
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateHexagonalNetwork(QLength width, QLength height, QLength vesselLength,
            bool fillDomain=false);

    /**
     * Creates a hexagonal network, but with the same length for all the vessels
     * @param width the widht
     * @param height the height
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateHexagonalNetworkEquilateral(QLength width, QLength height, QLength vesselLength=40_um,
            bool fillDomain=false);

 /**
     * Creates a hexagonal network, but with the same length for all the vessels and (max) vessel radius as input
     * @param width the widht
     * @param height the height
     * @param vesselLength the vessel length
     * @param main_radius max radius
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateHexagonalNetworkRadius(QLength width, QLength height, QLength vesselLength, QLength main_radius,
            bool fillDomain=false);

/**
     * Creates a dichotomous/forking network with decreasing length in y direction (and constant in x) for all the vessels and (max) vessel radius as input
     * @param order denotes order
     * @param main_length max length
     * @param main_radius max radius
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateDichotomousNetwork(unsigned order, QLength main_length, QLength main_radius,
            bool fillDomain=false);

/**
     * Creates a dichotomous network with decreasing length in y direction (and constant in x) for all the vessels, (max) vessel radius as input, and with uneven splitting of the radius
     * @param order denotes order
     * @param main_length max length
     * @param main_radius max radius
     * @param alpha level of heterogeneity (in radii between the two daughters)
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateDichotomousNetworkUneven(unsigned order, QLength main_length, QLength main_radius, double alpha,
            bool fillDomain=false);


/**
     * Creates a dichotomous network without corners with decreasing length in y direction (and constant in x) for all the vessels, (max) vessel radius as input, and with uneven splitting of the radius
     * @param order denotes order
     * @param main_length max length
     * @param main_radius max radius
     * @param alpha level of heterogeneity
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateDichotomousNetworkUnevenNoCorners(unsigned order, QLength main_length, QLength main_radius, double alpha,
            bool fillDomain=false);

/**
     * Creates a dichotomous network without corners with decreasing length in y direction and also decreasing (and further on increasing) length in x for all the vessels, (max) vessel radius as input, and with uneven splitting of the radius
     * @param order denotes order
     * @param main_length max length
     * @param main_radius max radius
     * @param alpha level of heterogeneity between daughters
     * @param theta level of decrease in length between the consecutive bifurcations
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateDichotomousNetworkUnevenNoCornersVaryDistance(unsigned order, QLength main_length, QLength main_radius, double alpha, double theta,
            bool fillDomain=false);


/**
     * Creates a dichotomous network without corners with decreasing length in y direction and also decreasing (and further on increasing) length in x for all the vessels, (max) vessel radius as input, with uneven splitting of the radius, and with vessel length following vessel radii
     * @param order denotes order
     * @param main_length max length
     * @param main_radius max radius
     * @param alpha level of heterogeneity between daughters
     * @param theta level of decrease in length between the consecutive bifurcations
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii(unsigned order, QLength main_length, QLength main_radius, double alpha, double theta, double lambda,
            bool fillDomain=false);

/**
     * Creates a dichotomous/forking network without corners with decreasing length in y direction (but more spreaded than initially) and also decreasing (and further on increasing) length in x for all the vessels, input (maximum) vessel radius as input, and with vessel length following vessel radii according to Murray's law
     * These networks will be used in our paper on CFL disruption and recovery effects
     * @param order denotes order
     * @param main_length length of the vertical projection of order-1 vessels
     * @param input_radius input vessel radius
     * @param twicelambda - vessel length divided by vessel radius... in other words, lambda (i.e. vessel length divided by vessel diameter) times 2
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateForkingNetworkNoCorners(unsigned order, QLength main_length, QLength input_radius, double twicelambda,
            bool fillDomain=false);
    /**
     * Creates a hexagonal repeating unit
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateHexagonalUnit(QLength vesselLength);

    /**
     * Creates a two  hexagonal repeating unit
     * @param vesselLength the vessel length
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateTwoHexagonalUnits(QLength vesselLength,  QLength inputRadius, double alpha);

    /**
     * Creates a bifurcation repeating unit
     * @param vesselLength the vessel length
     * @param startPosition the start position of the unit
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateBifurcationUnit(QLength vesselLength, Vertex<DIM> startPosition = Vertex<DIM>());

    VesselNetworkPtr<DIM> GenerateTrueForkedBifurcationUnit(QLength vesselLength, QLength inputRadius, QDimensionless alpha);


    VesselNetworkPtr<DIM> GenerateBranchingNetwork(unsigned order, QLength main_length, QLength input_radius, double twicelambda);

    /**
     * Creates a single vesselW
     * @param vesselLength the vessel length
     * @param startPosition the start position for the vessel
     * @param divisions the number of divisions
     * @param axis the alignment axis
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateSingleVessel(QLength vesselLength,
            Vertex<DIM> startPosition = Vertex<DIM>(), unsigned divisions = 0, unsigned axis = 2);

    /**
     * Creates an oval shaped network with one inlet and one outlet
     * @param scaleFactor a multiplication for from the initial unit length
     * @param num_increments the number of increments
     * @param a_param a width parameter
     * @param a_param a length parameter
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateOvalNetwork(QLength scaleFactor,
            unsigned num_increments = 40, double a_param = 0.5, double b_param = 1.0);
    /**
     * Generate a network on the edges of a Part
     * @param pPart the input part
     * @return a shared pointer to the vessel network
     */
    VesselNetworkPtr<DIM> GenerateFromPart(PartPtr<DIM> pPart);

    /**
     * Pattern Unit. Coincident nodes are automatically merged in this method.
     * @param pInputUnit the input unit
     * @param numberOfUnits the number of units in each direction
     */
    void PatternUnitByTranslation(VesselNetworkPtr<DIM> pInputUnit,
            std::array<unsigned, DIM> numberOfUnits);

    /**
     * Map the network onto a sphere
     * @param pInputUnit the input unit
     * @param radius the sphere radius
     * @param thickess the sphere thickness
     * @param azimuthExtent the azimuth extents
     * @param polarExtent the polar extents
     */
    void MapToSphere(VesselNetworkPtr<DIM> pInputUnit, QLength radius, QLength thickess,
                     double azimuthExtent, double polarExtent);

    /**
     * Set the reference length scale
     *
     * @param rLengthScale the reference length scale
     */
    void SetReferenceLengthScale(QLength rReferenceLength);

};

#endif /* VESSELNETWORKGENERATOR_HPP_ */
