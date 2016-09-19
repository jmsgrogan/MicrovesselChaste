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

#ifndef DIMENSIONALCHASTEPOINT_HPP_
#define DIMENSIONALCHASTEPOINT_HPP_

#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include "ChastePoint.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"

/**
 * This class is used in place of ChastePoint when units are important. It is needed as it is difficult to
 * interface Boost Units with c_vectors. As a result most point based classes (e.g. Vertex, VesselNode) use this
 * class for storing locations with units and calculating point distances.
 */
template<unsigned DIM>
class DimensionalChastePoint : public ChastePoint<DIM>, public boost::enable_shared_from_this<DimensionalChastePoint<DIM> >
{
    /**
     * The reference length scale for the point, default in microns. This is needed as units can't be combined
     * with c_vectors, which hold the point's location. If the length scale is changed the
     * values in mLocation will be changed accordingly. i.e. if the reference length scale is changed
     * from 1 micron to 40 micron, the value in mLocation will be divided by 40.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     */
    DimensionalChastePoint(double x, double y, double z, units::quantity<unit::length> referenceLength);

    /**
     * Constructor
     * @param a vector of x, y, z coordinates
     */
    DimensionalChastePoint(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength);

    /**
     * Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     */
    DimensionalChastePoint(double x = 0.0, double y = 0.0, double z = 0.0);

    /**
     * Constructor
     * @param a vector of x, y, z coordinates
     */
    DimensionalChastePoint(c_vector<double, DIM> coords);

    /**
     * Destructor
     */
    virtual ~DimensionalChastePoint();

    /**
     * Return the reference length scale for the point, default is micron
     *
     * @return a the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale() const;

    /**
     * Set the length scale used to dimensionalize the point location as stored in mLocation.
     * If you want locations to be in metres, for example, set it to 1.0*unit::metres. The point
     * location values are changed accordingly when this value is changed.
     *
     * @param lenthScale the reference length scale for point locations
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lenthScale);

    /**
     * Get the ratio of length scales between this point and the input point
     * @param rLocation the input point
     * @return the ratio of length scales between this point in the input point
     */
    double GetScalingFactor(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Get the distance between this point and the input point
     * @param rLocation the input point
     * @param checkDimensions check if the dimensions of the input point need to be scaled
     * @return the distance between this point and the input point
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions = true) const;

    static units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rStartLocation,
                                                     const DimensionalChastePoint<DIM>& rEndLocation,
                                                     const DimensionalChastePoint<DIM>& rProbeLocation,
                                                     bool checkDimensions = true);

    /**
     * Return a point midway between this point and the input point
     * @param rLocation the input point
     * @param checkDimensions check if the dimensions of the input point need to be scaled
     * @return a point midway between this point and the input point
     */
    DimensionalChastePoint<DIM> GetMidPoint(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions = true) const;

    static DimensionalChastePoint<DIM> GetPointProjection(const DimensionalChastePoint<DIM>& rStartLocation,
                                                          const DimensionalChastePoint<DIM>& rEndLocation,
                                                          const DimensionalChastePoint<DIM>& rProbeLocation,
                                                          bool projectToEnds = false,
                                                          bool checkDimensions = true);

    /**
     * Return the unit tangent to the segment formed by this point and the input point
     * @param rLocation the input point
     * @param checkDimensions check if the dimensions of the input point need to be scaled
     * @return the unit tangent to the segment formed by this point and the input point
     */
    c_vector<double, DIM> GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions = true) const;

    /**
     * Return true if the input point is coincident with this point
     * @param rLocation the input point
     * @param checkDimensions check if the dimensions of the input point need to be scaled
     * @return true if the input point is coincident with this point
     */
    bool IsCoincident(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions = true) const;

    void Translate(DimensionalChastePoint<DIM> rVector);


};

#endif /*DIMENSIONALCHASTEPOINT_HPP_*/
