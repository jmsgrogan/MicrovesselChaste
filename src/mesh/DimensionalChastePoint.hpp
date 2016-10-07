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
#include "ChasteSerialization.hpp"
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
class DimensionalChastePoint
{
    /**
     * Archiving
     */
    friend class boost::serialization::access;

    /**
     * Do the serialization
     * @param ar the archive
     * @param version the archive version
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mReferenceLength;
        ar & mLocation;
    }

protected:

    /** The location of the Point. */
    c_vector<double, DIM> mLocation;

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
     * @param referenceLength the reference length
     */
    DimensionalChastePoint(double x, double y, double z, units::quantity<unit::length> referenceLength);

    /**
     * Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
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
     * @param coords a vector of x, y, z coordinates
     */
    DimensionalChastePoint(c_vector<double, DIM> coords);

    /**
     * Destructor
     */
    virtual ~DimensionalChastePoint();

    /**
     * Return the reference length scale for the point, default is micron
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale() const;

    /**
     * @return the location of the Point.
     */
    c_vector<double, DIM>& rGetLocation();

    /**
     * @return the location of the Point.  Constant non-liberal variety.
     */
    const c_vector<double, DIM>& rGetLocation() const;

    /**
     * Get the ratio of length scales between this point and the input point
     * @param rLocation the input point
     * @return the ratio of length scales between this point in the input point
     */
    double GetScalingFactor(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Get the distance between this point and the input point
     * @param rLocation the input point
     * @return the distance between this point and the input point
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Get the distance between two points
     * @param rLocation1 the input point 1
     * @param rLocation2 the input point 2
     * @return the distance between this point and the input point
     */
    static units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation1,
                                                     const DimensionalChastePoint<DIM>& rLocation2);

    /**
     * Get the distance between the line defined by the start and end locations and the probe location.
     * @param rStartLocation the start location on the line
     * @param rEndLocation the end location on the line
     * @param rProbeLocation the probe location
     * @return the distance between the line and probe point
     */
    static units::quantity<unit::length> GetDistanceToLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                     const DimensionalChastePoint<DIM>& rEndLocation,
                                                     const DimensionalChastePoint<DIM>& rProbeLocation);

    /**
     * Get the dot product of the vectors between each point and the origin
     * @param rLocation1 the input point 1
     * @param rLocation2 the input point 2
     * @return the dot product of the vectors between each point and the origin
     */
    static units::quantity<unit::length> GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                                     const DimensionalChastePoint<DIM>& rLocation2);

    /**
     * Return a point midway between this point and the input point
     * @param rLocation the input point
     * @return a point midway between this point and the input point
     */
    DimensionalChastePoint<DIM> GetMidPoint(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Return the length of the vector between this point and the origin
     * @return the length of the vector between this point and the origin
     */
    units::quantity<unit::length> GetNorm2();



    /**
     * Return the projection of a point onto the line defined by the start and end locations
     * @param rStartLocation the start location on the line
     * @param rEndLocation the end location on the line
     * @param rProbeLocation the probe location
     * @param projectToEnds whether to project onto the end points
     * @param checkDimensions check if the dimensions of the input point need to be scaled
     * @return the projection of the probe point onto a line
     */
    static DimensionalChastePoint<DIM> GetPointProjectionOnLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                          const DimensionalChastePoint<DIM>& rEndLocation,
                                                          const DimensionalChastePoint<DIM>& rProbeLocation,
                                                          bool projectToEnds = false,
                                                          bool checkDimensions = true);

    /**
     * Return a point one unit from the origin in the direction along the vector between this point and the origin
     * @return a point one unit from the origin in the direction along the vector between this point and the origin
     */
    DimensionalChastePoint<DIM> GetUnitVector();

    /**
     * Return the unit tangent to the segment formed by this point and the input point
     * @param rLocation the input point
     * @return the unit tangent to the segment formed by this point and the input point
     */
    c_vector<double, DIM> GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Return true if the input point is coincident with this point
     * @param rLocation the input point
     * @return true if the input point is coincident with this point
     */
    bool IsCoincident(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * @return the vector mLocation.
     *
     * @param i the index of the vector to return
     */
    double operator[] (unsigned i) const;

    /**
     * Overload division from self
     * @return the resultant point
     *
     * @param rLocation the vector to be added to
     */
    DimensionalChastePoint<DIM>& operator/=(double factor);

    /**
     * Overload multiplication with self
     * @return the resultant point
     *
     * @param factor the scalar to be multiplied
     */
    DimensionalChastePoint<DIM>& operator*=(double factor);

    /**
     * Overload addition to SELF
     * @return the resultant point
     *
     * @param rLocation the vector to add
     */
    DimensionalChastePoint<DIM>& operator+=(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Overload subraction from self
     * @return the resultant point
     *
     * @param rLocation the vector to subtract
     */
    DimensionalChastePoint<DIM>& operator-=(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Set the length scale used to dimensionalize the point location as stored in mLocation. The point
     * location values are changed accordingly when this value is changed.
     *
     * @param lenthScale the reference length scale for point locations
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lenthScale);

    /**
     * Set one of the coordinates of the Point.
     *
     * @param i the index of the coordinate
     * @param value the value of the coordinate
     */
    void SetCoordinate(unsigned i, double value);

    /**
     * Translate the point along the supplied vector
     * @param rVector the translation vector
     */
    void Translate(DimensionalChastePoint<DIM> rVector);

    /**
     * Translate the point to the new point
     * @param rPoint the new point
     */
    void TranslateTo(DimensionalChastePoint<DIM> rPoint);
};

/**
 * Overload division
 * @return the resultant point
 * @param lhs the left hand part of the division operation
 * @param rLocation the right hand part of the division operation
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator/(DimensionalChastePoint<DIM> lhs, double factor);

/**
 * Overload multiplication
 * @return the resultant point
 * @param lhs the left hand part of the multiplication operation
 * @param factor the right hand part of the multiplication operation
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator*(DimensionalChastePoint<DIM> lhs, double factor);

/**
 * Overload addition
 * @return the resultant point
 * @param lhs the left hand part of the addition operation
 * @param rLocation the right hand part of the addition operation
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator+(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation);

/**
 * Overload subtract
 * @return the resultant point
 * @param lhs the left hand part of the subtract operation
 * @param rLocation the right hand part of the subtract operation
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator-(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation);

#endif /*DIMENSIONALCHASTEPOINT_HPP_*/
