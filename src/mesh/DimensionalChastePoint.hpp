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



#ifndef DIMENSIONALCHASTEPOINT_HPP_
#define DIMENSIONALCHASTEPOINT_HPP_

#include <vector>
#include "ChasteSerialization.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"

/**
 * This class is used in place of ChastePoint when units are important. It is needed to
 * interface Boost Units with c_vectors. It is a fundamental geometric feature for storing locations.
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
        ar & mIndex;
    }

protected:

    /**
     * The location of the Point.
     */
    c_vector<double, DIM> mLocation;

    /**
     * The reference length scale for the point, default in microns. This is needed as units can't be combined
     * with c_vectors, which hold the point's location. If the length scale is changed the
     * values in mLocation will be changed accordingly. i.e. if the reference length scale is changed
     * from 1 micron to 40 micron, the value in mLocation will be divided by 40.
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * An optional index
     */
    unsigned mIndex;

public:

    /**
     * Constructor. Be careful with the units of the default constructor. It should only be used for pre-allocating
     * vectors.
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     * @param referenceLength the reference length
     */
    DimensionalChastePoint(double x= 0.0, double y = 0.0, double z = 0.0, units::quantity<unit::length> referenceLength = 1.e-6*unit::metres);

    /**
     * Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     */
    DimensionalChastePoint(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength);

    /**
     * Factory Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static boost::shared_ptr<DimensionalChastePoint<DIM> > Create(double x, double y, double z, units::quantity<unit::length> referenceLength);

    /**
     * Factory Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static boost::shared_ptr<DimensionalChastePoint<DIM> > Create(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength);

    /**
     * Destructor
     */
    virtual ~DimensionalChastePoint();

    /**
     * Get the distance between this point and the input point
     * @param rLocation the input point
     * @return the distance between this point and the input point
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation) const;

    /**
     * Return the index
     * @return the point index
     */
    unsigned GetIndex();

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
     * Return the reference length scale for the point, default is micron
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale() const;

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.
     */
    c_vector<double, DIM> GetLocation(units::quantity<unit::length> scale);

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.  Constant non-liberal variety.
     */
    const c_vector<double, DIM> GetLocation(units::quantity<unit::length> scale) const;

    /**
     * Return a point one unit from the origin in the direction along the vector between this point and the origin
     * @return a point one unit from the origin in the direction along the vector between this point and the origin
     */
    c_vector<double, DIM> GetUnitVector() const;

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
     * Overload division from self
     * @return the resultant point
     * @param factor the vector to be added to
     */
    DimensionalChastePoint<DIM>& operator/=(double factor);

    /**
     * Overload multiplication with self
     * @return the resultant point
     * @param factor the scalar to be multiplied
     */
    DimensionalChastePoint<DIM>& operator*=(double factor);

    /**
     * Overload addition to SELF
     * @return the resultant point
     * @param rLocation the vector to add
     */
    DimensionalChastePoint<DIM>& operator+=(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Overload subraction from self
     * @return the resultant point
     * @param rLocation the vector to subtract
     */
    DimensionalChastePoint<DIM>& operator-=(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Rotate about the axis by the supplied angle
     *
     * @param axis the axis
     * @param angle the rotation ange
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Set the length scale used to dimensionalize the point location as stored in mLocation. The point
     * location values are changed accordingly when this value is changed.
     *
     * @param lenthScale the reference length scale for point locations
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lenthScale);

    /**
     * Set the index
     * @param index the point index
     */
    void SetIndex(unsigned index);

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
 * @return return the division result
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator/(DimensionalChastePoint<DIM> lhs, double factor)
{
    lhs /= factor;
    return lhs;
}

/**
 * Overload multiplication
 * @return the resultant point
 * @param lhs the left hand part of the multiplication operation
 * @param factor the right hand part of the multiplication operation
 * @return return the multiplication result
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator*(DimensionalChastePoint<DIM> lhs, double factor)
{
    lhs *= factor;
    return lhs;
}

/**
 * Overload addition
 * @return the resultant point
 * @param lhs the left hand part of the addition operation
 * @param rLocation the right hand part of the addition operation
 * @return return the addition result
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator+(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation)
{
    lhs += rLocation;
    return lhs;
}

/**
 * Overload subtract
 * @return the resultant point
 * @param lhs the left hand part of the subtract operation
 * @param rLocation the right hand part of the subtract operation
 * @return return the subtraction result
 */
template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator-(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation)
{
    lhs -= rLocation;
    return lhs;
}

#endif /*DIMENSIONALCHASTEPOINT_HPP_*/
