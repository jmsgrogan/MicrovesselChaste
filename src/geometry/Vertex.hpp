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

#ifndef Vertex_HPP_
#define Vertex_HPP_

#include <vector>
#include <map>
#include <string>
#include <boost/serialization/map.hpp>
#include "ChasteSerialization.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "VectorUnitCollection.hpp"
#include "BaseUnits.hpp"
#include "Exception.hpp"

/**
 * This class is used in place of ChastePoint when units are important. It is needed to
 * interface Boost Units with c_vectors and the VecQLength types.
 */
template<unsigned DIM>
class Vertex
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
        ar & mLocation;
        ar & mIndex;
        ar & mAttributes;
    }

protected:

    /**
     * The location of the Point relative to the origin.
     */
    VecQLength<DIM> mLocation;

    /**
     * The reference length scale for the point.
     */
    QLength mReferenceLength;

    /**
     * An optional index
     */
    unsigned mIndex;

    /**
     * Attributes, for when used as geometry vertices
     */
    std::map<std::string, double> mAttributes;

public:

    /**
     * Constructor.
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     */
    Vertex(QLength x= 0_m, QLength y = 0_m, QLength z = 0_m);

    /**
     * Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     */
    Vertex(const c_vector<double, DIM>& rCoords, QLength referenceLength);

    /**
     * Constructor
     * @param loc a vector of x, y, z coordinates
     */
    Vertex(VecQLength<DIM> loc);

    /**
     * Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     */
    Vertex(const double (&rCoords)[3], QLength referenceLength);

    /**
     * Factory Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     * @return a pointer to the point
     */
    static std::shared_ptr<Vertex<DIM> > Create(QLength x= 0_m, QLength y = 0_m, QLength z = 0_m);

    /**
     * Factory Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static std::shared_ptr<Vertex<DIM> > Create(const c_vector<double, DIM>& rCoords, QLength referenceLength);

    /**
     * Factory Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static std::shared_ptr<Vertex<DIM> > Create(VecQLength<DIM> loc);

    /**
     * Factory Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static std::shared_ptr<Vertex<DIM> > Create(const Vertex<DIM>& loc);

    /**
     * Factory Constructor
     * @param coords a vector of x, y, z coordinates
     * @param referenceLength the reference length
     * @return a pointer to the point
     */
    static std::shared_ptr<Vertex<DIM> > Create(const double (&rCoords)[3], QLength referenceLength);

    /**
     * Destructor
     */
    virtual ~Vertex();

    /**
     * Add an attribute
     * @param rAttribute the label
     * @param value the value
     */
    void AddAttribute(const std::string& rAttribute, double value);

    /**
     * Get the distance between this point and the input point
     * @param rLocation the input point
     * @return the distance between this point and the input point
     */
    QLength GetDistance(const VecQLength<DIM>& rLocation) const;

    /**
     * Get the distance between this point and the input point
     * @param rLocation the input point
     * @return the distance between this point and the input point
     */
    QLength GetDistance(const Vertex<DIM>& rLocation) const;

    /**
     * Return the index
     * @return the point index
     */
    unsigned GetIndex();

    /**
     * Return the attributes
     * @return the attributes
     */
    std::map<std::string, double> GetAttributes();

    /**
     * Return a point midway between this point and the input point
     * @param rLocation the input point
     * @return a point midway between this point and the input point
     */
    Vertex<DIM> GetMidPoint(const Vertex<DIM>& rLocation) const;

    /**
     * Return the length of the vector between this point and the origin
     * @return the length of the vector between this point and the origin
     */
    QLength GetNorm2();

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.
     */
    c_vector<double, 3> Convert3(QLength referenceLength) const;

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.
     */
    c_vector<double, DIM> Convert(QLength referenceLength) const;

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.
     */
    void Convert(double (&rLoc)[3], QLength referenceLength) const;

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.
     */
    VecQLength<DIM>& rGetLocation();

    /**
     * Return a non-dimensional location, normalized by the supplied length scale
     * @param scale the length scale for the point
     * @return the location of the Point.  Constant non-liberal variety.
     */
    const VecQLength<DIM>& rGetLocation() const;

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
    c_vector<double, DIM> GetUnitTangent(const Vertex<DIM>& rLocation) const;

    /**
     * Return true if the input point is coincident with this point
     * @param rLocation the input point
     * @return true if the input point is coincident with this point
     */
    bool IsCoincident(const Vertex<DIM>& rLocation) const;

    const QLength operator[] (unsigned i) const
    {
        if(i>=DIM)
        {
            EXCEPTION("Requested index out of bounds");
        }
        return mLocation[i];
    }

    /**
     * Overload division from self
     * @return the resultant point
     * @param factor the vector to be added to
     */
    Vertex<DIM>& operator/=(double factor);

    /**
     * Overload multiplication with self
     * @return the resultant point
     * @param factor the scalar to be multiplied
     */
    Vertex<DIM>& operator*=(double factor);

    /**
     * Overload addition to SELF
     * @return the resultant point
     * @param rLocation the vector to add
     */
    Vertex<DIM>& operator+=(const Vertex<DIM>& rLocation);

    /**
     * Overload subraction from self
     * @return the resultant point
     * @param rLocation the vector to subtract
     */
    Vertex<DIM>& operator-=(const Vertex<DIM>& rLocation);

    /**
     * Rotate about the axis by the supplied angle
     *
     * @param axis the axis
     * @param angle the rotation ange
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Set the index
     * @param index the point index
     */
    void SetIndex(unsigned index);

    /**
     * Translate the point along the supplied vector
     * @param rVector the translation vector
     */
    void Translate(const Vertex<DIM>& rVector);

    /**
     * Translate the point to the new point
     * @param rPoint the new point
     */
    void TranslateTo(const Vertex<DIM>& rPoint);

};

/**
 * Overload division
 * @return the resultant point
 * @param lhs the left hand part of the division operation
 * @param rLocation the right hand part of the division operation
 * @return return the division result
 */
template<unsigned DIM>
inline Vertex<DIM> operator/(Vertex<DIM> lhs, double factor)
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
inline Vertex<DIM> operator*(Vertex<DIM> lhs, double factor)
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
inline Vertex<DIM> operator+(Vertex<DIM> lhs, const Vertex<DIM>& rLocation)
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
inline Vertex<DIM> operator-(Vertex<DIM> lhs, const Vertex<DIM>& rLocation)
{
    lhs -= rLocation;
    return lhs;
}

/**
 * Define a simple vertex type
 */
template <unsigned DIM>
using VertexPtr = std::shared_ptr<Vertex<DIM> >;

#endif /*Vertex_HPP_*/
