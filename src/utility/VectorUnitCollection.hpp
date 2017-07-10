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

#ifndef VECTORUNITCOLLECTIONS_HPP
#define VECTORUNITCOLLECTIONS_HPP

#include <cmath>
#include <ratio>
#include "Exception.hpp"
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "ChasteSerialization.hpp"
#include "UnitCollection.hpp"

/**
* Vector container for Unit types
*/

template<typename RQuantity, unsigned DIM>
class RVectorQuantity
{
//    /**
//     * Archiving
//     */
//    friend class boost::serialization::access;
//
//    /**
//     * Do the serialize
//     * @param ar the archive
//     * @param version the archive version number
//     */
//    template<class Archive>
//    void serialize(Archive & ar, const unsigned int version)
//    {
//        ar & mValue;
//    }

    c_vector<double, DIM> mValue;

public:

    constexpr RVectorQuantity() :
    mValue(scalar_vector<double>(DIM, 1.0))
    {

    }

    constexpr RVectorQuantity(c_vector<double, DIM> val) :
        mValue(val)
    {

    }

    constexpr RVectorQuantity(double val) :
        mValue(scalar_vector<double>(DIM, val))
    {

    }

    // Returns the value of the quantity in multiples of the specified unit
    constexpr c_vector<double, DIM> Convert(const RQuantity& rhs) const
    {
        return mValue / rhs.getValue();
    }

    // returns the raw value of the quantity (should not be used)
    constexpr c_vector<double, DIM> getValue() const
    {
        return mValue;
    }

    constexpr operator c_vector<double, DIM>() const
    {
        return mValue;
    }

    constexpr RQuantity operator[] (unsigned i) const
    {
        return mValue[i]*RQuantity(1.0);
    }
};


// Predefined (physical unit) quantity types:
// ------------------------------------------

// Dimensionless
template<unsigned DIM>
using VecQDimensionless = RVectorQuantity<QDimensionless, DIM>;

// Length
template<unsigned DIM>
using VecQLength = RVectorQuantity<QLength, DIM>;
const VecQLength<2> QOrigin2(0.0);
const VecQLength<3> QOrigin3(0.0);

// Amount
template<unsigned DIM>
using VecQConcentration = RVectorQuantity<QConcentration, DIM>;


// Standard arithmetic operators:
// ------------------------------
template <typename Quantity, unsigned DIM>
constexpr RVectorQuantity<Quantity, DIM>
    operator+(const RVectorQuantity<Quantity, DIM>& lhs, const RVectorQuantity<Quantity, DIM>& rhs)
{
    return RVectorQuantity<Quantity, DIM>(lhs.getValue() + rhs.getValue());
}
template <typename Quantity, unsigned DIM>
constexpr RVectorQuantity<Quantity, DIM>
    operator-(const RVectorQuantity<Quantity, DIM>& lhs, const RVectorQuantity<Quantity, DIM>& rhs)
{
    return RVectorQuantity<Quantity, DIM>(lhs.getValue() - rhs.getValue());
}

template <typename Quantity, unsigned DIM>
constexpr RVectorQuantity<Quantity, DIM>
    operator*(const double& lhs, const RVectorQuantity<Quantity, DIM>& rhs)
{
    return RVectorQuantity<Quantity, DIM>(lhs*rhs.getValue());
}

template <typename Quantity, unsigned DIM>
constexpr RVectorQuantity<Quantity, DIM>
    operator/(const RVectorQuantity<Quantity, DIM>& rhs, double x)
{
    return RVectorQuantity<Quantity, DIM>(rhs.getValue() / x);
}


// Comparison operators for quantities:
// ------------------------------------
template <typename Quantity, unsigned DIM>
constexpr bool operator==(const RVectorQuantity<Quantity, DIM>& lhs, const RVectorQuantity<Quantity, DIM>& rhs)
{
    return (lhs.getValue() == rhs.getValue());
}
template <typename Quantity, unsigned DIM>
constexpr bool operator!=(const RVectorQuantity<Quantity, DIM>& lhs, RVectorQuantity<Quantity, DIM>& rhs)
{
    return (lhs.getValue() != rhs.getValue());
}

// Typesafe mathematical operations:
// ---------------------------------
template <typename Quantity, unsigned DIM>
constexpr Quantity
    Qnorm2(const RVectorQuantity<Quantity, DIM>& num)
{
    return norm2(num.getValue());
}

#endif /* VECTORUNITCOLLECTIONS_HPP */
