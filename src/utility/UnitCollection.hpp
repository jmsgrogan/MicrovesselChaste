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

#ifndef UNITCOLLECTIONS_HPP
#define UNITCOLLECTIONS_HPP

#include <cmath>
#include <ratio>
#include "Exception.hpp"
#include "ChasteSerialization.hpp"

/**
* Simple Unit Library based on a sample by Benjamin Jurke
* https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/#fn:3
*/
template<typename MassDim, typename LengthDim, typename TimeDim, typename AmountDim, typename AngleDim>
class RQuantity
{
    /**
     * Archiving
     */
    friend class boost::serialization::access;

    /**
     * Do the serialize
     * @param ar the archive
     * @param version the archive version number
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mValue;
    }

protected:

    /**
     * The value of the quantity
     */
    double mValue;

    /**
     * The symbol of the quantity
     */
    const char* mSymbol;

public:

    /**
     * Default value for a quantity
     */
    constexpr RQuantity() :
    	mValue(0.0),
		mSymbol("")
    {

    }

    /**
     * Set the quantity with a value
     */
    constexpr RQuantity(double val) :
        mValue(val),
        mSymbol("")
    {

    }

    /**
     * Set the quantity with a long double value
     */
    constexpr RQuantity(long double val) :
        mValue(static_cast<double>(val)),
        mSymbol("")
    {

    }

    /**
     * Set with a value and symbol
     */
    constexpr RQuantity(long double val, const char* rSymbol) :
        mValue(static_cast<double>(val)),
        mSymbol(rSymbol)
    {

    }

    /**
     * Convert to the supplied unit type
     */
    constexpr double Convert(const RQuantity& rhs) const
    {
        return mValue / rhs.mValue;
    }

    /**
     * Get the raw unit value (should not use)
     */
    constexpr double GetValue() const
    {
        return mValue;
    }

    /**
     * Double conversion type
     */
    constexpr operator double() const
    {
        return mValue;
    }
};


// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Amdim, _Adim, name) \
    typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Amdim>, std::ratio<_Adim>> name;

// Dimensionless
QUANTITY_TYPE(0, 0, 0, 0, 0, QDimensionless);

// Angle:
QUANTITY_TYPE(0, 0, 0, 0, 1, QAngle);

// Time
QUANTITY_TYPE(0, 0, 1, 0, 0, QTime);
QUANTITY_TYPE(0, 0, -1, 0, 0, QRate);

// Length
QUANTITY_TYPE(0, 1, 0, 0, 0, QLength );
QUANTITY_TYPE(0, 2, 0, 0, 0, QArea);
QUANTITY_TYPE(0, 3, 0, 0, 0, QVolume);
QUANTITY_TYPE(0, -1, 0, 0, 0, QPerLength);
QUANTITY_TYPE(0, -2, 0, 0, 0, QPerArea);

// Mass
QUANTITY_TYPE(1, 0, 0, 0, 0, QMass);
QUANTITY_TYPE(1, 0, -1, 0, 0, QMassFlowRate);
QUANTITY_TYPE(1, -2, -1, 0, 0, QMassFlux);

// Amount
QUANTITY_TYPE(0, 0, 0, 1, 0, QAmount);
QUANTITY_TYPE(0, 0, -1, 1, 0, QMolarFlowRate);
QUANTITY_TYPE(0, -3, -1, 1, 0, QConcentrationFlowRate);
QUANTITY_TYPE(0, -2, -1, 1, 0, QMolarFlux);
QUANTITY_TYPE(0, -5, -1, 1, 0, QConcentrationFlux);
QUANTITY_TYPE(0, -3, 0, 1, 0, QConcentration);
QUANTITY_TYPE(0, -4, 0, 1, 0, QConcentrationGradient);
QUANTITY_TYPE(0, 3, -1, -1, 0, QRatePerConcentration);
QUANTITY_TYPE(-1, 0, 0, 1, 0, QMolarMass);
QUANTITY_TYPE(0, -3, 0, 0, 0, QNumberDensity);

// Velocity
QUANTITY_TYPE(0, 1, -1, 0, 0, QVelocity);

// Force/pressure/stress
QUANTITY_TYPE(1, 1, -2, 0, 0, QForce);
QUANTITY_TYPE(1, -1, -2, 0, 0, QPressure);

// Flow
QUANTITY_TYPE(1, -1, -1, 0, 0, QDynamicViscosity);
QUANTITY_TYPE(0, 3, -1, 0, 0, QFlowRate);
QUANTITY_TYPE(1, -4, -1, 0, 0, QFlowImpedance);

// Diffusivity, Solubility, Permeability
QUANTITY_TYPE(0, 2, -1, 0, 0, QDiffusivity );
QUANTITY_TYPE(0, 5, -1, -1, 0, QDiffusivityPerConcentration);
QUANTITY_TYPE(-1, -2, 2, 1, 0, QSolubility);
QUANTITY_TYPE(-1, 1, 2, 0, 0, QVolumetricSolubility);
QUANTITY_TYPE(0, 1, -1, 0, 0, QMembranePermeability);

// Radiation
QUANTITY_TYPE(0, 2, -2, 0, 0, QAbsorbedDose);
QUANTITY_TYPE(0, -2, 2, 0, 0, QPerAbsorbedDose);
QUANTITY_TYPE(0, -4, 4, 0, 0, QPerAbsorbedDoseSquared);


// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<M, L, T, Am, A>
    operator+(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return RQuantity<M, L, T, Am, A>(lhs.GetValue() + rhs.GetValue());
}
template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<M, L, T, Am, A>
    operator-(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return RQuantity<M, L, T, Am, A>(lhs.GetValue() - rhs.GetValue());
}
template <typename M1, typename L1, typename T1, typename Am1, typename A1,
          typename M2, typename L2, typename T2, typename Am2, typename A2>
constexpr RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>,
                    std::ratio_add<T1, T2>, std::ratio_add<Am1, Am2>,
                    std::ratio_add<A1, A2>>
    operator*(const RQuantity<M1, L1, T1, Am1, A1>& lhs, const RQuantity<M2, L2, T2, Am2, A2>& rhs)
{
    return RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>,
                     std::ratio_add<T1, T2>, std::ratio_add<Am1, Am2>, std::ratio_add<A1, A2>>
                    (lhs.GetValue()*rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am, typename A>
constexpr RQuantity<M, L, T, Am, A>
    operator*(const double& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return RQuantity<M, L, T, Am, A>(lhs*rhs.GetValue());
}
template <typename M1, typename L1, typename T1, typename Am1, typename A1,
          typename M2, typename L2, typename T2, typename Am2, typename A2>
constexpr RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>, std::ratio_subtract<Am1, Am2>,
                    std::ratio_subtract<A1, A2>>
    operator/(const RQuantity<M1, L1, T1, Am1, A1>& lhs, const RQuantity<M2, L2, T2, Am2,A2>& rhs)
{
    return RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
                     std::ratio_subtract<T1, T2>,  std::ratio_subtract<Am1, Am2>,
                     std::ratio_subtract<A1, A2>>
                    (lhs.GetValue() / rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, Am>,
                    std::ratio_subtract<std::ratio<0>, A>>
    operator/(double x, const RQuantity<M, L, T, Am, A>& rhs)
{
    return RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
                     std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, Am>,
                     std::ratio_subtract<std::ratio<0>, A>>
                    (x / rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr RQuantity<M, L, T, Am, A>
    operator/(const RQuantity<M, L, T, Am, A>& rhs, double x)
{
    return RQuantity<M, L, T, Am, A>(rhs.GetValue() / x);
}


// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator==(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue() == rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator!=(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue() != rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator<=(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue() <= rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator>=(const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue() >= rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator< (const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue()<rhs.GetValue());
}
template <typename M, typename L, typename T,  typename Am,  typename A>
constexpr bool operator> (const RQuantity<M, L, T, Am, A>& lhs, const RQuantity<M, L, T, Am, A>& rhs)
{
    return (lhs.GetValue()>rhs.GetValue());
}


constexpr long double operator"" _pi(long double x)
    { return static_cast<double>(x) * 3.1415926535897932384626433832795; }
constexpr long double operator"" _pi(unsigned long long int x)
    { return static_cast<double>(x) * 3.1415926535897932384626433832795; }

// Predefined units:
// -----------------
namespace unit{

// Dimensionless
constexpr QDimensionless dimensionless(1.0);

// Angle
constexpr QAngle radians(1.0);
constexpr QAngle degrees = static_cast<double>(2_pi / 360.0) * radians;

// Time
constexpr QTime seconds(1.0, "s");
constexpr QTime minutes = 60.0 * seconds;
constexpr QTime hours = 60.0 * minutes;
constexpr QTime days = 24.0 * hours;
constexpr QRate per_second(1.0);
constexpr QRate per_minute = (1.0/60.0)*per_second;
constexpr QRate per_hour = (1.0/60.0)*per_minute;

// Length
constexpr QLength metres(1.0, "m");
constexpr QLength millimetres = metres / 1.e3;
constexpr QLength centimetres = metres / 10.0;
constexpr QLength microns = metres / 1.e6;
constexpr QArea metres_squared = metres*metres;
constexpr QVolume metres_cubed = metres*metres_squared;
constexpr QVolume microns_cubed = 1.e-18*metres_cubed;

constexpr QPerLength per_metre(1.0);
constexpr QPerArea per_metre_squared = per_metre*per_metre;

// Mass
constexpr QMass kg(1.0);
constexpr QMass grams = kg/1000.0;
constexpr QMassFlowRate kg_per_second(1.0);
constexpr QMassFlux kg_per_metre_squred_per_second(1.0);

// Amount
constexpr QAmount moles(1.0);
constexpr QMolarFlowRate mole_per_second(1.0);
constexpr QMolarFlowRate nanomole_per_hour = (1.e-9/3600.0)*mole_per_second;
constexpr QConcentrationFlowRate mole_per_metre_cubed_per_second(1.0);
constexpr QMolarFlux mole_per_metre_squared_per_second(1.0);
constexpr QConcentrationFlux mole_per_metre_pow5_per_second(1.0);
constexpr QConcentration mole_per_metre_cubed(1.0);
constexpr QConcentration molar = 1.e3*mole_per_metre_cubed;
constexpr QConcentration nanomolar = 1.e-9*molar;
constexpr QConcentrationGradient mole_per_metre_pow4(1.0);
constexpr QRatePerConcentration metre_cubed_per_mole_per_second(1.0);
constexpr QMolarMass mole_per_kg(1.0);
constexpr QNumberDensity per_metre_cubed(1.0);

// Velocity
constexpr QVelocity metres_per_second(1.0);
constexpr QVelocity microns_per_hour = (1.e-6/3600.0)*metres_per_second;

// Force, pressure, stress
constexpr QForce newtons(1.0);
constexpr QPressure pascals(1.0);
constexpr QPressure mmHg = 133.32239 * pascals;

// Flow
constexpr QDynamicViscosity poiseuille(1.0);
constexpr QFlowRate metre_cubed_per_second(1.0);
constexpr QFlowImpedance pascal_second_per_metre_cubed(1.0);

// Diffusivity, Solubility, Permeability
constexpr QDiffusivity metre_squared_per_second(1.0);
constexpr QDiffusivity micron_squared_per_hour = (1.e-12/3600.0)*metre_squared_per_second;
constexpr QDiffusivityPerConcentration metre_pow5_per_second_per_mole(1.0);
constexpr QSolubility mole_per_metre_cubed_per_pascal(1.0);
constexpr QVolumetricSolubility per_pascal(1.0);
constexpr QMembranePermeability metre_per_second(1.0);

// Radiation
constexpr QAbsorbedDose gray(1.0);
constexpr QPerAbsorbedDose per_gray(1.0);
constexpr QPerAbsorbedDoseSquared per_gray_squared(1.0);
}

// Physical unit literals:
// -----------------------

// literals for time units
constexpr QTime operator"" _s(long double x) { return QTime(x); };
constexpr QTime operator"" _min(long double x) { return static_cast<double>(x)*unit::minutes; };
constexpr QTime operator"" _h(long double x) { return static_cast<double>(x)*unit::hours; };
constexpr QTime operator"" _day(long double x) { return static_cast<double>(x)*unit::days; };
constexpr QTime operator"" _s(unsigned long long int x) { return QTime(static_cast<double>(x)); };
constexpr QTime operator"" _min(unsigned long long int x) { return static_cast<double>(x)*unit::minutes; };
constexpr QTime operator"" _h(unsigned long long int x) { return static_cast<double>(x)*unit::hours; };
constexpr QTime operator"" _day(unsigned long long int x) { return static_cast<double>(x)*unit::days; };

// literals for length units
constexpr QLength  operator"" _m(long double x) { return static_cast<double>(x)*unit::metres; }
constexpr QLength  operator"" _m(unsigned long long int  x) { return static_cast<double>(x)*unit::metres; }
constexpr QLength  operator"" _mm(long double x) { return static_cast<double>(x)*unit::millimetres; }
constexpr QLength  operator"" _mm(unsigned long long int  x) { return static_cast<double>(x)*unit::millimetres; }
constexpr QLength  operator"" _um(long double x) { return static_cast<double>(x)*unit::microns; }
constexpr QLength  operator"" _um(unsigned long long int  x) { return static_cast<double>(x)*unit::microns; }

// literals for mass units
constexpr QMass operator"" _kg(long double x) { return QMass(x); };
constexpr QMass operator"" _kg(unsigned long long int x) { return QMass(static_cast<double>(x)); };

// literals for amount units
constexpr QConcentration operator"" _M(long double x) {  return static_cast<double>(x)*unit::molar; };
constexpr QConcentration operator"" _M(unsigned long long int x) {  return static_cast<double>(x)*unit::molar; };
constexpr QConcentration operator"" _nM(long double x) {  return static_cast<double>(x)*unit::nanomolar; };
constexpr QConcentration operator"" _nM(unsigned long long int x) {  return static_cast<double>(x)*unit::nanomolar; };

// literals for force units
constexpr QForce operator"" _N(long double x) { return QForce(x); };
constexpr QForce operator"" _N(unsigned long long int x) { return QForce(static_cast<double>(x)); };

// literals for pressure units
constexpr QPressure operator"" _Pa(long double x) { return QPressure(x); };
constexpr QPressure operator"" _Pa(unsigned long long int x)
                                  { return QPressure(static_cast<double>(x)); };


// Angular unit literals:
// ----------------------

// literals for angle units
constexpr QAngle operator"" _rad(long double x) { return QAngle(x); };
constexpr QAngle operator"" _rad(unsigned long long int x) { return QAngle(static_cast<double>(x)); };
constexpr QAngle operator"" _deg(long double x) { return static_cast<double>(x)*unit::degrees; };
constexpr QAngle operator"" _deg(unsigned long long int x) { return static_cast<double>(x)*unit::degrees; };

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).Convert(1.0_##_y)



// Typesafe mathematical operations:
// ---------------------------------
template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
                    std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<Am, std::ratio<2>>,
                    std::ratio_divide<A, std::ratio<2>>>
    Qsqrt(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
                     std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<Am, std::ratio<2>>,
                     std::ratio_divide<A, std::ratio<2>>>
                    (sqrt(num.GetValue()));
}

template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<2>>, std::ratio_multiply<L, std::ratio<2>>,
                    std::ratio_multiply<T, std::ratio<2>>, std::ratio_multiply<Am, std::ratio<2>>,
                    std::ratio_multiply<A, std::ratio<2>>>
    Qpow2(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<std::ratio_multiply<M, std::ratio<2>>, std::ratio_multiply<L, std::ratio<2>>,
                     std::ratio_multiply<T, std::ratio<2>>, std::ratio_multiply<Am, std::ratio<2>>,
                     std::ratio_multiply<A, std::ratio<2>>>
                    (std::pow(num.GetValue() ,2));
}

template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<3>>, std::ratio_multiply<L, std::ratio<3>>,
                    std::ratio_multiply<T, std::ratio<3>>, std::ratio_multiply<Am, std::ratio<3>>,
                    std::ratio_multiply<A, std::ratio<3>>>
    Qpow3(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<std::ratio_multiply<M, std::ratio<3>>, std::ratio_multiply<L, std::ratio<3>>,
                     std::ratio_multiply<T, std::ratio<3>>, std::ratio_multiply<Am, std::ratio<3>>,
                     std::ratio_multiply<A, std::ratio<3>>>
                    (std::pow(num.GetValue() ,3));
}

template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<4>>, std::ratio_multiply<L, std::ratio<4>>,
                    std::ratio_multiply<T, std::ratio<4>>, std::ratio_multiply<Am, std::ratio<4>>,
                    std::ratio_multiply<A, std::ratio<4>>>
    Qpow4(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<std::ratio_multiply<M, std::ratio<4>>, std::ratio_multiply<L, std::ratio<4>>,
                     std::ratio_multiply<T, std::ratio<4>>, std::ratio_multiply<Am, std::ratio<4>>,
                     std::ratio_multiply<A, std::ratio<4>>>
                    (std::pow(num.GetValue() ,4));
}


template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<3>>, std::ratio_divide<L, std::ratio<3>>,
                    std::ratio_divide<T, std::ratio<3>>, std::ratio_divide<Am, std::ratio<3>>,
                    std::ratio_divide<A, std::ratio<3>>>
    Qcbrt(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<std::ratio_divide<M, std::ratio<3>>, std::ratio_divide<L, std::ratio<3>>,
                     std::ratio_divide<T, std::ratio<3>>, std::ratio_divide<Am, std::ratio<3>>,
                     std::ratio_divide<A, std::ratio<3>>>
                    (std::cbrt(num.GetValue()));
}

template <typename M, typename L, typename T, typename Am, typename A>
constexpr RQuantity<M, L, T, Am, A>
    Qabs(const RQuantity<M, L, T, Am, A>& num)
{
    return RQuantity<M, L, T, Am, A>(std::abs(num.GetValue()));
}

// Typesafe trigonometric operations
inline double Qsin(const QAngle &num)
{
    return std::sin(num.GetValue());
}
inline double Qcos(const QAngle &num)
{
    return std::cos(num.GetValue());
}
inline double Qtan(const QAngle &num)
{
    return std::tan(num.GetValue());
}

#endif /* UNITCOLLECTIONS_HPP */
