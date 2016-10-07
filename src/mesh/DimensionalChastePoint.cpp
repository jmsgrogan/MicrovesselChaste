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

#include "UblasIncludes.hpp"
#include "DimensionalChastePoint.hpp"

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(double x, double y, double z, units::quantity<unit::length> referenceLength) :
        mReferenceLength(referenceLength)
{
    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    if (DIM > 0)
    {
        mLocation[0] = x;
    }
    if (DIM > 1)
    {
        mLocation[1] = y;
    }
    if (DIM > 2)
    {
        mLocation[2] = z;
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength) :
        mReferenceLength(referenceLength)
{
    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    for (unsigned i=0; i<DIM; i++)
    {
        mLocation(i) = coords.at(i);
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(double x, double y, double z) :
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    if (DIM > 0)
    {
        mLocation[0] = x;
    }
    if (DIM > 1)
    {
        mLocation[1] = y;
    }
    if (DIM > 2)
    {
        mLocation[2] = z;
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(c_vector<double, DIM> coords) :
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    for (unsigned i=0; i<DIM; i++)
    {
        mLocation(i) = coords.at(i);
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::~DimensionalChastePoint()
{

}

template<unsigned DIM>
c_vector<double, DIM>& DimensionalChastePoint<DIM>::rGetLocation(units::quantity<unit::length> scale)
{
    return mLocation*(scale/mReferenceLength);
}

template<unsigned DIM>
const c_vector<double, DIM>& DimensionalChastePoint<DIM>::rGetLocation(units::quantity<unit::length> scale) const
{
    return mLocation*(scale/mReferenceLength);
}

template<unsigned DIM>
double DimensionalChastePoint<DIM>::operator[] (unsigned i) const
{
    assert(i<DIM);
    return mLocation(i);
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetReferenceLengthScale() const
{
    return mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation) const
{
    return norm_2(rLocation.rGetLocation(mReferenceLength) - mLocation)*mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetNorm2()
{
    return norm_2(mLocation)*mReferenceLength;
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitVector()
{
    return mLocation/norm_2(mLocation);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetMidPoint(const DimensionalChastePoint<DIM>& rLocation) const
{
    return DimensionalChastePoint<DIM>((rLocation.rGetLocation(mReferenceLength) + mLocation) / 2.0, mReferenceLength);
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation) const
{
    return (rLocation.rGetLocation(mReferenceLength)- mLocation) / (GetDistance(rLocation)/mReferenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator/=(double factor)
{
    mLocation /= factor;
    return *this;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator*=(double factor)
{
    mLocation *= factor;
    return *this;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator+=(const DimensionalChastePoint<DIM>& rLocation)
{
    mLocation += rLocation.rGetLocation(mReferenceLength);
    return *this;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator-=(const DimensionalChastePoint<DIM>& rLocation)
{
    mLocation -= rLocation.rGetLocation(mReferenceLength);
    return *this;
}

template<unsigned DIM>
bool DimensionalChastePoint<DIM>::IsCoincident(const DimensionalChastePoint<DIM>& rLocation) const
{
    double scaling_length = rLocation.GetReferenceLengthScale()/mReferenceLength;
    bool returned_value = true;
    for (unsigned dim=0; dim<DIM; dim++)
    {
        if (rLocation[dim]*scaling_length != mLocation[dim])
        {
            returned_value = false;
            break;
        }
    }
    return returned_value;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::SetCoordinate(unsigned i, double value)
{
    assert(i < DIM);
    mLocation(i) = value;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::SetReferenceLengthScale(units::quantity<unit::length> lenthScale)
{
    if(lenthScale == 0.0*unit::metres)
    {
        EXCEPTION("Attempted to assign a zero length scale");
    }
    mLocation *= (mReferenceLength/lenthScale);
    mReferenceLength = lenthScale;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::Translate(DimensionalChastePoint<DIM> rVector)
{
    mLocation += rVector.rGetLocation(mReferenceLength);
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::TranslateTo(DimensionalChastePoint<DIM> rPoint)
{
    mLocation = rPoint.rGetLocation(mReferenceLength);
}

// Explicit instantiation
template class DimensionalChastePoint<2>;
template class DimensionalChastePoint<3>;
