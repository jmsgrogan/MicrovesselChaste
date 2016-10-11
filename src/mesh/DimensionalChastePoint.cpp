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
        mReferenceLength(referenceLength),
        mIndex(0)
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
        mReferenceLength(referenceLength),
        mIndex(0)
{
    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    for (unsigned i=0; i<DIM; i++)
    {
        mLocation(i) = coords[i];
    }
}

template<unsigned DIM>
boost::shared_ptr<DimensionalChastePoint<DIM> > DimensionalChastePoint<DIM>::Create(double x, double y, double z, units::quantity<unit::length> referenceLength)
{
    MAKE_PTR_ARGS(DimensionalChastePoint<DIM>, p_point, (x, y, z, referenceLength));
    return p_point;
}

template<unsigned DIM>
boost::shared_ptr<DimensionalChastePoint<DIM> > DimensionalChastePoint<DIM>::Create(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength)
{
    MAKE_PTR_ARGS(DimensionalChastePoint<DIM>, p_point, (coords, referenceLength));
    return p_point;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::~DimensionalChastePoint()
{

}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetLocation(units::quantity<unit::length> scale)
{
    return mLocation*(mReferenceLength/scale);
}

template<unsigned DIM>
const c_vector<double, DIM> DimensionalChastePoint<DIM>::GetLocation(units::quantity<unit::length> scale) const
{
    return mLocation*(mReferenceLength/scale);
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetReferenceLengthScale() const
{
    return mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation) const
{
    return norm_2(rLocation.GetLocation(mReferenceLength) - mLocation)*mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetNorm2()
{
    return norm_2(mLocation)*mReferenceLength;
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitVector() const
{
    return mLocation/norm_2(mLocation);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetMidPoint(const DimensionalChastePoint<DIM>& rLocation) const
{
    return DimensionalChastePoint<DIM>((rLocation.GetLocation(mReferenceLength) + mLocation) / 2.0, mReferenceLength);
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation) const
{
    return (rLocation.GetLocation(mReferenceLength)- mLocation) / (GetDistance(rLocation)/mReferenceLength);
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
    mLocation += rLocation.GetLocation(mReferenceLength);
    return *this;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator-=(const DimensionalChastePoint<DIM>& rLocation)
{
    mLocation -= rLocation.GetLocation(mReferenceLength);
    return *this;
}

template<unsigned DIM>
bool DimensionalChastePoint<DIM>::IsCoincident(const DimensionalChastePoint<DIM>& rLocation) const
{
    bool returned_value = true;
    c_vector<double, DIM> comparison_loc = rLocation.GetLocation(mReferenceLength);
    for (unsigned dim=0; dim<DIM; dim++)
    {
        if (comparison_loc[dim] != mLocation[dim])
        {
            returned_value = false;
            break;
        }
    }
    return returned_value;
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
void DimensionalChastePoint<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    double sin_a = std::sin(angle);
    double cos_a = std::cos(angle);
    c_vector<double, 3> unit_axis = axis / norm_2(axis);
    if(DIM==2 and unit_axis[2]!= 1.0)
    {
        EXCEPTION("2D rotation is about z axis only");
    }

    c_vector<double, DIM> old_location = this->mLocation;
    c_vector<double, DIM> new_location;
    if(DIM==3)
    {
        double dot_product = inner_prod(old_location, unit_axis);
        new_location[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + old_location[0] * cos_a
                    + (-unit_axis[2] * old_location[1] + unit_axis[1] * old_location[2]) * sin_a);
        new_location[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + old_location[1] * cos_a
                    + (unit_axis[2] * old_location[0] - unit_axis[0] * old_location[2]) * sin_a);
        new_location[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + old_location[2] * cos_a
                    + (-unit_axis[1] * old_location[0] + unit_axis[0] * old_location[1]) * sin_a);
    }
    else
    {
        new_location[0] = old_location[0] * cos_a;
        new_location[1] = old_location[1] * cos_a;
    }

    this->mLocation = new_location;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::Translate(DimensionalChastePoint<DIM> rVector)
{
    mLocation += rVector.GetLocation(mReferenceLength);
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::TranslateTo(DimensionalChastePoint<DIM> rPoint)
{
    mLocation = rPoint.GetLocation(mReferenceLength);
}

template<unsigned DIM>
unsigned DimensionalChastePoint<DIM>::GetIndex()
{
    return mIndex;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::SetIndex(unsigned index)
{
    mIndex = index;
}

// Explicit instantiation
template class DimensionalChastePoint<2>;
template class DimensionalChastePoint<3>;
