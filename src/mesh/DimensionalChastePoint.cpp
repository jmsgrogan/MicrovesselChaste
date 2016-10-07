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
c_vector<double, DIM>& DimensionalChastePoint<DIM>::rGetLocation()
{
    return mLocation;
}

template<unsigned DIM>
const c_vector<double, DIM>& DimensionalChastePoint<DIM>::rGetLocation() const
{
    return mLocation;
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
double DimensionalChastePoint<DIM>::GetScalingFactor(const DimensionalChastePoint<DIM>& rLocation) const
{
    return rLocation.GetReferenceLengthScale()/mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation) const
{
    return norm_2(rLocation.rGetLocation()*GetScalingFactor(rLocation) - mLocation)*mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation1,
                                                                       const DimensionalChastePoint<DIM>& rLocation2)
{
    units::quantity<unit::length> reference_length_1 = rLocation1.GetReferenceLengthScale();
    units::quantity<unit::length> reference_length_2 = rLocation2.GetReferenceLengthScale();

    if(reference_length_1 == 0.0*unit::metres)
    {
        EXCEPTION("Point one has zero reference length");
    }

    return norm_2(rLocation2.rGetLocation()*(reference_length_2/reference_length_1) - rLocation1.rGetLocation())*reference_length_1;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                                                       const DimensionalChastePoint<DIM>& rLocation2)
{
    units::quantity<unit::length> reference_length_1 = rLocation1.GetReferenceLengthScale();
    units::quantity<unit::length> reference_length_2 = rLocation2.GetReferenceLengthScale();
    if(reference_length_1 == 0.0*unit::metres)
    {
        EXCEPTION("Point one has zero reference length");
    }

    return inner_prod(rLocation2.rGetLocation()*(reference_length_2/reference_length_1) , rLocation1.rGetLocation())*reference_length_1;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetNorm2()
{
    return norm_2(mLocation)*mReferenceLength;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetUnitVector()
{
    return DimensionalChastePoint<DIM>(mLocation/norm_2(mLocation), mReferenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetPointProjectionOnLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                      const DimensionalChastePoint<DIM>& rEndLocation,
                                                      const DimensionalChastePoint<DIM>& rProbeLocation,
                                                      bool projectToEnds,
                                                      bool checkDimensions)
{
    DimensionalChastePoint<DIM> segment_vector = rEndLocation - rStartLocation;
    DimensionalChastePoint<DIM> point_vector = rProbeLocation - rStartLocation;
    units::quantity<unit::length> dp_segment_point = GetDotProduct(segment_vector, point_vector);
    units::quantity<unit::length> dp_segment_segment = GetDotProduct(segment_vector, segment_vector);

    if (dp_segment_point <= 0.0*unit::metres || dp_segment_segment <= dp_segment_point)
    {
        if(!projectToEnds)
        {
            EXCEPTION("Projection of point is outside segment.");
        }
        else
        {
            units::quantity<unit::length> dist1 = (rStartLocation - rEndLocation).GetNorm2();
            units::quantity<unit::length> dist2 = (rEndLocation - rProbeLocation).GetNorm2();
            if(dist1 <= dist2)
            {
                return rStartLocation;
            }
            else
            {
                return rEndLocation;
            }
        }
    }
    // Point projection is inside segment, get distance to point projection
    return rStartLocation + (dp_segment_point / dp_segment_segment) * segment_vector;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistanceToLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                 const DimensionalChastePoint<DIM>& rEndLocation,
                                                 const DimensionalChastePoint<DIM>& rProbeLocation)
{
    DimensionalChastePoint<DIM> segment_vector = rEndLocation - rStartLocation;
    units::quantity<unit::length> dp_segment_point = GetDotProduct(segment_vector, rProbeLocation - rStartLocation);
    // Point projection is outside segment, return node0 distance
    if (dp_segment_point <= 0.0*unit::metres)
    {
        return rStartLocation.GetDistance(rProbeLocation);
    }

    units::quantity<unit::length> dp_segment_segment = GetDotProduct(segment_vector, segment_vector);
    // Point projection is outside segment, return node1 distance
    if (dp_segment_segment <= dp_segment_point)
    {
        return rEndLocation.GetDistance(rProbeLocation);
    }

    // Point projection is inside segment, get distance to point projection
    double projection_ratio = dp_segment_point / dp_segment_segment;
    DimensionalChastePoint<DIM> projected_location = rStartLocation + projection_ratio * segment_vector - rProbeLocation;
    return projected_location.GetNorm2();
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetMidPoint(const DimensionalChastePoint<DIM>& rLocation) const
{
    return DimensionalChastePoint<DIM>((rLocation.rGetLocation()*GetScalingFactor(rLocation) + mLocation) / 2.0, mReferenceLength);
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation) const
{
    return (rLocation.rGetLocation()*GetScalingFactor(rLocation) - mLocation) / (GetDistance(rLocation)/mReferenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator/=(double factor)
{
    mLocation /= factor;
    return *this;
}

template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator/(DimensionalChastePoint<DIM> lhs, double factor)
{
    lhs /= factor;
    return lhs;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator*=(double factor)
{
    mLocation *= factor;
    return *this;
}

template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator*(DimensionalChastePoint<DIM> lhs, double factor)
{
    lhs *= factor;
    return lhs;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator+=(const DimensionalChastePoint<DIM>& rLocation)
{
    mLocation += rLocation.rGetLocation()*rLocation.GetReferenceLengthScale()/mReferenceLength;
    return *this;
}

template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator+(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation)
{
    lhs += rLocation;
    return lhs;
}

template<unsigned DIM>
DimensionalChastePoint<DIM>& DimensionalChastePoint<DIM>::operator-=(const DimensionalChastePoint<DIM>& rLocation)
{
    mLocation -= rLocation.rGetLocation()*rLocation.GetReferenceLengthScale()/mReferenceLength;
    return *this;
}

template<unsigned DIM>
inline DimensionalChastePoint<DIM> operator-(DimensionalChastePoint<DIM> lhs, const DimensionalChastePoint<DIM>& rLocation)
{
    lhs -= rLocation;
    return lhs;
}

template<unsigned DIM>
bool DimensionalChastePoint<DIM>::IsCoincident(const DimensionalChastePoint<DIM>& rLocation) const
{
    double scaling_length = GetScalingFactor(rLocation);

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
    mLocation += rVector.rGetLocation()*GetScalingFactor(rVector);
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::TranslateTo(DimensionalChastePoint<DIM> rPoint)
{
    mLocation = rPoint.rGetLocation()*GetScalingFactor(rPoint);
}

// Explicit instantiation
template class DimensionalChastePoint<2>;
template class DimensionalChastePoint<3>;
