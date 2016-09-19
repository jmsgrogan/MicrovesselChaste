/*

 Copyright (c) 2005-2015, University of Oxford.
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
        ChastePoint<DIM>(x, y, z),
        mReferenceLength(referenceLength)
{
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(c_vector<double, DIM> coords, units::quantity<unit::length> referenceLength) :
        ChastePoint<DIM>(coords),
        mReferenceLength(referenceLength)
{
}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(double x, double y, double z) :
        ChastePoint<DIM>(x, y, z),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned DIM>
DimensionalChastePoint<DIM>::DimensionalChastePoint(c_vector<double, DIM> coords) :
        ChastePoint<DIM>(coords),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned DIM>
DimensionalChastePoint<DIM>::~DimensionalChastePoint()
{
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetReferenceLengthScale() const
{
    return mReferenceLength;
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::SetReferenceLengthScale(units::quantity<unit::length> lenthScale)
{
    c_vector<double, DIM> temp_location = ChastePoint<DIM>::rGetLocation() * (mReferenceLength/lenthScale);
    ChastePoint<DIM>::rGetLocation() = temp_location;
    mReferenceLength = lenthScale;
}

template<unsigned DIM>
double DimensionalChastePoint<DIM>::GetScalingFactor(const DimensionalChastePoint<DIM>& rLocation) const
{
    double scaling_length = 1.0;

    if(mReferenceLength == 0.0*unit::metres)
    {
        EXCEPTION("Point has zero reference length");
    }

    if(mReferenceLength != rLocation.GetReferenceLengthScale())
    {
        scaling_length = rLocation.GetReferenceLengthScale()/mReferenceLength;
    }

    return scaling_length;
}

template<unsigned DIM>
bool DimensionalChastePoint<DIM>::IsCoincident(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions) const
{
    double scaling_length = 1.0;
    if(checkDimensions)
    {
        scaling_length = GetScalingFactor(rLocation);
    }

    bool returned_value = true;
    for (unsigned dim=0; dim<DIM; dim++)
    {
        if (rLocation[dim]*scaling_length != ChastePoint<DIM>::rGetLocation()[dim])
        {
            returned_value = false;
            break;
        }
    }
    return returned_value;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions) const
{
    double scaling_length = 1.0;
    if(checkDimensions)
    {
        scaling_length = GetScalingFactor(rLocation);
    }
    return norm_2(rLocation.rGetLocation()*scaling_length - ChastePoint<DIM>::rGetLocation())*mReferenceLength;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetPointProjection(const DimensionalChastePoint<DIM>& rStartLocation,
                                                      const DimensionalChastePoint<DIM>& rEndLocation,
                                                      const DimensionalChastePoint<DIM>& rProbeLocation,
                                                      bool projectToEnds,
                                                      bool checkDimensions)
{
    double scaling_length_end = 1.0;
    double scaling_length_probe = 1.0;
    if(checkDimensions)
    {
        if(rStartLocation.GetReferenceLengthScale() == 0.0*unit::metres)
        {
            EXCEPTION("Point has zero reference length");
        }

        if(rStartLocation.GetReferenceLengthScale() != rEndLocation.GetReferenceLengthScale())
        {
            scaling_length_end = rEndLocation.GetReferenceLengthScale()/rStartLocation.GetReferenceLengthScale();
        }

        if(rStartLocation.GetReferenceLengthScale() != rProbeLocation.GetReferenceLengthScale())
        {
            scaling_length_probe = rProbeLocation.GetReferenceLengthScale()/rStartLocation.GetReferenceLengthScale();
        }
    }

    c_vector<double, DIM> start_location = rStartLocation.rGetLocation();
    c_vector<double, DIM> end_location = rEndLocation.rGetLocation()*scaling_length_end;
    c_vector<double, DIM> probe_location = rProbeLocation.rGetLocation()*scaling_length_probe;

    c_vector<double, DIM> segment_vector = end_location - start_location;
    c_vector<double, DIM> point_vector = probe_location - start_location;

    double dp_segment_point = inner_prod(segment_vector, point_vector);
    double dp_segment_segment = inner_prod(segment_vector, segment_vector);

    if (dp_segment_point <= 0.0 || dp_segment_segment <= dp_segment_point)
    {
        if(!projectToEnds)
        {
            EXCEPTION("Projection of point is outside segment.");
        }
        else
        {
            double dist1 = norm_2(start_location - probe_location);
            double dist2 = norm_2(end_location - probe_location);
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
    double projection_ratio = dp_segment_point / dp_segment_segment;
    DimensionalChastePoint<DIM> projected_point = DimensionalChastePoint<DIM>(start_location + projection_ratio * segment_vector,
                                                                              rStartLocation.GetReferenceLengthScale());
    return projected_point;
}

template<unsigned DIM>
units::quantity<unit::length> DimensionalChastePoint<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rStartLocation,
                                                 const DimensionalChastePoint<DIM>& rEndLocation,
                                                 const DimensionalChastePoint<DIM>& rProbeLocation,
                                                 bool checkDimensions)
{
    double scaling_length_end = 1.0;
    double scaling_length_probe = 1.0;
    if(checkDimensions)
    {
        if(rStartLocation.GetReferenceLengthScale() == 0.0*unit::metres)
        {
            EXCEPTION("Point has zero reference length");
        }

        if(rStartLocation.GetReferenceLengthScale() != rEndLocation.GetReferenceLengthScale())
        {
            scaling_length_end = rEndLocation.GetReferenceLengthScale()/rStartLocation.GetReferenceLengthScale();
        }

        if(rStartLocation.GetReferenceLengthScale() != rProbeLocation.GetReferenceLengthScale())
        {
            scaling_length_probe = rProbeLocation.GetReferenceLengthScale()/rStartLocation.GetReferenceLengthScale();
        }
    }

    c_vector<double, DIM> start_location = rStartLocation.rGetLocation();
    c_vector<double, DIM> end_location = rEndLocation.rGetLocation()*scaling_length_end;
    c_vector<double, DIM> probe_location = rProbeLocation.rGetLocation()*scaling_length_probe;

    c_vector<double, DIM> segment_vector = end_location - start_location;
    double dp_segment_point = inner_prod(segment_vector, probe_location - start_location);
    // Point projection is outside segment, return node0 distance
    if (dp_segment_point <= 0.0)
    {
        return rStartLocation.GetDistance(rProbeLocation);
    }

    double dp_segment_segment = inner_prod(segment_vector, segment_vector);
    // Point projection is outside segment, return node1 distance
    if (dp_segment_segment <= dp_segment_point)
    {
        return rEndLocation.GetDistance(rProbeLocation);
    }

    // Point projection is inside segment, get distance to point projection
    double projection_ratio = dp_segment_point / dp_segment_segment;
    return norm_2(start_location + projection_ratio * segment_vector - probe_location) * rStartLocation.GetReferenceLengthScale();
}

template<unsigned DIM>
DimensionalChastePoint<DIM> DimensionalChastePoint<DIM>::GetMidPoint(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions) const
{
    double scaling_length = 1.0;
    if(checkDimensions)
    {
        scaling_length = GetScalingFactor(rLocation);
    }

    return DimensionalChastePoint<DIM>((rLocation.rGetLocation()*scaling_length + ChastePoint<DIM>::rGetLocation()) / 2.0, mReferenceLength);
}

template<unsigned DIM>
c_vector<double, DIM> DimensionalChastePoint<DIM>::GetUnitTangent(const DimensionalChastePoint<DIM>& rLocation, bool checkDimensions) const
{
    double scaling_length = 1.0;
    if(checkDimensions)
    {
        scaling_length = GetScalingFactor(rLocation);
    }

    return (rLocation.rGetLocation()*scaling_length - ChastePoint<DIM>::rGetLocation()) / (GetDistance(rLocation)/mReferenceLength);
}

template<unsigned DIM>
void DimensionalChastePoint<DIM>::Translate(DimensionalChastePoint<DIM> rVector)
{
    // Save the original reference length
    units::quantity<unit::length> original_length = mReferenceLength;

    // Change to the vector length
    SetReferenceLengthScale(rVector.GetReferenceLengthScale());

    // Update the position using raw c_vectors
    ChastePoint<DIM>::rGetLocation()=ChastePoint<DIM>::rGetLocation()+rVector.rGetLocation();

    // Change back to the original reference length
    SetReferenceLengthScale(original_length);
}

// Explicit instantiation
template class DimensionalChastePoint<2> ;
template class DimensionalChastePoint<3> ;
