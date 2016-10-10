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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkBox.h>
#include <vtkTetra.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellLocator.h>
#include "Exception.hpp"
#include "GeometryTools.hpp"
#include "UblasVectorInclude.hpp"
#include "UblasIncludes.hpp"
#include "UblasCustomFunctions.hpp"

template<unsigned DIM>
units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation1,
                                                                       const DimensionalChastePoint<DIM>& rLocation2)
{
    units::quantity<unit::length> reference_length = rLocation1.GetReferenceLengthScale();
    return norm_2(rLocation2.GetLocation(reference_length) - rLocation1.GetLocation(reference_length))*reference_length;
}

template<unsigned DIM>
units::quantity<unit::area> GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                          const DimensionalChastePoint<DIM>& rLocation2)
{
    units::quantity<unit::length> reference_length = rLocation1.GetReferenceLengthScale();
    return inner_prod(rLocation2.GetLocation(reference_length), rLocation1.GetLocation(reference_length))*reference_length*reference_length;
}

template<unsigned DIM>
units::quantity<unit::length> GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                          const c_vector<double, DIM>& rLocation2)
{
    units::quantity<unit::length> reference_length_1 = rLocation1.GetReferenceLengthScale();
    return inner_prod(rLocation2 , rLocation1.GetLocation(reference_length_1))*reference_length_1;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> GetPointProjectionOnLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                      const DimensionalChastePoint<DIM>& rEndLocation,
                                                      const DimensionalChastePoint<DIM>& rProbeLocation,
                                                      bool projectToEnds,
                                                      bool checkDimensions)
{
    DimensionalChastePoint<DIM> segment_vector = rEndLocation - rStartLocation;
    DimensionalChastePoint<DIM> point_vector = rProbeLocation - rStartLocation;
    units::quantity<unit::area> dp_segment_point = GetDotProduct(segment_vector, point_vector);
    units::quantity<unit::area> dp_segment_segment = GetDotProduct(segment_vector, segment_vector);

    if (dp_segment_point <= 0.0*unit::metres*unit::metres || dp_segment_segment <= dp_segment_point)
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
    return rStartLocation + segment_vector*(dp_segment_point / dp_segment_segment);
}

template<unsigned DIM>
units::quantity<unit::length> GetDistanceToLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
                                                 const DimensionalChastePoint<DIM>& rEndLocation,
                                                 const DimensionalChastePoint<DIM>& rProbeLocation)
{
    DimensionalChastePoint<DIM> segment_vector = rEndLocation - rStartLocation;
    units::quantity<unit::area> dp_segment_point = GetDotProduct(segment_vector, rProbeLocation - rStartLocation);
    // Point projection is outside segment, return node0 distance
    if (dp_segment_point <= 0.0*unit::metres*unit::metres)
    {
        return rStartLocation.GetDistance(rProbeLocation);
    }

    units::quantity<unit::area> dp_segment_segment = GetDotProduct(segment_vector, segment_vector);
    // Point projection is outside segment, return node1 distance
    if (dp_segment_segment <= dp_segment_point)
    {
        return rEndLocation.GetDistance(rProbeLocation);
    }

    // Point projection is inside segment, get distance to point projection
    double projection_ratio = dp_segment_point / dp_segment_segment;
    DimensionalChastePoint<DIM> projected_location = rStartLocation + segment_vector*projection_ratio - rProbeLocation;
    return projected_location.GetNorm2();
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > GetProbeLocationsExternalPoint(DimensionalChastePoint<DIM> rCentrePoint,
                                                                         units::quantity<unit::length> probeLength)
{
    unsigned num_probes = (2*DIM)+1;
    std::vector<DimensionalChastePoint<DIM> > probe_locations(num_probes);
    units::quantity<unit::length> length_scale = rCentrePoint.GetReferenceLengthScale();

    double normalized_probe_length = probeLength/length_scale;
    probe_locations[0] = rCentrePoint;
    probe_locations[1] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,0), length_scale);
    probe_locations[2] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,0), length_scale);
    probe_locations[3] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,1), length_scale);
    probe_locations[4] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,1), length_scale);
    if(DIM==3)
    {
        probe_locations[5] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,2), length_scale);
        probe_locations[6] = rCentrePoint + DimensionalChastePoint<DIM>(normalized_probe_length * unit_vector<double>(DIM,2), length_scale);
    }
    return probe_locations;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > GetProbeLocationsInternalPoint(DimensionalChastePoint<DIM> rInitialDirection,
                                                                         DimensionalChastePoint<DIM> rCentralPoint,
                                                                         DimensionalChastePoint<DIM> rRotationAxis,
                                                                         units::quantity<unit::length> probeLength,
                                                                         units::quantity<unit::plane_angle> angle)
{
    unsigned num_probes = 2*DIM - 1;
    std::vector<DimensionalChastePoint<DIM> > probe_locations(num_probes);
    probe_locations[0] = rCentralPoint;
    c_vector<double, DIM> unit_axis = rRotationAxis.GetUnitVector();
    units::quantity<unit::length> length_scale = rCentralPoint.GetReferenceLengthScale();
    double normalized_probe_length = probeLength/length_scale;

    c_vector<double, DIM> new_direction = RotateAboutAxis<DIM>(rInitialDirection.GetUnitVector(), unit_axis, angle);
    new_direction /= norm_2(new_direction);
    probe_locations[1] = rCentralPoint + DimensionalChastePoint<DIM>(new_direction*normalized_probe_length, length_scale);

    c_vector<double, DIM> new_direction_r1 = RotateAboutAxis<DIM>(new_direction, unit_axis, M_PI*unit::radians);
    new_direction_r1 /= norm_2(new_direction_r1);
    probe_locations[2] = rCentralPoint + DimensionalChastePoint<DIM>(new_direction_r1*normalized_probe_length, length_scale);
    if(DIM==3)
    {
        c_vector<double, DIM> new_direction_r2 = RotateAboutAxis<DIM>(new_direction, unit_axis, M_PI/2.0*unit::radians);
        new_direction_r2 /= norm_2(new_direction_r2);
        probe_locations[3] = rCentralPoint + DimensionalChastePoint<DIM>(new_direction_r2*normalized_probe_length, length_scale);

        c_vector<double, DIM> new_direction_r3 = RotateAboutAxis<DIM>(new_direction, unit_axis, 3.0*M_PI/2.0*unit::radians);
        new_direction_r3 /= norm_2(new_direction_r3);
        probe_locations[4] = rCentralPoint + DimensionalChastePoint<DIM>(new_direction_r3*normalized_probe_length, length_scale);
    }

    return probe_locations;
}

template<unsigned DIM>
bool IsPointInCone(const DimensionalChastePoint<DIM>& rPoint,
                   const DimensionalChastePoint<DIM>& rApex,
                   const DimensionalChastePoint<DIM>& rBase,
                   double aperture)
{
    DimensionalChastePoint<DIM> apex_to_point = rApex - rPoint;
    DimensionalChastePoint<DIM> apex_to_base = rApex - rBase;
    units::quantity<unit::length> dist_apex_base = apex_to_base.GetNorm2();
    units::quantity<unit::area> dp_point_base = GetDotProduct(apex_to_point, apex_to_base);
    bool in_infinite_cone = dp_point_base / (apex_to_point.GetNorm2() * dist_apex_base) > std::cos(aperture/2.0);
    if(!in_infinite_cone)
    {
        return false;
    }
    return dp_point_base / dist_apex_base < dist_apex_base;
}

template<unsigned DIM>
bool IsPointInBox(const DimensionalChastePoint<DIM>& rPoint,
                  const DimensionalChastePoint<DIM>& rLocation, units::quantity<unit::length> spacing)
{
    bool point_in_box = false;
    units::quantity<unit::length> point_length_scale = rPoint.GetReferenceLengthScale();
    c_vector<double, DIM> location_in_point_scale = rLocation.GetLocation(point_length_scale);
    double dimensionless_spacing = spacing/point_length_scale;

    bool inside_left = rPoint[0] >= location_in_point_scale[0] -dimensionless_spacing/2.0;
    bool inside_right = rPoint[0] <= location_in_point_scale[0] + dimensionless_spacing/2.0;
    if(inside_left && inside_right)
    {
        if(rPoint[1] >= location_in_point_scale[1] -dimensionless_spacing/2.0 && rPoint[1] <=
                location_in_point_scale[1] + dimensionless_spacing/2.0)
        {
            if(DIM==3)
            {
                if(rPoint[2] >= location_in_point_scale[2] -dimensionless_spacing/2.0 && rPoint[2] <=
                        location_in_point_scale[2] + dimensionless_spacing/2.0)
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        }
    }
    return point_in_box;
}

template<unsigned DIM>
bool IsPointInTetra(const DimensionalChastePoint<DIM>& rPoint, const std::vector<DimensionalChastePoint<DIM> >& locations)
{
    units::quantity<unit::length> scale_factor = rPoint.GetReferenceLengthScale();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints> :: New();
    c_vector<double, DIM> loc0 = locations[0].GetLocation(scale_factor);
    c_vector<double, DIM> loc1 = locations[1].GetLocation(scale_factor);
    c_vector<double, DIM> loc2 = locations[2].GetLocation(scale_factor);
    c_vector<double, DIM> loc3 = locations[3].GetLocation(scale_factor);
    if(DIM==3)
    {
        points->InsertNextPoint(&loc0[0]);
        points->InsertNextPoint(&loc1[0]);
        points->InsertNextPoint(&loc2[0]);
        points->InsertNextPoint(&loc3[0]);
    }
    else
    {
        points->InsertNextPoint(loc0[0], loc0[1], 0.0);
        points->InsertNextPoint(loc1[0], loc1[1], 0.0);
        points->InsertNextPoint(loc2[0], loc2[1], 0.0);
        points->InsertNextPoint(loc3[0], loc3[1], 0.0);
    }

    vtkSmartPointer<vtkUnstructuredGrid> p_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    p_grid->SetPoints(points);
    vtkIdType ptIds[] = {0, 1, 2, 3};
    p_grid->InsertNextCell(VTK_TETRA, 4, ptIds);

    vtkSmartPointer<vtkCellLocator> p_locator = vtkSmartPointer<vtkCellLocator>::New();
    p_locator->SetDataSet(p_grid);
    p_locator->Update();
    c_vector<double, DIM> probe_location = rPoint.GetLocation(scale_factor);
    int in_tetra = p_locator->FindCell(&probe_location[0]);
    if(in_tetra == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

template<unsigned DIM>
units::quantity<unit::length> LengthOfLineInBox(const DimensionalChastePoint<DIM>& rStartPoint,
                         const DimensionalChastePoint<DIM>& rEndPoint,
                         const DimensionalChastePoint<DIM>& rLocation, units::quantity<unit::length> spacing)
{
    // If the line is fully in the box return its length
    bool point1_in_box = IsPointInBox<DIM>(rStartPoint, rLocation, spacing);
    bool point2_in_box = IsPointInBox<DIM>(rEndPoint, rLocation, spacing);
    if(point1_in_box && point2_in_box)
    {
        return GetDistance(rEndPoint, rStartPoint);
    }
    else
    {
        units::quantity<unit::length> scale_factor = rLocation.GetReferenceLengthScale();
        double dimensionless_spacing = spacing/scale_factor;

        c_vector<double,6> dimensionless_bounds;
        dimensionless_bounds[0] = rLocation[0] - dimensionless_spacing/2.0;
        dimensionless_bounds[1] = rLocation[0] + dimensionless_spacing/2.0;
        dimensionless_bounds[2] = rLocation[1] - dimensionless_spacing/2.0;
        dimensionless_bounds[3] = rLocation[1] + dimensionless_spacing/2.0;
        if(DIM==3)
        {
            dimensionless_bounds[4] = rLocation[2] - dimensionless_spacing/2.0;
            dimensionless_bounds[5] = rLocation[2] + dimensionless_spacing/2.0;
        }
        else
        {
            dimensionless_bounds[4] = 0.0;
            dimensionless_bounds[5] = 0.0;
        }

        double t1;
        double t2;
        int plane1;
        int plane2;
        c_vector<double,DIM> intercept_1;
        c_vector<double,DIM> intercept_2;

        c_vector<double,3> dimensionless_start = rStartPoint.GetLocation(scale_factor);
        c_vector<double,3> dimensionless_end = rEndPoint.GetLocation(scale_factor);
        int in_box = vtkBox::IntersectWithLine(&dimensionless_bounds[0], &dimensionless_start[0], &dimensionless_end[0],
                                               t1, t2, &intercept_1[0], &intercept_2[0], plane1, plane2);

        if(point1_in_box)
        {
            return norm_2(intercept_2 - dimensionless_start)*scale_factor;
        }

        if(point2_in_box)
        {
            return norm_2(intercept_1 - dimensionless_end)*scale_factor;
        }

        if(in_box)
        {
            return norm_2(intercept_2 - intercept_1)*scale_factor;
        }
        else
        {
            return 0.0*scale_factor;
        }
    }
}

template<unsigned DIM>
units::quantity<unit::length> LengthOfLineInTetra(const DimensionalChastePoint<DIM>& rStartPoint,
                           const DimensionalChastePoint<DIM>& rEndPoint,
                           const std::vector<DimensionalChastePoint<DIM> >& locations)
{
    bool point1_in_tetra = IsPointInTetra<DIM>(rStartPoint, locations);
    bool point2_in_tetra = IsPointInTetra<DIM>(rEndPoint, locations);

    if(point1_in_tetra && point2_in_tetra)
    {
        return GetDistance(rEndPoint, rStartPoint);
    }
    else
    {
        int line_crosses;

        units::quantity<unit::length> scale_factor = rStartPoint.GetReferenceLengthScale();
        c_vector<double, DIM> loc0 = locations[0].GetLocation(scale_factor);
        c_vector<double, DIM> loc1 = locations[1].GetLocation(scale_factor);
        c_vector<double, DIM> loc2 = locations[2].GetLocation(scale_factor);
        c_vector<double, DIM> loc3 = locations[3].GetLocation(scale_factor);

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints> :: New();
        points->InsertNextPoint(&loc0[0]);
        points->InsertNextPoint(&loc1[0]);
        points->InsertNextPoint(&loc2[0]);
        points->InsertNextPoint(&loc3[0]);

        vtkSmartPointer<vtkUnstructuredGrid> p_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
        p_grid->SetPoints(points);

        vtkIdType ptIds[] = {0, 1, 2, 3};
        p_grid->InsertNextCell(VTK_TETRA, 4, ptIds);

        double t;
        c_vector<double,DIM> intersection;
        c_vector<double,DIM> parametric_intersection;
        int subId;

        c_vector<double,3> dimensionless_start = rStartPoint.GetLocation(scale_factor);
        c_vector<double,3> dimensionless_end = rEndPoint.GetLocation(scale_factor);

        if(point1_in_tetra)
        {
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_start[0], &dimensionless_end[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            return norm_2(intersection - dimensionless_start)*scale_factor;
        }

        if(point2_in_tetra)
        {
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_end[0], &dimensionless_start[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            return norm_2(intersection - dimensionless_end)*scale_factor;
        }

        line_crosses = p_grid->GetCell(0)->IntersectWithLine(&dimensionless_start[0], &dimensionless_end[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
        if(line_crosses)
        {
            c_vector<double,DIM> intersection2;
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_end[0], &dimensionless_start[0], 1.e-6, t, &intersection2[0], &parametric_intersection[0], subId);
            return norm_2(intersection - intersection2)*scale_factor;
        }
        else
        {
            return 0.0*scale_factor;
        }
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM> OffsetAlongVector(const DimensionalChastePoint<DIM>& rVector, units::quantity<unit::length> offset)
{
    return DimensionalChastePoint<DIM>(rVector.GetUnitVector() * double(offset/rVector.GetReferenceLengthScale()), rVector.GetReferenceLengthScale());
}

template<unsigned DIM>
DimensionalChastePoint<DIM> OffsetAlongVector(const c_vector<double, DIM>& rVector, units::quantity<unit::length> offset,
                                              units::quantity<unit::length> referenceLength)
{
    return DimensionalChastePoint<DIM>(rVector * double(offset/referenceLength), referenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RotateAboutAxis(const DimensionalChastePoint<DIM>& rDirection,
                                      const DimensionalChastePoint<DIM>& rAxis, units::quantity<unit::plane_angle> angle)
{
    double sin_a = units::sin(angle);
    double cos_a = units::cos(angle);
    c_vector<double, DIM> unit_axis = rAxis.GetUnitVector();
    units::quantity<unit::length> dot_product = GetDotProduct(rDirection, unit_axis);
    units::quantity<unit::length> length_scale = rDirection.GetReferenceLengthScale();
    double dimensionless_dot_product = dot_product/length_scale;
    c_vector<double, DIM> initial_direction = zero_vector<double>(DIM);
    DimensionalChastePoint<DIM> new_direction(initial_direction, length_scale);
    if(DIM==3)
    {
        new_direction[0] = (unit_axis[0] * dimensionless_dot_product * (1.0 - cos_a) + rDirection[0] * cos_a
                    + (-unit_axis[2] * rDirection[1] + unit_axis[1] * rDirection[2]) * sin_a);
        new_direction[1] = (unit_axis[1] * dimensionless_dot_product * (1.0 - cos_a) + rDirection[1] * cos_a
                    + (unit_axis[2] * rDirection[0] - unit_axis[0] * rDirection[2]) * sin_a);
        new_direction[2] = (unit_axis[2] * dimensionless_dot_product * (1.0 - cos_a) + rDirection[2] * cos_a
                    + (-unit_axis[1] * rDirection[0] + unit_axis[0] * rDirection[1]) * sin_a);
    }
    else
    {
        new_direction[0] = unit_axis[0] * dimensionless_dot_product * (1.0 - cos_a) + rDirection[0] * cos_a;
        new_direction[1] = unit_axis[1] * dimensionless_dot_product * (1.0 - cos_a) + rDirection[1] * cos_a;
    }
    return new_direction;
}

template<unsigned DIM>
c_vector<double, DIM> RotateAboutAxis(const c_vector<double, DIM>& rDirection,
                                      const c_vector<double, DIM>& rAxis, units::quantity<unit::plane_angle> angle)
{
    double sin_a = units::sin(angle);
    double cos_a = units::cos(angle);
    c_vector<double, DIM> unit_axis = rAxis/norm_2(rAxis);
    double dot_product = inner_prod(rDirection, unit_axis);
    c_vector<double, DIM> new_direction = zero_vector<double>(DIM);

    if(DIM==3)
    {
        new_direction[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + rDirection[0] * cos_a
                    + (-unit_axis[2] * rDirection[1] + unit_axis[1] * rDirection[2]) * sin_a);
        new_direction[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + rDirection[1] * cos_a
                    + (unit_axis[2] * rDirection[0] - unit_axis[0] * rDirection[2]) * sin_a);
        new_direction[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + rDirection[2] * cos_a
                    + (-unit_axis[1] * rDirection[0] + unit_axis[0] * rDirection[1]) * sin_a);
    }
    else
    {
        new_direction[0] = unit_axis[0] * dot_product * (1.0 - cos_a) + rDirection[0] * cos_a;
        new_direction[1] = unit_axis[1] * dot_product * (1.0 - cos_a) + rDirection[1] * cos_a;
    }
    return new_direction;
}
