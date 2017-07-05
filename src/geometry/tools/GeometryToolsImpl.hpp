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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkBox.h>
#include <vtkTetra.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellLocator.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include "Exception.hpp"
#include "GeometryTools.hpp"
#include "UblasVectorInclude.hpp"
#include "UblasIncludes.hpp"
#include "UblasCustomFunctions.hpp"

template<unsigned DIM>
c_vector<double, DIM> GetArbitaryUnitNormal(c_vector<double, DIM> direction)
{
    c_vector<double, DIM> normal;
    if(DIM==2 or direction[2]==0.0)
    {
        if(direction[1] == 0.0)
        {
            normal[0] = 0.0;
            normal[1] = 1.0;
        }
        else
        {
            normal[0] = 1.0;
            normal[1] = -direction[0] /direction[1];
        }
    }
    else
    {
        if(std::abs(direction[0]) + std::abs(direction[1]) == 0.0)
        {
            normal[0] = 1.0;
            normal[1] = 1.0;
        }
        else
        {
            normal[0] = 1.0;
            normal[1] = 1.0;
            normal[2] = -(direction[0] + direction[1])/direction[2];
        }
    }
    return normal/norm_2(normal);
}

template<unsigned DIM>
QLength GetDistance(const DimensionalChastePoint<DIM>& rLocation1,
                                                                       const DimensionalChastePoint<DIM>& rLocation2)
{
    QLength reference_length = rLocation1.GetReferenceLengthScale();
    return norm_2(rLocation2.GetLocation(reference_length) - rLocation1.GetLocation(reference_length))*reference_length;
}

template<unsigned DIM>
units::quantity<unit::area> GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                          const DimensionalChastePoint<DIM>& rLocation2)
{
    QLength reference_length = rLocation1.GetReferenceLengthScale();
    return inner_prod(rLocation2.GetLocation(reference_length), rLocation1.GetLocation(reference_length))*reference_length*reference_length;
}

template<unsigned DIM>
QLength GetDotProduct(const DimensionalChastePoint<DIM>& rLocation1,
                                          const c_vector<double, DIM>& rLocation2)
{
    QLength reference_length_1 = rLocation1.GetReferenceLengthScale();
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
            QLength dist1 = (rStartLocation - rEndLocation).GetNorm2();
            QLength dist2 = (rEndLocation - rProbeLocation).GetNorm2();
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
QLength GetDistanceToLineSegment(const DimensionalChastePoint<DIM>& rStartLocation,
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
vtkSmartPointer<vtkPoints> GetProbeLocationsExternalPoint(DimensionalChastePoint<DIM> rCentrePoint,
        DimensionalChastePoint<DIM> currentDirection, QLength probeLength,
        unsigned numDivisions)
{
    QLength length_scale = rCentrePoint.GetReferenceLengthScale();
    c_vector<double, DIM> central_point = rCentrePoint.GetLocation(length_scale);
    c_vector<double, DIM> current_direction = currentDirection.GetUnitVector();

    double normalized_probe_length = probeLength/length_scale;
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    if(DIM==2)
    {
        p_points->InsertNextPoint(central_point[0], central_point[1], 0.0);
        c_vector<double, 3> rotation_axis;
        rotation_axis[0] = 0.0;
        rotation_axis[1] = 0.0;
        rotation_axis[2] = 1.0;
        units::quantity<unit::plane_angle> angle_increment = (2.0*M_PI/double(numDivisions))*unit::radians;
        for(unsigned idx=0;idx<numDivisions;idx++)
        {
            if(idx!=unsigned(numDivisions/2))
            {
                units::quantity<unit::plane_angle> angle = angle_increment*double(idx);
                c_vector<double, DIM> rotated_loc = RotateAboutAxis<DIM>(current_direction, rotation_axis, angle);
                c_vector<double, DIM> loc = central_point+normalized_probe_length * rotated_loc;
                p_points->InsertNextPoint(loc[0], loc[1], 0.0);
            }
        }
    }
    else if(DIM==3)
    {
        vtkSmartPointer<vtkSphereSource> p_sphere_source = vtkSmartPointer<vtkSphereSource>::New();
        p_sphere_source->SetCenter(&central_point[0]);
        p_sphere_source->SetRadius(normalized_probe_length);
        p_sphere_source->SetThetaResolution(numDivisions);
        p_sphere_source->SetPhiResolution(numDivisions);
        p_sphere_source->Update();
        vtkSmartPointer<vtkPolyData> p_sphere = p_sphere_source->GetOutput();
        c_vector<double, DIM> exluded_point = central_point-normalized_probe_length * current_direction;
        for(unsigned idx=0;idx<p_sphere->GetNumberOfPoints();idx++)
        {
            c_vector<double, DIM> point_loc;
            p_sphere->GetPoint(idx, &point_loc[0]);
            p_points->InsertNextPoint(&point_loc[0]);
        }
    }
    return p_points;
}

template<unsigned DIM>
vtkSmartPointer<vtkPoints> GetProbeLocationsInternalPoint(DimensionalChastePoint<DIM> rInitialDirection,
                                                                         DimensionalChastePoint<DIM> rCentralPoint,
                                                                         DimensionalChastePoint<DIM> rRotationAxis,
                                                                         QLength probeLength,
                                                                         units::quantity<unit::plane_angle> angle)
{
    QLength length_scale = rCentralPoint.GetReferenceLengthScale();
    c_vector<double, DIM> central_point = rCentralPoint.GetLocation(length_scale);
    c_vector<double, DIM> initial_direction = rInitialDirection.GetLocation(length_scale);
    c_vector<double, DIM> rotation_axis = rRotationAxis.GetLocation(length_scale);
    double normalized_probe_length = probeLength/length_scale;

    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    if(DIM==3)
    {
        p_points->InsertNextPoint(&central_point[0]);
        c_vector<double, DIM> unit_axis = rotation_axis/norm_2(rotation_axis);
        c_vector<double, DIM> new_direction = RotateAboutAxis<DIM>(initial_direction/norm_2(initial_direction), unit_axis, angle);
        new_direction /= norm_2(new_direction);
        c_vector<double, DIM> new_loc1 = central_point + normalized_probe_length*new_direction;
        p_points->InsertNextPoint(&new_loc1[0]);

        c_vector<double, DIM> new_direction_r1 = RotateAboutAxis<DIM>(new_direction, unit_axis, M_PI*unit::radians);
        new_direction_r1 /= norm_2(new_direction_r1);
        c_vector<double, DIM> new_loc2 = central_point + normalized_probe_length*new_direction_r1;
        p_points->InsertNextPoint(&new_loc2[0]);

        c_vector<double, DIM> new_direction_r2 = RotateAboutAxis<DIM>(new_direction, unit_axis, M_PI/2.0*unit::radians);
        new_direction_r2 /= norm_2(new_direction_r2);
        c_vector<double, DIM> new_loc3 = central_point + normalized_probe_length*new_direction_r2;
        p_points->InsertNextPoint(&new_loc3[0]);

        c_vector<double, DIM> new_direction_r3 = RotateAboutAxis<DIM>(new_direction, unit_axis, 3.0*M_PI/2.0*unit::radians);
        new_direction_r3 /= norm_2(new_direction_r3);
        c_vector<double, DIM> new_loc4 = central_point + normalized_probe_length*new_direction_r3;
        p_points->InsertNextPoint(&new_loc4[0]);
    }
    else
    {
        p_points->InsertNextPoint(central_point[0], central_point[1], 0.0);
        c_vector<double, DIM> new_direction = normalized_probe_length*(initial_direction/norm_2(initial_direction));
        c_vector<double, DIM> new_loc1 = central_point + new_direction;
        c_vector<double, DIM> new_loc2 = central_point - new_direction;
        p_points->InsertNextPoint(new_loc1[0], new_loc1[1], 0.0);
        p_points->InsertNextPoint(new_loc2[0], new_loc2[1], 0.0);
    }
    return p_points;
}

template<unsigned DIM>
bool IsPointInCone(const DimensionalChastePoint<DIM>& rPoint,
                   const DimensionalChastePoint<DIM>& rApex,
                   const DimensionalChastePoint<DIM>& rBase,
                   double aperture)
{
    DimensionalChastePoint<DIM> apex_to_point = rApex - rPoint;
    DimensionalChastePoint<DIM> apex_to_base = rApex - rBase;
    QLength dist_apex_base = apex_to_base.GetNorm2();
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
        const c_vector<double, 6>& rBoundingBox, QLength lengthScale)
{
    bool point_in_box = false;
    c_vector<double, DIM> dimensionless_point = rPoint.GetLocation(lengthScale);

    bool inside_left = dimensionless_point[0] >= rBoundingBox[0];
    bool inside_right = dimensionless_point[0] <= rBoundingBox[1];
    if(inside_left && inside_right)
    {
        if(dimensionless_point[1] >= rBoundingBox[2] && dimensionless_point[1] <= rBoundingBox[3])
        {
            if(DIM==3)
            {
                return (dimensionless_point[2] >= rBoundingBox[4] && dimensionless_point[2] <= rBoundingBox[5]);
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
bool IsPointInBox(const DimensionalChastePoint<DIM>& rPoint,
                  const DimensionalChastePoint<DIM>& rLocation, QLength spacing)
{
    bool point_in_box = false;
    QLength point_length_scale = rPoint.GetReferenceLengthScale();
    c_vector<double, DIM> location_in_point_scale = rLocation.GetLocation(point_length_scale);
    c_vector<double, DIM> dimensionless_point = rPoint.GetLocation(point_length_scale);
    double dimensionless_spacing = spacing/point_length_scale;

    bool inside_left = dimensionless_point[0] >= location_in_point_scale[0] -dimensionless_spacing/2.0;
    bool inside_right = dimensionless_point[0] <= location_in_point_scale[0] + dimensionless_spacing/2.0;
    if(inside_left && inside_right)
    {
        if(dimensionless_point[1] >= location_in_point_scale[1] -dimensionless_spacing/2.0 && dimensionless_point[1] <=
                location_in_point_scale[1] + dimensionless_spacing/2.0)
        {
            if(DIM==3)
            {
                if(dimensionless_point[2] >= location_in_point_scale[2] -dimensionless_spacing/2.0 && dimensionless_point[2] <=
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
    QLength scale_factor = rPoint.GetReferenceLengthScale();
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
        EXCEPTION("PointInTetra only works in 3d");
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
QLength LengthOfLineInBox(const DimensionalChastePoint<DIM>& rStartPoint,
                         const DimensionalChastePoint<DIM>& rEndPoint,
                         const c_vector<double, 6>& rBoundingBox,
                         QLength lengthScale)
{
    // If neither point is in the box return 0
    bool point1_in_box = IsPointInBox<DIM>(rStartPoint, rBoundingBox, lengthScale);
    bool point2_in_box = IsPointInBox<DIM>(rEndPoint, rBoundingBox, lengthScale);

    double t1;
    double t2;
    int plane1;
    int plane2;
    c_vector<double,3> intercept_1;
    c_vector<double,3> intercept_2;
    c_vector<double,3> dimensionless_start;
    dimensionless_start[0] = rStartPoint.GetLocation(lengthScale)[0];
    dimensionless_start[1] = rStartPoint.GetLocation(lengthScale)[1];
    if(DIM==3)
    {
        dimensionless_start[2] = rStartPoint.GetLocation(lengthScale)[2];
    }
    else
    {
        dimensionless_start[2] = 0.0;
    }
    c_vector<double,3> dimensionless_end;
    dimensionless_end[0] = rEndPoint.GetLocation(lengthScale)[0];
    dimensionless_end[1] = rEndPoint.GetLocation(lengthScale)[1];
    if(DIM==3)
    {
        dimensionless_end[2] = rEndPoint.GetLocation(lengthScale)[2];
    }
    else
    {
        dimensionless_end[2] = 0.0;
    }

    int crosses = vtkBox::IntersectWithLine(&rBoundingBox[0], &dimensionless_start[0], &dimensionless_end[0],
            t1, t2, &intercept_1[0], &intercept_2[0], plane1, plane2);

    if(!point1_in_box && !point2_in_box and !crosses)
    {
        return 0.0*lengthScale;
    }
    else
    {
        return norm_2(dimensionless_start - dimensionless_end)*(t2-t1)*lengthScale;
    }
}

template<unsigned DIM>
QLength LengthOfLineInBox(const DimensionalChastePoint<DIM>& rStartPoint,
                         const DimensionalChastePoint<DIM>& rEndPoint,
                         const DimensionalChastePoint<DIM>& rLocation,
                         QLength spacing)
{

    QLength scale_factor = rLocation.GetReferenceLengthScale();

    double dimensionless_spacing = spacing/scale_factor;
    c_vector<double, 3> dimensionless_location;
    dimensionless_location[0] = rLocation.GetLocation(scale_factor)[0];
    dimensionless_location[1] = rLocation.GetLocation(scale_factor)[1];
    if(DIM==3)
    {
        dimensionless_location[2] = rLocation.GetLocation(scale_factor)[2];
    }
    else
    {
        dimensionless_location[2] = 0.0;
    }

    c_vector<double,6> dimensionless_bounds;
    dimensionless_bounds[0] = dimensionless_location[0] - dimensionless_spacing/2.0;
    dimensionless_bounds[1] = dimensionless_location[0] + dimensionless_spacing/2.0;
    dimensionless_bounds[2] = dimensionless_location[1] - dimensionless_spacing/2.0;
    dimensionless_bounds[3] = dimensionless_location[1] + dimensionless_spacing/2.0;
    if(DIM==3)
    {
        dimensionless_bounds[4] = dimensionless_location[2] - dimensionless_spacing/2.0;
        dimensionless_bounds[5] = dimensionless_location[2] + dimensionless_spacing/2.0;
    }
    else
    {
        dimensionless_bounds[4] = -1.0;
        dimensionless_bounds[5] = 1.0;
    }

    return LengthOfLineInBox(rStartPoint,rEndPoint,dimensionless_bounds, scale_factor);
}

template<unsigned DIM>
QLength LengthOfLineInTetra(const DimensionalChastePoint<DIM>& rStartPoint,
                           const DimensionalChastePoint<DIM>& rEndPoint,
                           const std::vector<DimensionalChastePoint<DIM> >& locations)
{
    if (DIM==2)
    {
        EXCEPTION("This method expects 3D inputs");
    }
    bool point1_in_tetra = IsPointInTetra<DIM>(rStartPoint, locations);
    bool point2_in_tetra = IsPointInTetra<DIM>(rEndPoint, locations);

    if(point1_in_tetra && point2_in_tetra)
    {
        return GetDistance(rEndPoint, rStartPoint);
    }
    else
    {
        int line_crosses;

        QLength scale_factor = rStartPoint.GetReferenceLengthScale();
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
DimensionalChastePoint<DIM> OffsetAlongVector(const DimensionalChastePoint<DIM>& rVector, QLength offset)
{
    return DimensionalChastePoint<DIM>(rVector.GetUnitVector() * double(offset/rVector.GetReferenceLengthScale()), rVector.GetReferenceLengthScale());
}

template<unsigned DIM>
DimensionalChastePoint<DIM> OffsetAlongVector(const c_vector<double, DIM>& rVector, QLength offset,
                                              QLength referenceLength)
{
    return DimensionalChastePoint<DIM>(rVector * double(offset/referenceLength), referenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RotateAboutAxis(const DimensionalChastePoint<DIM>& rDirection,
                                      const DimensionalChastePoint<3>& rAxis, units::quantity<unit::plane_angle> angle)
{
    double sin_a = units::sin(angle);
    double cos_a = units::cos(angle);
    c_vector<double, DIM> new_direction;
    QLength length_scale = rDirection.GetReferenceLengthScale();
    c_vector<double, DIM> dimensionless_direction = rDirection.GetLocation(length_scale);
    if(DIM==3)
    {
        c_vector<double, 3> unit_axis = rAxis.GetUnitVector();
        QLength dot_product = GetDotProduct(rDirection, unit_axis);
        double dimensionless_dot_product = dot_product/length_scale;
        new_direction[0] = (unit_axis[0] * dimensionless_dot_product * (1.0 - cos_a) + dimensionless_direction[0] * cos_a
                    + (-unit_axis[2] * dimensionless_direction[1] + unit_axis[1] * dimensionless_direction[2]) * sin_a);
        new_direction[1] = (unit_axis[1] * dimensionless_dot_product * (1.0 - cos_a) + dimensionless_direction[1] * cos_a
                    + (unit_axis[2] * dimensionless_direction[0] - unit_axis[0] * dimensionless_direction[2]) * sin_a);
        new_direction[2] = (unit_axis[2] * dimensionless_dot_product * (1.0 - cos_a) + dimensionless_direction[2] * cos_a
                    + (-unit_axis[1] * dimensionless_direction[0] + unit_axis[0] * dimensionless_direction[1]) * sin_a);
    }
    else
    {
        new_direction[0] = dimensionless_direction[0] * cos_a - dimensionless_direction[1]*sin_a;
        new_direction[1] = dimensionless_direction[0] * sin_a + dimensionless_direction[1]*cos_a;
    }
    return DimensionalChastePoint<DIM>(new_direction, length_scale);
}

template<unsigned DIM>
c_vector<double, DIM> RotateAboutAxis(const c_vector<double, DIM>& rDirection,
                                      const c_vector<double, 3>& rAxis, units::quantity<unit::plane_angle> angle)
{
    double sin_a = units::sin(angle);
    double cos_a = units::cos(angle);
    c_vector<double, DIM> new_direction = zero_vector<double>(DIM);

    if(DIM==3)
    {
        c_vector<double, 3> unit_axis = rAxis/norm_2(rAxis);
        double dot_product = inner_prod(rDirection, unit_axis);
        new_direction[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + rDirection[0] * cos_a
                    + (-unit_axis[2] * rDirection[1] + unit_axis[1] * rDirection[2]) * sin_a);
        new_direction[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + rDirection[1] * cos_a
                    + (unit_axis[2] * rDirection[0] - unit_axis[0] * rDirection[2]) * sin_a);
        new_direction[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + rDirection[2] * cos_a
                    + (-unit_axis[1] * rDirection[0] + unit_axis[0] * rDirection[1]) * sin_a);
    }
    else
    {
        new_direction[0] = rDirection[0] * cos_a - rDirection[1]*sin_a;
        new_direction[1] = rDirection[0] * sin_a + rDirection[1]*cos_a;
    }
    return new_direction;
}
