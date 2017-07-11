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
#include "UnitCollection.hpp"

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
QLength GetDistance(const VecQLength<DIM>& rLocation1, const VecQLength<DIM>& rLocation2)
{
    return Qnorm_2(rLocation2 - rLocation1);
}

template<unsigned DIM>
QArea GetDotProduct(const VecQLength<DIM>& rLocation1, const VecQLength<DIM>& rLocation2)
{
    QLength reference_length = 1_m;
    return inner_prod(rLocation2.Convert(reference_length), rLocation1.Convert(reference_length))*reference_length*reference_length;
}

template<unsigned DIM>
QLength GetDotProduct(const VecQLength<DIM>& rLocation1,
                                          const c_vector<double, DIM>& rLocation2)
{
    QLength reference_length_1 = 1_m;
    return inner_prod(rLocation2 , rLocation1.Convert(reference_length_1))*reference_length_1;
}

template<unsigned DIM>
VecQLength<DIM> GetPointProjectionOnLineSegment(const VecQLength<DIM>& rStartLocation,
                                                      const VecQLength<DIM>& rEndLocation,
                                                      const VecQLength<DIM>& rProbeLocation,
                                                      bool projectToEnds,
                                                      bool checkDimensions)
{
    VecQLength<DIM> segment_vector = rEndLocation - rStartLocation;
    VecQLength<DIM> point_vector = rProbeLocation - rStartLocation;
    QArea dp_segment_point = GetDotProduct(segment_vector, point_vector);
    QArea dp_segment_segment = GetDotProduct(segment_vector, segment_vector);

    if (dp_segment_point <= 0.0*unit::metres_squared || dp_segment_segment <= dp_segment_point)
    {
        if(!projectToEnds)
        {
            EXCEPTION("Projection of point is outside segment.");
        }
        else
        {
            QLength dist1 = Qnorm_2(rStartLocation - rEndLocation);
            QLength dist2 = Qnorm_2(rEndLocation - rProbeLocation);
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
    return rStartLocation + (dp_segment_point / dp_segment_segment)*segment_vector;
}

template<unsigned DIM>
QLength GetDistanceToLineSegment(const Vertex<DIM>& rStartLocation, const Vertex<DIM>& rEndLocation,
                                                 const Vertex<DIM>& rProbeLocation)
{
    VecQLength<DIM> segment_vector = (rEndLocation - rStartLocation).rGetLocation();
    QArea dp_segment_point = GetDotProduct(segment_vector, (rProbeLocation - rStartLocation).rGetLocation());
    // Point projection is outside segment, return node0 distance
    if (dp_segment_point <= 0.0*unit::metres_squared)
    {
        return rStartLocation.GetDistance(rProbeLocation);
    }

    QArea dp_segment_segment = GetDotProduct(segment_vector, segment_vector);
    // Point projection is outside segment, return node1 distance
    if (dp_segment_segment <= dp_segment_point)
    {
        return rEndLocation.GetDistance(rProbeLocation);
    }

    // Point projection is inside segment, get distance to point projection
    double projection_ratio = dp_segment_point / dp_segment_segment;
    VecQLength<DIM> scaled_location = projection_ratio*segment_vector;
    VecQLength<DIM> projected_location = rStartLocation.rGetLocation() + scaled_location - rProbeLocation.rGetLocation();
    return Qnorm_2(projected_location);
}

template<unsigned DIM>
bool IsPointInCone(const Vertex<DIM>& rPoint,
                   const Vertex<DIM>& rApex,
                   const Vertex<DIM>& rBase,
                   double aperture)
{
    Vertex<DIM> apex_to_point = rApex - rPoint;
    Vertex<DIM> apex_to_base = rApex - rBase;
    QLength dist_apex_base = Qnorm_2(apex_to_base.rGetLocation());
    QArea dp_point_base = GetDotProduct(apex_to_point.rGetLocation(), apex_to_base.rGetLocation());
    bool in_infinite_cone = dp_point_base / (Qnorm_2(apex_to_point.rGetLocation()) * dist_apex_base) > std::cos(aperture/2.0);
    if(!in_infinite_cone)
    {
        return false;
    }
    return dp_point_base / dist_apex_base < dist_apex_base;
}

template<unsigned DIM>
bool IsPointInBox(const VecQLength<DIM>& rPoint, const c_vector<double, 6>& rBoundingBox, QLength lengthScale)
{
    bool point_in_box = false;
    c_vector<double, DIM> dimensionless_point = rPoint.Convert(lengthScale);

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
bool IsPointInBox(const VecQLength<DIM>& rPoint,
                  const VecQLength<DIM>& rLocation, QLength spacing, QLength lengthScale)
{
    bool point_in_box = false;
    c_vector<double, DIM> location_in_point_scale = rLocation.GetLocation(lengthScale);
    c_vector<double, DIM> dimensionless_point = rPoint.GetLocation(lengthScale);
    double dimensionless_spacing = spacing/lengthScale;

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
bool IsPointInTetra(const VecQLength<DIM>& rPoint, const std::vector<VecQLength<DIM> >& locations, QLength lengthScale)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints> :: New();
    c_vector<double, DIM> loc0 = locations[0].Convert(lengthScale);
    c_vector<double, DIM> loc1 = locations[1].Convert(lengthScale);
    c_vector<double, DIM> loc2 = locations[2].Convert(lengthScale);
    c_vector<double, DIM> loc3 = locations[3].Convert(lengthScale);
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
    c_vector<double, DIM> probe_location = rPoint.Convert(lengthScale);
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
QLength LengthOfLineInBox(const VecQLength<DIM>& rStartPoint,
                         const VecQLength<DIM>& rEndPoint,
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
    dimensionless_start[0] = rStartPoint.Convert(lengthScale)[0];
    dimensionless_start[1] = rStartPoint.Convert(lengthScale)[1];
    if(DIM==3)
    {
        dimensionless_start[2] = rStartPoint.Convert(lengthScale)[2];
    }
    else
    {
        dimensionless_start[2] = 0.0;
    }
    c_vector<double,3> dimensionless_end;
    dimensionless_end[0] = rEndPoint.Convert(lengthScale)[0];
    dimensionless_end[1] = rEndPoint.Convert(lengthScale)[1];
    if(DIM==3)
    {
        dimensionless_end[2] = rEndPoint.Convert(lengthScale)[2];
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
QLength LengthOfLineInBox(const VecQLength<DIM>& rStartPoint,
                         const VecQLength<DIM>& rEndPoint,
                         const VecQLength<DIM>& rLocation,
                         QLength spacing, QLength lengthScale)
{
    double dimensionless_spacing = spacing/lengthScale;
    c_vector<double, 3> dimensionless_location;
    dimensionless_location[0] = rLocation.Convert(lengthScale)[0];
    dimensionless_location[1] = rLocation.Convert(lengthScale)[1];
    if(DIM==3)
    {
        dimensionless_location[2] = rLocation.Convert(lengthScale)[2];
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

    return LengthOfLineInBox(rStartPoint,rEndPoint,dimensionless_bounds, lengthScale);
}

template<unsigned DIM>
QLength LengthOfLineInTetra(const VecQLength<DIM>& rStartPoint,
                           const VecQLength<DIM>& rEndPoint,
                           const std::vector<VecQLength<DIM> >& locations,
                           QLength lengthScale)
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
        c_vector<double, DIM> loc0 = locations[0].Convert(lengthScale);
        c_vector<double, DIM> loc1 = locations[1].Convert(lengthScale);
        c_vector<double, DIM> loc2 = locations[2].Convert(lengthScale);
        c_vector<double, DIM> loc3 = locations[3].Convert(lengthScale);

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

        c_vector<double,3> dimensionless_start = rStartPoint.Convert(lengthScale);
        c_vector<double,3> dimensionless_end = rEndPoint.Convert(lengthScale);

        if(point1_in_tetra)
        {
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_start[0], &dimensionless_end[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            return norm_2(intersection - dimensionless_start)*lengthScale;
        }

        if(point2_in_tetra)
        {
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_end[0], &dimensionless_start[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
            return norm_2(intersection - dimensionless_end)*lengthScale;
        }

        line_crosses = p_grid->GetCell(0)->IntersectWithLine(&dimensionless_start[0], &dimensionless_end[0], 1.e-6, t, &intersection[0], &parametric_intersection[0], subId);
        if(line_crosses)
        {
            c_vector<double,DIM> intersection2;
            p_grid->GetCell(0)->IntersectWithLine(&dimensionless_end[0], &dimensionless_start[0], 1.e-6, t, &intersection2[0], &parametric_intersection[0], subId);
            return norm_2(intersection - intersection2)*lengthScale;
        }
        else
        {
            return 0.0*lengthScale;
        }
    }
}

template<unsigned DIM>
VecQLength<DIM> OffsetAlongVector(const VecQLength<DIM>& rVector, QLength offset)
{
    c_vector<double, DIM> dir = rVector.Convert(offset);
    dir/=norm_2(dir);
    return rVector + VecQLength<DIM>(dir, offset);
}

template<unsigned DIM>
VecQLength<DIM> OffsetAlongVector(const c_vector<double, DIM>& rVector, QLength offset,
                                              QLength referenceLength)
{
    return VecQLength<DIM>(rVector * double(offset/referenceLength), referenceLength);
}

template<unsigned DIM>
VecQLength<DIM> RotateAboutAxis(const VecQLength<DIM>& rDirection, const c_vector<double, 3>& axis, QAngle angle)
{
    double sin_a = Qsin(angle);
    double cos_a = Qcos(angle);
    c_vector<double, DIM> new_direction;
    QLength length_scale = 1_m;
    c_vector<double, DIM> dimensionless_direction = rDirection.Convert(length_scale);
    if(DIM==3)
    {
        c_vector<double, DIM> unit_axis = axis/norm_2(axis);
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
    return VecQLength<DIM>(new_direction, length_scale);
}

template<unsigned DIM>
c_vector<double, DIM> RotateAboutAxis(const c_vector<double, DIM>& rDirection,
                                      const c_vector<double, 3>& rAxis, QAngle angle)
{
    double sin_a = Qsin(angle);
    double cos_a = Qcos(angle);
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
