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
#include <vtkPolygon.h>
#include <stdlib.h>
#include "Exception.hpp"
#include "Vessel.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "UblasCustomFunctions.hpp"
#include "UblasIncludes.hpp"
#include "DimensionalChastePoint.hpp"
#include "Part.hpp"
#include "BaseUnits.hpp"

#include "VesselSurfaceGenerator.hpp"

template<unsigned DIM>
VesselSurfaceGenerator<DIM>::VesselSurfaceGenerator(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork) :
        mpVesselNetwork(pVesselNetwork),
        mpSurface(vtkSmartPointer<vtkPolyData>::New()),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
}

template<unsigned DIM>
VesselSurfaceGenerator<DIM>::~VesselSurfaceGenerator()
{
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > VesselSurfaceGenerator<DIM>::GetHoles()
{
    std::vector<DimensionalChastePoint<DIM> > hole_locations;
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();
    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        hole_locations.push_back(segments[idx]->GetMidPoint());
    }
    return hole_locations;
}

template<unsigned DIM>
std::vector<std::vector<PolygonPtr<DIM> > > VesselSurfaceGenerator<DIM>::GetSurface()
{
    if(DIM==2)
    {
        EXCEPTION("The surface generator currently only works in 3D");
    }
    // Define the global axes
    c_vector<double, DIM> z_axis = unit_vector<double>(DIM,2);
    c_vector<double, DIM> y_axis = unit_vector<double>(DIM,1);

    // Generate a surface for each segment
    std::vector<std::vector<PolygonPtr<DIM> > > segment_polygons;
    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();

    for (unsigned idx = 0; idx < segments.size(); idx++)
    {
        std::shared_ptr<VesselNode<DIM> > p_start_node = segments[idx]->GetNode(0);
        std::shared_ptr<VesselNode<DIM> > p_end_node = segments[idx]->GetNode(1);
        c_vector<double, DIM> segment_tangent = segments[idx]->GetUnitTangent();

        // Create the precursor points
        std::vector<c_vector<double, DIM> > start_points = MakeCircle(p_start_node->GetRadius()/mReferenceLength);
        std::vector<c_vector<double, DIM> > end_points = MakeCircle(p_end_node->GetRadius()/mReferenceLength);

        double angle = std::acos(inner_prod(z_axis, segment_tangent));
        if (std::abs(inner_prod(z_axis, segment_tangent)) < 1.0 - 1.e-6)
        {
            c_vector<double, DIM> axis = VectorProduct(z_axis, segment_tangent);
            RotateAboutAxis(start_points, axis, angle);
            RotateAboutAxis(end_points, axis, angle);
        }

        // Get the bisection planes at the nodes
        std::vector<vtkSmartPointer<vtkPlane> > start_planes;
        std::vector<vtkSmartPointer<vtkPlane> > end_planes;
        c_vector<double, DIM> average_start_normal;
        c_vector<double, DIM> average_end_normal;

        if (p_start_node->GetNumberOfSegments() == 1)
        {
            c_vector<double, DIM> node_location = p_start_node->rGetLocation().GetLocation(mReferenceLength);
            vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
            if(DIM==3)
            {
                p_plane->SetOrigin(node_location[0], node_location[1], node_location[2]);
                p_plane->SetNormal(segment_tangent[0], segment_tangent[1], segment_tangent[2]);
            }
            else
            {
                p_plane->SetOrigin(node_location[0], node_location[1], 0.0);
                p_plane->SetNormal(segment_tangent[0], segment_tangent[1], 0.0);
            }
            start_planes.push_back(p_plane);
        }
        else
        {
            std::vector<std::shared_ptr<VesselSegment<DIM> > > node_segments = p_start_node->GetSegments();
            for (unsigned jdx = 0; jdx < node_segments.size(); jdx++)
            {
                if (node_segments[jdx] != segments[idx])
                {
                    c_vector<double, DIM> other_segment_tangent = node_segments[jdx]->GetUnitTangent();
                    if (node_segments[jdx]->GetNode(0) == p_start_node)
                    {
                        other_segment_tangent = -other_segment_tangent;
                    }

                    average_start_normal += VectorProduct(segment_tangent, other_segment_tangent);
                    c_vector<double, DIM> node_location = p_start_node->rGetLocation().GetLocation(mReferenceLength);
                    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
                    if(DIM==2)
                    {
                        p_plane->SetOrigin(node_location[0], node_location[1], 0.0);
                    }
                    else
                    {
                        p_plane->SetOrigin(node_location[0], node_location[1], node_location[2]);
                    }

                    c_vector<double, DIM> bisection_vector = segment_tangent + other_segment_tangent;
                    bisection_vector /= norm_2(bisection_vector);
                    if(DIM==2)
                    {
                        p_plane->SetNormal(bisection_vector[0], bisection_vector[1], 0.0);
                    }
                    else
                    {
                        p_plane->SetNormal(bisection_vector[0], bisection_vector[1], bisection_vector[2]);
                    }
                    start_planes.push_back(p_plane);
                }
            }
        }

        if (p_end_node->GetNumberOfSegments() == 1)
        {
            c_vector<double, DIM> node_location = p_end_node->rGetLocation().GetLocation(mReferenceLength);
            vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
            p_plane->SetOrigin(node_location[0], node_location[1], node_location[2]);
            p_plane->SetNormal(segment_tangent[0], segment_tangent[1], segment_tangent[2]);
            end_planes.push_back(p_plane);
        }
        else
        {
            std::vector<std::shared_ptr<VesselSegment<DIM> > > node_segments = p_end_node->GetSegments();
            for (unsigned jdx = 0; jdx < node_segments.size(); jdx++)
            {
                if (node_segments[jdx] != segments[idx])
                {
                    c_vector<double, DIM> other_segment_tangent = node_segments[jdx]->GetUnitTangent();
                    if (node_segments[jdx]->GetNode(1) == p_end_node)
                    {
                        other_segment_tangent = -other_segment_tangent;
                    }
                    average_end_normal += VectorProduct(segment_tangent, other_segment_tangent);

                    c_vector<double, DIM> node_location = p_end_node->rGetLocation().GetLocation(mReferenceLength);
                    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
                    if(DIM==2)
                    {
                        p_plane->SetOrigin(node_location[0], node_location[1], 0.0);
                    }
                    else
                    {
                        p_plane->SetOrigin(node_location[0], node_location[1], node_location[2]);
                    }

                    c_vector<double, DIM> bisection_vector = segment_tangent + other_segment_tangent;
                    bisection_vector /= norm_2(bisection_vector);
                    if(DIM==2)
                    {
                        p_plane->SetNormal(bisection_vector[0], bisection_vector[1], 0.0);
                    }
                    else
                    {
                        p_plane->SetNormal(bisection_vector[0], bisection_vector[1], bisection_vector[2]);
                    }
                    end_planes.push_back(p_plane);
                }
            }
        }

        // Project the precursor points onto the first plane they hit
        Translate(start_points, segments[idx]->GetMidPoint().GetLocation(mReferenceLength));
        Translate(end_points, segments[idx]->GetMidPoint().GetLocation(mReferenceLength));

        std::vector<c_vector<double, DIM> > projected_start_points = start_points;
        std::vector<c_vector<double, DIM> > projected_end_points = end_points;

        for (unsigned jdx = 0; jdx < start_planes.size(); jdx++)
        {
            if (jdx == 0)
            {
                ProjectOnPlane(projected_start_points, -segment_tangent, 2.0 * (segments[idx]->GetLength()/mReferenceLength), start_planes[jdx]);
            }
            else
            {
                std::vector<c_vector<double, DIM> > candidate_points = start_points;
                ProjectOnPlane(candidate_points, -segment_tangent, 2.0 * (segments[idx]->GetLength()/mReferenceLength), start_planes[jdx]);
                for (unsigned mdx = 0; mdx < projected_start_points.size(); mdx++)
                {
                    if (norm_2(candidate_points[mdx] - start_points[mdx])
                            < norm_2(projected_start_points[mdx] - start_points[mdx]))
                    {
                        projected_start_points[mdx] = candidate_points[mdx];
                    }
                }
            }
        }

        for (unsigned jdx = 0; jdx < end_planes.size(); jdx++)
        {
            if (jdx == 0)
            {
                ProjectOnPlane(projected_end_points, -segment_tangent, 2.0 * (segments[idx]->GetLength()/mReferenceLength),end_planes[jdx]);
            }
            else
            {
                std::vector<c_vector<double, DIM> > candidate_points = end_points;
                ProjectOnPlane(candidate_points, -segment_tangent, 2.0 * (segments[idx]->GetLength()/mReferenceLength), end_planes[jdx]);
                for (unsigned mdx = 0; mdx < projected_end_points.size(); mdx++)
                {
                    if (norm_2(candidate_points[mdx] - end_points[mdx])
                            < norm_2(projected_end_points[mdx] - end_points[mdx]))
                    {
                        projected_end_points[mdx] = candidate_points[mdx];
                    }
                }
            }
        }

        // Create the vertices and polygons
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > start_vertices;
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > end_vertices;

        for (unsigned jdx = 0; jdx < projected_start_points.size(); jdx++)
        {
            start_vertices.push_back(DimensionalChastePoint<DIM>::Create(projected_start_points[jdx], mReferenceLength));
        }
        for (unsigned jdx = 0; jdx < projected_end_points.size(); jdx++)
        {
            end_vertices.push_back(DimensionalChastePoint<DIM>::Create(projected_end_points[jdx], mReferenceLength));
        }

        //
        std::vector<PolygonPtr<DIM> > polygons;
        for (unsigned jdx = 0; jdx < start_vertices.size(); jdx++)
        {
            unsigned index2;
            if (jdx != start_vertices.size() - 1)
            {
                index2 = jdx + 1;
            }
            else
            {
                index2 = 0;
            }
            PolygonPtr<DIM> p_polygon = Polygon<DIM>::Create(start_vertices[jdx]);
            p_polygon->AddVertex(start_vertices[index2]);
            p_polygon->AddVertex(end_vertices[index2]);
            p_polygon->AddVertex(end_vertices[jdx]);
            polygons.push_back(p_polygon);
        }
        // If a node has connectivity one add an end-cap
        if (segments[idx]->GetNode(0)->GetNumberOfSegments() == 1)
        {
            polygons.push_back(Polygon<DIM>::Create(start_vertices));
        }

        if (segments[idx]->GetNode(1)->GetNumberOfSegments() == 1)
        {
            polygons.push_back(Polygon<DIM>::Create(end_vertices));
        }
        segment_polygons.push_back(polygons);
    }
    return segment_polygons;
}

template<unsigned DIM>
std::vector<PolygonPtr<DIM> > VesselSurfaceGenerator<DIM>::GetSurfacePolygons()
{
    std::vector<std::vector<PolygonPtr<DIM> > > segment_polygons = GetSurface();
    std::vector<PolygonPtr<DIM> > polygons;
    for (unsigned idx = 0; idx < segment_polygons.size(); idx++)
    {
        for (unsigned jdx = 0; jdx < segment_polygons[idx].size(); jdx++)
        {
            polygons.push_back(segment_polygons[idx][jdx]);
        }
    }
    return polygons;
}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> VesselSurfaceGenerator<DIM>::GetVtkSurface()
{
    std::vector<std::vector<PolygonPtr<DIM> > > segment_polygons = GetSurface();

    // Add the polygons to a part
    Part<DIM> part;
    for (unsigned idx = 0; idx < segment_polygons.size(); idx++)
    {
        for (unsigned jdx = 0; jdx < segment_polygons[idx].size(); jdx++)
        {
            part.AddPolygon(segment_polygons[idx][jdx]->GetVertices(), true);
        }
    }

    mpSurface = part.GetVtk();
    return mpSurface;
}

template<unsigned DIM>
std::vector<c_vector<double, DIM> > VesselSurfaceGenerator<DIM>::MakeCircle(double radius, unsigned numberOfSegments)
{

    double increment = 2.0 * M_PI / double(numberOfSegments);
    double angle = 0.0;
    std::vector<c_vector<double, DIM> > points;

    for (unsigned idx = 0; idx < numberOfSegments; idx++)
    {
        c_vector<double, DIM> point;
        point[0] = radius * std::cos(angle);
        point[1] = radius * std::sin(angle);
        point[2] = 0.0;
        points.push_back(point);
        angle += increment;
    }
    return points;
}

template<unsigned DIM>
void VesselSurfaceGenerator<DIM>::ProjectOnPlane(std::vector<c_vector<double, DIM> >& rPoints,
                                                 c_vector<double, DIM> directionVector, double length,
                                                 vtkSmartPointer<vtkPlane> plane)
{
    for (unsigned idx = 0; idx < rPoints.size(); idx++)
    {
        c_vector<double, DIM> point_on_line = rPoints[idx] + length * directionVector;
        c_vector<double, DIM> projected_point;
        double parametric_distance;
        plane->IntersectWithLine(&rPoints[idx][0], &point_on_line[0], parametric_distance, &projected_point[0]);
        rPoints[idx] = projected_point;
    }
}

template<unsigned DIM>
void VesselSurfaceGenerator<DIM>::RotateAboutAxis(std::vector<c_vector<double, DIM> >& rPoints, c_vector<double, DIM> axis,
                                                  double angle)
{
    double sin_a = std::sin(angle);
    double cos_a = std::cos(angle);
    c_vector<double, DIM> unit_axis = axis / norm_2(axis);

    for (unsigned idx = 0; idx < rPoints.size(); idx++)
    {
        double dot_product = inner_prod(rPoints[idx], unit_axis);
        c_vector<double, DIM> new_point;
        new_point[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + rPoints[idx][0] * cos_a
                + (-unit_axis[2] * rPoints[idx][1] + unit_axis[1] * rPoints[idx][2]) * sin_a);
        new_point[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + rPoints[idx][1] * cos_a
                + (unit_axis[2] * rPoints[idx][0] - unit_axis[0] * rPoints[idx][2]) * sin_a);
        new_point[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + rPoints[idx][2] * cos_a
                + (-unit_axis[1] * rPoints[idx][0] + unit_axis[0] * rPoints[idx][1]) * sin_a);

        rPoints[idx] = new_point;
    }
}

template<unsigned DIM>
void VesselSurfaceGenerator<DIM>::Translate(std::vector<c_vector<double, DIM> >& rPoints,
                                            c_vector<double, DIM> translationVector)
{
    for (unsigned idx = 0; idx < rPoints.size(); idx++)
    {
        rPoints[idx] += translationVector;
    }
}

template class VesselSurfaceGenerator<2> ;
template class VesselSurfaceGenerator<3> ;
