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

#include <math.h>
#include <vector>
#include "UblasIncludes.hpp"
#include "Polygon.hpp"
#include "DimensionalChastePoint.hpp"
#include "MappableGridGenerator.hpp"
#include "BaseUnits.hpp"

MappableGridGenerator::MappableGridGenerator() :
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
}

MappableGridGenerator::~MappableGridGenerator()
{

}

boost::shared_ptr<Part<3> > MappableGridGenerator::GeneratePlane(unsigned numX, unsigned numY, bool withEndCaps)
{
    // Make a regular grid of polygons
    // Front vertices
    std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices;
    for(unsigned jdx=0; jdx< numY; jdx++)
    {
        for(unsigned idx=0; idx<numX; idx++)
        {
            double x_location = double(idx);
            double y_location = double(jdx);
            double z_location = 0.0;

            // Create the vertices
            vertices.push_back(DimensionalChastePoint<3>::Create(x_location, y_location, z_location, mReferenceLength));
        }
    }

    // Back vertices
    for(unsigned jdx=0; jdx< numY; jdx++)
    {
        for(unsigned idx=0; idx<numX; idx++)
        {
            double x_location = double(idx);
            double y_location = double(jdx);
            double z_location = 1.0;

            // Create the vertices
            vertices.push_back(DimensionalChastePoint<3>::Create(x_location, y_location, z_location, mReferenceLength));

        }
    }

    // Make the polygons
    // Front face
    std::vector<boost::shared_ptr<Polygon<3> > > polygons;
    for(unsigned jdx=0; jdx< numY - 1; jdx++)
    {
        for(unsigned idx=0; idx<numX - 1; idx++)
        {
            unsigned front_left_index = idx + numX * jdx;
            unsigned front_right_index = idx + 1 + numX * jdx;
            unsigned front_left_top_index = idx + numX * (jdx+1);
            unsigned front_right_top_index = idx + 1 + numX * (jdx+1);

            std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
            poly_vertices.push_back(vertices[front_left_index]);
            poly_vertices.push_back(vertices[front_right_index]);
            poly_vertices.push_back(vertices[front_right_top_index]);
            poly_vertices.push_back(vertices[front_left_top_index]);
            polygons.push_back(Polygon<3>::Create(poly_vertices));
        }
    }

    // Back face
    for(unsigned jdx=0; jdx< numY - 1; jdx++)
    {
        for(unsigned idx=0; idx<numX - 1; idx++)
        {
            unsigned front_left_index = idx + numX * jdx + numX*numY;
            unsigned front_right_index = idx + 1 + numX * jdx + numX*numY;
            unsigned front_left_top_index = idx + numX * (jdx+1) + numX*numY;
            unsigned front_right_top_index = idx + 1 + numX * (jdx+1) + numX*numY;

            std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
            poly_vertices.push_back(vertices[front_left_index]);
            poly_vertices.push_back(vertices[front_right_index]);
            poly_vertices.push_back(vertices[front_right_top_index]);
            poly_vertices.push_back(vertices[front_left_top_index]);
            polygons.push_back(Polygon<3>::Create(poly_vertices));
        }
    }

    if(withEndCaps)
    {
        // Left face
        for(unsigned jdx=0; jdx< numY - 1; jdx++)
        {
            unsigned front_index = numX * jdx;
            unsigned top_front_index = numX * (jdx+1);
            unsigned back_index = numX * jdx + numX*numY;
            unsigned top_back_index = numX * (jdx+1) + numX*numY;

            std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
            poly_vertices.push_back(vertices[front_index]);
            poly_vertices.push_back(vertices[top_front_index]);
            poly_vertices.push_back(vertices[top_back_index]);
            poly_vertices.push_back(vertices[back_index]);
            polygons.push_back(Polygon<3>::Create(poly_vertices));
        }

        // Right face
        for(unsigned jdx=0; jdx< numY - 1; jdx++)
        {
            unsigned front_index = numX * (jdx+1) - 1;
            unsigned top_front_index = numX * (jdx+2) - 1;
            unsigned back_index = numX * (jdx + 1) - 1 + numX*numY;
            unsigned top_back_index = numX * (jdx+2) -1 + numX*numY;

            std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
            poly_vertices.push_back(vertices[front_index]);
            poly_vertices.push_back(vertices[top_front_index]);
            poly_vertices.push_back(vertices[top_back_index]);
            poly_vertices.push_back(vertices[back_index]);
            polygons.push_back(Polygon<3>::Create(poly_vertices));
        }
    }

    // Bottom face
    for(unsigned idx=0; idx< numX - 1; idx++)
    {
        unsigned front_index = idx;
        unsigned front_right_index = idx + 1;
        unsigned back_index = idx + numX*numY;
        unsigned back_right_index = idx + 1 + numX*numY;

        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
        poly_vertices.push_back(vertices[front_index]);
        poly_vertices.push_back(vertices[front_right_index]);
        poly_vertices.push_back(vertices[back_right_index]);
        poly_vertices.push_back(vertices[back_index]);
        polygons.push_back(Polygon<3>::Create(poly_vertices));
    }

    // Top face
    for(unsigned idx=0; idx< numX - 1; idx++)
    {
        unsigned front_index = idx + numX*(numY-1);
        unsigned front_right_index = idx + 1 + numX*(numY-1);
        unsigned back_index = idx + + numX*(numY-1) + numX*numY;
        unsigned back_right_index = idx + numX*(numY-1) + 1 + numX*numY;

        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > poly_vertices;
        poly_vertices.push_back(vertices[front_index]);
        poly_vertices.push_back(vertices[front_right_index]);
        poly_vertices.push_back(vertices[back_right_index]);
        poly_vertices.push_back(vertices[back_index]);
        polygons.push_back(Polygon<3>::Create(poly_vertices));
    }

    // Create a part
    boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        p_part->AddPolygon(polygons[idx], true);
    }
    return p_part;
}

boost::shared_ptr<Part<3> > MappableGridGenerator::GenerateCylinder(double cylinderRadius,
                                               double cylinderThickness,
                                               double cylinderHeight,
                                               unsigned numX,
                                               unsigned numY,
                                               double cylinderAngle)
{
    if(cylinderAngle > 2.0 * M_PI)
    {
        EXCEPTION("The cylinder angle should be <= 2*pi");
    }

    boost::shared_ptr<Part<3> > p_part = GeneratePlane(numX, numY, !(cylinderAngle == 2.0 * M_PI));

    // Get the part extents
    std::vector<units::quantity<unit::length> > bbox = p_part->GetBoundingBox();

    // Get the vertices
    std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices = p_part->GetVertices();
    for(unsigned idx =0; idx<vertices.size(); idx++)
    {
        c_vector<double, 3> vertex_location = vertices[idx]->GetLocation(mReferenceLength);
        double x_frac = vertex_location[0]*mReferenceLength / (bbox[1] - bbox[0]);
        double angle = x_frac * cylinderAngle;

        double y_frac = vertex_location[1]*mReferenceLength / (bbox[3] - bbox[2]);
        double height = y_frac * cylinderHeight;

        double z_frac = vertex_location[2]*mReferenceLength / (bbox[5] - bbox[4]);
        double radius = cylinderRadius - cylinderThickness * z_frac;

        // Get the new x
        c_vector<double, 3> new_position;
        new_position[0] = radius * std::cos(angle);

        // Get the new y
        new_position[1] = height;

        // Get the new z
        new_position[2] = radius * std::sin(angle);
        vertices[idx]->TranslateTo(DimensionalChastePoint<3>(new_position, mReferenceLength));
    }

    p_part->MergeCoincidentVertices();
    return p_part;
}

boost::shared_ptr<Part<3> > MappableGridGenerator::GenerateHemisphere(double sphereRadius,
                                               double sphereThickness,
                                               unsigned numX,
                                               unsigned numY,
                                               double sphereAzimuthAngle,
                                               double spherePolarAngle)
{
    if(sphereAzimuthAngle >= 2.0 * M_PI)
    {
        EXCEPTION("The azimuth angle should be < 2*pi");
    }

    if(spherePolarAngle >= M_PI)
    {
        EXCEPTION("The polar angle should be < pi");
    }

    boost::shared_ptr<Part<3> > p_part = GeneratePlane(numX, numY);

    // The part extents
    std::vector<units::quantity<unit::length> > bbox = p_part->GetBoundingBox();

    // Get the vertices
    std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices = p_part->GetVertices();
    for(unsigned idx =0; idx<vertices.size(); idx++)
    {
        c_vector<double, 3> vertex_location = vertices[idx]->GetLocation(mReferenceLength);
        double x_frac = vertex_location[0]*mReferenceLength / (bbox[1] - bbox[0]);
        double azimuth_angle = x_frac * sphereAzimuthAngle;

        double y_frac = vertex_location[1]*mReferenceLength / (bbox[3] - bbox[2]);
        double polar_angle = y_frac * spherePolarAngle;

        double z_frac = vertex_location[2]*mReferenceLength / (bbox[5] - bbox[4]);
        double radius = sphereRadius - sphereThickness * z_frac;

        // Get the new x
        c_vector<double, 3> new_position;
        new_position[0] = radius * std::cos(azimuth_angle) * std::sin(polar_angle);

        // Get the new y
        new_position[1] = radius * std::cos(polar_angle);

        // Get the new z
        new_position[2] = radius * std::sin(azimuth_angle) * std::sin(polar_angle);

        vertices[idx]->TranslateTo(DimensionalChastePoint<3>(new_position, mReferenceLength));
    }

    p_part->MergeCoincidentVertices();
    return p_part;
}
