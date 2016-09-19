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

#ifndef MAPPABLEGRIDGENERATOR_HPP_
#define MAPPABLEGRIDGENERATOR_HPP_

#include "SmartPointers.hpp"
#include "Part.hpp"

/**
 * Generate a part consisting of a grid of unit rectangles or cuboids. It is useful for mapping
 * onto more complex geometries for subsequent meshing. 3-D only.
 */
class MappableGridGenerator
{

public:
    /**
     * Constructor
     */
    MappableGridGenerator();

    /**
     * Destructor
     */
    ~MappableGridGenerator();

    /**
     * Generate a planar grid with one block in the z direction
     * @param numX number of blocks in the x direction
     * @param numY number of blocks in the y direction
     * @param withEndCaps include polygons at the ends of the plane, turned off for closed cylinder generation
     * @return pointer to a Part representation of the plane
     */
    boost::shared_ptr<Part<3> > GeneratePlane(unsigned numX, unsigned numY, bool withEndCaps = true);

    /**
     * Generate a cylindrical grid, where the planar grid is mapped on to a cylinder
     * of specified dimensions.
     * @param cylinderRadius radius of the cylinder
     * @param cylinderThickness thickness of the  cylinder
     * @param cylinderAngle sweep angle of the cylinder (radians)
     * @param cylinderHeight height of the cylinder
     * @param numX number of blocks around the cylinder circumference
     * @param numY number of blocks over the cylinder height
     * @return pointer to a Part representation of the cylinder
     */
    boost::shared_ptr<Part<3> > GenerateCylinder(double cylinderRadius,
                                                   double cylinderThickness,
                                                   double cylinderHeight,
                                                   unsigned numX,
                                                   unsigned numY,
                                                   double cylinderAngle = 2.0 * M_PI);

    /**
     * Generate a hemispherical grid, where the planar grid is mapped on to a sphere
     * of specified dimensions.
     * @param sphereRadius radius of the sphere
     * @param sphereThickness thickness of the  cylinder
     * @param sphereAzimuthAngle azimuth angle of the hemisphere (radians)
     * @param spherePolarAngle polar angle of the hemisphere (radians)
     * @param numX number of blocks around the cylinder circumference
     * @param numY number of blocks over the cylinder height
     * @return pointer to a Part representation of the hemisphere
     */
    boost::shared_ptr<Part<3> > GenerateHemisphere(double sphereRadius,
                                                   double sphereThickness,
                                                   unsigned numX,
                                                   unsigned numY,
                                                   double sphereAzimuthAngle = 2.0 * M_PI,
                                                   double spherePolarAngle = 0.5 * M_PI);
};

#endif /*MAPPABLEGRIDGENERATOR_HPP_*/
