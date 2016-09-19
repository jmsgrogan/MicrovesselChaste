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

#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include "DimensionalChastePoint.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"

/**
 * A point in 3d space.
 */
class Vertex : public DimensionalChastePoint<3>, public boost::enable_shared_from_this<Vertex>
{
    /**
     *  An optional index
     */
    unsigned mIndex;

public:

    /**
     * Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     */
    Vertex(double x, double y, double z, units::quantity<unit::length> referenceLength);

    /**
     * Constructor
     * @param a vector of x, y, z coordinates
     */
    Vertex(c_vector<double, 3> coords, units::quantity<unit::length> referenceLength);

    /**
     * Constructor
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     */
    Vertex(double x = 0.0, double y = 0.0, double z = 0.0);

    /**
     * Constructor
     * @param a vector of x, y, z coordinates
     */
    Vertex(c_vector<double, 3> coords);

    /**
     * Factory constructor method
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     * @return a shared pointer to a new vertex
     */
    static boost::shared_ptr<Vertex> Create(double x, double y, double z, units::quantity<unit::length> referenceLength);

    /**
     * Factory constructor method
     * @param a vector of x, y, z coordinates
     * @return a shared pointer to a new vertex
     */
    static boost::shared_ptr<Vertex> Create(c_vector<double, 3> coords, units::quantity<unit::length> referenceLength);

    /**
     * Factory constructor method
     * @param x x position of vertex
     * @param y y position of vertex
     * @param z z position of vertex
     * @return a shared pointer to a new vertex
     */
    static boost::shared_ptr<Vertex> Create(double x = 0.0, double y = 0.0, double z = 0.0);

    /**
     * Factory constructor method
     * @param a vector of x, y, z coordinates
     * @return a shared pointer to a new vertex
     */
    static boost::shared_ptr<Vertex> Create(c_vector<double, 3> coords);

    /**
     * Desctructor
     */
    ~Vertex();

    /**
     * Return the index
     * @return the vertex index
     */
    unsigned GetIndex();

    /**
     * Rotate about the specified axis by the specified angle
     * @param axis the rotation axis
     * @param angle the rotation angle
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Set the index
     * @param index the vertex index
     */
    void SetIndex(unsigned index);

    /**
     * Move the vertex along the translation vector
     * @param translationVector the new location is the original + the translationVector
     */
    void Translate(c_vector<double, 3> translationVector);
};

#endif /*VERTEX_HPP_*/
