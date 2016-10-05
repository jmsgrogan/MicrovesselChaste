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

#ifndef POLYGON_HPP_
#define POLYGON_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "UblasVectorInclude.hpp"
#include "SmartPointers.hpp"
#include "Vertex.hpp"
#include "UnitCollection.hpp"

/**
 * A collection of planar vertices, joined in the order they are added.
 */

class Polygon
{
    /**
     * The vertices of the polygon. They should be co-planar.
     * Vertices should be unique, this is not ensured in the class.
     */
    std::vector<boost::shared_ptr<Vertex> > mVertices;

public:

    /**
     * Constructor
     * @param vertices a vector of planar vertices, to be joined in the order they are added.
     */
    Polygon(std::vector<boost::shared_ptr<Vertex> > vertices);

    /**
     * Constructor
     * @param pVertex a vertex
     */
    Polygon(boost::shared_ptr<Vertex> pVertex);

    /**
     * Factory constructor method
     * @param vertices a vector of planar vertices, to be joined in the order they are added.
     * @return a shared pointer to a new polygon
     */
    static boost::shared_ptr<Polygon> Create(std::vector<boost::shared_ptr<Vertex> > vertices);

    /**
     * Factory constructor method
     * @param pVertex a vertex
     * @return a shared pointer to a new polygon
     */
    static boost::shared_ptr<Polygon> Create(boost::shared_ptr<Vertex> pVertex);

    /**
     * Desctructor
     */
    ~Polygon();

    /**
     * Add vertices
     * @param vertices a vector of planar vertices, to be joined in the order they are added.
     */
    void AddVertices(std::vector<boost::shared_ptr<Vertex> > vertices);

    /**
     * Add vertex
     * @param pVertex a vertex to be added. It is best to add multiple vertices at once.
     */
    void AddVertex(boost::shared_ptr<Vertex> pVertex);

    /**
     * Return true if the specified location is in the polygon, uses vtk point in polygon.
     * @param rLocation the location to be tested
     * @return true if the location is in the polygon
     */
    bool ContainsPoint(const DimensionalChastePoint<3>& rLocation);

    /**
     * Return the bounding box of the polygon
     * @return the bounding box (xmin, xmax, ymin, ymax, zmin, zmax)
     */
    c_vector<double, 6> GetBoundingBox();

    /**
     * Return the centroid of the polygon
     * @return the centroid of the polygon
     */
    DimensionalChastePoint<3> GetCentroid();

    /**
     * Return the distance to the polygon's plane
     * @param rLocation the location of the point to get the distance from
     * @return the distance to the plane containing the polygon
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<3>& rLocation);

    /**
     * Return the shortest distance to the polygon's edges
     * @param rLocation the location of the point to get the distance from
     * @return the shortest distance to the polygon edges
     */
    units::quantity<unit::length> GetDistanceToEdges(const DimensionalChastePoint<3>& rLocation);

    /**
     * Return the polygon's plane
     * @return a vtk plane on the polygon's plane
     */
    vtkSmartPointer<vtkPlane> GetPlane();

    /**
     * Return the normal to the polygon, must have 3 or more points
     * @return the normal to the polygon's plane
     */
    c_vector<double, 3> GetNormal();

    /**
     * Return the vertices
     * @param idx index of the vertex to return
     * @return pointer to the indexed vertex
     */
    boost::shared_ptr<Vertex > GetVertex(unsigned idx);

    /**
     * Return the vertices
     * @return the polygon's vertices
     */
    std::vector<boost::shared_ptr<Vertex> > GetVertices();

    /**
     * Return a pointer to a VtkPolygon representation.
     * @return a vtk polygon representation of the polygon
     */
    vtkSmartPointer<vtkPolygon> GetVtkPolygon();

    /**
     * Return the polygon vertices as a set of VtkPoints.
     * @return a pair consisting of vtk representation of the vertices and corresponding ids
     */
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > GetVtkVertices();

    /**
     * Replace an exiting vertex with the passed in one.
     * @param idx the index of the vertex to be replaced
     * @param pVertex the new vertex
     */
    void ReplaceVertex(unsigned idx, boost::shared_ptr<Vertex > pVertex);

    /**
     * Rotate about the specified axis by the specified angle
     * @param axis the rotation axis
     * @param angle the rotation angle
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Move the polygon along the translation vector
     * @param translationVector the new location is the original + the translationVector
     */
    void Translate(c_vector<double, 3> translationVector);
};

#endif /* POLYGON_HPP_*/
