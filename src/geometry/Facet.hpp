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

#ifndef FACET_HPP_
#define FACET_HPP_

#include <vector>
#include <map>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "Polygon.hpp"
#include "DimensionalChastePoint.hpp"

/**
 * A collection of planar polygons
 */
template<unsigned DIM>
class Facet
{
    /**
     * The polygons making up the facet
     */
    std::vector<boost::shared_ptr<Polygon<DIM> > > mPolygons;

    /**
     * Unique vertices in the facet. This is not always up-to-date.
     * Use GetVertices() to ensure up-to-date vertices are used.
     */
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > mVertices;

    /**
     * Whether mVertices is up-to-date. This should be set false when new
     * polygons are added.
     */
    bool mVerticesUpToDate;

    /**
     * Data container, useful for specifying boundary conditions on facets
     */
    std::map<std::string, double> mData;

    /**
     * A label for the application of boundary conditions
     */
    std::string mLabel;

    /**
     * The reference length scale
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     * @param polygons a facet is made from these polygons
     */
    Facet(std::vector<boost::shared_ptr<Polygon<DIM> > > polygons);

    /**
     * Constructor
     * @param pPolygon a single polygon for the facet
     */
    Facet(boost::shared_ptr<Polygon<DIM> > pPolygon);

    /**
     * Factory constructor method
     * @param polygons planar polygons
     * @return a shared pointer to a new facet
     */
    static boost::shared_ptr<Facet<DIM> > Create(std::vector<boost::shared_ptr<Polygon<DIM> > > polygons);

    /**
     * Factory constructor method
     * @param pPolygon a polygon
     * @return a smart pointer to a new facet
     */
    static boost::shared_ptr<Facet<DIM> > Create(boost::shared_ptr<Polygon<DIM> > pPolygon);

    /**
     * Desctructor
     */
    ~Facet();

    /**
     * Add polygons
     * @param polygons planar polygons
     */
    void AddPolygons(std::vector<boost::shared_ptr<Polygon<DIM> > > polygons);

    /**
     * Add polygon
     * @param pPolygon a polygon
     */
    void AddPolygon(boost::shared_ptr<Polygon<DIM> > pPolygon);

    /**
     * Return true if the specified location is in the facet
     * @param location the location to be tested
     * @return true if the location is in the facet
     */
    bool ContainsPoint(const DimensionalChastePoint<DIM>& location);

    /**
     * Return the bounding box of the facet
     * @return the bounding box (xmin, xmax, ymin, ymax, zmin, zmax)
     */
    std::vector<units::quantity<unit::length> > GetBoundingBox();

    /**
     * Return the centroid of the facet
     * @return the centroid of the facet
     */
    DimensionalChastePoint<DIM> GetCentroid();

    /**
     * Return the distance to the facet
     * @param rLocation reference to the location of the point for distance calculation
     * @return the distance to the facet
     */
    units::quantity<unit::length> GetDistance(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Get the label for boundary conditions
     * @return label the boundary condition label
     */
    std::string GetLabel();

    /**
     * Return the facet's plane
     * @return a vtk plane on the facet's plane
     */
    vtkSmartPointer<vtkPlane> GetPlane();

    /**
     * Return the normal to the facet
     * @return the normal to the facet
     */
    c_vector<double, DIM> GetNormal();

    /**
     * Return the polygons
     * @return the polygons making up the facet
     */
    std::vector<boost::shared_ptr<Polygon<DIM> > > GetPolygons();

    /**
     * Return the vertices
     * @return the unique vertices in the facet
     */
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > GetVertices();

    /**
     * Return the facet vertices as a set of VtkPoints.
     * @return the facet vertices as a set of VtkPoints.
     */
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > GetVtkVertices();

    /**
     * Rotate about the specified axis by the specified angle
     * @param axis the rotation axis
     * @param angle the rotation angle
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Set the label for boundary conditions
     * @param label the boundary condition label
     */
    void SetLabel(const std::string& label);

    /**
     * Move the facet along the translation vector
     * @param translationVector the new location is the original + the translationVector
     */
    void Translate(DimensionalChastePoint<DIM> translationVector);

    /**
     * Update the mVertices member
     */
    void UpdateVertices();

};

#endif /*FACET_HPP_*/
