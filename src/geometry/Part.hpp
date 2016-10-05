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

#ifndef PART_HPP_
#define PART_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "UblasVectorInclude.hpp"
#include "Vertex.hpp"
#include "Polygon.hpp"
#include "Facet.hpp"
#include "VesselNetwork.hpp"
#include "DimensionalChastePoint.hpp"
#include "UnitCollection.hpp"
#include "GeometryWriter.hpp"

/**
 * A geometric feature described using a PLC (piecewise linear complex) description
 * (see the tetgen manual for details:http://wias-berlin.de/software/tetgen/).
 * These descriptions allow parts to be meshed using triangle or tetgen.
 */
template<unsigned DIM>
class Part
{
    /**
     * Planar collections of polygons
     */
    std::vector<boost::shared_ptr<Facet> > mFacets;

    /**
     * A vtk representation of the part
     */
    vtkSmartPointer<vtkPolyData> mVtkPart;

    /**
     * The locations of hole markers (see PLC definition)
     */
    std::vector<DimensionalChastePoint<DIM> > mHoleMarkers;

    /**
     * The locations of region markers (see PLC definition)
     */
    std::vector<DimensionalChastePoint<DIM> > mRegionMarkers;

    /**
     * The reference length scale
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * Is the vtk representation up-to-date
     */
    bool mVtkIsUpToDate;

public:

    /**
     * Constructor
     */
    Part();

    /**
     * Factory constructor method
     * @return a shared pointer to a new part
     */
    static boost::shared_ptr<Part<DIM> > Create();

    /**
     * Destructor
     */
    ~Part();

    /**
     * Add a circle to the part. If a target facet is not specified the default position is normal to the z-axis.
     * @param radius the circle radius
     * @param centre the centre of the circle
     * @param numSegments the number of linear segments the circle is described with
     * @return polygon corresponding to the circle, useful for further operations, such as extrusion.
     */
    boost::shared_ptr<Polygon> AddCircle(units::quantity<unit::length> radius,
                                         DimensionalChastePoint<DIM> centre,
                                         unsigned numSegments = 24);

    /**
     * Add a cylinder to the part.
     * @param radius the radius
     * @param depth the depth
     * @param centre the centre of the base
     * @param numSegments the number of line segments the base is described with
     */
    void AddCylinder(units::quantity<unit::length> radius, units::quantity<unit::length> depth, DimensionalChastePoint<DIM> centre,
                     unsigned numSegments = 24);

    /**
     * Add a cuboid to the part.
     * @param sizeX the dimension in x
     * @param sizeY the dimension in y
     * @param sizeZ the dimension in z
     * @param origin the bottom, left, front corner
     */
    void AddCuboid(units::quantity<unit::length> sizeX,
                   units::quantity<unit::length> sizeY,
                   units::quantity<unit::length> sizeZ,
                   DimensionalChastePoint<DIM> origin);

    /**
     * Add a hole marker to the part
     * @param location the location of the hole
     */
    void AddHoleMarker(DimensionalChastePoint<DIM> location);

    /**
     * Add a polygon described by a vector or vertices. The vertices should be planar. This is not
     * checked.
     * @param vertices a vector of vertices making up the polygon
     * @param newFacet whether to add a new facet
     * @param pFacet an optional facet that the circle can be generated on
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    boost::shared_ptr<Polygon> AddPolygon(std::vector<boost::shared_ptr<Vertex> > vertices,
                                          bool newFacet = false,
                                          boost::shared_ptr<Facet> pFacet = boost::shared_ptr<Facet>());

    /**
     * Add a polygon
     * @param pPolygon a polygon to add to the part
     * @param newFacet whether to add a new facet
     * @param pFacet an optional facet that the polygon can be generated on
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    boost::shared_ptr<Polygon> AddPolygon(boost::shared_ptr<Polygon> pPolygon,
                                          bool newFacet = false,
                                          boost::shared_ptr<Facet> pFacet = boost::shared_ptr<Facet>());

    /**
     * Add a rectangle to the part, oriented by default with out of plane direction along the z-axis.
     * @param sizeX the dimension in the x direction
     * @param sizeY the dimension in the y direction
     * @param origin the bottom left corner
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    boost::shared_ptr<Polygon> AddRectangle(units::quantity<unit::length> sizeX,
                                            units::quantity<unit::length> sizeY,
                                            DimensionalChastePoint<DIM> origin);

    /**
     * Add a vessel network to the part.
     * @param pVesselNetwork the vessel network to be added
     * @param surface true if a surface representation of the network is required
     */
    void AddVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork, bool surface = false);

    /**
     * Remove vessels outside the part
     * @param pVesselNetwork the vessel network to be pruned
     */
    void BooleanWithNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Extrude the part along the z-axis, inserting planar faces in place of edges.
     * @param pPolygon the polygon to extrude
     * @param distance the extrusion distance
     */
    void Extrude(boost::shared_ptr<Polygon> pPolygon, units::quantity<unit::length> distance);

    /**
     * Return the bounding box
     * @return the bounding box of the part (xmin, xmax, ymin, ymax, zmin, zmax)
     */
    c_vector<double, 2*DIM> GetBoundingBox();

    /**
     * Return the indices of the grid that are inside the part
     * @param num_x number of grid points in x
     * @param num_y number of grid points in y
     * @param num_z number of grid points in z
     * @param spacing the grid spacing
     * @return a vector of grid indices
     */
    std::vector<unsigned> GetContainingGridIndices(unsigned num_x, unsigned num_y = 1, unsigned num_z = 1, double spacing = 1.0);

    /**
     * Return the hole marker locations
     * @return the hole marker locations
     */
    std::vector<DimensionalChastePoint<DIM> > GetHoleMarkers();

    /**
     * Return the facets
     * @return the facets
     */
    std::vector<boost::shared_ptr<Facet> > GetFacets();

    /**
     * Return the FIRST facet found on the point. Strict method, returns exception if there is no facet at the point.
     * @param rLocation the probe point
     * @return the FIRST found facet on the point.
     */
    boost::shared_ptr<Facet> GetFacet(const DimensionalChastePoint<3>& rLocation);

    /**
     * Return the polygons
     * @return the polygons
     */
    std::vector<boost::shared_ptr<Polygon> > GetPolygons();

    /**
     * Return the reference length scale
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale();

    /**
     * Return the segment indexes, used for 2D meshing
     * @return the indices of vertices corresponding to segments (edges) in the part
     */
    std::vector<std::pair<unsigned, unsigned> > GetSegmentIndices();

    /**
     * Return the unique vertices
     * @return the unique vertices
     */
    std::vector<boost::shared_ptr<Vertex> > GetVertices();

    /**
     * Return the vertex locations
     * @return the vertex locations
     */
    std::vector<c_vector<double, DIM> > GetVertexLocations();

    /**
     * Return the a vtk polydata representation of the part
     * @return a vtk representation of the part
     */
    vtkSmartPointer<vtkPolyData> GetVtk();

    /**
     * Is the point inside the part
     * @param location the location of the point
     * @return bool true if the point is inside the part
     */
    bool IsPointInPart(DimensionalChastePoint<DIM> location);

    /**
     * Is the point inside the part
     * @param location the location of the point
     * @return bool true if the point is inside the part
     */
    std::vector<bool> IsPointInPart(const std::vector<DimensionalChastePoint<DIM> >& location);

    /**
     * Merge vertices that overlap in polygons and facets
     */
    void MergeCoincidentVertices();

    /**
     * Set the reference length scale
     * @param referenceLength the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> referenceLength);

    /**
     * Move the part along the translation vector
     * @param vector the vector to move the part along
     */
    void Translate(c_vector<double, DIM> vector);

    /**
     * Write the part to file in vtk format
     * @param rFilename the path to the file to be written, without extension
     * @param format the output format
     */
    void Write(const std::string& rFilename, GeometryFormat::Value format = GeometryFormat::VTP);

};

#endif /*PART_HPP_*/
