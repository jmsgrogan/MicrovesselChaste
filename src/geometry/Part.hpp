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

#ifndef PART_HPP_
#define PART_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellLocator.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include "ChasteSerialization.hpp"
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "UblasVectorInclude.hpp"
#include "DimensionalChastePoint.hpp"
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
     * Archiving
     */
    friend class boost::serialization::access;

    /**
     * Do the serialize
     * @param ar the archive
     * @param version the archive version number
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mFacets;
        ar & mHoleMarkers;
        ar & mRegionMarkers;
        ar & mReferenceLength;
        ar & mAttributes;
        ar & mAttributeKeys;
    }

    /**
     * Planar collections of polygons
     */
    std::vector<std::shared_ptr<Facet<DIM> > > mFacets;

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
    std::vector<std::pair<DimensionalChastePoint<DIM>, unsigned> > mRegionMarkers;

    /**
     * The reference length scale
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * Is the vtk representation up-to-date
     */
    bool mVtkIsUpToDate;

    /**
     * Attributes for the part
     */
    std::map<std::string, double> mAttributes;

    /**
     * Handy for point/cell location operations
     */
    vtkSmartPointer<vtkCellLocator> mpVtkCellLocator;

    /**
     * Map of attribute labels for mesh attribute numbers
     */
    std::map<unsigned, std::string> mAttributeKeys;

public:

    /**
     * Constructor
     */
    Part();

    /**
     * Factory constructor method
     * @return a shared pointer to a new part
     */
    static std::shared_ptr<Part<DIM> > Create();

    /**
     * Destructor
     */
    ~Part();

    /**
     * Add an attribute to the polygon
     * @param rLabel the attribute label
     * @param value the attribute value
     */
    void AddAttribute(const std::string& rLabel, double value);

    /**
     * Label a polygon edge if it is found
     * @param loc the search point
     * @param rLabel the label
     * @return true if an edge is found
     */
    void AddAttributeToEdgeIfFound(DimensionalChastePoint<DIM> loc, const std::string& rLabel, double value);

    /**
     * Label all polygons
     * @param value the value
     * @param rLabel the label
     * @return true if an edge is found
     */
    void AddAttributeToPolygons(const std::string& rLabel, double value);

    /**
     * Label all polygons
     * @param value the value
     * @param rLabel the label
     * @return true if an edge is found
     */
    void AddAttributeToPolygonIfFound(DimensionalChastePoint<DIM> loc, const std::string& rLabel, double value);

    /**
     * Add a circle to the part. If a target facet is not specified the default position is normal to the z-axis.
     * @param radius the circle radius
     * @param centre the centre of the circle
     * @param numSegments the number of linear segments the circle is described with
     * @return polygon corresponding to the circle, useful for further operations, such as extrusion.
     */
    std::shared_ptr<Polygon<DIM> > AddCircle(units::quantity<unit::length> radius,
                                         DimensionalChastePoint<DIM> centre, unsigned numSegments = 24);

    /**
     * Add a cylinder to the part.
     * @param radius the radius
     * @param depth the depth
     * @param centre the centre of the base
     * @param numSegments the number of line segments the base is described with
     */
    void AddCylinder(units::quantity<unit::length> radius, units::quantity<unit::length> depth,
            DimensionalChastePoint<DIM> centre, unsigned numSegments = 24);

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
     * Add a region marker to the part
     * @param location the location of the region
     * @param value the region value
     */
    void AddRegionMarker(DimensionalChastePoint<DIM> location, unsigned value);

    /**
     * Add a polygon described by a vector or vertices. The vertices should be planar. This is not
     * checked.
     * @param vertices a vector of vertices making up the polygon
     * @param newFacet whether to add a new facet
     * @param pFacet an optional facet that the circle can be generated on
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    std::shared_ptr<Polygon<DIM> > AddPolygon(std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices,
                                          bool newFacet = false,
                                          std::shared_ptr<Facet<DIM> > pFacet = std::shared_ptr<Facet<DIM> >());

    /**
     * Add a polygon
     * @param pPolygon a polygon to add to the part
     * @param newFacet whether to add a new facet
     * @param pFacet an optional facet that the polygon can be generated on
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    std::shared_ptr<Polygon<DIM> > AddPolygon(std::shared_ptr<Polygon<DIM> > pPolygon,
                                          bool newFacet = false,
                                          std::shared_ptr<Facet<DIM> > pFacet = std::shared_ptr<Facet<DIM> >());

    /**
     * Add a rectangle to the part, oriented by default with out of plane direction along the z-axis.
     * @param sizeX the dimension in the x direction
     * @param sizeY the dimension in the y direction
     * @param origin the bottom left corner
     * @return the new polygon, useful for further operations, such as extrusion.
     */
    std::shared_ptr<Polygon<DIM> > AddRectangle(units::quantity<unit::length> sizeX,
                                            units::quantity<unit::length> sizeY,
                                            DimensionalChastePoint<DIM> origin);

    /**
     * Add a vessel network to the part.
     * @param pVesselNetwork the vessel network to be added
     * @param surface true if a surface representation of the network is required
     * @param removeVesselRegion remove vessel region from meshes
     */
    void AddVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork,
            bool surface = false, bool removeVesselRegion = true);

    /**
     * Add a part to the existing one. This takes all the polygons from the incoming part and
     * adds them to new facets on the existing part.
     * @param pPart the part to be appended
     */
    void AppendPart(std::shared_ptr<Part<DIM> > pPart);

    /**
     * Remove vessels outside the part
     * @param pVesselNetwork the vessel network to be pruned
     */
    void BooleanWithNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Extrude the part along the z-axis, inserting planar faces in place of edges.
     * @param pPolygon the polygon to extrude
     * @param distance the extrusion distance
     */
    void Extrude(std::shared_ptr<Polygon<DIM> > pPolygon, units::quantity<unit::length> distance);

    /**
     * Return the bounding box
     * @return the bounding box of the part (xmin, xmax, ymin, ymax, zmin, zmax)
     */
    std::vector<units::quantity<unit::length> > GetBoundingBox();

    /**
     * Return the indices of the grid that are inside the part
     * @param num_x number of grid points in x
     * @param num_y number of grid points in y
     * @param num_z number of grid points in z
     * @param spacing the grid spacing
     * @return a vector of grid indices
     */
    std::vector<unsigned> GetContainingGridIndices(unsigned num_x, unsigned num_y,
            unsigned num_z, units::quantity<unit::length> spacing);

    /**
     * Return the hole marker locations
     * @return the hole marker locations
     */
    std::vector<DimensionalChastePoint<DIM> > GetHoleMarkers();

    /**
     * Return the region marker locations
     * @return the region marker locations
     */
    std::vector<std::pair<DimensionalChastePoint<DIM>, unsigned> > GetRegionMarkers();

    /**
     * Return the part attributes
     * @return the part attributes
     */
    std::map<std::string, double> GetAttributes();

    /**
     * Return the attribute keys for the mesh
     * @return the attribute keys for the mesh
     */
    std::map<unsigned, std::string> GetAttributesKeysForMesh(bool update = true);

    /**
     * Return the facets
     * @return the facets
     */
    std::vector<std::shared_ptr<Facet<DIM> > > GetFacets();

    /**
     * Return the FIRST facet found on the point. Strict method, returns exception if there is no facet at the point.
     * @param rLocation the probe point
     * @return the FIRST found facet on the point.
     */
    std::shared_ptr<Facet<DIM> > GetFacet(const DimensionalChastePoint<DIM>& rLocation);

    /**
     * Return the polygons
     * @return the polygons
     */
    std::vector<std::shared_ptr<Polygon<DIM> > > GetPolygons();

    /**
     * Return the reference length scale
     * @return the reference length scale
     */
    units::quantity<unit::length> GetReferenceLengthScale();

    /**
     * Return the segment indexes, used for 2D meshing
     * @return the indices of vertices corresponding to segments (edges) in the part
     */
    std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > GetSegmentIndices();

    /**
     * Return the unique vertices
     * @return the unique vertices
     */
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > GetVertices();

    /**
     * Return the vertex locations
     * @return the vertex locations
     */
    std::vector<DimensionalChastePoint<DIM> > GetVertexLocations();

    /**
     * Return the a vtk polydata representation of the part
     * @return a vtk representation of the part
     */
    vtkSmartPointer<vtkPolyData> GetVtk(bool includeEdges = false);

    /**
     * Given an attribute value map return a suitable key. Assumes the
     * part attribute map is already up to date.
     * @param rAttributes the attributes
     * @return a duitable key
     */
    unsigned GetKeyForAttributes(std::map<std::string, double> rAttributes);

    /**
     * Return true if the edge at the input point has the supplied label
     * @param loc the search point
     * @param rLabel the label
     * @return true if an edge is found
     */
    bool EdgeHasAttribute(DimensionalChastePoint<DIM> loc, const std::string& rLabel);

    /**
     * Is the point inside the part
     * @param location the location of the point
     * @return bool true if the point is inside the part
     */
    bool IsPointInPart(DimensionalChastePoint<DIM> location);

    /**
     * Is the point inside the part
     * @param pPoints vtk points
     * @return bool true if the point is inside the part
     */
    std::vector<bool> IsPointInPart(vtkSmartPointer<vtkPoints> pPoints);

    /**
     * Merge vertices that overlap in polygons and facets
     */
    void MergeCoincidentVertices();

    /**
     * Rotate about the specified axis by the specified angle
     * @param axis the rotation axis
     * @param angle the rotation angle
     */
    void RotateAboutAxis(c_vector<double, 3> axis, double angle);

    /**
     * Set the reference length scale
     * @param referenceLength the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> referenceLength);

    /**
     * Move the part along the translation vector
     * @param vector the vector to move the part along
     */
    void Translate(DimensionalChastePoint<DIM> vector);

    /**
     * Write the part to file in vtk format
     * @param rFilename the path to the file to be written, without extension
     * @param format the output format
     */
    void Write(const std::string& rFilename, GeometryFormat::Value format = GeometryFormat::VTP, bool includeEdges = false);

};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(Part, 2)
EXPORT_TEMPLATE_CLASS1(Part, 3)

#endif /*PART_HPP_*/
