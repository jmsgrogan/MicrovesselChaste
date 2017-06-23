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

#include <math.h>
#include <set>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkCleanPolyData.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkVersion.h>
#include <vtkLine.h>
#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "Part.hpp"
#include "GeometryTools.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
Part<DIM>::Part() :
        mFacets(),
        mVtkPart(vtkSmartPointer<vtkPolyData>()),
        mHoleMarkers(),
        mRegionMarkers(),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mVtkIsUpToDate(false),
        mAttributes(),
        mpVtkCellLocator(),
        mAttributeKeys()
{
}

template<unsigned DIM>
std::shared_ptr<Part<DIM> > Part<DIM>::Create()
{
    return std::make_shared<Part<DIM> >();

}

template<unsigned DIM>
Part<DIM>::~Part()
{

}

template<unsigned DIM>
void Part<DIM>::AddAttribute(const std::string& rLabel, double value)
{
    mAttributes[rLabel] = value;
}

template<unsigned DIM>
void Part<DIM>::AddAttributeToEdgeIfFound(DimensionalChastePoint<DIM> loc, const std::string& rLabel, double value)
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        polygons[idx]->AddAttributeToEdgeIfFound(loc, rLabel, value);
    }
}

template<unsigned DIM>
void Part<DIM>::AddAttributeToPolygonIfFound(DimensionalChastePoint<DIM> loc, const std::string& rLabel, double value)
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        if(polygons[idx]->ContainsPoint(loc))
        {
            polygons[idx]->AddAttribute(rLabel, value);
        }
    }
}

template<unsigned DIM>
void Part<DIM>::AddAttributeToPolygons(const std::string& rLabel, double value)
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        polygons[idx]->AddAttribute(rLabel, value);
    }
}

template<unsigned DIM>
std::shared_ptr<Polygon<DIM> > Part<DIM>::AddCircle(units::quantity<unit::length> radius,
                                                DimensionalChastePoint<DIM> centre, unsigned numSegments)
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    double seg_angle = 2.0 * M_PI / double(numSegments);
    for (unsigned idx = 0; idx < numSegments; idx++)
    {
        double angle = seg_angle * double(idx);
        double x = (radius * std::cos(angle) + centre.GetLocation(mReferenceLength)[0]*centre.GetReferenceLengthScale())/mReferenceLength;
        double y = (radius * std::sin(angle) + centre.GetLocation(mReferenceLength)[1]*centre.GetReferenceLengthScale())/mReferenceLength;
        if(DIM==3)
        {
            vertices.push_back(DimensionalChastePoint<DIM>::Create(x, y, centre.GetLocation(mReferenceLength)[2], mReferenceLength));
        }
        else
        {
            vertices.push_back(DimensionalChastePoint<DIM>::Create(x, y, 0.0, mReferenceLength));
        }
    }
    mVtkIsUpToDate = false;
    return AddPolygon(vertices);
}

template<unsigned DIM>
void Part<DIM>::AddCylinder(units::quantity<unit::length> radius,
                            units::quantity<unit::length> depth,
                            DimensionalChastePoint<DIM> centre,
                            unsigned numSegments)
{
    std::shared_ptr<Polygon<DIM> > p_circle = AddCircle(radius, centre, numSegments);
    Extrude(p_circle, depth);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddCuboid(units::quantity<unit::length> sizeX,
                          units::quantity<unit::length> sizeY,
                          units::quantity<unit::length> sizeZ,
                          DimensionalChastePoint<DIM> origin)
{
    std::shared_ptr<Polygon<DIM> > p_rectangle = AddRectangle(sizeX, sizeY, origin);
    Extrude(p_rectangle, sizeZ);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddHoleMarker(DimensionalChastePoint<DIM> hole)
{
    mHoleMarkers.push_back(hole);
}

template<unsigned DIM>
void Part<DIM>::AddRegionMarker(DimensionalChastePoint<DIM> region, unsigned value)
{
    mRegionMarkers.push_back(std::pair<DimensionalChastePoint<DIM>, unsigned>(region, value));
}

template<unsigned DIM>
void Part<DIM>::AppendPart(std::shared_ptr<Part<DIM> > pPart)
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = pPart->GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        this->AddPolygon(polygons[idx], true);
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
std::shared_ptr<Polygon<DIM> > Part<DIM>::AddPolygon(std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices, bool newFacet,
                                                                                   std::shared_ptr<Facet<DIM> > pFacet)
{
    std::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(vertices);
    AddPolygon(p_polygon, newFacet, pFacet);
    mVtkIsUpToDate = false;
    return p_polygon;
}

template<unsigned DIM>
std::shared_ptr<Polygon<DIM> > Part<DIM>::AddPolygon(std::shared_ptr<Polygon<DIM> > pPolygon, bool newFacet,
                                                 std::shared_ptr<Facet<DIM> > pFacet)
{
    if (!pFacet)
    {
        if (mFacets.size() == 0 || newFacet)
        {
            mFacets.push_back(Facet<DIM>::Create(pPolygon));
        }
        else
        {
            mFacets[0]->AddPolygon(pPolygon);
        }
    }
    else
    {
        pFacet->AddPolygon(pPolygon);
    }
    mVtkIsUpToDate = false;
    return pPolygon;
}

template<unsigned DIM>
std::shared_ptr<Polygon<DIM> > Part<DIM>::AddRectangle(units::quantity<unit::length> sizeX,
                                                   units::quantity<unit::length> sizeY,
                                                   DimensionalChastePoint<DIM> origin)
{
    c_vector<double, DIM> dimensionless_origin = origin.GetLocation(mReferenceLength);
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    if(DIM==3)
    {
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0], dimensionless_origin[1], dimensionless_origin[2], mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0] + sizeX/mReferenceLength,
                                                               dimensionless_origin[1], dimensionless_origin[2],mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0] + sizeX/mReferenceLength,
                                                               dimensionless_origin[1] + sizeY/mReferenceLength, dimensionless_origin[2], mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0], dimensionless_origin[1] + sizeY/mReferenceLength, dimensionless_origin[2],mReferenceLength));
    }
    else
    {
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0], dimensionless_origin[1], 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0] + sizeX/mReferenceLength, dimensionless_origin[1], 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0] + sizeX/mReferenceLength,
                                                               dimensionless_origin[1] + sizeY/mReferenceLength, 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(dimensionless_origin[0], dimensionless_origin[1] + sizeY/mReferenceLength, 0.0, mReferenceLength));
    }
    mVtkIsUpToDate = false;
    return AddPolygon(vertices);
}

template<unsigned DIM>
void Part<DIM>::AddVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork, bool surface,
        bool removeVesselRegion)
{
    if (!surface)
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
        std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = pVesselNetwork->GetNodes();
        for (unsigned idx = 0; idx < nodes.size(); idx++)
        {
            units::quantity<unit::length> length_Scale = nodes[idx]->rGetLocation().GetReferenceLengthScale();
            vertices.push_back(DimensionalChastePoint<DIM>::Create(nodes[idx]->rGetLocation().GetLocation(length_Scale), length_Scale));
        }

        // If vertices lie on any existing facets add the vertex to the facet
        for (unsigned kdx = 0; kdx < vertices.size(); kdx++)
        {
            for (unsigned idx = 0; idx < mFacets.size(); idx++)
            {
                if(mFacets[idx]->ContainsPoint(*(vertices[kdx])))
                {
                    mFacets[idx]->AddPolygon(Polygon<DIM>::Create(vertices[kdx]));
                }
            }
        }

        // Create polygons and facets for each vessel
        std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
        for (unsigned idx = 0; idx < vessels.size(); idx++)
        {
            std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = vessels[idx]->GetSegments();
            std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > segment_vertices;
            for (unsigned jdx = 0; jdx < segments.size(); jdx++)
            {
                unsigned node0_index = pVesselNetwork->GetNodeIndex(segments[jdx]->GetNode(0));
                unsigned node1_index = pVesselNetwork->GetNodeIndex(segments[jdx]->GetNode(1));
                if (jdx == 0)
                {
                    segment_vertices.push_back(vertices[node0_index]);
                }
                segment_vertices.push_back(vertices[node1_index]);
            }
            std::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(segment_vertices);
            mFacets.push_back(Facet<DIM>::Create(p_polygon));
        }
    }
    else
    {
        // Add any polygons on existing facets to the facet
        VesselSurfaceGenerator<DIM> generator(pVesselNetwork);
        std::vector<std::shared_ptr<Polygon<DIM> > > polygons = generator.GetSurfacePolygons();
        std::vector<bool> polygon_on_facet;

        for (unsigned idx = 0; idx < polygons.size(); idx++)
        {
            polygons[idx]->AddAttribute("Vessel Wall", 1.0);
            bool on_facet = false;
            DimensionalChastePoint<DIM> poly_centroid = polygons[idx]->GetCentroid();
            for (unsigned jdx = 0; jdx < mFacets.size(); jdx++)
            {
                if (mFacets[jdx]->ContainsPoint(poly_centroid))
                {
                    on_facet = true;
                    mFacets[jdx]->AddPolygon(polygons[idx]);
                }
            }
            polygon_on_facet.push_back(on_facet);
        }

        // Create polygons and facets for each vessel
        for (unsigned idx = 0; idx < polygons.size(); idx++)
        {
            if (!polygon_on_facet[idx])
            {
                mFacets.push_back(Facet<DIM>::Create(polygons[idx]));
            }
        }

        if(removeVesselRegion)
        {
            std::vector<DimensionalChastePoint<DIM> > hole_locations = generator.GetHoles();
            for(unsigned idx=0; idx<hole_locations.size(); idx++)
            {
                AddHoleMarker(hole_locations[idx]);
            }
        }
        else
        {
            std::vector<DimensionalChastePoint<DIM> > region_locations = generator.GetHoles();
            for(unsigned idx=0; idx<region_locations.size(); idx++)
            {
                AddRegionMarker(region_locations[idx], 2.0);
            }
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
bool Part<DIM>::EdgeHasAttribute(DimensionalChastePoint<DIM> loc, const std::string& rLabel)
{
    bool edge_has_label = false;
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        if(polygons[idx]->EdgeHasAttribute(loc, rLabel))
        {
            return true;
        }
    }
    return edge_has_label;
}

template<unsigned DIM>
void Part<DIM>::Extrude(std::shared_ptr<Polygon<DIM> > pPolygon, units::quantity<unit::length> depth)
{
    if(DIM==2)
    {
        EXCEPTION("Only parts in 3D space can be extruded.");
    }
    // Loop through the vertices and create new ones at the offset depth
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > original_vertices = pPolygon->GetVertices();
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > new_vertices;
    for (unsigned idx = 0; idx < original_vertices.size(); idx++)
    {
        c_vector<double, DIM> vertex_location = original_vertices[idx]->GetLocation(mReferenceLength);
        if(DIM==2)
        {
            new_vertices.push_back(DimensionalChastePoint<DIM>::Create(vertex_location[0], vertex_location[1], depth/mReferenceLength, mReferenceLength));
        }
        else
        {
            new_vertices.push_back(DimensionalChastePoint<DIM>::Create(vertex_location[0], vertex_location[1],
                                                                       vertex_location[2] + depth/mReferenceLength, mReferenceLength));
        }
    }

    // Every straight edge is now a planar face, with 3 new edges ordered in CCW
    for (unsigned idx = 0; idx < original_vertices.size(); idx++)
    {
        unsigned index2;
        if (idx != original_vertices.size() - 1)
        {
            index2 = idx + 1;
        }
        else
        {
            index2 = 0;
        }
        std::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(original_vertices[idx]);
        p_polygon->AddVertex(original_vertices[index2]);
        p_polygon->AddVertex(new_vertices[index2]);
        p_polygon->AddVertex(new_vertices[idx]);
        mFacets.push_back(Facet<DIM>::Create(p_polygon));
    }

    // Close the lid
    std::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(new_vertices);
    mFacets.push_back(Facet<DIM>::Create(p_polygon));
    mVtkIsUpToDate = false;
}

template <unsigned DIM>
void Part<DIM>::BooleanWithNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    // Remove any vessel with both nodes outside the domain
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    for(unsigned idx=0;idx<vessels.size();idx++)
    {
        if(!IsPointInPart(vessels[idx]->GetStartNode()->rGetLocation()) &&
                !IsPointInPart(vessels[idx]->GetEndNode()->rGetLocation()))
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
        }
    }
    pVesselNetwork->UpdateAll();
}

template<unsigned DIM>
std::map<std::string, double> Part<DIM>::GetAttributes()
{
    return mAttributes;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > Part<DIM>::GetHoleMarkers()
{
    return mHoleMarkers;
}

template<unsigned DIM>
std::vector<std::pair<DimensionalChastePoint<DIM>, unsigned> > Part<DIM>::GetRegionMarkers()
{
    return mRegionMarkers;
}

template<unsigned DIM>
units::quantity<unit::length> Part<DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned DIM>
std::map<unsigned, std::string> Part<DIM>::GetAttributesKeysForMesh(bool update)
{
    if(update)
    {
        // Collect all attribute labels
        std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
        std::set<std::string> attribute_labels;

        for(unsigned idx=0; idx<polygons.size(); idx++)
        {
            std::map<std::string, double> attribute_map = polygons[idx]->GetAttributes();
            for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
            {
                attribute_labels.insert(it->first);
            }

            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->GetEdgeAttributes();
            for(unsigned jdx=0;jdx<edge_atts_per_polygon.size();jdx++)
            {
                for(std::map<std::string, double>::iterator it = edge_atts_per_polygon[jdx].begin();
                        it != edge_atts_per_polygon[jdx].end(); ++it)
                {
                    attribute_labels.insert(it->first);
                }
            }
        }
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
        for(unsigned idx=0;idx<vertices.size(); idx++)
        {
            std::map<std::string, double> attribute_map = vertices[idx]->GetAttributes();
            for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
            {
                attribute_labels.insert(it->first);
            }
        }
        mAttributeKeys.clear();
        unsigned counter = 3; // Start on 3 as unlabelled regions default to 0, outer bounds default to 1 and vessel walls to 2
        std::set<std::string>::iterator it;
        for (it = attribute_labels.begin(); it != attribute_labels.end(); ++it)
        {
            mAttributeKeys[counter] = *it;
            counter++;
        }
    }
    return mAttributeKeys;
}

template<unsigned DIM>
std::shared_ptr<Facet<DIM> > Part<DIM>::GetFacet(const DimensionalChastePoint<DIM>& location)
{
    for(unsigned idx=0; idx<mFacets.size(); idx++)
    {
        if(mFacets[idx]->ContainsPoint(location))
        {
            return mFacets[idx];
        }
    }
    EXCEPTION("No facet found at input location");
}

template<unsigned DIM>
std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > Part<DIM>::GetVertices()
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    std::set<std::shared_ptr<DimensionalChastePoint<DIM> > > unique_vertices;
    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > polygon_vertices = polygons[idx]->GetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }

    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    vertices.insert(vertices.end(), unique_vertices.begin(), unique_vertices.end());

    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        vertices[idx]->SetIndex(idx);
    }
    return vertices;
}

template<unsigned DIM>
std::vector<unsigned> Part<DIM>::GetContainingGridIndices(unsigned num_x, unsigned num_y, unsigned num_z, units::quantity<unit::length> spacing)
{
    double scaled_spacing = spacing/mReferenceLength;
    std::vector<unsigned> location_indices;
    for(unsigned kdx=0; kdx<num_z; kdx++)
    {
        for(unsigned jdx=0; jdx<num_y; jdx++)
        {
            for(unsigned idx=0; idx<num_x; idx++)
            {
                DimensionalChastePoint<DIM> location(double(idx) * scaled_spacing, double(jdx) * scaled_spacing, double(kdx) * scaled_spacing, mReferenceLength);
                unsigned index = idx + num_x * jdx + num_x * num_y * kdx;
                if(IsPointInPart(location))
                {
                    location_indices.push_back(index);
                }
            }
        }
    }
    return location_indices;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > Part<DIM>::GetVertexLocations()
{
    std::vector<DimensionalChastePoint<DIM> > vertex_locs;
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        vertex_locs.push_back(*vertices[idx]);
    }
    return vertex_locs;
}

template<unsigned DIM>
std::vector<std::shared_ptr<Polygon<DIM> > > Part<DIM>::GetPolygons()
{
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons;
    for (unsigned idx = 0; idx < mFacets.size(); idx++)
    {
        std::vector<std::shared_ptr<Polygon<DIM> > > facet_polygons = mFacets[idx]->GetPolygons();
        polygons.insert(polygons.end(), facet_polygons.begin(), facet_polygons.end());
    }
    return polygons;
}

template<unsigned DIM>
std::vector<units::quantity<unit::length> > Part<DIM>::GetBoundingBox()
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    c_vector<double, 6> box;

    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        c_vector<double, DIM> vertex_location = vertices[idx]->GetLocation(mReferenceLength);
        for (unsigned jdx = 0; jdx < DIM; jdx++)
        {
            if (idx == 0)
            {
                box[2 * jdx] = vertex_location[jdx];
                box[2 * jdx + 1] = vertex_location[jdx];
            }
            else
            {
                if (vertex_location[jdx] < box[2 * jdx])
                {
                    box[2 * jdx] = vertex_location[jdx];
                }
                if (vertex_location[jdx] > box[2 * jdx + 1])
                {
                    box[2 * jdx + 1] = vertex_location[jdx];
                }
            }
        }
    }

    std::vector<units::quantity<unit::length> > box_vector(6, 0.0*unit::metres);
    for(unsigned idx=0; idx<6; idx++)
    {
        box_vector[idx] = box[idx] * mReferenceLength;
    }

    return box_vector;
}

template<unsigned DIM>
std::vector<std::shared_ptr<Facet<DIM> > > Part<DIM>::GetFacets()
{
    return mFacets;
}

template<unsigned DIM>
unsigned Part<DIM>::GetKeyForAttributes(std::map<std::string, double> rAttributes)
{
    unsigned key=2;
    for(std::map<std::string, double>::iterator it = rAttributes.begin(); it != rAttributes.end(); ++it)
    {
        if(it->second>0.0)
        {
            // Find the key
            for(std::map<unsigned, std::string>::iterator it2 = mAttributeKeys.begin(); it2 != mAttributeKeys.end(); ++it2)
            {
                if(it->first == it2->second)
                {
                    key = it2->first;
                    break;
                }
            }
        }
    }
    return key;
}

template<unsigned DIM>
std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > Part<DIM>::GetSegmentIndices()
{
    // Make sure the vertex indexes are up-to-date.
    GetVertices();

    std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > indexes;
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = mFacets[0]->GetPolygons();
    std::map<unsigned, std::string> attribute_keys = this->GetAttributesKeysForMesh(true);

    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        bool use_attributes = false;
        std::vector<std::map<std::string, double> > attributes = polygons[idx]->GetEdgeAttributes();
        if((attributes.size() == polygons[idx]->GetVertices().size())and attribute_keys.size()>0)
        {
            use_attributes = true;
        }

        if (polygons[idx]->GetVertices().size() == 2)
        {
            unsigned key = 2;
            if(use_attributes)
            {
                key = GetKeyForAttributes(attributes[0]);
            }
            std::pair<unsigned, unsigned> vertex_indices(polygons[idx]->GetVertices()[0]->GetIndex(),
                    polygons[idx]->GetVertices()[1]->GetIndex());

            indexes.push_back(
                    std::pair<std::pair<unsigned, unsigned>, unsigned > (vertex_indices, key));
        }
        else if (polygons[idx]->GetVertices().size() > 1)
        {
            std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = polygons[idx]->GetVertices();
            for (unsigned jdx = 0; jdx < vertices.size() - 1; jdx++)
            {
                unsigned key = 2;
                if(use_attributes)
                {
                    key = GetKeyForAttributes(attributes[jdx]);
                }
                std::pair<unsigned, unsigned> vertex_indices(vertices[jdx]->GetIndex(), vertices[jdx + 1]->GetIndex());
                indexes.push_back(
                        std::pair<std::pair<unsigned, unsigned>, unsigned >(vertex_indices, key));
            }
            unsigned key = 2;
            if(use_attributes)
            {
                key = GetKeyForAttributes(attributes[vertices.size() - 1]);
            }
            std::pair<unsigned, unsigned> vertex_indices(vertices[vertices.size() - 1]->GetIndex(), vertices[0]->GetIndex());
            indexes.push_back(
                    std::pair<std::pair<unsigned, unsigned>, unsigned >(vertex_indices, key));
        }
    }
    return indexes;
}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> Part<DIM>::GetVtk(bool includeEdges)
{
    if(mVtkIsUpToDate)
    {
        return mVtkPart;
    }

    // Get unique edge and polygon labels
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    std::set<std::string> polygon_labels;
    std::set<std::string> edge_labels;
    std::set<std::string> vertex_labels;

    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        std::map<std::string, double> attribute_map = polygons[idx]->GetAttributes();
        for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
        {
            polygon_labels.insert(it->first);
        }

        if(includeEdges)
        {
            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->GetEdgeAttributes();
            for(unsigned jdx=0;jdx<edge_atts_per_polygon.size();jdx++)
            {
                for(std::map<std::string, double>::iterator it = edge_atts_per_polygon[jdx].begin();
                        it != edge_atts_per_polygon[jdx].end(); ++it)
                {
                    edge_labels.insert(it->first);
                }
            }
        }
    }
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    for(unsigned idx=0;idx<vertices.size(); idx++)
    {
        std::map<std::string, double> attribute_map = vertices[idx]->GetAttributes();
        for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
        {
            vertex_labels.insert(it->first);
        }
    }

    std::vector<vtkSmartPointer<vtkDoubleArray> > vtk_cell_data;
    std::vector<vtkSmartPointer<vtkDoubleArray> > vtk_vertex_data;

    std::set<std::string>::iterator it;
    for (it = polygon_labels.begin(); it != polygon_labels.end(); it++)
    {
        vtkSmartPointer<vtkDoubleArray> p_facet_info = vtkSmartPointer<vtkDoubleArray>::New();
        p_facet_info->SetNumberOfComponents(1);
        p_facet_info->SetName((*it).c_str());
        vtk_cell_data.push_back(p_facet_info);
    }
    for (it = edge_labels.begin(); it != edge_labels.end(); it++)
    {
        vtkSmartPointer<vtkDoubleArray> p_facet_info = vtkSmartPointer<vtkDoubleArray>::New();
        p_facet_info->SetNumberOfComponents(1);
        p_facet_info->SetName((*it).c_str());
        vtk_cell_data.push_back(p_facet_info);
    }
    for (it = vertex_labels.begin(); it != vertex_labels.end(); it++)
    {
        vtkSmartPointer<vtkDoubleArray> p_facet_info = vtkSmartPointer<vtkDoubleArray>::New();
        p_facet_info->SetNumberOfComponents(1);
        p_facet_info->SetName((*it).c_str());
        vtk_vertex_data.push_back(p_facet_info);
    }

    // Set up the vertices
    vtkSmartPointer<vtkPolyData> p_part_data = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();
    p_part_data->Allocate(1, 1);

    for(unsigned idx=0;idx<vertices.size();idx++)
    {
        vertices[idx]->SetIndex(idx);
        c_vector<double, DIM> vertex_location = vertices[idx]->GetLocation(mReferenceLength);
        if(DIM==3)
        {
            p_vertices->InsertNextPoint(vertex_location[0], vertex_location[1], vertex_location[2]);
        }
        else
        {
            p_vertices->InsertNextPoint(vertex_location[0], vertex_location[1], 0.0);
        }

        for(unsigned jdx=0;jdx<vtk_vertex_data.size();jdx++)
        {
            std::map<std::string, double> attribute_map = vertices[idx]->GetAttributes();
            bool found_attribute = false;
            for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
            {
                if(it->first == vtk_vertex_data[jdx]->GetName())
                {
                    vtk_vertex_data[jdx]->InsertNextTuple1(it->second);
                    found_attribute = true;
                    break;
                }
            }
            if(!found_attribute)
            {
                vtk_vertex_data[jdx]->InsertNextTuple1(0.0);
            }
        }
    }

    // Insertion order is important here, lines first then polys
    if(includeEdges)
    {
        for (vtkIdType idx = 0; idx < vtkIdType(polygons.size()); idx++)
        {
            std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > local_vertices = polygons[idx]->GetVertices();
            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->GetEdgeAttributes();
            for (vtkIdType jdx = 0; jdx < vtkIdType(local_vertices.size()); jdx++)
            {
                if(jdx>0)
                {
                    vtkSmartPointer<vtkLine> p_line = vtkSmartPointer<vtkLine>::New();
                    p_line->GetPointIds()->SetNumberOfIds(2);
                    p_line->GetPointIds()->SetId(0, local_vertices[jdx-1]->GetIndex());
                    p_line->GetPointIds()->SetId(1, local_vertices[jdx]->GetIndex());
                    for(unsigned kdx=0;kdx<vtk_cell_data.size();kdx++)
                    {
                        bool found_attribute = false;
                        for(std::map<std::string, double>::iterator it = edge_atts_per_polygon[jdx-1].begin();
                                it != edge_atts_per_polygon[jdx-1].end(); ++it)
                        {
                            if(it->first == vtk_cell_data[kdx]->GetName())
                            {
                                vtk_cell_data[kdx]->InsertNextTuple1(it->second);
                                found_attribute = true;
                                break;
                            }
                        }
                        if(!found_attribute)
                        {
                            vtk_cell_data[kdx]->InsertNextTuple1(0.0);
                        }
                    }
                    p_part_data->InsertNextCell(p_line->GetCellType(), p_line->GetPointIds());
                }
                if(jdx==vtkIdType(local_vertices.size())-1)
                {
                    vtkSmartPointer<vtkLine> p_line = vtkSmartPointer<vtkLine>::New();
                    p_line->GetPointIds()->SetNumberOfIds(2);
                    p_line->GetPointIds()->SetId(0, local_vertices[jdx]->GetIndex());
                    p_line->GetPointIds()->SetId(1, local_vertices[0]->GetIndex());
                    for(unsigned kdx=0;kdx<vtk_cell_data.size();kdx++)
                    {
                        bool found_attribute = false;
                        for(std::map<std::string, double>::iterator it = edge_atts_per_polygon[jdx].begin();
                                it != edge_atts_per_polygon[jdx].end(); ++it)
                        {
                            if(it->first == vtk_cell_data[kdx]->GetName())
                            {
                                vtk_cell_data[kdx]->InsertNextTuple1(it->second);
                                found_attribute = true;
                                break;
                            }
                        }
                        if(!found_attribute)
                        {
                            vtk_cell_data[kdx]->InsertNextTuple1(0.0);
                        }
                    }
                    p_part_data->InsertNextCell(p_line->GetCellType(), p_line->GetPointIds());
                }
            }
        }
    }

    // Add polygons
    for (vtkIdType idx = 0; idx < vtkIdType(polygons.size()); idx++)
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > local_vertices = polygons[idx]->GetVertices();
        vtkSmartPointer<vtkPolygon> p_polygon = vtkSmartPointer<vtkPolygon>::New();
        p_polygon->GetPointIds()->SetNumberOfIds(local_vertices.size());
        for (vtkIdType jdx = 0; jdx < vtkIdType(local_vertices.size()); jdx++)
        {
            p_polygon->GetPointIds()->SetId(jdx, local_vertices[jdx]->GetIndex());
        }
        p_part_data->InsertNextCell(p_polygon->GetCellType(), p_polygon->GetPointIds());

        for(unsigned jdx=0;jdx<vtk_cell_data.size();jdx++)
        {
            bool found_attribute = false;
            std::map<std::string, double> attribute_map = polygons[idx]->GetAttributes();
            for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
            {
                if(it->first == vtk_cell_data[jdx]->GetName())
                {
                    vtk_cell_data[jdx]->InsertNextTuple1(it->second);
                    found_attribute = true;
                    break;
                }
            }
            if(!found_attribute)
            {
                vtk_cell_data[jdx]->InsertNextTuple1(0.0);
            }
        }
    }

    p_part_data->SetPoints(p_vertices);
    for(unsigned idx=0;idx<vtk_cell_data.size();idx++)
    {
        if(vtk_cell_data[idx]->GetNumberOfTuples()>0)
        {
            p_part_data->GetCellData()->AddArray(vtk_cell_data[idx]);
        }
    }
    for(unsigned idx=0;idx<vtk_vertex_data.size();idx++)
    {
        if(vtk_vertex_data[idx]->GetNumberOfTuples()>0)
        {
            p_part_data->GetPointData()->AddArray(vtk_vertex_data[idx]);
        }
    }

    vtkSmartPointer<vtkCleanPolyData> p_clean_data = vtkSmartPointer<vtkCleanPolyData>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_clean_data->SetInput(p_part_data);
    #else
        p_clean_data->SetInputData(p_part_data);
    #endif

    if(!includeEdges)
    {
//        vtkSmartPointer<vtkPolyDataNormals> p_normals = vtkSmartPointer<vtkPolyDataNormals>::New();
//        p_normals->SetInputConnection(p_clean_data->GetOutputPort());
//        p_normals->ComputePointNormalsOn();
//        p_normals->ComputeCellNormalsOn();
//        p_normals->Update();
//        mVtkPart = p_normals->GetOutput();
          p_clean_data->Update();
          mVtkPart = p_clean_data->GetOutput();
    }
    else
    {
        p_clean_data->Update();
        mVtkPart = p_clean_data->GetOutput();
    }

    mpVtkCellLocator = vtkSmartPointer<vtkCellLocator>::New();
    mpVtkCellLocator->SetDataSet(mVtkPart);
    mpVtkCellLocator->BuildLocator();
    mVtkIsUpToDate = true;
    return mVtkPart;
}

template<unsigned DIM>
bool Part<DIM>::IsPointInPart(DimensionalChastePoint<DIM> location)
{
    vtkSmartPointer<vtkPolyData> p_part = GetVtk();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();

    if(DIM==3)
    {
        p_points->InsertNextPoint(location.GetLocation(mReferenceLength)[0], location.GetLocation(mReferenceLength)[1], location.GetLocation(mReferenceLength)[2]);
    }
    else
    {
        p_points->InsertNextPoint(location.GetLocation(mReferenceLength)[0], location.GetLocation(mReferenceLength)[1], 0.0);
    }

    vtkSmartPointer<vtkPolyData> p_point_data = vtkSmartPointer<vtkPolyData>::New();
    p_point_data->SetPoints(p_points);

    //Points inside test
    vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    #if VTK_MAJOR_VERSION <= 5
        selectEnclosedPoints->SetInput(p_point_data);
    #else
        selectEnclosedPoints->SetInputData(p_point_data);
    #endif
    #if VTK_MAJOR_VERSION <= 5
        selectEnclosedPoints->SetSurface(p_part);
    #else
        selectEnclosedPoints->SetSurfaceData(p_part);
    #endif
    selectEnclosedPoints->Update();

    return selectEnclosedPoints->IsInside(0);
}

template<unsigned DIM>
std::vector<bool> Part<DIM>::IsPointInPart(vtkSmartPointer<vtkPoints> pPoints)
{
    vtkSmartPointer<vtkPolyData> p_part = GetVtk();
    vtkSmartPointer<vtkPolyData> p_point_data = vtkSmartPointer<vtkPolyData>::New();
    p_point_data->SetPoints(pPoints);

    //Points inside test
    vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    #if VTK_MAJOR_VERSION <= 5
        selectEnclosedPoints->SetInput(p_point_data);
    #else
        selectEnclosedPoints->SetInputData(p_point_data);
    #endif
    #if VTK_MAJOR_VERSION <= 5
        selectEnclosedPoints->SetSurface(p_part);
    #else
        selectEnclosedPoints->SetSurfaceData(p_part);
    #endif
    selectEnclosedPoints->Update();

    std::vector<bool> is_inside(pPoints->GetNumberOfPoints());
    for(unsigned idx=0; idx<pPoints->GetNumberOfPoints(); idx++)
    {
        is_inside[idx] = selectEnclosedPoints->IsInside(idx);
    }
    return is_inside;
}

template<unsigned DIM>
void Part<DIM>::MergeCoincidentVertices()
{
    // Loop through the nodes of each polygon. If it is in another polygon, replace it.
    std::vector<std::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        for(unsigned jdx=0; jdx<polygons.size(); jdx++)
        {
            if(idx != jdx)
            {
                std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > p1_verts = polygons[idx]->GetVertices();
                std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > p2_verts = polygons[jdx]->GetVertices();
                for(unsigned mdx=0; mdx<p1_verts.size(); mdx++)
                {
                    for(unsigned ndx=0; ndx<p2_verts.size(); ndx++)
                    {
                        if(GetDistance(*(p2_verts[ndx]), *(p1_verts[mdx]))< 1.e-6*p2_verts[ndx]->GetReferenceLengthScale())
                        {
                            polygons[jdx]->ReplaceVertex(ndx, polygons[idx]->GetVertex(mdx));
                        }
                    }
                }
            }
        }
    }

    for(unsigned idx=0; idx<mFacets.size(); idx++)
    {
        mFacets[idx]->UpdateVertices();
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    {
        for (unsigned idx = 0; idx < vertices.size(); idx++)
        {
            vertices[idx]->RotateAboutAxis(axis, angle);
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::SetReferenceLengthScale(units::quantity<unit::length> referenceLength)
{
    mReferenceLength = referenceLength;
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Translate(DimensionalChastePoint<DIM> vector)
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    {
        for (unsigned idx = 0; idx < vertices.size(); idx++)
        {
            vertices[idx]->Translate(vector);
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Write(const std::string& fileName, GeometryFormat::Value format, bool includeEdges)
{
    GeometryWriter writer;
    writer.SetFileName(fileName);
    writer.AddInput(GetVtk(includeEdges));
    writer.SetOutputFormat(format);
    writer.Write();
}

// Explicit instantiation
template class Part<2>;
template class Part<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(Part, 2)
EXPORT_TEMPLATE_CLASS1(Part, 3)
