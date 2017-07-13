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
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellLocator.h>
#include <vtkCleanPolyData.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkVersion.h>
#include <vtkLine.h>
#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolygon.h>
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "Part.hpp"
#include "GeometryTools.hpp"
#include "BaseUnits.hpp"
#include "Exception.hpp"

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
PartPtr<DIM> Part<DIM>::Create()
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
void Part<DIM>::AddAttributeToEdgeIfFound(const Vertex<DIM>& rLoc, const std::string& rLabel, double value)
{
    for(auto& polygon:GetPolygons())
    {
        polygon->AddAttributeToEdgeIfFound(rLoc, rLabel, value);
    }
}

template<unsigned DIM>
void Part<DIM>::AddAttributeToPolygonIfFound(const Vertex<DIM>& rLoc, const std::string& rLabel, double value)
{
    for(auto& polygon:GetPolygons())
    {
        if(polygon->ContainsPoint(rLoc))
        {
            polygon->AddAttribute(rLabel, value);
        }
    }
}

template<unsigned DIM>
void Part<DIM>::AddAttributeToPolygons(const std::string& rLabel, double value)
{
    for(auto& polygon:GetPolygons())
    {
        polygon->AddAttribute(rLabel, value);
    }
}

template<unsigned DIM>
PolygonPtr<DIM> Part<DIM>::AddCircle(QLength radius, Vertex<DIM> centre, unsigned numSegments)
{
    std::vector<VertexPtr<DIM> > vertices;
    double seg_angle = 2.0 * M_PI / double(numSegments);
    for (unsigned idx = 0; idx < numSegments; idx++)
    {
        double angle = seg_angle * double(idx);
        QLength x = (radius * std::cos(angle) + centre[0]);
        QLength y = (radius * std::sin(angle) + centre[1]);
        if(DIM==3)
        {
            vertices.push_back(Vertex<DIM>::Create(x, y, centre[2]));
        }
        else
        {
            vertices.push_back(Vertex<DIM>::Create(x, y, 0_m));
        }
    }
    mVtkIsUpToDate = false;
    return AddPolygon(vertices);
}

template<unsigned DIM>
void Part<DIM>::AddCylinder(QLength radius, QLength depth, Vertex<DIM> centre, unsigned numSegments)
{
    PolygonPtr<DIM> p_circle = AddCircle(radius, centre, numSegments);
    Extrude(p_circle, depth);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddCuboid(QLength sizeX, QLength sizeY, QLength sizeZ, Vertex<DIM> origin)
{
    PolygonPtr<DIM> p_rectangle = AddRectangle(sizeX, sizeY, origin);
    Extrude(p_rectangle, sizeZ);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddHoleMarker(Vertex<DIM> hole)
{
    mHoleMarkers.push_back(hole);
}

template<unsigned DIM>
void Part<DIM>::AddRegionMarker(Vertex<DIM> region, unsigned value)
{
    mRegionMarkers.push_back(std::pair<Vertex<DIM>, unsigned>(region, value));
}

template<unsigned DIM>
void Part<DIM>::AppendPart(PartPtr<DIM> pPart)
{
    for(auto& polygon:pPart->GetPolygons())
    {
        this->AddPolygon(polygon, true);
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
PolygonPtr<DIM> Part<DIM>::AddPolygon(const std::vector<VertexPtr<DIM> >& vertices, bool newFacet, FacetPtr<DIM> pFacet)
{
    auto p_polygon = Polygon<DIM>::Create(vertices);
    AddPolygon(p_polygon, newFacet, pFacet);
    mVtkIsUpToDate = false;
    return p_polygon;
}

template<unsigned DIM>
PolygonPtr<DIM> Part<DIM>::AddPolygon(PolygonPtr<DIM> pPolygon, bool newFacet, FacetPtr<DIM> pFacet)
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
PolygonPtr<DIM> Part<DIM>::AddRectangle(QLength sizeX, QLength sizeY, Vertex<DIM> origin)
{
    std::vector<VertexPtr<DIM> > vertices;
    if(DIM==3)
    {
        vertices.push_back(Vertex<DIM>::Create(origin[0], origin[1], origin[2]));
        vertices.push_back(Vertex<DIM>::Create(origin[0] + sizeX, origin[1], origin[2]));
        vertices.push_back(Vertex<DIM>::Create(origin[0] + sizeX, origin[1] + sizeY, origin[2]));
        vertices.push_back(Vertex<DIM>::Create(origin[0], origin[1] + sizeY, origin[2]));
    }
    else
    {
        vertices.push_back(Vertex<DIM>::Create(origin[0], origin[1], 0.0_m));
        vertices.push_back(Vertex<DIM>::Create(origin[0] + sizeX, origin[1], 0.0_m));
        vertices.push_back(Vertex<DIM>::Create(origin[0] + sizeX, origin[1] + sizeY, 0.0_m));
        vertices.push_back(Vertex<DIM>::Create(origin[0], origin[1] + sizeY, 0.0_m));
    }
    mVtkIsUpToDate = false;
    return AddPolygon(vertices);
}

template<unsigned DIM>
void Part<DIM>::AddVesselNetwork(VesselNetworkPtr<DIM> pVesselNetwork, bool surface, bool removeVesselRegion)
{
    if (!surface)
    {
        std::vector<VertexPtr<DIM> > vertices;
        std::vector<VesselNodePtr<DIM> > nodes = pVesselNetwork->GetNodes();
        for (unsigned idx = 0; idx < nodes.size(); idx++)
        {
            vertices.push_back(Vertex<DIM>::Create(nodes[idx]->rGetLocation()));
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
        std::vector<VesselPtr<DIM> > vessels = pVesselNetwork->GetVessels();
        for (unsigned idx = 0; idx < vessels.size(); idx++)
        {
            std::vector<VesselSegmentPtr<DIM> > segments = vessels[idx]->GetSegments();
            std::vector<VertexPtr<DIM> > segment_vertices;
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
            mFacets.push_back(Facet<DIM>::Create(Polygon<DIM>::Create(segment_vertices)));
        }
    }
    else
    {
        // Add any polygons on existing facets to the facet
        VesselSurfaceGenerator<DIM> generator(pVesselNetwork);
        std::vector<PolygonPtr<DIM> > polygons = generator.GetSurfacePolygons();
        std::vector<bool> polygon_on_facet;

        for (unsigned idx = 0; idx < polygons.size(); idx++)
        {
            polygons[idx]->AddAttribute("Vessel Wall", 1.0);
            bool on_facet = false;
            Vertex<DIM> poly_centroid = polygons[idx]->GetCentroid();
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
            std::vector<Vertex<DIM> > hole_locations = generator.GetHoles();
            for(unsigned idx=0; idx<hole_locations.size(); idx++)
            {
                AddHoleMarker(hole_locations[idx]);
            }
        }
        else
        {
            std::vector<Vertex<DIM> > region_locations = generator.GetHoles();
            for(unsigned idx=0; idx<region_locations.size(); idx++)
            {
                AddRegionMarker(region_locations[idx], 2.0);
            }
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
bool Part<DIM>::EdgeHasAttribute(const Vertex<DIM>& rLoc, const std::string& rLabel)
{
    bool edge_has_label = false;
    std::vector<PolygonPtr<DIM> > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        if(polygons[idx]->EdgeHasAttribute(rLoc, rLabel))
        {
            return true;
        }
    }
    return edge_has_label;
}

template<unsigned DIM>
void Part<DIM>::Extrude(PolygonPtr<DIM> pPolygon, QLength depth)
{
    if(DIM==2)
    {
        EXCEPTION("Only parts in 3D space can be extruded.");
    }
    // Loop through the vertices and create new ones at the offset depth
    std::vector<VertexPtr<DIM> > original_vertices = pPolygon->rGetVertices();
    std::vector<VertexPtr<DIM> > new_vertices;
    for (const auto& vertex:original_vertices)
    {
        if(DIM==2)
        {
            new_vertices.push_back(Vertex<DIM>::Create((*vertex)[0], (*vertex)[1], depth));
        }
        else
        {
            new_vertices.push_back(Vertex<DIM>::Create((*vertex)[0], (*vertex)[1], (*vertex)[2] + depth));
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
        PolygonPtr<DIM> p_polygon = Polygon<DIM>::Create(original_vertices[idx]);
        p_polygon->AddVertex(original_vertices[index2]);
        p_polygon->AddVertex(new_vertices[index2]);
        p_polygon->AddVertex(new_vertices[idx]);
        mFacets.push_back(Facet<DIM>::Create(p_polygon));
    }

    // Close the lid
    PolygonPtr<DIM> p_polygon = Polygon<DIM>::Create(new_vertices);
    mFacets.push_back(Facet<DIM>::Create(p_polygon));
    mVtkIsUpToDate = false;
}

template <unsigned DIM>
void Part<DIM>::BooleanWithNetwork(VesselNetworkPtr<DIM> pVesselNetwork)
{
    // Remove any vessel with both nodes outside the domain
    for(auto& vessel:pVesselNetwork->GetVessels())
    {
        if(!IsPointInPart(vessel->GetStartNode()->rGetLocation()) &&
                !IsPointInPart(vessel->GetEndNode()->rGetLocation()))
        {
            pVesselNetwork->RemoveVessel(vessel, true);
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
std::vector<Vertex<DIM> > Part<DIM>::GetHoleMarkers()
{
    return mHoleMarkers;
}

template<unsigned DIM>
std::vector<std::pair<Vertex<DIM>, unsigned> > Part<DIM>::GetRegionMarkers()
{
    return mRegionMarkers;
}

template<unsigned DIM>
QLength Part<DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned DIM>
std::map<unsigned, std::string> Part<DIM>::GetAttributesKeysForMesh(bool update)
{
    if(update)
    {
        // Collect all attribute labels
        std::vector<PolygonPtr<DIM> > polygons = GetPolygons();
        std::set<std::string> attribute_labels;

        for(unsigned idx=0; idx<polygons.size(); idx++)
        {
            std::map<std::string, double> attribute_map = polygons[idx]->rGetAttributes();
            for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
            {
                attribute_labels.insert(it->first);
            }

            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->rGetEdgeAttributes();
            for(unsigned jdx=0;jdx<edge_atts_per_polygon.size();jdx++)
            {
                for(std::map<std::string, double>::iterator it = edge_atts_per_polygon[jdx].begin();
                        it != edge_atts_per_polygon[jdx].end(); ++it)
                {
                    attribute_labels.insert(it->first);
                }
            }
        }
        std::vector<VertexPtr<DIM> > vertices = GetVertices();
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
FacetPtr<DIM> Part<DIM>::GetFacet(const Vertex<DIM>& location)
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
std::vector<VertexPtr<DIM> > Part<DIM>::GetVertices()
{
    std::vector<PolygonPtr<DIM> > polygons = GetPolygons();
    std::set<VertexPtr<DIM> > unique_vertices;
    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<VertexPtr<DIM> > polygon_vertices = polygons[idx]->rGetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }

    std::vector<VertexPtr<DIM> > vertices;
    vertices.insert(vertices.end(), unique_vertices.begin(), unique_vertices.end());
    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        vertices[idx]->SetIndex(idx);
    }
    return vertices;
}

template<unsigned DIM>
std::vector<unsigned> Part<DIM>::GetContainingGridIndices(unsigned num_x, unsigned num_y, unsigned num_z, QLength spacing)
{
    std::vector<unsigned> location_indices;
    for(unsigned kdx=0; kdx<num_z; kdx++)
    {
        for(unsigned jdx=0; jdx<num_y; jdx++)
        {
            for(unsigned idx=0; idx<num_x; idx++)
            {
                Vertex<DIM> location(double(idx) * spacing, double(jdx) * spacing, double(kdx) * spacing);
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
std::vector<PolygonPtr<DIM> > Part<DIM>::GetPolygons()
{
    std::vector<PolygonPtr<DIM> > polygons;
    for (unsigned idx = 0; idx < mFacets.size(); idx++)
    {
        std::vector<PolygonPtr<DIM> > facet_polygons = mFacets[idx]->GetPolygons();
        polygons.insert(polygons.end(), facet_polygons.begin(), facet_polygons.end());
    }
    return polygons;
}

template<unsigned DIM>
std::array<QLength, 6> Part<DIM>::GetBoundingBox()
{
    std::vector<VertexPtr<DIM> > vertices = GetVertices();
    std::array<QLength, 6> box_vector;
    unsigned counter = 0;
    for(const auto& vertex:vertices)
    {
        for (unsigned jdx = 0; jdx < DIM; jdx++)
        {
            if (counter == 0)
            {
                box_vector[2 * jdx] = (*vertex)[jdx];
                box_vector[2 * jdx + 1] = (*vertex)[jdx];
            }
            else
            {
                if ((*vertex)[jdx] < box_vector[2 * jdx])
                {
                    box_vector[2 * jdx] = (*vertex)[jdx];
                }
                if ((*vertex)[jdx] > box_vector[2 * jdx + 1])
                {
                    box_vector[2 * jdx + 1] = (*vertex)[jdx];
                }
            }
        }
        counter++;
    }
    return box_vector;
}

template<unsigned DIM>
std::vector<FacetPtr<DIM> > Part<DIM>::GetFacets()
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
    std::vector<PolygonPtr<DIM> > polygons = mFacets[0]->GetPolygons();

    std::map<unsigned, std::string> attribute_keys = this->GetAttributesKeysForMesh(true);

    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<VertexPtr<DIM> > poly_verts = polygons[idx]->rGetVertices();

        bool use_attributes = false;
        std::vector<std::map<std::string, double> > attributes = polygons[idx]->rGetEdgeAttributes();
        if((attributes.size() == poly_verts.size())and attribute_keys.size()>0)
        {
            use_attributes = true;
        }

        if (polygons[idx]->rGetVertices().size() == 2)
        {
            unsigned key = 2;
            if(use_attributes)
            {
                key = GetKeyForAttributes(attributes[0]);
            }
            std::pair<unsigned, unsigned> vertex_indices(poly_verts[0]->GetIndex(),
                    poly_verts[1]->GetIndex());

            indexes.push_back(
                    std::pair<std::pair<unsigned, unsigned>, unsigned > (vertex_indices, key));
        }
        else if (poly_verts.size() > 2)
        {
            for (unsigned jdx = 0; jdx < poly_verts.size() - 1; jdx++)
            {
                unsigned key = 2;
                if(use_attributes)
                {
                    key = GetKeyForAttributes(attributes[jdx]);
                }
                std::pair<unsigned, unsigned> vertex_indices(poly_verts[jdx]->GetIndex(), poly_verts[jdx + 1]->GetIndex());
                indexes.push_back(
                        std::pair<std::pair<unsigned, unsigned>, unsigned >(vertex_indices, key));
            }
            unsigned key = 2;
            if(use_attributes)
            {
                key = GetKeyForAttributes(attributes[poly_verts.size() - 1]);
            }
            std::pair<unsigned, unsigned> vertex_indices(poly_verts[poly_verts.size() - 1]->GetIndex(), poly_verts[0]->GetIndex());
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
    std::vector<PolygonPtr<DIM> > polygons = GetPolygons();
    std::set<std::string> polygon_labels;
    std::set<std::string> edge_labels;
    std::set<std::string> vertex_labels;

    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        std::map<std::string, double> attribute_map = polygons[idx]->rGetAttributes();
        for(std::map<std::string, double>::iterator it = attribute_map.begin(); it != attribute_map.end(); ++it)
        {
            polygon_labels.insert(it->first);
        }

        if(includeEdges)
        {
            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->rGetEdgeAttributes();
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
    std::vector<VertexPtr<DIM> > vertices = GetVertices();
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

    unsigned counter=0;
    for(auto& vertex:vertices)
    {
        vertex->SetIndex(counter);
        double loc[3];
        vertex->Convert(loc, mReferenceLength);
        p_vertices->InsertNextPoint(&loc[0]);

        for(unsigned jdx=0;jdx<vtk_vertex_data.size();jdx++)
        {
            std::map<std::string, double> attribute_map = vertex->GetAttributes();
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
        counter++;
    }

    // Insertion order is important here, lines first then polys
    if(includeEdges)
    {
        for (vtkIdType idx = 0; idx < vtkIdType(polygons.size()); idx++)
        {
            std::vector<VertexPtr<DIM> > local_vertices = polygons[idx]->rGetVertices();
            std::vector<std::map<std::string, double> > edge_atts_per_polygon = polygons[idx]->rGetEdgeAttributes();
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
        std::vector<VertexPtr<DIM> > local_vertices = polygons[idx]->rGetVertices();
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
            std::map<std::string, double> attribute_map = polygons[idx]->rGetAttributes();
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
bool Part<DIM>::IsPointInPart(const Vertex<DIM>& rLoc)
{
    vtkSmartPointer<vtkPolyData> p_part = GetVtk();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    double loc[3];
    rLoc.Convert(loc, mReferenceLength);
    p_points->InsertNextPoint(&loc[0]);

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
    std::vector<PolygonPtr<DIM> > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        for(unsigned jdx=0; jdx<polygons.size(); jdx++)
        {
            if(idx != jdx)
            {
                std::vector<VertexPtr<DIM> > p1_verts = polygons[idx]->rGetVertices();
                std::vector<VertexPtr<DIM> > p2_verts = polygons[jdx]->rGetVertices();
                for(unsigned mdx=0; mdx<p1_verts.size(); mdx++)
                {
                    for(unsigned ndx=0; ndx<p2_verts.size(); ndx++)
                    {
                        if(GetDistance((p2_verts[ndx])->rGetLocation(), (p1_verts[mdx])->rGetLocation())< 1.e-6*mReferenceLength)
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
    std::vector<VertexPtr<DIM> > vertices = GetVertices();
    {
        for (unsigned idx = 0; idx < vertices.size(); idx++)
        {
            vertices[idx]->RotateAboutAxis(axis, angle);
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::SetReferenceLengthScale(QLength referenceLength)
{
    mReferenceLength = referenceLength;
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Translate(const Vertex<DIM>& vector)
{
    std::vector<VertexPtr<DIM> > vertices = GetVertices();
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
