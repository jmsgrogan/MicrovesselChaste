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
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include "vtkCleanPolyData.h"
#include "vtkSelectEnclosedPoints.h"
#include <vtkVersion.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "Part.hpp"
#include "GeometryTools.hpp"

template<unsigned DIM>
Part<DIM>::Part() :
        mFacets(),
        mVtkPart(vtkSmartPointer<vtkPolyData>()),
        mHoleMarkers(),
        mRegionMarkers(),
        mReferenceLength(1.e-6 * unit::metres),
        mVtkIsUpToDate(false)
{
}

template<unsigned DIM>
boost::shared_ptr<Part<DIM> > Part<DIM>::Create()
{
    MAKE_PTR(Part<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
Part<DIM>::~Part()
{

}

template<unsigned DIM>
boost::shared_ptr<Polygon<DIM> > Part<DIM>::AddCircle(units::quantity<unit::length> radius,
                                                DimensionalChastePoint<DIM> centre, unsigned numSegments)
{
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    double seg_angle = 2.0 * M_PI / double(numSegments);
    for (unsigned idx = 0; idx < numSegments; idx++)
    {
        double angle = seg_angle * double(idx);
        double x = (radius * std::cos(angle) + centre[0]*centre.GetReferenceLengthScale())/mReferenceLength;
        double y = (radius * std::sin(angle) + centre[1]*centre.GetReferenceLengthScale())/mReferenceLength;
        if(DIM==3)
        {
            vertices.push_back(DimensionalChastePoint<DIM>::Create(x, y, centre[2], mReferenceLength));
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
    boost::shared_ptr<Polygon<DIM> > p_circle = AddCircle(radius, centre, numSegments);
    Extrude(p_circle, depth);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddCuboid(units::quantity<unit::length> sizeX,
                          units::quantity<unit::length> sizeY,
                          units::quantity<unit::length> sizeZ,
                          DimensionalChastePoint<DIM> origin)
{
    boost::shared_ptr<Polygon<DIM> > p_rectangle = AddRectangle(sizeX, sizeY, origin);
    Extrude(p_rectangle, sizeZ);
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::AddHoleMarker(DimensionalChastePoint<DIM> hole)
{
    mHoleMarkers.push_back(hole);
}

template<unsigned DIM>
boost::shared_ptr<Polygon<DIM> > Part<DIM>::AddPolygon(std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices, bool newFacet,
                                                                                   boost::shared_ptr<Facet<DIM> > pFacet)
{
    boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(vertices);
    AddPolygon(p_polygon, newFacet, pFacet);
    mVtkIsUpToDate = false;
    return p_polygon;
}

template<unsigned DIM>
boost::shared_ptr<Polygon<DIM> > Part<DIM>::AddPolygon(boost::shared_ptr<Polygon<DIM> > pPolygon, bool newFacet,
                                                 boost::shared_ptr<Facet<DIM> > pFacet)
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
boost::shared_ptr<Polygon<DIM> > Part<DIM>::AddRectangle(units::quantity<unit::length> sizeX,
                                                   units::quantity<unit::length> sizeY,
                                                   DimensionalChastePoint<DIM> origin)
{
    origin.SetReferenceLengthScale(mReferenceLength);

    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    if(DIM==3)
    {
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0], origin[1], origin[2], mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0] + sizeX/mReferenceLength, origin[1], origin[2],mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0] + sizeX/mReferenceLength, origin[1] + sizeY/mReferenceLength, origin[2], mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0], origin[1] + sizeY/mReferenceLength, origin[2],mReferenceLength));
    }
    else
    {
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0], origin[1], 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0] + sizeX/mReferenceLength, origin[1], 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0] + sizeX/mReferenceLength, origin[1] + sizeY/mReferenceLength, 0.0, mReferenceLength));
        vertices.push_back(DimensionalChastePoint<DIM>::Create(origin[0], origin[1] + sizeY/mReferenceLength, 0.0, mReferenceLength));
    }
    mVtkIsUpToDate = false;
    return AddPolygon(vertices);
}

template<unsigned DIM>
void Part<DIM>::AddVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork, bool surface)
{
    if (!surface)
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pVesselNetwork->GetNodes();
        for (unsigned idx = 0; idx < nodes.size(); idx++)
        {
            units::quantity<unit::length> length_Scale = nodes[idx]->rGetLocation().GetReferenceLengthScale();
            vertices.push_back(DimensionalChastePoint<DIM>::Create(nodes[idx]->rGetLocation().rGetLocation(length_Scale), length_Scale));
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
        std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
        for (unsigned idx = 0; idx < vessels.size(); idx++)
        {
            std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = vessels[idx]->GetSegments();
            std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > segment_vertices;
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
            boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(segment_vertices);
            mFacets.push_back(Facet<DIM>::Create(p_polygon));
        }
    }
    else
    {
        // Add any polygons on existing facets to the facet
        VesselSurfaceGenerator<DIM> generator(pVesselNetwork);
        std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = generator.GetSurfacePolygons();
        std::vector<bool> polygon_on_facet;

        for (unsigned idx = 0; idx < polygons.size(); idx++)
        {
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

        std::vector<DimensionalChastePoint<DIM> > hole_locations = generator.GetHoles();
        for(unsigned idx=0; idx<hole_locations.size(); idx++)
        {
            AddHoleMarker(hole_locations[idx]);
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Extrude(boost::shared_ptr<Polygon<DIM> > pPolygon, units::quantity<unit::length> depth)
{
    if(DIM==2)
    {
        EXCEPTION("Only parts in 3D space can be extruded.");
    }
    // Loop through the vertices and create new ones at the offset depth
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > original_vertices = pPolygon->GetVertices();
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > new_vertices;
    for (unsigned idx = 0; idx < original_vertices.size(); idx++)
    {
        if(DIM==2)
        {
            new_vertices.push_back(DimensionalChastePoint<DIM>::Create((*original_vertices[idx])[0], (*original_vertices[idx])[1],
                                                                       depth/original_vertices[idx]->GetReferenceLengthScale(),
                                                  original_vertices[idx]->GetReferenceLengthScale()));
        }
        else
        {
            new_vertices.push_back(DimensionalChastePoint<DIM>::Create((*original_vertices[idx])[0], (*original_vertices[idx])[1],
                                                                       (*original_vertices[idx])[2] + depth/original_vertices[idx]->GetReferenceLengthScale(),
                                                  original_vertices[idx]->GetReferenceLengthScale()));
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
        boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(original_vertices[idx]);
        p_polygon->AddVertex(original_vertices[index2]);
        p_polygon->AddVertex(new_vertices[index2]);
        p_polygon->AddVertex(new_vertices[idx]);
        mFacets.push_back(Facet<DIM>::Create(p_polygon));
    }

    // Close the lid
    boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(new_vertices);
    mFacets.push_back(Facet<DIM>::Create(p_polygon));
    mVtkIsUpToDate = false;
}

template <unsigned DIM>
void Part<DIM>::BooleanWithNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    // Remove any vessel with both nodes outside the domain
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
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
std::vector<DimensionalChastePoint<DIM> > Part<DIM>::GetHoleMarkers()
{
    return mHoleMarkers;
}

template<unsigned DIM>
units::quantity<unit::length> Part<DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned DIM>
boost::shared_ptr<Facet<DIM> > Part<DIM>::GetFacet(const DimensionalChastePoint<DIM>& location)
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
std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > Part<DIM>::GetVertices()
{
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    std::set<boost::shared_ptr<DimensionalChastePoint<DIM> > > unique_vertices;
    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > polygon_vertices = polygons[idx]->GetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }

    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices;
    vertices.insert(vertices.end(), unique_vertices.begin(), unique_vertices.end());

    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        vertices[idx]->SetIndex(idx);
    }
    return vertices;
}

template<unsigned DIM>
std::vector<unsigned> Part<DIM>::GetContainingGridIndices(unsigned num_x, unsigned num_y, unsigned num_z, double spacing)
{
    std::vector<unsigned> location_indices;
    for(unsigned kdx=0; kdx<num_z; kdx++)
    {
        for(unsigned jdx=0; jdx<num_y; jdx++)
        {
            for(unsigned idx=0; idx<num_x; idx++)
            {
                DimensionalChastePoint<DIM> location(double(idx) * spacing, double(jdx) * spacing, double(kdx) * spacing);
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
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        vertex_locs.push_back(*vertices[idx]);
    }
    return vertex_locs;
}

template<unsigned DIM>
std::vector<boost::shared_ptr<Polygon<DIM> > > Part<DIM>::GetPolygons()
{
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons;
    for (unsigned idx = 0; idx < mFacets.size(); idx++)
    {
        std::vector<boost::shared_ptr<Polygon<DIM> > > facet_polygons = mFacets[idx]->GetPolygons();
        polygons.insert(polygons.end(), facet_polygons.begin(), facet_polygons.end());
    }
    return polygons;
}

template<unsigned DIM>
c_vector<double, 2*DIM> Part<DIM>::GetBoundingBox()
{
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    c_vector<double, 2*DIM> box;

    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        for (unsigned jdx = 0; jdx < DIM; jdx++)
        {
            if (idx == 0)
            {
                box[2 * jdx] = (*vertices[idx])[jdx];
                box[2 * jdx + 1] = (*vertices[idx])[jdx];
            }
            else
            {
                if ((*vertices[idx])[jdx] < box[2 * jdx])
                {
                    box[2 * jdx] = (*vertices[idx])[jdx];
                }
                if ((*vertices[idx])[jdx] > box[2 * jdx + 1])
                {
                    box[2 * jdx + 1] = (*vertices[idx])[jdx];
                }
            }
        }
    }
    return box;
}

template<unsigned DIM>
std::vector<boost::shared_ptr<Facet<DIM> > > Part<DIM>::GetFacets()
{
    return mFacets;
}

template<unsigned DIM>
std::vector<std::pair<unsigned, unsigned> > Part<DIM>::GetSegmentIndices()
{
    // Make sure the vertex indexes are up-to-date.
    GetVertices();

    std::vector<std::pair<unsigned, unsigned> > indexes;
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = mFacets[0]->GetPolygons();

    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        if (polygons[idx]->GetVertices().size() == 2)
        {
            indexes.push_back(
                    std::pair<unsigned, unsigned>(polygons[idx]->GetVertices()[0]->GetIndex(),
                                                  polygons[idx]->GetVertices()[1]->GetIndex()));
        }
        else if (polygons[idx]->GetVertices().size() > 1)
        {
            std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = polygons[idx]->GetVertices();
            for (unsigned jdx = 0; jdx < vertices.size() - 1; jdx++)
            {
                indexes.push_back(
                        std::pair<unsigned, unsigned>(vertices[jdx]->GetIndex(), vertices[jdx + 1]->GetIndex()));
            }
            indexes.push_back(
                    std::pair<unsigned, unsigned>(vertices[vertices.size() - 1]->GetIndex(), vertices[0]->GetIndex()));

        }
    }
    return indexes;
}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> Part<DIM>::GetVtk()
{
    if(mVtkIsUpToDate)
    {
        return mVtkPart;
    }

    vtkSmartPointer<vtkPolyData> p_part_data = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();

    p_part_data->Allocate(1, 1);
    // Loop through each polygon, collect the vertices and set correct point ids

    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    unsigned vert_counter = 0;
    for (vtkIdType idx = 0; idx < vtkIdType(polygons.size()); idx++)
    {
        std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = polygons[idx]->GetVertices();
        vtkSmartPointer<vtkPolygon> p_polygon = vtkSmartPointer<vtkPolygon>::New();
        p_polygon->GetPointIds()->SetNumberOfIds(vertices.size());
        for (vtkIdType jdx = 0; jdx < vtkIdType(vertices.size()); jdx++)
        {
            if(DIM==3)
            {
                p_vertices->InsertNextPoint((*vertices[jdx])[0], (*vertices[jdx])[1], (*vertices[jdx])[2]);
            }
            else
            {
                p_vertices->InsertNextPoint((*vertices[jdx])[0], (*vertices[jdx])[1], 0.0);
            }

            p_polygon->GetPointIds()->SetId(jdx, vert_counter);
            vert_counter++;
        }
        p_part_data->InsertNextCell(p_polygon->GetCellType(), p_polygon->GetPointIds());
    }
    p_part_data->SetPoints(p_vertices);

    vtkSmartPointer<vtkCleanPolyData> p_clean_data = vtkSmartPointer<vtkCleanPolyData>::New();
    p_clean_data->SetInputData(p_part_data);
    p_clean_data->Update();

    mVtkPart = p_clean_data->GetOutput();
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
        p_points->InsertNextPoint(location[0], location[1], location[2]);
    }
    else
    {
        p_points->InsertNextPoint(location[0], location[1], 0.0);
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
std::vector<bool> Part<DIM>::IsPointInPart(const std::vector<DimensionalChastePoint<DIM> >& rLocations)
{
    vtkSmartPointer<vtkPolyData> p_part = GetVtk();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();

    for(unsigned idx=0; idx<rLocations.size(); idx++)
    {
        if(DIM==3)
        {
            p_points->InsertNextPoint(rLocations[idx][0], rLocations[idx][1], rLocations[idx][2]);
        }
        else
        {
            p_points->InsertNextPoint(rLocations[idx][0], rLocations[idx][1], 0.0);
        }
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

    std::vector<bool> is_inside(rLocations.size());
    for(unsigned idx=0; idx<rLocations.size(); idx++)
    {
        is_inside[idx] = selectEnclosedPoints->IsInside(idx);
    }
    return is_inside;
}

template<unsigned DIM>
void Part<DIM>::MergeCoincidentVertices()
{
    // Loop through the nodes of each polygon. If it is in another polygon, replace it.
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = GetPolygons();
    for(unsigned idx=0; idx<polygons.size(); idx++)
    {
        for(unsigned jdx=0; jdx<polygons.size(); jdx++)
        {
            if(idx != jdx)
            {
                std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > p1_verts = polygons[idx]->GetVertices();
                std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > p2_verts = polygons[jdx]->GetVertices();
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
void Part<DIM>::SetReferenceLengthScale(units::quantity<unit::length> referenceLength)
{
    mReferenceLength = referenceLength;
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Translate(c_vector<double, DIM> vector)
{
    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    {
        for (unsigned idx = 0; idx < vertices.size(); idx++)
        {
            vertices[idx]->Translate(vector);
        }
    }
    mVtkIsUpToDate = false;
}

template<unsigned DIM>
void Part<DIM>::Write(const std::string& fileName, GeometryFormat::Value format)
{
    GeometryWriter writer;
    writer.SetFileName(fileName);
    writer.SetInput(GetVtk());
    writer.SetOutputFormat(format);
    writer.Write();
}

// Explicit instantiation
template class Part<2> ;
template class Part<3> ;
