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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkIdTypeArray.h>
#include <vtkTriangle.h>
#include <vtkDoubleArray.h>
#include <vtkLine.h>
#include <vtkPolygon.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "Exception.hpp"
#include "GeometryTools.hpp"
#include "BaseUnits.hpp"

#include "Polygon.hpp"

template<unsigned DIM>
Polygon<DIM>::Polygon() :
        mVertices(),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mEdgeAttributes(),
        mAttributes(),
        mVtkRepresentationUpToDate(false),
        mpVtkRepresentation()
{

}

template<unsigned DIM>
Polygon<DIM>::Polygon(const std::vector<VertexPtr<DIM> >& vertices) :
        mVertices(vertices),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mEdgeAttributes(),
        mAttributes(),
        mVtkRepresentationUpToDate(false),
        mpVtkRepresentation()
{

}

template<unsigned DIM>
Polygon<DIM>::Polygon(VertexPtr<DIM> pVertex) :
        mVertices(),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mEdgeAttributes(),
        mAttributes(),
        mVtkRepresentationUpToDate(false),
        mpVtkRepresentation()
{
    mVertices.push_back(pVertex);
}

template<unsigned DIM>
PolygonPtr<DIM> Polygon<DIM>::Create(const std::vector<VertexPtr<DIM> >& vertices)
{
    return std::make_shared<Polygon<DIM> >(vertices);
}

template<unsigned DIM>
PolygonPtr<DIM> Polygon<DIM>::Create(VertexPtr<DIM> pVertex)
{
    return std::make_shared<Polygon<DIM> >(pVertex);
}

template<unsigned DIM>
Polygon<DIM>::~Polygon()
{
}

template<unsigned DIM>
void Polygon<DIM>::AddAttribute(const std::string& rLabel, double value)
{
    mAttributes[rLabel] = value;
}

template<unsigned DIM>
bool Polygon<DIM>::AddAttributeToEdgeIfFound(const Vertex<DIM>& rLoc, const std::string& rLabel, double value)
{
    // Cycle through the edges, check if it contains point, label if found
    vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
    unsigned num_edges = p_polygon->GetNumberOfEdges();
    bool edge_found = false;
    for(unsigned idx=0; idx<num_edges;idx++)
    {
        vtkSmartPointer<vtkPoints> p_line = p_polygon->GetEdge(idx)->GetPoints();
        double start_point[3];
        p_line->GetPoint(0, &start_point[0]);
        double end_point[3];
        p_line->GetPoint(1, &end_point[0]);
        Vertex<DIM> start_loc(start_point, mReferenceLength);
        Vertex<DIM> end_loc(end_point, mReferenceLength);
        if(GetDistanceToLineSegment(start_loc, end_loc, rLoc)/mReferenceLength<1.e-3)
        {
            mEdgeAttributes[idx][rLabel]=value;
            edge_found = true;
        }
    }
    return edge_found;
}

template<unsigned DIM>
void Polygon<DIM>::AddAttributeToAllEdges(const std::string& rLabel, double value)
{
    GetVtkPolygon();
    for(auto& attribute:mEdgeAttributes)
    {
        attribute[rLabel] = value;
    }
}

template<unsigned DIM>
void Polygon<DIM>::AddVertices(const std::vector<VertexPtr<DIM> >& vertices)
{
    // Add the new vertices
    mVertices.insert(mVertices.end(), vertices.begin(), vertices.end());
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void Polygon<DIM>::AddVertex(VertexPtr<DIM> pVertex)
{
    mVertices.push_back(pVertex);
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
const std::vector<std::map<std::string, double> >& Polygon<DIM>::rGetEdgeAttributes()
{
    GetVtkPolygon();
    return mEdgeAttributes;
}

template<unsigned DIM>
const std::map<std::string, double>& Polygon<DIM>::rGetAttributes()
{
    return mAttributes;
}

template<unsigned DIM>
VertexPtr<DIM> Polygon<DIM>::GetVertex(unsigned idx)
{
    if(idx >= mVertices.size())
    {
        EXCEPTION("Requested vertex index out of range");
    }
    else
    {
        return mVertices[idx];
    }
}

template<unsigned DIM>
const std::vector<VertexPtr<DIM> >& Polygon<DIM>::rGetVertices() const
{
    return mVertices;
}

template<unsigned DIM>
std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > Polygon<DIM>::GetVtkVertices()
{
    vtkSmartPointer<vtkIdTypeArray> p_vertexIds = vtkSmartPointer<vtkIdTypeArray>::New();
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();
    p_vertices->SetNumberOfPoints(mVertices.size());
    p_vertexIds->SetNumberOfTuples(mVertices.size());
    for (vtkIdType idx = 0; idx < vtkIdType(mVertices.size()); idx++)
    {
        double loc[3];
        mVertices[idx]->Convert(loc, mReferenceLength);
        p_vertices->SetPoint(idx, &loc[0]);
        p_vertexIds->SetValue(idx, idx);
    }
    return std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(p_vertices, p_vertexIds);
}

template<unsigned DIM>
vtkSmartPointer<vtkPolygon> Polygon<DIM>::GetVtkPolygon()
{
    if(!mVtkRepresentationUpToDate)
    {
        vtkSmartPointer<vtkPoints> p_vertices = GetVtkVertices().first;
        mpVtkRepresentation = vtkSmartPointer<vtkPolygon>::New();
        mpVtkRepresentation->GetPoints()->SetNumberOfPoints(p_vertices->GetNumberOfPoints());
        mpVtkRepresentation->GetPointIds()->SetNumberOfIds(p_vertices->GetNumberOfPoints());
        for (vtkIdType idx = 0; idx < p_vertices->GetNumberOfPoints(); idx++)
        {
            mpVtkRepresentation->GetPoints()->SetPoint(idx, p_vertices->GetPoint(idx));
            mpVtkRepresentation->GetPointIds()->SetId (idx, idx);
        }

        // Also set up edge labels
        mEdgeAttributes = std::vector<std::map<std::string, double> >(p_vertices->GetNumberOfPoints());
        mVtkRepresentationUpToDate = true;
    }
    return mpVtkRepresentation;
}

template<unsigned DIM>
bool Polygon<DIM>::EdgeHasAttribute(const Vertex<DIM>& rLoc, const std::string& rLabel)
{
    // Cycle through the edges, check if it contains point, label if found
    bool edge_found = false;
    vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
    unsigned num_edges = p_polygon->GetNumberOfEdges();
    for(unsigned idx=0; idx<num_edges;idx++)
    {
        if(mEdgeAttributes[idx].count(rLabel))
        {
            vtkSmartPointer<vtkPoints> p_line = p_polygon->GetEdge(idx)->GetPoints();
            double start_point[3];
            p_line->GetPoint(0, &start_point[0]);
            double end_point[3];
            p_line->GetPoint(1, &end_point[0]);
            Vertex<DIM> start_loc(start_point, mReferenceLength);
            Vertex<DIM> end_loc(end_point, mReferenceLength);
            if(GetDistanceToLineSegment(start_loc, end_loc, rLoc)/mReferenceLength<1.e-3)
            {
                return true;
            }
        }
    }
    return edge_found;
}

template<unsigned DIM>
bool Polygon<DIM>::HasAttribute(const std::string& rLabel) const
{
    return(mAttributes.count(rLabel)>0);
}

template<unsigned DIM>
Vertex<DIM> Polygon<DIM>::GetCentroid()
{
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > verts = GetVtkVertices();
    double centroid[3];
    vtkPolygon::ComputeCentroid(verts.second, verts.first, centroid);
    return Vertex<DIM>(centroid, mReferenceLength);
}

template<unsigned DIM>
std::array<QLength, 6> Polygon<DIM>::GetBoundingBox()
{
    c_vector<double, 6> box;
    GetVtkPolygon()->GetPoints()->GetBounds(&box[0]);
    std::array<QLength, 6> box_vector;
    for(unsigned idx=0; idx<6; idx++)
    {
        box_vector[idx] = box[idx] * mReferenceLength;
    }
    return box_vector;
}

template<unsigned DIM>
QLength Polygon<DIM>::GetDistance(const Vertex<DIM>& rLocation)
{
    double point[3];
    rLocation.Convert(point, mReferenceLength);
    double distance = GetPlane()->DistanceToPlane(&point[0]);
    return distance * mReferenceLength;
}

template<unsigned DIM>
QLength Polygon<DIM>::GetDistanceToEdges(const Vertex<DIM>& rLocation)
{
    double point[3];
    rLocation.Convert(point, mReferenceLength);

    vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
    double bounds[6];
    double closest[6];
    p_polygon->GetPoints()->GetBounds(bounds);
    double distance = p_polygon->DistanceToPolygon(
            point, p_polygon->GetPoints()->GetNumberOfPoints(),
            static_cast<double*>(p_polygon->GetPoints()->GetData()->GetVoidPointer(0)), bounds, closest);
    return distance * mReferenceLength;
}

template<unsigned DIM>
vtkSmartPointer<vtkPlane> Polygon<DIM>::GetPlane()
{
    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
    c_vector<double, 3> centroid = GetCentroid().Convert3(mReferenceLength);
    c_vector<double, 3> normal = GetNormal();
    p_plane->SetOrigin(&centroid[0]);
    p_plane->SetNormal(&normal[0]);
    return p_plane;
}

template<unsigned DIM>
c_vector<double, 3> Polygon<DIM>::GetNormal()
{
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    double loc1[3];
    double loc2[3];
    double loc3[3];
    vertex_data.first->GetPoint(0, loc1);
    vertex_data.first->GetPoint(1, loc2);
    vertex_data.first->GetPoint(2, loc3);
    c_vector<double, 3> in_normal;
    vtkTriangle::ComputeNormal(loc1, loc2, loc3, &in_normal[0]);
    return in_normal;
}

template<unsigned DIM>
bool Polygon<DIM>::ContainsPoint(const Vertex<DIM>& rLocation, double localTolerance)
{
    bool contains_point = false;
    if (mVertices.size() >= 3)
    {
        c_vector<double, 3> vertex_location = rLocation.Convert3(mReferenceLength);

        vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
        double n[3];
        p_polygon->ComputeNormal(p_polygon->GetPoints()->GetNumberOfPoints(),
                                 static_cast<double*>(p_polygon->GetPoints()->GetData()->GetVoidPointer(0)), n);

        double bounds[6];
        p_polygon->GetPoints()->GetBounds(bounds);

        if(localTolerance>0.0)
        {
            if(bounds[1]-bounds[0]<localTolerance)
            {
                bounds[0]-=localTolerance/2.0;
                bounds[1]+=localTolerance/2.0;
            }
            if(bounds[3]-bounds[2]<localTolerance)
            {
                bounds[2]-=localTolerance/2.0;
                bounds[3]+=localTolerance/2.0;
            }
            if(bounds[5]-bounds[4]<localTolerance)
            {
                bounds[4]-=localTolerance/2.0;
                bounds[5]+=localTolerance/2.0;
            }
        }

        int contains = p_polygon->PointInPolygon(
                &vertex_location[0], p_polygon->GetPoints()->GetNumberOfPoints(),
                static_cast<double*>(p_polygon->GetPoints()->GetData()->GetVoidPointer(0)), bounds, n);
        if (contains == 1)
        {
            contains_point = true;
        }
    }
    return contains_point;
}

template<unsigned DIM>
void Polygon<DIM>::ReplaceVertex(unsigned idx, VertexPtr<DIM> pVertex)
{
    if(idx >= mVertices.size())
    {
        EXCEPTION("Requested vertex index out of range");
    }
    else
    {
        mVertices[idx] = pVertex;
    }
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void Polygon<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    for(unsigned idx=0; idx<mVertices.size(); idx++)
    {
        mVertices[idx]->RotateAboutAxis(axis, angle);
    }
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void Polygon<DIM>::Translate(const Vertex<DIM>& translationVector)
{
    for(unsigned idx=0; idx<mVertices.size(); idx++)
    {
        mVertices[idx]->Translate(translationVector);
    }
    mVtkRepresentationUpToDate = false;
}

// Explicit instantiation
template class Polygon<2>;
template class Polygon<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(Polygon, 2)
EXPORT_TEMPLATE_CLASS1(Polygon, 3)

