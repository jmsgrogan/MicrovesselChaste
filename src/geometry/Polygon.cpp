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
Polygon<DIM>::Polygon(std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices) :
        mVertices(vertices),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mEdgeAttributes(),
        mAttributes(),
        mVtkRepresentationUpToDate(false),
        mpVtkRepresentation()
{

}

template<unsigned DIM>
Polygon<DIM>::Polygon(std::shared_ptr<DimensionalChastePoint<DIM> > pVertex) :
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
std::shared_ptr<Polygon<DIM> > Polygon<DIM>::Create(std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices)
{
    return std::make_shared<Polygon<DIM> >(vertices);
}

template<unsigned DIM>
std::shared_ptr<Polygon<DIM> > Polygon<DIM>::Create(std::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
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
bool Polygon<DIM>::AddAttributeToEdgeIfFound(DimensionalChastePoint<DIM> loc,
        const std::string& rLabel, double value)
{
    // Cycle through the edges, check if it contains point, label if found
    vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
    unsigned num_edges = p_polygon->GetNumberOfEdges();
    bool edge_found = false;
    for(unsigned idx=0; idx<num_edges;idx++)
    {
        vtkSmartPointer<vtkPoints> p_line = p_polygon->GetEdge(idx)->GetPoints();
        c_vector<double, 3> start_point;
        p_line->GetPoint(0, &start_point[0]);
        c_vector<double, 3> end_point;
        p_line->GetPoint(1, &end_point[0]);
        DimensionalChastePoint<DIM> start_loc = DimensionalChastePoint<DIM>(start_point[0], start_point[1],
                start_point[2], mReferenceLength);
        DimensionalChastePoint<DIM> end_loc = DimensionalChastePoint<DIM>(end_point[0], end_point[1],
                end_point[2], mReferenceLength);
        if(GetDistanceToLineSegment(start_loc, end_loc, loc)/mReferenceLength<1.e-3)
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
    for(unsigned idx=0;idx<mEdgeAttributes.size();idx++)
    {
        mEdgeAttributes[idx][rLabel] = value;
    }
}

template<unsigned DIM>
void Polygon<DIM>::AddVertices(std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices)
{
    // Add the new vertices
    mVertices.insert(mVertices.end(), vertices.begin(), vertices.end());
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void Polygon<DIM>::AddVertex(std::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
{
    mVertices.push_back(pVertex);
    mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
std::vector<std::map<std::string, double> > Polygon<DIM>::GetEdgeAttributes()
{
    GetVtkPolygon();
    return mEdgeAttributes;
}

template<unsigned DIM>
std::map<std::string, double> Polygon<DIM>::GetAttributes()
{
    return mAttributes;
}

template<unsigned DIM>
std::shared_ptr<DimensionalChastePoint<DIM> > Polygon<DIM>::GetVertex(unsigned idx)
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
std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > Polygon<DIM>::GetVertices()
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
        c_vector<double, DIM> vertex_location = mVertices[idx]->GetLocation(mReferenceLength);
        if(DIM==3)
        {
            p_vertices->SetPoint(idx, vertex_location[0], vertex_location[1], vertex_location[2]);
        }
        else
        {
            p_vertices->SetPoint(idx, vertex_location[0], vertex_location[1], 0.0);
        }
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
bool Polygon<DIM>::EdgeHasAttribute(DimensionalChastePoint<DIM> loc, const std::string& rLabel)
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
            c_vector<double, 3> start_point;
            p_line->GetPoint(0, &start_point[0]);
            c_vector<double, 3> end_point;
            p_line->GetPoint(1, &end_point[0]);
            DimensionalChastePoint<DIM> start_loc = DimensionalChastePoint<DIM>(start_point[0], start_point[1],
                    start_point[2], mReferenceLength);
            DimensionalChastePoint<DIM> end_loc = DimensionalChastePoint<DIM>(end_point[0], end_point[1],
                    end_point[2], mReferenceLength);
            if(GetDistanceToLineSegment(start_loc, end_loc, loc)/mReferenceLength<1.e-3)
            {
                return true;
            }
        }
    }
    return edge_found;
}

template<unsigned DIM>
bool Polygon<DIM>::HasAttribute(const std::string& rLabel)
{
    return(mAttributes.count(rLabel)>0);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> Polygon<DIM>::GetCentroid()
{
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > verts = GetVtkVertices();
    c_vector<double, 3> centroid;
    vtkPolygon::ComputeCentroid(verts.second, verts.first, &centroid[0]);
    if(DIM==3)
    {
        return DimensionalChastePoint<DIM>(centroid, mReferenceLength);
    }
    else
    {
        return DimensionalChastePoint<DIM>(centroid[0], centroid[1], 0.0, mReferenceLength);
    }
}

template<unsigned DIM>
std::vector<units::quantity<unit::length> > Polygon<DIM>::GetBoundingBox()
{
    c_vector<double, 6> box;
    GetVtkPolygon()->GetPoints()->GetBounds(&box[0]);

    std::vector<units::quantity<unit::length> > box_vector(6);
    for(unsigned idx=0; idx<6; idx++)
    {
        box_vector[idx] = box[idx] * mReferenceLength;
    }
    return box_vector;
}

template<unsigned DIM>
units::quantity<unit::length> Polygon<DIM>::GetDistance(const DimensionalChastePoint<DIM>& location)
{
    double point[3];
    for (unsigned idx = 0; idx < DIM; idx++)
    {
        point[idx] = location.GetLocation(mReferenceLength)[idx];
    }
    if(DIM==2)
    {
        point[2] = 0.0;
    }

    vtkSmartPointer<vtkPlane> p_plane = GetPlane();
    double distance = p_plane->DistanceToPlane(&point[0]);
    return distance * mReferenceLength;
}

template<unsigned DIM>
units::quantity<unit::length> Polygon<DIM>::GetDistanceToEdges(const DimensionalChastePoint<DIM>& location)
{
    double point[3];
    for (unsigned idx = 0; idx < DIM; idx++)
    {
        point[idx] = location.GetLocation(mReferenceLength)[idx];
    }
    if(DIM==2)
    {
        point[2] = 0.0;
    }

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
    c_vector<double, DIM> centroid = GetCentroid().GetLocation(mReferenceLength);
    c_vector<double, DIM> normal = GetNormal();
    if(DIM==3)
    {
        p_plane->SetOrigin(centroid[0], centroid[1], centroid[2]);
        p_plane->SetNormal(normal[0], normal[1], normal[2]);
    }
    else
    {
        p_plane->SetOrigin(centroid[0], centroid[1], 0.0);
        p_plane->SetNormal(normal[0], normal[1], 0.0);
    }
    return p_plane;
}

template<unsigned DIM>
c_vector<double, DIM> Polygon<DIM>::GetNormal()
{
    if (mVertices.size() < 3)
    {
        EXCEPTION("At least 3 vertices are required to generate a normal.");
    }

    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    double loc1[3];
    double loc2[3];
    double loc3[3];
    vertex_data.first->GetPoint(0, loc1);
    vertex_data.first->GetPoint(1, loc2);
    vertex_data.first->GetPoint(2, loc3);
    c_vector<double, 3> in_normal;
    vtkTriangle::ComputeNormal(loc1, loc2, loc3, &in_normal[0]);
    if(DIM==3)
    {
        return in_normal;
    }
    else
    {
        c_vector<double, 2> normal;
        normal[0] = in_normal[0];
        normal[1] = in_normal[1];
        return normal;
    }
}

template<unsigned DIM>
bool Polygon<DIM>::ContainsPoint(const DimensionalChastePoint<DIM>& location, double localTolerance)
{
    bool contains_point = false;
    if (mVertices.size() >= 3)
    {
        c_vector<double, DIM> vertex_location = location.GetLocation(mReferenceLength);
        double point[3];
        for (unsigned idx = 0; idx < DIM; idx++)
        {
            point[idx] = vertex_location[idx];
        }
        if(DIM==2)
        {
            point[2] = 0.0;
        }

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
                point, p_polygon->GetPoints()->GetNumberOfPoints(),
                static_cast<double*>(p_polygon->GetPoints()->GetData()->GetVoidPointer(0)), bounds, n);
        if (contains == 1)
        {
            contains_point = true;
        }
    }
    return contains_point;
}

template<unsigned DIM>
void Polygon<DIM>::ReplaceVertex(unsigned idx, std::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
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
void Polygon<DIM>::Translate(DimensionalChastePoint<DIM> translationVector)
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

