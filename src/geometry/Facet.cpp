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

#include <set>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkIdTypeArray.h>
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkPoints.h>
#include <vtkPlane.h>
#include "BaseUnits.hpp"
#include "Exception.hpp"
#include "Facet.hpp"

template<unsigned DIM>
Facet<DIM>::Facet() :
        mPolygons(),
        mVertices(),
        mVerticesUpToDate(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned DIM>
Facet<DIM>::Facet(std::vector<PolygonPtr<DIM> > polygons) :
        mPolygons(polygons),
        mVertices(),
        mVerticesUpToDate(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned DIM>
Facet<DIM>::Facet(PolygonPtr<DIM> pPolygon) :
        mPolygons(),
        mVertices(),
        mVerticesUpToDate(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
    mPolygons.push_back(pPolygon);
}

template<unsigned DIM>
std::shared_ptr<Facet<DIM> > Facet<DIM>::Create(std::vector<PolygonPtr<DIM> > polygons)
{
    return std::make_shared<Facet<DIM> >(polygons);
}

template<unsigned DIM>
std::shared_ptr<Facet<DIM> > Facet<DIM>::Create(PolygonPtr<DIM> pPolygon)
{
    return std::make_shared<Facet<DIM> >(pPolygon);
}

template<unsigned DIM>
Facet<DIM>::~Facet()
{
}

template<unsigned DIM>
void Facet<DIM>::AddPolygons(std::vector<PolygonPtr<DIM> > polygons)
{
    mPolygons.insert(mPolygons.end(), polygons.begin(), polygons.end());
    mVerticesUpToDate = false;
}

template<unsigned DIM>
void Facet<DIM>::AddPolygon(PolygonPtr<DIM> pPolygon)
{
    mPolygons.push_back(pPolygon);
    mVerticesUpToDate = false;
}

template<unsigned DIM>
bool Facet<DIM>::ContainsPoint(const Vertex<DIM>& rLocation)
{
    bool contains_point = false;
    for(auto& polygon:mPolygons)
    {
        if (polygon->ContainsPoint(rLocation))
        {
            contains_point = true;
            break;
        }
    }
    return contains_point;
}

template<unsigned DIM>
std::array<QLength, 6> Facet<DIM>::GetBoundingBox()
{
    std::array<QLength, 6> box_vector;

    unsigned counter = 0;
    for(auto& vertex:rGetVertices())
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
Vertex<DIM> Facet<DIM>::GetCentroid()
{
    double centroid[3];
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    vtkPolygon::ComputeCentroid(vertex_data.second, vertex_data.first, centroid);
    return Vertex<DIM>(centroid, mReferenceLength);
}

template<unsigned DIM>
QLength Facet<DIM>::GetDistance(const Vertex<DIM>& rLocation)
{
    double location_array[3];
    rLocation.Convert(location_array, mReferenceLength);
    double distance = GetPlane()->DistanceToPlane(&location_array[0]);
    return distance*mReferenceLength;
}

template<unsigned DIM>
c_vector<double, 3> Facet<DIM>::GetNormal()
{
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    double loc1[3];
    double loc2[3];
    double loc3[3];
    vertex_data.first->GetPoint(0, loc1);
    vertex_data.first->GetPoint(1, loc2);
    vertex_data.first->GetPoint(2, loc3);
    c_vector<double, 3> normal;
    vtkTriangle::ComputeNormal(loc1, loc2, loc3, &normal[0]);
    return normal;
}

template<unsigned DIM>
vtkSmartPointer<vtkPlane> Facet<DIM>::GetPlane()
{
    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
    c_vector<double, 3> centroid = GetCentroid().Convert3(mReferenceLength);
    c_vector<double, 3> normal = GetNormal();
    p_plane->SetOrigin(&centroid[0]);
    p_plane->SetNormal(&normal[0]);
    return p_plane;
}

template<unsigned DIM>
std::vector<PolygonPtr<DIM> > Facet<DIM>::GetPolygons()
{
    return mPolygons;
}

template<unsigned DIM>
const std::vector<VertexPtr<DIM> >& Facet<DIM>::rGetVertices()
{
    if (!mVerticesUpToDate)
    {
        UpdateVertices();
    }
    return mVertices;
}

template<unsigned DIM>
std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > Facet<DIM>::GetVtkVertices()
{
    vtkSmartPointer<vtkIdTypeArray> p_vertexIds = vtkSmartPointer<vtkIdTypeArray>::New();
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();
    std::vector<VertexPtr<DIM> > vertices = rGetVertices();
    p_vertices->SetNumberOfPoints(vertices.size());
    for (vtkIdType idx = 0; idx < vtkIdType(vertices.size()); idx++)
    {
        c_vector<double, 3> vertex_location = vertices[idx]->Convert3(mReferenceLength);
        p_vertices->SetPoint(idx, &vertex_location[0]);
        p_vertexIds->InsertNextValue(idx);
    }
    return std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(p_vertices, p_vertexIds);
}

template<unsigned DIM>
void Facet<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    if (!mVerticesUpToDate)
    {
        UpdateVertices();
    }
    for(auto& vertex:mVertices)
    {
        vertex->RotateAboutAxis(axis, angle);
    }
}

template<unsigned DIM>
void Facet<DIM>::Translate(const Vertex<DIM>& rTranslationVector)
{
    if (!mVerticesUpToDate)
    {
        UpdateVertices();
    }
    for(auto& vertex:mVertices)
    {
        vertex->Translate(rTranslationVector);
    }
}

template<unsigned DIM>
void Facet<DIM>::UpdateVertices()
{
    std::set<VertexPtr<DIM> > unique_vertices;
    for (unsigned idx = 0; idx < mPolygons.size(); idx++)
    {
        std::vector<VertexPtr<DIM> > polygon_vertices = mPolygons[idx]->rGetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }
    mVertices = std::vector<VertexPtr<DIM> >();
    mVertices.insert(mVertices.end(), unique_vertices.begin(), unique_vertices.end());
    for (unsigned idx = 0; idx < mVertices.size(); idx++)
    {
        mVertices[idx]->SetIndex(idx);
    }
    mVerticesUpToDate = true;
}

// Explicit instantiation
template class Facet<2>;
template class Facet<3>;

#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(Facet, 2)
EXPORT_TEMPLATE_CLASS1(Facet, 3)
