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
Facet<DIM>::Facet(std::vector<std::shared_ptr<Polygon<DIM> > > polygons) :
        mPolygons(polygons),
        mVertices(),
        mVerticesUpToDate(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned DIM>
Facet<DIM>::Facet(std::shared_ptr<Polygon<DIM> > pPolygon) :
        mPolygons(),
        mVertices(),
        mVerticesUpToDate(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{
    mPolygons.push_back(pPolygon);
}

template<unsigned DIM>
std::shared_ptr<Facet<DIM> > Facet<DIM>::Create(std::vector<std::shared_ptr<Polygon<DIM> > > polygons)
{
    return std::make_shared<Facet<DIM> >(polygons);
}

template<unsigned DIM>
std::shared_ptr<Facet<DIM> > Facet<DIM>::Create(std::shared_ptr<Polygon<DIM> > pPolygon)
{
    return std::make_shared<Facet<DIM> >(pPolygon);
}

template<unsigned DIM>
Facet<DIM>::~Facet()
{
}

template<unsigned DIM>
void Facet<DIM>::AddPolygons(std::vector<std::shared_ptr<Polygon<DIM> > > polygons)
{
    mPolygons.insert(mPolygons.end(), polygons.begin(), polygons.end());
    mVerticesUpToDate = false;
}

template<unsigned DIM>
void Facet<DIM>::AddPolygon(std::shared_ptr<Polygon<DIM> > pPolygon)
{
    mPolygons.push_back(pPolygon);
    mVerticesUpToDate = false;
}

template<unsigned DIM>
bool Facet<DIM>::ContainsPoint(const DimensionalChastePoint<DIM>& location)
{
    bool contains_point = false;
    for (unsigned idx = 0; idx < mPolygons.size(); idx++)
    {
        if (mPolygons[idx]->ContainsPoint(location))
        {
            contains_point = true;
            break;
        }
    }
    return contains_point;
}

template<unsigned DIM>
std::vector<QLength > Facet<DIM>::GetBoundingBox()
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

    std::vector<QLength > box_vector(6);
    for(unsigned idx=0; idx<6; idx++)
    {
        box_vector[idx] = box[idx] * mReferenceLength;
    }

    return box_vector;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> Facet<DIM>::GetCentroid()
{
    double centroid[3];
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    vtkPolygon::ComputeCentroid(vertex_data.second, vertex_data.first, centroid);
    c_vector<double, DIM> return_centroid;
    for (unsigned idx = 0; idx < DIM; idx++)
    {
        return_centroid[idx] = centroid[idx];
    }
    if(DIM==3)
    {
        return DimensionalChastePoint<DIM>(return_centroid, mReferenceLength);
    }
    else
    {
        return DimensionalChastePoint<DIM>(return_centroid[0], return_centroid[1], 0.0, mReferenceLength);
    }
}

template<unsigned DIM>
QLength Facet<DIM>::GetDistance(const DimensionalChastePoint<DIM>& rLocation)
{
    double location_array[3];
    for(unsigned idx=0; idx<DIM;idx++)
    {
        location_array[idx] = rLocation.GetLocation(mReferenceLength)[idx];
    }
    if(DIM==2)
    {
        location_array[2] = 0.0;
    }

    vtkSmartPointer<vtkPlane> p_plane = GetPlane();
    double distance = p_plane->DistanceToPlane(&location_array[0]);
    return distance*mReferenceLength;
}

template<unsigned DIM>
c_vector<double, DIM> Facet<DIM>::GetNormal()
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    if (vertices.size() < 3)
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
    c_vector<double, 3> normal;
    vtkTriangle::ComputeNormal(loc1, loc2, loc3, &normal[0]);
    if(DIM==3)
    {
        return normal;
    }
    else
    {
        c_vector<double, 2> normal_2d;
        normal_2d[0] = normal[0];
        normal_2d[1] = normal[1];
        return normal_2d;
    }
}

template<unsigned DIM>
vtkSmartPointer<vtkPlane> Facet<DIM>::GetPlane()
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
std::vector<std::shared_ptr<Polygon<DIM> > > Facet<DIM>::GetPolygons()
{
    return mPolygons;
}

template<unsigned DIM>
std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > Facet<DIM>::GetVertices()
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
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();

    p_vertices->SetNumberOfPoints(vertices.size());
    for (vtkIdType idx = 0; idx < vtkIdType(vertices.size()); idx++)
    {
        c_vector<double, DIM> vertex_location = vertices[idx]->GetLocation(mReferenceLength);
        if(DIM==3)
        {
            p_vertices->SetPoint(idx, vertex_location[0], vertex_location[1], vertex_location[2]);
        }
        else
        {
            p_vertices->SetPoint(idx, vertex_location[0], vertex_location[1], 0.0);
        }
        p_vertexIds->InsertNextValue(idx);
    }
    return std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(p_vertices, p_vertexIds);
}

template<unsigned DIM>
void Facet<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    for(unsigned idx=0; idx<vertices.size(); idx++)
    {
        vertices[idx]->RotateAboutAxis(axis, angle);
    }
}

template<unsigned DIM>
void Facet<DIM>::Translate(DimensionalChastePoint<DIM> translationVector)
{
    std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > vertices = GetVertices();
    for(unsigned idx=0; idx<vertices.size(); idx++)
    {
        vertices[idx]->Translate(translationVector);
    }
}

template<unsigned DIM>
void Facet<DIM>::UpdateVertices()
{
    std::set<std::shared_ptr<DimensionalChastePoint<DIM> > > unique_vertices;
    for (unsigned idx = 0; idx < mPolygons.size(); idx++)
    {
        std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > > polygon_vertices = mPolygons[idx]->GetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }
    mVertices = std::vector<std::shared_ptr<DimensionalChastePoint<DIM> > >();
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
