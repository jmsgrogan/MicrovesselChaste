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

#include <set>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkIdTypeArray.h>
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include "Exception.hpp"

#include "Facet.hpp"

Facet::Facet(std::vector<boost::shared_ptr<Polygon> > polygons) :
        mPolygons(polygons),
        mVertices(),
        mVerticesUpToDate(false),
        mData(),
        mLabel()
{

}

Facet::Facet(boost::shared_ptr<Polygon> pPolygon) :
        mPolygons(),
        mVertices(),
        mVerticesUpToDate(false),
        mData()
{
    mPolygons.push_back(pPolygon);
}

boost::shared_ptr<Facet> Facet::Create(std::vector<boost::shared_ptr<Polygon> > polygons)
{
    MAKE_PTR_ARGS(Facet, pSelf, (polygons));
    return pSelf;
}

boost::shared_ptr<Facet> Facet::Create(boost::shared_ptr<Polygon> pPolygon)
{
    MAKE_PTR_ARGS(Facet, pSelf, (pPolygon));
    return pSelf;
}

Facet::~Facet()
{
}

void Facet::AddPolygons(std::vector<boost::shared_ptr<Polygon> > polygons)
{
    mPolygons.insert(mPolygons.end(), polygons.begin(), polygons.end());
    mVerticesUpToDate = false;
}

void Facet::AddPolygon(boost::shared_ptr<Polygon> pPolygon)
{
    mPolygons.push_back(pPolygon);
    mVerticesUpToDate = false;
}
bool Facet::ContainsPoint(const DimensionalChastePoint<3>& location)
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

c_vector<double, 6> Facet::GetBoundingBox()
{
    std::vector<boost::shared_ptr<Vertex> > vertices = GetVertices();
    c_vector<double, 6> box;

    for (unsigned idx = 0; idx < vertices.size(); idx++)
    {
        for (unsigned jdx = 0; jdx < 3; jdx++)
        {
            if (idx == 0)
            {
                box[2 * jdx] = vertices[idx]->rGetLocation()[jdx];
                box[2 * jdx + 1] = vertices[idx]->rGetLocation()[jdx];
            }
            else
            {
                if (vertices[idx]->rGetLocation()[jdx] < box[2 * jdx])
                {
                    box[2 * jdx] = vertices[idx]->rGetLocation()[jdx];
                }
                if (vertices[idx]->rGetLocation()[jdx] > box[2 * jdx + 1])
                {
                    box[2 * jdx + 1] = vertices[idx]->rGetLocation()[jdx];
                }
            }
        }
    }
    return box;
}

DimensionalChastePoint<3> Facet::GetCentroid()
{
    double centroid[3];
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    vtkPolygon::ComputeCentroid(vertex_data.second, vertex_data.first, centroid);
    c_vector<double, 3> return_centroid;
    for (unsigned idx = 0; idx < 3; idx++)
    {
        return_centroid[idx] = centroid[idx];
    }
    return DimensionalChastePoint<3>(return_centroid);
}

double Facet::GetData(const std::string& label)
{
    return mData[label];
}

std::string Facet::GetLabel()
{
    return mLabel;
}

double Facet::GetDistance(const DimensionalChastePoint<3>& location)
{
    double location_array[3];
    for(unsigned idx=0; idx<3;idx++)
    {
        location_array[idx] = location[idx];
    }

    vtkSmartPointer<vtkPlane> p_plane = GetPlane();
    double distance = p_plane->DistanceToPlane(&location_array[0]);
    return distance;
}

c_vector<double, 3> Facet::GetNormal()
{
    std::vector<boost::shared_ptr<Vertex> > vertices = GetVertices();
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
    return normal;
}

vtkSmartPointer<vtkPlane> Facet::GetPlane()
{
    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
    DimensionalChastePoint<3> centroid = GetCentroid();
    p_plane->SetOrigin(centroid[0], centroid[1], centroid[2]);

    c_vector<double, 3> normal = GetNormal();
    p_plane->SetNormal(normal[0], normal[1], normal[2]);
    return p_plane;
}

std::vector<boost::shared_ptr<Polygon> > Facet::GetPolygons()
{
    return mPolygons;
}

std::vector<boost::shared_ptr<Vertex> > Facet::GetVertices()
{
    if (!mVerticesUpToDate)
    {
        UpdateVertices();
    }
    return mVertices;
}

std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > Facet::GetVtkVertices()
{
    vtkSmartPointer<vtkIdTypeArray> p_vertexIds = vtkSmartPointer<vtkIdTypeArray>::New();
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();
    std::vector<boost::shared_ptr<Vertex> > vertices = GetVertices();

    p_vertices->SetNumberOfPoints(vertices.size());
    for (vtkIdType idx = 0; idx < vtkIdType(vertices.size()); idx++)
    {
        c_vector<double, 3> location = vertices[idx]->rGetLocation();
        p_vertices->SetPoint(idx, location[0], location[1], location[2]);
        p_vertexIds->InsertNextValue(idx);
    }
    return std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(p_vertices, p_vertexIds);
}

void Facet::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    std::vector<boost::shared_ptr<Vertex> > vertices = GetVertices();
    for(unsigned idx=0; idx<vertices.size(); idx++)
    {
        vertices[idx]->RotateAboutAxis(axis, angle);
    }
}

void Facet::SetData(const std::string& label, double value)
{
    mData[label] = value;
}

void Facet::SetLabel(const std::string& label)
{
    mLabel= label;
}

void Facet::Translate(c_vector<double, 3> translationVector)
{
    std::vector<boost::shared_ptr<Vertex> > vertices = GetVertices();
    for(unsigned idx=0; idx<vertices.size(); idx++)
    {
        vertices[idx]->Translate(translationVector);
    }
}

void Facet::UpdateVertices()
{
    std::set<boost::shared_ptr<Vertex> > unique_vertices;
    for (unsigned idx = 0; idx < mPolygons.size(); idx++)
    {
        std::vector<boost::shared_ptr<Vertex> > polygon_vertices = mPolygons[idx]->GetVertices();
        std::copy(polygon_vertices.begin(), polygon_vertices.end(),
                  std::inserter(unique_vertices, unique_vertices.end()));
    }
    mVertices = std::vector<boost::shared_ptr<Vertex> >();
    mVertices.insert(mVertices.end(), unique_vertices.begin(), unique_vertices.end());

    for (unsigned idx = 0; idx < mVertices.size(); idx++)
    {
        mVertices[idx]->SetIndex(idx);
    }
    mVerticesUpToDate = true;
}
