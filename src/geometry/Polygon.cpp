/*

 Copyright (c) 2005-2015, University of Oxford.
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
#include "Exception.hpp"
#include "Polygon.hpp"

template<unsigned DIM>
Polygon<DIM>::Polygon(std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices) :
        mVertices(vertices)
{

}

template<unsigned DIM>
Polygon<DIM>::Polygon(boost::shared_ptr<DimensionalChastePoint<DIM> > pVertex) :
        mVertices()
{
    mVertices.push_back(pVertex);
}

template<unsigned DIM>
boost::shared_ptr<Polygon<DIM> > Polygon<DIM>::Create(std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices)
{
    MAKE_PTR_ARGS(Polygon<DIM> , pSelf, (vertices));
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<Polygon<DIM> > Polygon<DIM>::Create(boost::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
{
    MAKE_PTR_ARGS(Polygon<DIM> , pSelf, (pVertex));
    return pSelf;
}

template<unsigned DIM>
Polygon<DIM>::~Polygon()
{
}

template<unsigned DIM>
void Polygon<DIM>::AddVertices(std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices)
{
    // Add the new vertices
    mVertices.insert(mVertices.end(), vertices.begin(), vertices.end());
}

template<unsigned DIM>
void Polygon<DIM>::AddVertex(boost::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
{
    mVertices.push_back(pVertex);
}

template<unsigned DIM>
boost::shared_ptr<DimensionalChastePoint<DIM> > Polygon<DIM>::GetVertex(unsigned idx)
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
std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > Polygon<DIM>::GetVertices()
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
        if(DIM==3)
        {
            p_vertices->SetPoint(idx, (*mVertices[idx])[0], (*mVertices[idx])[1], (*mVertices[idx])[2]);
        }
        else
        {
            p_vertices->SetPoint(idx, (*mVertices[idx])[0], (*mVertices[idx])[1], 0.0);
        }
        p_vertexIds->SetValue(idx, idx);
    }
    return std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(p_vertices, p_vertexIds);
}

template<unsigned DIM>
vtkSmartPointer<vtkPolygon> Polygon<DIM>::GetVtkPolygon()
{
    vtkSmartPointer<vtkPoints> p_vertices = GetVtkVertices().first;
    vtkSmartPointer<vtkPolygon> p_polygon = vtkSmartPointer<vtkPolygon>::New();
    p_polygon->GetPoints()->SetNumberOfPoints(p_vertices->GetNumberOfPoints());
    for (vtkIdType idx = 0; idx < p_vertices->GetNumberOfPoints(); idx++)
    {
        p_polygon->GetPoints()->SetPoint(idx, p_vertices->GetPoint(idx));
    }
    return p_polygon;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> Polygon<DIM>::GetCentroid()
{
    c_vector<double, 3> centroid;
    std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> > vertex_data = GetVtkVertices();
    vtkPolygon::ComputeCentroid(vertex_data.second, vertex_data.first, &centroid[0]);

    if(DIM==3)
    {
        return DimensionalChastePoint<DIM>(centroid);
    }
    else
    {
        return DimensionalChastePoint<DIM>(centroid[0], centroid[1]);
    }

}

template<unsigned DIM>
c_vector<double, 6> Polygon<DIM>::GetBoundingBox()
{
    vtkSmartPointer<vtkPolygon> p_polygon = GetVtkPolygon();
    c_vector<double, 6> box;
    p_polygon->GetPoints()->GetBounds(&box[0]);
    return box;
}

template<unsigned DIM>
units::quantity<unit::length> Polygon<DIM>::GetDistance(const DimensionalChastePoint<DIM>& location)
{
    double point[3];
    for (unsigned idx = 0; idx < DIM; idx++)
    {
        point[idx] = location[idx];
    }
    if(DIM==2)
    {
        point[2] = 0.0;
    }

    vtkSmartPointer<vtkPlane> p_plane = GetPlane();
    double distance = p_plane->DistanceToPlane(&point[0]);
    return distance * location.GetReferenceLengthScale();
}

template<unsigned DIM>
units::quantity<unit::length> Polygon<DIM>::GetDistanceToEdges(const DimensionalChastePoint<DIM>& location)
{
    double point[3];
    for (unsigned idx = 0; idx < DIM; idx++)
    {
        point[idx] = location[idx];
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

    return distance * location.GetReferenceLengthScale();
}

template<unsigned DIM>
vtkSmartPointer<vtkPlane> Polygon<DIM>::GetPlane()
{
    vtkSmartPointer<vtkPlane> p_plane = vtkSmartPointer<vtkPlane>::New();
    DimensionalChastePoint<DIM> centroid = GetCentroid();
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
bool Polygon<DIM>::ContainsPoint(const DimensionalChastePoint<DIM>& location)
{
    bool contains_point = false;
    if (mVertices.size() >= 3)
    {
        double point[3];
        for (unsigned idx = 0; idx < DIM; idx++)
        {
            point[idx] = location[idx];
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
void Polygon<DIM>::ReplaceVertex(unsigned idx, boost::shared_ptr<DimensionalChastePoint<DIM> > pVertex)
{
    if(idx >= mVertices.size())
    {
        EXCEPTION("Requested vertex index out of range");
    }
    else
    {
        mVertices[idx] = pVertex;
    }
}

template<unsigned DIM>
void Polygon<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    for(unsigned idx=0; idx<mVertices.size(); idx++)
    {
        mVertices[idx]->RotateAboutAxis(axis, angle);
    }
}

template<unsigned DIM>
void Polygon<DIM>::Translate(c_vector<double, DIM> translationVector)
{
    for(unsigned idx=0; idx<mVertices.size(); idx++)
    {
        mVertices[idx]->Translate(translationVector);
    }
}

// Explicit instantiation
template class Polygon<2>;
template class Polygon<3>;

