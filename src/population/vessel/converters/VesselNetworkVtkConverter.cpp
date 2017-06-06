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

#include <vtkMergePoints.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <iostream>
#include <math.h>
#include <cfloat>
#include "VesselNetworkVtkConverter.hpp"

template <unsigned DIM>
VesselNetworkVtkConverter<DIM>::VesselNetworkVtkConverter()
{

}

template <unsigned DIM>
VesselNetworkVtkConverter<DIM>::~VesselNetworkVtkConverter()
{

}

template <unsigned DIM>
boost::shared_ptr<VesselNetworkVtkConverter<DIM> > VesselNetworkVtkConverter<DIM>::Create()
{
    MAKE_PTR(VesselNetworkVtkConverter<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> VesselNetworkVtkConverter<DIM>::GetVtkRepresentation(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();

    vtkSmartPointer<vtkPoints> p_node_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> p_lines = vtkSmartPointer<vtkCellArray>::New();

    if(nodes.size()>0)
    {
        units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            nodes[idx]->SetComparisonId(idx);
            c_vector<double, DIM> loc = nodes[idx]->rGetLocation().GetLocation(length_scale);
            if(DIM==2)
            {
                p_node_points->InsertNextPoint(loc[0], loc[1], 0.0);
            }
            else
            {
                p_node_points->InsertNextPoint(&loc[0]);
            }
        }

        for(unsigned idx=0;idx<segments.size();idx++)
        {
            vtkSmartPointer<vtkLine> p_line = vtkSmartPointer<vtkLine>::New();
            p_line->GetPointIds()->InsertId(0, segments[idx]->GetNode(0)->GetComparisonId());
            p_line->GetPointIds()->InsertId(1, segments[idx]->GetNode(1)->GetComparisonId());
            p_lines->InsertNextCell(p_line);
        }
    }

    vtkSmartPointer<vtkPolyData> p_vtk_network = vtkSmartPointer<vtkPolyData>::New();
    p_vtk_network->SetPoints(p_node_points);
    p_vtk_network->SetLines(p_lines);
    return p_vtk_network;
}

template<unsigned DIM>
vtkSmartPointer<vtkPolyData> VesselNetworkVtkConverter<DIM>::GetGlobalVtkRepresentation(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pNetwork->GetNodes();
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = pNetwork->GetVesselSegments();

    vtkSmartPointer<vtkPoints> p_node_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> p_lines = vtkSmartPointer<vtkCellArray>::New();

    if(nodes.size()>0)
    {
        units::quantity<unit::length> length_scale = BaseUnits::Instance()->GetReferenceLengthScale();
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            nodes[idx]->SetComparisonId(idx);
            c_vector<double, DIM> loc = nodes[idx]->rGetLocation().GetLocation(length_scale);
            if(DIM==2)
            {
                p_node_points->InsertNextPoint(loc[0], loc[1], 0.0);
            }
            else
            {
                p_node_points->InsertNextPoint(&loc[0]);
            }
        }

        for(unsigned idx=0;idx<segments.size();idx++)
        {
            vtkSmartPointer<vtkLine> p_line = vtkSmartPointer<vtkLine>::New();
            p_line->GetPointIds()->InsertId(0, segments[idx]->GetNode(0)->GetComparisonId());
            p_line->GetPointIds()->InsertId(1, segments[idx]->GetNode(1)->GetComparisonId());
            p_lines->InsertNextCell(p_line);
        }
    }

    vtkSmartPointer<vtkPolyData> p_vtk_network = vtkSmartPointer<vtkPolyData>::New();
    p_vtk_network->SetPoints(p_node_points);
    p_vtk_network->SetLines(p_lines);
    return p_vtk_network;
}

// Explicit instantiation
template class VesselNetworkVtkConverter<2>;
template class VesselNetworkVtkConverter<3>;

