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

#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkTetra.h>
#include <vtkGenericCell.h>
#include "Exception.hpp"
#include "Warnings.hpp"
#include "Facet.hpp"
#include "Polygon.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "Element.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractTetrahedralMesh.hpp"
#include "VtkMeshWriter.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::DiscreteContinuumMesh() :
    mAttributes(),
    mVolumes(),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
    mpVtkMesh(),
    mpVtkCellLocator(vtkSmartPointer<vtkCellLocator>::New()),
    mVtkRepresentationUpToDate(false),
    mNodalData()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::Create()
{
    MAKE_PTR(DiscreteContinuumMesh<ELEMENT_DIM>, pSelf);
    return pSelf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::~DiscreteContinuumMesh()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::length> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkUnstructuredGrid> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetAsVtkUnstructuredGrid()
{
    if(!mVtkRepresentationUpToDate)
    {
        mpVtkMesh = vtkSmartPointer<vtkUnstructuredGrid>::New();
        vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkDoubleArray> p_point_data = vtkSmartPointer<vtkDoubleArray>::New();
        p_point_data->SetName("Nodal Values");

        std::vector<c_vector<double, SPACE_DIM> > node_locations = GetNodeLocations();
        p_vtk_points->SetNumberOfPoints(node_locations.size());
        for(unsigned idx=0; idx<node_locations.size(); idx++)
        {
            if(SPACE_DIM==3)
            {
                p_vtk_points->InsertPoint(idx, node_locations[idx][0], node_locations[idx][1], node_locations[idx][2]);
            }
            else
            {
                p_vtk_points->InsertPoint(idx, node_locations[idx][0], node_locations[idx][1], 0.0);
            }

            if(mNodalData.size()==node_locations.size())
            {
                p_point_data->InsertNextTuple1(mNodalData[idx]);
            }
        }
        mpVtkMesh->SetPoints(p_vtk_points);
        if(mNodalData.size()==node_locations.size())
        {
            mpVtkMesh->GetPointData()->AddArray(p_point_data);
        }

        // Add vtk tets or triangles
        std::vector<std::vector<unsigned> > element_connectivity =  GetConnectivity();
        unsigned num_elements = element_connectivity.size();
        mpVtkMesh->Allocate(num_elements, num_elements);

        for(unsigned idx=0; idx<num_elements; idx++)
        {
            if(ELEMENT_DIM==3)
            {
                vtkSmartPointer<vtkTetra> p_vtk_element = vtkSmartPointer<vtkTetra>::New();
                unsigned num_nodes = element_connectivity[idx].size();
                for(unsigned jdx=0; jdx<num_nodes; jdx++)
                {
                    p_vtk_element->GetPointIds()->SetId(jdx, element_connectivity[idx][jdx]);
                }
                mpVtkMesh->InsertNextCell(p_vtk_element->GetCellType(), p_vtk_element->GetPointIds());
            }
            else
            {
                vtkSmartPointer<vtkTriangle> p_vtk_element = vtkSmartPointer<vtkTriangle>::New();
                unsigned num_nodes = element_connectivity[idx].size();
                for(unsigned jdx=0; jdx<num_nodes; jdx++)
                {
                    p_vtk_element->GetPointIds()->SetId(jdx, element_connectivity[idx][jdx]);
                }
                mpVtkMesh->InsertNextCell(p_vtk_element->GetCellType(), p_vtk_element->GetPointIds());
            }
        }
        mpVtkCellLocator = vtkSmartPointer<vtkCellLocator>::New();
        mpVtkCellLocator->SetDataSet(mpVtkMesh);
        mpVtkCellLocator->BuildLocator();

        mVtkRepresentationUpToDate = true;
    }
    return mpVtkMesh;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<unsigned> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetElementRegionMarkers()
{
    return mAttributes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<c_vector<double, SPACE_DIM> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetElementCentroids()
{
    std::vector<c_vector<double, SPACE_DIM> > centroids(this->GetNumElements());
    for(unsigned idx=0; idx<this->GetNumElements(); idx++)
    {
        centroids[idx] = this->GetElement(idx)->CalculateCentroid();
    }
    return centroids;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkCellLocator> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetVtkCellLocator()
{
    return mpVtkCellLocator;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<double> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetElementVolumes()
{
    if(mVolumes.size()!=this->GetNumElements())
    {
        mVolumes.clear();
        for(unsigned idx=0; idx<this->GetNumElements(); idx++)
        {
            double determinant = 0.0;
            c_matrix<double, ELEMENT_DIM, SPACE_DIM> jacobian;
            this->GetElement(idx)->CalculateJacobian(jacobian, determinant);
            mVolumes.push_back(this->GetElement(idx)->GetVolume(determinant));
        }
    }
    return mVolumes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetNodalData(std::vector<double> rNodalValues)
{
    mNodalData.clear();
    for(unsigned idx=0; idx<rNodalValues.size(); idx++)
    {
        mNodalData.push_back(rNodalValues[idx]);
    }
    mVtkRepresentationUpToDate = false;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<std::vector<unsigned> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetConnectivity()
{
    std::vector<std::vector<unsigned> > connectivity;
    unsigned num_elements = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumElements();
    for (unsigned idx = 0; idx < num_elements; idx++)
    {
        std::vector<unsigned> node_indexes;
        unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetElement(idx)->GetNumNodes();
        for (unsigned jdx = 0; jdx < num_nodes; jdx++)
        {
            node_indexes.push_back(AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetElement(idx)->GetNodeGlobalIndex(jdx));
        }
        connectivity.push_back(node_indexes);
    }
    return connectivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<c_vector<double, SPACE_DIM> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetNodeLocations()
{
    unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumNodes();
    std::vector<c_vector<double, SPACE_DIM> > locations(num_nodes);
    for (unsigned idx = 0; idx < num_nodes; idx++)
    {
        locations[idx] = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNode(idx)->rGetLocation();
    }
    return locations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<DimensionalChastePoint<SPACE_DIM> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetNodeLocationsAsPoints()
{
    unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumNodes();
    std::vector<DimensionalChastePoint<SPACE_DIM> > locations(num_nodes);
    for (unsigned idx = 0; idx < num_nodes; idx++)
    {
        locations[idx] = DimensionalChastePoint<SPACE_DIM>(AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNode(idx)->rGetLocation(), mReferenceLength);
    }
    return locations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::ImportDiscreteContinuumMeshFromTetgen(tetgen::tetgenio& mesherOutput, unsigned numberOfElements,
                                                          int *elementList, unsigned numberOfFaces, int *faceList,
                                                          int *edgeMarkerList)
{
    unsigned nodes_per_element = mesherOutput.numberofcorners;

    assert(nodes_per_element == ELEMENT_DIM + 1 || nodes_per_element == (ELEMENT_DIM + 1) * (ELEMENT_DIM + 2) / 2);

    for (unsigned i = 0; i < this->mBoundaryElements.size(); i++)
    {
        delete this->mBoundaryElements[i];
    }
    for (unsigned i = 0; i < this->mElements.size(); i++)
    {
        delete this->mElements[i];
    }
    for (unsigned i = 0; i < this->mNodes.size(); i++)
    {
        delete this->mNodes[i];
    }

    this->mNodes.clear();
    this->mElements.clear();
    this->mBoundaryElements.clear();
    this->mBoundaryNodes.clear();

    // Construct the nodes
    for (unsigned node_index = 0; node_index < (unsigned) mesherOutput.numberofpoints; node_index++)
    {
        this->mNodes.push_back(new Node<SPACE_DIM>(node_index, &mesherOutput.pointlist[node_index * SPACE_DIM], false));
    }

    // Construct the elements
    this->mElements.reserve(numberOfElements);

    unsigned real_element_index = 0;
    for (unsigned element_index = 0; element_index < numberOfElements; element_index++)
    {
        std::vector<Node<SPACE_DIM>*> nodes;
        for (unsigned j = 0; j < ELEMENT_DIM + 1; j++)
        {
            unsigned global_node_index = elementList[element_index * (nodes_per_element) + j];
            assert(global_node_index < this->mNodes.size());
            nodes.push_back(this->mNodes[global_node_index]);

        }

        /*
         * For some reason, tetgen in library mode makes its initial Delaunay mesh
         * with very thin slivers. Hence we expect to ignore some of the elements!
         */
        Element<ELEMENT_DIM, SPACE_DIM>* p_element;
        try
        {
            p_element = new Element<ELEMENT_DIM, SPACE_DIM>(real_element_index, nodes);

            // Shouldn't throw after this point
            this->mElements.push_back(p_element);

            // Add the internals to quadratics
            for (unsigned j = ELEMENT_DIM + 1; j < nodes_per_element; j++)
            {
                unsigned global_node_index = elementList[element_index * nodes_per_element + j];
                assert(global_node_index < this->mNodes.size());
                this->mElements[real_element_index]->AddNode(this->mNodes[global_node_index]);
                this->mNodes[global_node_index]->AddElement(real_element_index);
                this->mNodes[global_node_index]->MarkAsInternal();
            }
            real_element_index++;
        }
        catch (Exception &)
        {
            if (SPACE_DIM == 2)
            {
                WARNING("Triangle has produced a zero area (collinear) element");
            }
            else
            {
                WARNING("Tetgen has produced a zero volume (coplanar) element");
            }
        }
    }

    // Construct the BoundaryElements (and mark boundary nodes)
    unsigned next_boundary_element_index = 0;
    for (unsigned boundary_element_index = 0; boundary_element_index < numberOfFaces; boundary_element_index++)
    {
        /*
         * Tetgen produces only boundary faces (set edgeMarkerList to NULL).
         * Triangle marks which edges are on the boundary.
         */
        if (edgeMarkerList == NULL || edgeMarkerList[boundary_element_index] == 1)
        {
            std::vector<Node<SPACE_DIM>*> nodes;
            for (unsigned j = 0; j < ELEMENT_DIM; j++)
            {
                unsigned global_node_index = faceList[boundary_element_index * ELEMENT_DIM + j];
                assert(global_node_index < this->mNodes.size());
                nodes.push_back(this->mNodes[global_node_index]);
                if (!nodes[j]->IsBoundaryNode())
                {
                    nodes[j]->SetAsBoundaryNode();
                    this->mBoundaryNodes.push_back(nodes[j]);
                }
            }

            /*
             * For some reason, tetgen in library mode makes its initial Delaunay mesh
             * with very thin slivers. Hence we expect to ignore some of the elements!
             */
            BoundaryElement<ELEMENT_DIM - 1, SPACE_DIM>* p_b_element;
            try
            {
                p_b_element = new BoundaryElement<ELEMENT_DIM - 1, SPACE_DIM>(next_boundary_element_index, nodes);
                this->mBoundaryElements.push_back(p_b_element);
                next_boundary_element_index++;
            }
            catch (Exception &)
            {
                // Tetgen is feeding us lies  //Watch this space for coverage
                assert(SPACE_DIM == 3);
            }
        }
    }

    this->RefreshJacobianCachedData();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetAttributes(std::vector<unsigned> attributes)
{
    mAttributes = attributes;
}

// Explicit instantiation
template class DiscreteContinuumMesh<2> ;
template class DiscreteContinuumMesh<3> ;
