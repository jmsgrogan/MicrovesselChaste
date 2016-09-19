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

#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkTetra.h>
#include "Exception.hpp"
#include "Warnings.hpp"
#include "Facet.hpp"
#include "Polygon.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "Element.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractTetrahedralMesh.hpp"
#include "VtkMeshWriter.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::DiscreteContinuumMesh() :
    mMaxElementArea(0.0),
    mpDomain(),
    mpVtkDomain(),
    mStlFilePath(),
    mHoles(),
    mRegions(),
    mAttributes(),
    mReferenceLength(1.e-6 * unit::metres),
    mpVtkMesh(vtkSmartPointer<vtkUnstructuredGrid>::New()),
    mVtkRepresentationUpToDate(false)
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
vtkSmartPointer<vtkUnstructuredGrid> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetAsVtkUnstructuredGrid()
{
    if(!mVtkRepresentationUpToDate)
    {
        mpVtkMesh = vtkSmartPointer<vtkUnstructuredGrid>::New();
        vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();
        std::vector<std::vector<double> > node_locations = GetNodeLocations();
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
        }
        mpVtkMesh->SetPoints(p_vtk_points);

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
        mVtkRepresentationUpToDate = true;
    }
    return mpVtkMesh;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetDomain(boost::shared_ptr<Part<SPACE_DIM> > pDomain)
{
    mpDomain = pDomain;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetDomain(vtkSmartPointer<vtkPolyData> pDomain)
{
    mpVtkDomain = pDomain;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetDomain(const std::string& rPathToStl)
{
    mStlFilePath = rPathToStl;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetMaxElementArea(double maxElementArea)
{
    mMaxElementArea = maxElementArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetHoles(std::vector<DimensionalChastePoint<SPACE_DIM> > holes)
{
    mHoles = holes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetRegionMarkers(std::vector<DimensionalChastePoint<SPACE_DIM> > regionMarkers)
{
    mRegions = regionMarkers;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::Update()
{
    // For 2D parts use triangle
    if (ELEMENT_DIM == 2)
    {
        if (SPACE_DIM == 2)
        {
            Mesh2d();
        }
        else
        {
            if(mpDomain)
            {
                c_vector<double, 2 * ELEMENT_DIM> bounding_box = mpDomain->GetBoundingBox();
                if (std::abs(bounding_box[4]) < 1.e-6 && std::abs(bounding_box[5]) < 1.e-6)
                {
                   Mesh2d();
                }
                else
                {
                    EXCEPTION("2D meshing is only supported for parts with z=0.");
                }
            }
            else if(mpVtkDomain)
            {
                Mesh2d();
            }
            else
            {
                EXCEPTION("2d meshing is not supported for STL files.");
            }
        }
    }
    // For 3d use tetgen
    else
    {
        if(mpDomain)
        {
            c_vector<double, 2 * ELEMENT_DIM> bounding_box = mpDomain->GetBoundingBox();
            if (std::abs(bounding_box[4]) < 1.e-6 && std::abs(bounding_box[5]) < 1.e-6)
            {
                EXCEPTION("The part is two-dimensional, use the 2D meshing functionality.");
            }
            else
            {
                Mesh3d();
            }
        }
        else if(mpVtkDomain)
        {
            Mesh3d();
        }
        else
        {
            MeshStl3d();
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::Mesh2d()
{
    struct triangulateio mesher_input, mesher_output;
    this->InitialiseTriangulateIo(mesher_input);
    this->InitialiseTriangulateIo(mesher_output);

    // Cases: have just domain, have just vtk domain, have vtk domain and domain
    if(mpDomain and !mpVtkDomain)
    {
        std::vector<c_vector<double, SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
        unsigned num_vertices = vertex_locations.size();
        mesher_input.pointlist = (double *) malloc(num_vertices * 2 * sizeof(double));
        mesher_input.numberofpoints = int(num_vertices);
        for (unsigned idx = 0; idx < num_vertices; idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx] = vertex_locations[idx][jdx];
            }
        }

        std::vector<std::pair<unsigned, unsigned> > segments = mpDomain->GetSegmentIndices();
        unsigned num_segments = segments.size();
        mesher_input.segmentlist = (int *) malloc(num_segments * 2 * sizeof(int));
        mesher_input.numberofsegments = int(num_segments);
        for (unsigned idx = 0; idx < num_segments; idx++)
        {
            mesher_input.segmentlist[2 * idx] = int(segments[idx].first);
            mesher_input.segmentlist[2 * idx + 1] = int(segments[idx].second);
        }
    }
    else if(!mpDomain and mpVtkDomain)
    {
        unsigned num_points = mpVtkDomain->GetNumberOfPoints();
        mesher_input.pointlist = (double *) malloc(num_points * 2 * sizeof(double));
        mesher_input.numberofpoints = int(num_points);
        for (unsigned idx = 0; idx < num_points; idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx] = mpVtkDomain->GetPoints()->GetPoint(idx)[jdx];
            }
        }

        unsigned num_segments = mpVtkDomain->GetNumberOfLines();
        mpVtkDomain->GetLines()->InitTraversal();
        mesher_input.segmentlist = (int *) malloc(num_segments * 2 * sizeof(int));
        mesher_input.numberofsegments = int(num_segments);
        vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();
        for (unsigned idx = 0; idx < num_segments; idx++)
        {
            mpVtkDomain->GetLines()->GetNextCell(p_id_list);
            mesher_input.segmentlist[2 * idx] = int(p_id_list->GetId(0));
            mesher_input.segmentlist[2 * idx + 1] = int(p_id_list->GetId(1));
        }
    }
    else if(mpDomain and mpVtkDomain)
    {
        unsigned num_points = mpVtkDomain->GetNumberOfPoints();
        std::vector<c_vector<double, SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
        unsigned num_vertices = vertex_locations.size();

        mesher_input.pointlist = (double *) malloc((num_points+num_vertices) * 2 * sizeof(double));
        mesher_input.numberofpoints = int(num_points+num_vertices);
        for (unsigned idx = 0; idx < num_points; idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx] = mpVtkDomain->GetPoints()->GetPoint(idx)[jdx];
            }
        }
        for (unsigned idx = 0; idx < num_vertices; idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx + 2*num_points] = vertex_locations[idx][jdx];
            }
        }

        unsigned num_segments = mpVtkDomain->GetNumberOfLines();
        std::vector<std::pair<unsigned, unsigned> > segments = mpDomain->GetSegmentIndices();
        unsigned num_part_segments = segments.size();
        mpVtkDomain->GetLines()->InitTraversal();

        mesher_input.segmentlist = (int *) malloc((num_segments+num_part_segments) * 2 * sizeof(int));
        mesher_input.numberofsegments = int(num_segments+num_part_segments);
        vtkSmartPointer<vtkIdList> p_id_list = vtkSmartPointer<vtkIdList>::New();
        for (unsigned idx = 0; idx < num_segments; idx++)
        {
            mpVtkDomain->GetLines()->GetNextCell(p_id_list);
            mesher_input.segmentlist[2 * idx] = int(p_id_list->GetId(0));
            mesher_input.segmentlist[2 * idx + 1] = int(p_id_list->GetId(1));
        }
        for (unsigned idx = 0; idx < num_part_segments; idx++)
        {
            mesher_input.segmentlist[2 * idx + 2 * num_segments] = int(segments[idx].first + num_points);
            mesher_input.segmentlist[2 * idx + 1 + 2*num_segments] = int(segments[idx].second + num_points);
        }

    }
    else
    {
        EXCEPTION("Either a part, vtk surface or both are required for 2d meshing");
    }

    if (mHoles.size() > 0)
    {
        mesher_input.holelist = (double *) malloc(mHoles.size() * 2 * sizeof(double));
        mesher_input.numberofholes = int(mHoles.size());
        for (unsigned idx = 0; idx < mHoles.size(); idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.holelist[2 * idx + jdx] = mHoles[idx][jdx];
            }
        }
    }

    if (mRegions.size() > 0)
    {
        mesher_input.regionlist = (double *) malloc(mRegions.size() * 4 * sizeof(double));
        mesher_input.numberofregions = int(mRegions.size());
        for (unsigned idx = 0; idx < mRegions.size(); idx++)
        {
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.regionlist[4 * idx + jdx] = mRegions[idx][jdx];
            }
            mesher_input.regionlist[4 * idx + 2] = 1.0;
            mesher_input.regionlist[4 * idx + 3] = mMaxElementArea;
        }
    }

    std::string mesher_command = "pqQze";
    if (mMaxElementArea > 0.0)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea);
    }
    if(mRegions.size()>0)
    {
        if(mRegions.size()>0)
        {
            mesher_command += "A";
        }
    }

    triangulate((char*) mesher_command.c_str(), &mesher_input, &mesher_output, NULL);

    this->ImportFromMesher(mesher_output, mesher_output.numberoftriangles, mesher_output.trianglelist,
                           mesher_output.numberofedges, mesher_output.edgelist, mesher_output.edgemarkerlist);

    if(mRegions.size()>0)
    {
        unsigned num_elements = mesher_output.numberoftriangles;
        for(unsigned idx=0; idx< num_elements; idx++)
        {
            mAttributes.push_back(double(mesher_output.triangleattributelist[idx]));
        }
    }

    //Tidy up triangle
    this->FreeTriangulateIo(mesher_input);
    this->FreeTriangulateIo(mesher_output);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<unsigned> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetElementRegionMarkers()
{
    return mAttributes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::Mesh3d()
{
    std::vector<c_vector<double, SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
    std::vector<DimensionalChastePoint<SPACE_DIM> > hole_locations = mpDomain->GetHoleMarkers();
    unsigned num_vertices = vertex_locations.size();
    unsigned num_holes = hole_locations.size();
    std::vector<boost::shared_ptr<Facet> > facets = mpDomain->GetFacets();
    unsigned num_facets = facets.size();

    class tetgen::tetgenio mesher_input, mesher_output;

    tetgen::tetgenio::facet *f;
    tetgen::tetgenio::polygon *p;
    mesher_input.pointlist = new double[(num_vertices) * 3];
    mesher_input.numberofpoints = num_vertices;

    for (unsigned idx = 0; idx < num_vertices; idx++)
    {
        for (unsigned jdx = 0; jdx < 3; jdx++)
        {
            mesher_input.pointlist[3 * idx + jdx] = vertex_locations[idx][jdx];
        }
    }

    // Add the holes
    mesher_input.holelist = new double[(num_holes) * 3];
    mesher_input.numberofholes = num_holes;
    for (unsigned idx = 0; idx < num_holes; idx++)
    {
        for (unsigned jdx = 0; jdx < 3; jdx++)
        {
            mesher_input.holelist[3 * idx + jdx] = hole_locations[idx][jdx];
        }
    }

    mesher_input.numberoffacets = num_facets;
    mesher_input.facetlist = new tetgen::tetgenio::facet[num_facets];
    mesher_input.facetmarkerlist = new int[num_facets];
    for (unsigned idx = 0; idx < num_facets; idx++)
    {
        mesher_input.facetmarkerlist[idx] = 0;
        f = &mesher_input.facetlist[idx];
        std::vector<boost::shared_ptr<Polygon> > polygons = facets[idx]->GetPolygons();
        f->numberofpolygons = polygons.size();
        f->polygonlist = new tetgen::tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;
        for (unsigned jdx = 0; jdx < polygons.size(); jdx++)
        {
            p = &f->polygonlist[jdx];
            p->numberofvertices = polygons[jdx]->GetVertices().size();
            p->vertexlist = new int[p->numberofvertices];
            for (unsigned kdx = 0; kdx < polygons[jdx]->GetVertices().size(); kdx++)
            {
                p->vertexlist[kdx] = int(polygons[jdx]->GetVertices()[kdx]->GetIndex());
            }
        }
    }

    if(mHoles.size() > 0)
    {
        unsigned num_holes = mHoles.size();
        mesher_input.holelist = new double[(num_holes) * 3];
        mesher_input.numberofholes = num_holes;
        for (unsigned idx = 0; idx < num_holes; idx++)
        {
            for (unsigned jdx = 0; jdx < 3; jdx++)
            {
                mesher_input.holelist[3 * idx + jdx] = mHoles[idx][jdx];
            }
        }
    }

    std::string mesher_command = "pqQz";
    if (mMaxElementArea > 0.0)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea);
    }

    // Library call
    tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);
    this->ImportFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                           mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL);

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::MeshStl3d()
{
    class tetgen::tetgenio mesher_input, mesher_output;
    char * writable = new char[mStlFilePath.size() + 1];
    std::copy(mStlFilePath.begin(), mStlFilePath.end(), writable);
    writable[mStlFilePath.size()] = '\0';
    mesher_input.load_stl(writable);

    if(mHoles.size() > 0)
    {
        unsigned num_holes = mHoles.size();
        mesher_input.holelist = new double[(num_holes) * 3];
        mesher_input.numberofholes = num_holes;
        for (unsigned idx = 0; idx < num_holes; idx++)
        {
            for (unsigned jdx = 0; jdx < 3; jdx++)
            {
                mesher_input.holelist[3 * idx + jdx] = mHoles[idx][jdx];
            }
        }
    }

    std::string mesher_command = "pqQz";
    if (mMaxElementArea > 0.0)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea);
    }

    // Library call
    tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);

    this->ImportFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                           mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL);

    delete[] writable;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::ImportFromTetgen(tetgen::tetgenio& mesherOutput, unsigned numberOfElements,
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
            node_indexes.push_back(
                    AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetElement(idx)->GetNodeGlobalIndex(jdx));
        }
        connectivity.push_back(node_indexes);
    }
    return connectivity;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<std::vector<double> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetNodeLocations()
{
    std::vector<std::vector<double> > locations;
    unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumNodes();
    for (unsigned idx = 0; idx < num_nodes; idx++)
    {
        c_vector<double, SPACE_DIM> location =
                AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNode(idx)->rGetLocation();
        std::vector<double> vec_location;
        for (unsigned jdx = 0; jdx < SPACE_DIM; jdx++)
        {
            vec_location.push_back(location[jdx]);
        }
        locations.push_back(vec_location);
    }
    return locations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::vector<DimensionalChastePoint<SPACE_DIM> > DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetNodeLocationsAsPoints()
{
    std::vector<DimensionalChastePoint<SPACE_DIM> > locations;
    unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumNodes();
    for (unsigned idx = 0; idx < num_nodes; idx++)
    {
        locations.push_back(DimensionalChastePoint<SPACE_DIM>(AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNode(idx)->rGetLocation()));
    }
    return locations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::InitialiseTriangulateIo(triangulateio& mesherIo)
{
    mesherIo.numberofpoints = 0;
    mesherIo.pointlist = NULL;
    mesherIo.numberofpointattributes = 0;
    mesherIo.pointattributelist = (double *) NULL;
    mesherIo.pointmarkerlist = (int *) NULL;
    mesherIo.segmentlist = NULL;
    mesherIo.segmentmarkerlist = (int *) NULL;
    mesherIo.numberofsegments = 0;
    mesherIo.numberofholes = 0;
    mesherIo.numberofregions = 0;
    mesherIo.trianglelist = (int *) NULL;
    mesherIo.triangleattributelist = (double *) NULL;
    mesherIo.numberoftriangleattributes = 0;
    mesherIo.edgelist = (int *) NULL;
    mesherIo.edgemarkerlist = (int *) NULL;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::FreeTriangulateIo(triangulateio& mesherIo)
{
    if (mesherIo.numberofpoints != 0)
    {
        mesherIo.numberofpoints = 0;
        free(mesherIo.pointlist);
    }

    if (mesherIo.numberofsegments != 0)
    {
        mesherIo.numberofsegments = 0;
        free(mesherIo.segmentlist);
    }

    // These (and the above) should actually be safe since we explicity set to NULL above
    free(mesherIo.pointattributelist);
    free(mesherIo.pointmarkerlist);
    free(mesherIo.segmentmarkerlist);
    free(mesherIo.trianglelist);
    free(mesherIo.triangleattributelist);
    free(mesherIo.edgelist);
    free(mesherIo.edgemarkerlist);
}

// Explicit instantiation
template class DiscreteContinuumMesh<2> ;
template class DiscreteContinuumMesh<3> ;
