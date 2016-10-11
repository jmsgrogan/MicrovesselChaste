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
#include "Exception.hpp"
#include "Warnings.hpp"
#include "Facet.hpp"
#include "Polygon.hpp"
#include "Element.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractTetrahedralMesh.hpp"
#include "VtkMeshWriter.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::DiscreteContinuumMeshGenerator() :
    mMaxElementArea(0.0 * unit::metres * unit::metres* unit::metres),
    mpMesh(),
    mpDomain(),
    mpVtkDomain(),
    mStlFilePath(),
    mHoles(),
    mRegions(),
    mAttributes(),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM> > DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Create()
{
    MAKE_PTR(DiscreteContinuumMeshGenerator<ELEMENT_DIM>, pSelf);
    return pSelf;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::~DiscreteContinuumMeshGenerator()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::GetMesh()
{
    if(!mpMesh)
    {
        EXCEPTION("No mesh has been generated.");
    }
    return mpMesh;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetDomain(boost::shared_ptr<Part<SPACE_DIM> > pDomain)
{
    mpDomain = pDomain;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetDomain(vtkSmartPointer<vtkPolyData> pDomain)
{
    mpVtkDomain = pDomain;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetDomain(const std::string& rPathToStl)
{
    mStlFilePath = rPathToStl;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetMaxElementArea(double maxElementArea)
{
    mMaxElementArea = maxElementArea;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetHoles(std::vector<DimensionalChastePoint<SPACE_DIM> > holes)
{
    mHoles = holes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetRegionMarkers(std::vector<DimensionalChastePoint<SPACE_DIM> > regionMarkers)
{
    mRegions = regionMarkers;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Update()
{
    // Create a mesh
    mpMesh = boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> >(new DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>());

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
                std::vector<units::quantity<unit::length> > bounding_box = mpDomain->GetBoundingBox();
                if (units::abs(bounding_box[4]) < 1.e-12*unit::metres && units::abs(bounding_box[5]) < 1.e-12*unit::metres)
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
            std::vector<units::quantity<unit::length> > bounding_box = mpDomain->GetBoundingBox();
            if (units::abs(bounding_box[4]) < 1.e-12*unit::metres && units::abs(bounding_box[5]) < 1.e-12*unit::metres)
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
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Mesh2d()
{
    struct triangulateio mesher_input, mesher_output;
    this->InitialiseTriangulateIo(mesher_input);
    this->InitialiseTriangulateIo(mesher_output);

    // Cases: have just domain, have just vtk domain, have vtk domain and domain
    if(mpDomain and !mpVtkDomain)
    {
        std::vector<DimensionalChastePoint<SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
        unsigned num_vertices = vertex_locations.size();
        mesher_input.pointlist = (double *) malloc(num_vertices * 2 * sizeof(double));
        mesher_input.numberofpoints = int(num_vertices);
        for (unsigned idx = 0; idx < num_vertices; idx++)
        {
            c_vector<double, SPACE_DIM> vertex_location = vertex_locations[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx] = vertex_location[jdx];
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
        std::vector<DimensionalChastePoint<SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
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
            c_vector<double, SPACE_DIM> vertex_location = vertex_locations[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.pointlist[2 * idx + jdx + 2*num_points] = vertex_location[jdx];
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
            c_vector<double, SPACE_DIM> hole_location = mHoles[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.holelist[2 * idx + jdx] = hole_location[jdx];
            }
        }
    }

    if (mRegions.size() > 0)
    {
        mesher_input.regionlist = (double *) malloc(mRegions.size() * 4 * sizeof(double));
        mesher_input.numberofregions = int(mRegions.size());
        for (unsigned idx = 0; idx < mRegions.size(); idx++)
        {
            c_vector<double, SPACE_DIM> region_location = mRegions[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 2; jdx++)
            {
                mesher_input.regionlist[4 * idx + jdx] = region_location[jdx];
            }
            mesher_input.regionlist[4 * idx + 2] = 1.0;
            mesher_input.regionlist[4 * idx + 3] = mMaxElementArea;
        }
    }

    std::string mesher_command = "pqQze";
    if (mMaxElementArea > 0.0*unit::metres*unit::metres*unit::metres)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea/(units::pow<3>(mReferenceLength)));
    }
    if(mRegions.size()>0)
    {
        if(mRegions.size()>0)
        {
            mesher_command += "A";
        }
    }

    triangulate((char*) mesher_command.c_str(), &mesher_input, &mesher_output, NULL);

    mpMesh->ImportFromMesher(mesher_output, mesher_output.numberoftriangles, mesher_output.trianglelist,
                           mesher_output.numberofedges, mesher_output.edgelist, mesher_output.edgemarkerlist);

    mAttributes.clear();
    if(mRegions.size()>0)
    {
        unsigned num_elements = mesher_output.numberoftriangles;
        for(unsigned idx=0; idx< num_elements; idx++)
        {
            mAttributes.push_back(double(mesher_output.triangleattributelist[idx]));
        }
        mpMesh->SetAttributes(mAttributes);
    }

    //Tidy up triangle
    this->FreeTriangulateIo(mesher_input);
    this->FreeTriangulateIo(mesher_output);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Mesh3d()
{
    std::vector<DimensionalChastePoint<SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
    std::vector<DimensionalChastePoint<SPACE_DIM> > hole_locations = mpDomain->GetHoleMarkers();
    unsigned num_vertices = vertex_locations.size();
    unsigned num_holes = hole_locations.size();
    std::vector<boost::shared_ptr<Facet<SPACE_DIM> > > facets = mpDomain->GetFacets();
    unsigned num_facets = facets.size();

    class tetgen::tetgenio mesher_input, mesher_output;

    tetgen::tetgenio::facet *f;
    tetgen::tetgenio::polygon *p;
    mesher_input.pointlist = new double[(num_vertices) * 3];
    mesher_input.numberofpoints = num_vertices;

    for (unsigned idx = 0; idx < num_vertices; idx++)
    {
        c_vector<double, SPACE_DIM> vertex_location = vertex_locations[idx].GetLocation(mReferenceLength);
        for (unsigned jdx = 0; jdx < 3; jdx++)
        {
            mesher_input.pointlist[3 * idx + jdx] = vertex_location[jdx];
        }
    }

    // Add the holes
    mesher_input.holelist = new double[(num_holes) * 3];
    mesher_input.numberofholes = num_holes;
    for (unsigned idx = 0; idx < num_holes; idx++)
    {
        c_vector<double, SPACE_DIM> hole_location = hole_locations[idx].GetLocation(mReferenceLength);
        for (unsigned jdx = 0; jdx < 3; jdx++)
        {
            mesher_input.holelist[3 * idx + jdx] = hole_location[jdx];
        }
    }

    mesher_input.numberoffacets = num_facets;
    mesher_input.facetlist = new tetgen::tetgenio::facet[num_facets];
    mesher_input.facetmarkerlist = new int[num_facets];
    for (unsigned idx = 0; idx < num_facets; idx++)
    {
        mesher_input.facetmarkerlist[idx] = 0;
        f = &mesher_input.facetlist[idx];
        std::vector<boost::shared_ptr<Polygon<SPACE_DIM> > > polygons = facets[idx]->GetPolygons();
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
            c_vector<double, SPACE_DIM> hole_location = mHoles[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 3; jdx++)
            {
                mesher_input.holelist[3 * idx + jdx] = hole_location[jdx];
            }
        }
    }

    std::string mesher_command = "pqQz";
    if (mMaxElementArea > 0.0*unit::metres*unit::metres*unit::metres)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea/units::pow<3>(mReferenceLength));
    }

    // Library call
    tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);
    mpMesh->ImportDiscreteContinuumMeshFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                           mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL);

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::MeshStl3d()
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
            c_vector<double, SPACE_DIM> hole_location = mHoles[idx].GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 3; jdx++)
            {
                mesher_input.holelist[3 * idx + jdx] = hole_location[jdx];
            }
        }
    }

    std::string mesher_command = "pqQz";
    if (mMaxElementArea >  0.0*unit::metres*unit::metres*unit::metres)
    {
        mesher_command += "a" + boost::lexical_cast<std::string>(mMaxElementArea/units::pow<3>(mReferenceLength));
    }

    // Library call
    tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);

    mpMesh->ImportDiscreteContinuumMeshFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                           mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL);

    delete[] writable;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::InitialiseTriangulateIo(triangulateio& mesherIo)
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
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::FreeTriangulateIo(triangulateio& mesherIo)
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
template class DiscreteContinuumMeshGenerator<2> ;
template class DiscreteContinuumMeshGenerator<3> ;
