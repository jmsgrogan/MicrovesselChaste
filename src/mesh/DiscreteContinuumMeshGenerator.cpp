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
std::shared_ptr<DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM> > DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Create()
{
    return std::make_shared<DiscreteContinuumMeshGenerator<ELEMENT_DIM> >();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::~DiscreteContinuumMeshGenerator()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::GetMesh()
{
    if(!mpMesh)
    {
        EXCEPTION("No mesh has been generated.");
    }
    return mpMesh;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetDomain(std::shared_ptr<Part<SPACE_DIM> > pDomain)
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
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::SetMaxElementArea(units::quantity<unit::volume> maxElementArea)
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
    mpMesh = std::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> >(new DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>());

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
                std::vector<QLength > bounding_box = mpDomain->GetBoundingBox();
                if (units::abs(bounding_box[4]) < 1.e-12*unit::metres && units::abs(bounding_box[5]) < 1.e-12*unit::metres)
                {
                   Mesh2d();
                   this->mpMesh->SetAttributesKeys(mpDomain->GetAttributesKeysForMesh(false));
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
            std::vector<QLength > bounding_box = mpDomain->GetBoundingBox();
            if (units::abs(bounding_box[4]) < 1.e-12*unit::metres && units::abs(bounding_box[5]) < 1.e-12*unit::metres)
            {
                EXCEPTION("The part is two-dimensional, use the 2D meshing functionality.");
            }
            else
            {
                Mesh3d();
                this->mpMesh->SetAttributesKeys(mpDomain->GetAttributesKeysForMesh(false));
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

        std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > segments = mpDomain->GetSegmentIndices();
        unsigned num_segments = segments.size();
        mesher_input.segmentlist = (int *) malloc(num_segments * 2 * sizeof(int));
        mesher_input.numberofsegments = int(num_segments);
        mesher_input.segmentmarkerlist = (int *) malloc(num_segments * sizeof(int));
        for (unsigned idx = 0; idx < num_segments; idx++)
        {
            mesher_input.segmentlist[2 * idx] = int(segments[idx].first.first);
            mesher_input.segmentlist[2 * idx + 1] = int(segments[idx].first.second);
            mesher_input.segmentmarkerlist[idx] = int(segments[idx].second);
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
        std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned > > segments = mpDomain->GetSegmentIndices();
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
            mesher_input.segmentlist[2 * idx + 2 * num_segments] = int(segments[idx].first.first + num_points);
            mesher_input.segmentlist[2 * idx + 1 + 2*num_segments] = int(segments[idx].first.second + num_points);
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

    std::vector<std::pair<DimensionalChastePoint<SPACE_DIM>, unsigned> > region_locations;
    if(mpDomain)
    {
        region_locations = mpDomain->GetRegionMarkers();
        if (mRegions.size() > 0 or region_locations.size()>0)
        {
            unsigned num_regions = mRegions.size() + region_locations.size();
            mesher_input.regionlist = (double *) malloc(num_regions * 4 * sizeof(double));
            mesher_input.numberofregions = int(num_regions);
            for (unsigned idx = 0; idx < mRegions.size(); idx++)
            {
                c_vector<double, SPACE_DIM> region_location = mRegions[idx].GetLocation(mReferenceLength);
                for (unsigned jdx = 0; jdx < 2; jdx++)
                {
                    mesher_input.regionlist[4 * idx + jdx] = region_location[jdx];
                }
                mesher_input.regionlist[4 * idx + 2] = 1.0;
                mesher_input.regionlist[4 * idx + 3] = mMaxElementArea/units::pow<3>(mReferenceLength);
            }
            for (unsigned idx = 0; idx < region_locations.size(); idx++)
            {
                c_vector<double, SPACE_DIM> region_location = region_locations[idx].first.GetLocation(mReferenceLength);
                for (unsigned jdx = 0; jdx < 2; jdx++)
                {
                    mesher_input.regionlist[4 * idx + jdx] = region_location[jdx];
                }
                mesher_input.regionlist[4 * idx + 2] = region_locations[idx].second;
                mesher_input.regionlist[4 * idx + 3] = mMaxElementArea/units::pow<3>(mReferenceLength);
            }
        }
    }

    std::string mesher_command = "pqQze";
    if (mMaxElementArea > 0.0*unit::metres*unit::metres*unit::metres)
    {
        double mesh_size = mMaxElementArea/units::pow<3>(mReferenceLength);
        mesher_command += "a" + boost::lexical_cast<std::string>(mesh_size);
    }

    if(mRegions.size()>0 or region_locations.size()>0)
    {
        mesher_command += "A";
    }

    triangulate((char*) mesher_command.c_str(), &mesher_input, &mesher_output, NULL);

    this->mpMesh->ImportDiscreteContinuumMeshFromTri(mesher_output, mesher_output.numberoftriangles, mesher_output.trianglelist,
                           mesher_output.numberofedges, mesher_output.edgelist, mesher_output.edgemarkerlist, NULL, 0, NULL);

    mAttributes.clear();
    if(mRegions.size()>0 or region_locations.size()>0)
    {
        unsigned num_elements = mesher_output.numberoftriangles;
        for(unsigned idx=0; idx< num_elements; idx++)
        {
            this->mpMesh->GetElement(idx)->AddElementAttribute(double(mesher_output.triangleattributelist[idx]));
        }
    }

    //Tidy up triangle
    this->FreeTriangulateIo(mesher_input);
    this->FreeTriangulateIo(mesher_output);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>::Mesh3d()
{
    // Only do this on master as tetgen is serial
    if(PetscTools::AmMaster())
    {
        std::vector<DimensionalChastePoint<SPACE_DIM> > vertex_locations = mpDomain->GetVertexLocations();
        std::vector<DimensionalChastePoint<SPACE_DIM> > hole_locations = mpDomain->GetHoleMarkers();
        std::vector<std::pair<DimensionalChastePoint<SPACE_DIM>, unsigned> > region_locations = mpDomain->GetRegionMarkers();

        unsigned num_vertices = vertex_locations.size();
        unsigned num_regions = region_locations.size();
        unsigned num_holes = hole_locations.size();

        std::vector<std::shared_ptr<Facet<SPACE_DIM> > > facets = mpDomain->GetFacets();
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

        // Add the regions
        mesher_input.regionlist = new double[(num_regions) * 5];
        mesher_input.numberofregions = num_regions;
        for (unsigned idx = 0; idx < num_regions; idx++)
        {
            c_vector<double, SPACE_DIM> region_location = region_locations[idx].first.GetLocation(mReferenceLength);
            for (unsigned jdx = 0; jdx < 3; jdx++)
            {
                mesher_input.regionlist[5 * idx + jdx] = region_location[jdx];
            }
            mesher_input.regionlist[5 * idx + 3] = double(region_locations[idx].second);
            mesher_input.regionlist[5 * idx + 4] = double(mMaxElementArea/units::pow<3>(mReferenceLength));
        }

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

        // Get attribute keys
        std::map<unsigned, std::string> attribute_keys = mpDomain->GetAttributesKeysForMesh(true);

        for (unsigned idx = 0; idx < num_facets; idx++)
        {
            // Use the first polygon to get the marker
            std::vector<std::shared_ptr<Polygon<SPACE_DIM> > > polygons = facets[idx]->GetPolygons();
            mesher_input.facetmarkerlist[idx] = 0;
            if(polygons.size()>0)
            {
                std::map<std::string, double> attributes = polygons[0]->GetAttributes();
                unsigned key=0;

                // Are any attributes active, if so set their value in the marker list
                for(std::map<std::string, double>::iterator it = attributes.begin(); it != attributes.end(); ++it)
                {
                    if(it->second>0.0)
                    {
                        // Find the key
                        for(std::map<unsigned, std::string>::iterator it2 = attribute_keys.begin();
                                it2 != attribute_keys.end(); ++it2)
                        {
                            if(it->first == it2->second)
                            {
                                key = it2->first;
                                break;
                            }
                        }
                    }
                }
                mesher_input.facetmarkerlist[idx] = int(key);
            }

            f = &mesher_input.facetlist[idx];
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
            double mesh_size = mMaxElementArea/units::pow<3>(mReferenceLength);
            mesher_command += "a" + boost::lexical_cast<std::string>(mesh_size);
        }
        if(num_regions>0)
        {
            mesher_command += "A";
        }

        // Library call - only do this on master
        tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);
        this->mpMesh->ImportDiscreteContinuumMeshFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                               mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL, mesher_output.trifacemarkerlist,
                               mesher_output.numberoftetrahedronattributes, mesher_output.tetrahedronattributelist);

        // Tetgen doesn't have multiple labels per facet, so if we have more than one polygon per facet
        // we need to do the labelling 'manually'. This is 'slow' but we don't expect to have many
        // cases of facets with multiple polygons.
        for(unsigned idx=0;idx<facets.size();idx++)
        {
            if(facets[idx]->GetPolygons().size()>1)
            {
                // Get original label
                std::map<std::string, double> attributes = facets[idx]->GetPolygons()[0]->GetAttributes();
                unsigned key=0;

                // Are any attributes active, if so set their value in the marker list
                for(std::map<std::string, double>::iterator it = attributes.begin(); it != attributes.end(); ++it)
                {
                    if(it->second>0.0)
                    {
                        // Find the key
                        for(std::map<unsigned, std::string>::iterator it2 = attribute_keys.begin();
                                it2 != attribute_keys.end(); ++it2)
                        {
                            if(it->first == it2->second)
                            {
                                key = it2->first;
                                break;
                            }
                        }
                    }
                }

                // Find boundary elements with this label
                for(unsigned jdx=0;jdx<this->mpMesh->GetNumBoundaryElements();jdx++)
                {
                    if(this->mpMesh->GetBoundaryElement(jdx)->GetUnsignedAttribute()==key)
                    {
                        c_vector<double, SPACE_DIM> centroid = this->mpMesh->GetBoundaryElement(jdx)->CalculateCentroid();
                        for(unsigned kdx=1;kdx<facets[idx]->GetPolygons().size(); kdx++)
                        {
                            c_vector<double, SPACE_DIM> cent = facets[idx]->GetPolygons()[kdx]->GetCentroid().GetLocation(mReferenceLength);
                            if(facets[idx]->GetPolygons()[kdx]->ContainsPoint(
                                    DimensionalChastePoint<SPACE_DIM>(centroid, mReferenceLength), 1.e-3))
                            {
                                std::map<std::string, double> attributes = facets[idx]->GetPolygons()[kdx]->GetAttributes();
                                unsigned key=0;

                                // Are any attributes active, if so set their value in the marker list
                                for(std::map<std::string, double>::iterator it = attributes.begin(); it != attributes.end(); ++it)
                                {
                                    if(it->second>0.0)
                                    {
                                        // Find the key
                                        for(std::map<unsigned, std::string>::iterator it2 = attribute_keys.begin();
                                                it2 != attribute_keys.end(); ++it2)
                                        {
                                            if(it->first == it2->second)
                                            {
                                                key = it2->first;
                                                break;
                                            }
                                        }
                                    }
                                }
                                this->mpMesh->GetBoundaryElement(jdx)->SetAttribute(double(key));
                            }
                        }
                    }
                }
            }
        }

        // if we are running parallel don't assume the resulting mesh is the same on all procs. Do a write-read
        // to make sure it is.
        if(PetscTools::IsParallel())
        {
            {
                TrianglesMeshWriter<ELEMENT_DIM,SPACE_DIM> mesh_writer("", "temp_mesh");
                mesh_writer.WriteFilesUsingMesh(*mpMesh);
            }
        }
    }

    // if we are running parallel everyone needs to read the mesh back in
    if(PetscTools::IsParallel())
    {
        PetscTools::Barrier();
        OutputFileHandler output_handler("", false);
        std::string output_dir = output_handler.GetOutputDirectoryFullPath();
        TrianglesMeshReader<ELEMENT_DIM,SPACE_DIM> mesh_reader(output_dir+"temp_mesh");
        mpMesh = std::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> >(new DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>);
        mpMesh->ConstructFromMeshReader(mesh_reader);
    }
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
        double mesh_size = mMaxElementArea/units::pow<3>(mReferenceLength);
        mesher_command += "a" + boost::lexical_cast<std::string>(mesh_size);
    }

    // Library call
    tetgen::tetrahedralize((char*) mesher_command.c_str(), &mesher_input, &mesher_output);
    this->mpMesh->ImportDiscreteContinuumMeshFromTetgen(mesher_output, mesher_output.numberoftetrahedra, mesher_output.tetrahedronlist,
                           mesher_output.numberoftrifaces, mesher_output.trifacelist, NULL, NULL, 0, NULL);

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
