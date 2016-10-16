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

#include "Exception.hpp"
#include <boost/filesystem.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkTetra.h>
#include <vtkGeometryFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkSTLWriter.h>
#include <vtkCleanPolyData.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkDataSetTriangleFilter.h>
#include <vtkExtractUnstructuredGrid.h>
#include <vtkDataSetSurfaceFilter.h>
//#include <dolfin.h>
#include "MultiFormatMeshWriter.hpp"

template<unsigned DIM>
MultiFormatMeshWriter<DIM>::MultiFormatMeshWriter()
    : mpVtkMesh(),
      mpMesh(),
      mFilepath(),
      mOutputFormat(MeshFormat::VTU)
{

}

template<unsigned DIM>
boost::shared_ptr<MultiFormatMeshWriter<DIM> > MultiFormatMeshWriter<DIM>::Create()
{
    MAKE_PTR(MultiFormatMeshWriter<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
MultiFormatMeshWriter<DIM>::~MultiFormatMeshWriter()
{

}

template<unsigned DIM>
void MultiFormatMeshWriter<DIM>::SetMesh(vtkSmartPointer<vtkUnstructuredGrid> pMesh)
{
    mpVtkMesh = pMesh;
}

template<unsigned DIM>
void MultiFormatMeshWriter<DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    mpMesh = pMesh;
}

template<unsigned DIM>
void MultiFormatMeshWriter<DIM>::SetFilename(const std::string& filename)
{
    mFilepath = filename;
}

template<unsigned DIM>
void MultiFormatMeshWriter<DIM>::SetOutputFormat(MeshFormat::Value outputFormat)
{
    mOutputFormat = outputFormat;
}

template<unsigned DIM>
void MultiFormatMeshWriter<DIM>::Write()
{
    if(mFilepath == "")
    {
        EXCEPTION("Output file not specified for mesh writer");
    }

    if(mOutputFormat == MeshFormat::VTU or mOutputFormat == MeshFormat::STL)
    {
        // If there is a DiscreteContinuum mesh convert it to vtk format first
        if(mpMesh)
        {
            vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();
            std::vector<c_vector<double, DIM> > node_locations = mpMesh->GetNodeLocations();
            p_vtk_points->SetNumberOfPoints(node_locations.size());
            for(unsigned idx=0; idx<node_locations.size(); idx++)
            {
                if(DIM==3)
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
            std::vector<std::vector<unsigned> > element_connectivity =  mpMesh->GetConnectivity();
            unsigned num_elements = element_connectivity.size();
            mpVtkMesh->Allocate(num_elements, num_elements);

            for(unsigned idx=0; idx<num_elements; idx++)
            {
                if(DIM==3)
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
        }

        if(!mpVtkMesh)
        {
            EXCEPTION("No mesh has been set, cannot do write.");
        }

        if(mOutputFormat == MeshFormat::VTU)
        {
            vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_writer1 = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
            p_writer1->SetFileName((mFilepath + ".vtu").c_str());
            #if VTK_MAJOR_VERSION <= 5
                p_writer1->SetInput(mpVtkMesh);
            #else
                p_writer1->SetInputData(mpVtkMesh);
            #endif
            p_writer1->Write();
        }
        else
        {
            vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_geom_filter->SetInput(mpVtkMesh);
            #else
                p_geom_filter->SetInputData(mpVtkMesh);
            #endif

            p_geom_filter->Update();

            vtkSmartPointer<vtkTriangleFilter> p_tri_filter = vtkSmartPointer<vtkTriangleFilter>::New();
            p_tri_filter->SetInputConnection(p_geom_filter->GetOutputPort());

            vtkSmartPointer<vtkCleanPolyData> p_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New();
            p_clean_filter->SetInputConnection(p_tri_filter->GetOutputPort());
            p_clean_filter->Update();

            vtkSmartPointer<vtkSTLWriter> p_writer1 = vtkSmartPointer<vtkSTLWriter>::New();
            p_writer1->SetFileName((mFilepath + ".stl").c_str());

            #if VTK_MAJOR_VERSION <= 5
                p_writer1->SetInput(p_clean_filter->GetOutput());
            #else
                p_writer1->SetInputData(p_clean_filter->GetOutput());
            #endif
            p_writer1->SetFileTypeToASCII();
            p_writer1->Write();
        }
    }
//    else if (mOutputFormat == MeshFormat::DOLFIN)
//    {
//        // Use DiscreteContinuum mesh directly
//        if(mpMesh)
//        {
//            dolfin::MeshEditor editor;
//            dolfin::Mesh dolfin_mesh;
//            editor.open(dolfin_mesh, DIM, DIM);
//
//            std::vector<std::vector<double> > node_locations = mpMesh->GetNodeLocations();
//            std::vector<std::vector<unsigned> > element_connectivity =  mpMesh->GetConnectivity();
//            editor.init_vertices(node_locations.size());
//            editor.init_cells(element_connectivity.size());
//
//            for(unsigned idx=0; idx<node_locations.size(); idx++)
//            {
//                if(DIM==2)
//                {
//                    editor.add_vertex(idx, node_locations[idx][0],
//                                      node_locations[idx][1]);
//                }
//                else
//                {
//                    editor.add_vertex(idx, node_locations[idx][0],
//                                      node_locations[idx][1],
//                                      node_locations[idx][2]);
//                }
//
//            }
//
//            for(unsigned idx=0; idx<element_connectivity.size(); idx++)
//            {
//                if(DIM==2)
//                {
//                    editor.add_cell(idx, element_connectivity[idx][0],
//                                    element_connectivity[idx][1],
//                                    element_connectivity[idx][2]);
//                }
//                else
//                {
//                    editor.add_cell(idx, element_connectivity[idx][0],
//                                    element_connectivity[idx][1],
//                                    element_connectivity[idx][2],
//                                    element_connectivity[idx][3]);
//                }
//            }
//
//            editor.close();
//            dolfin_mesh.init();
//
//            // Write the mesh to file
//            dolfin::File mesh_file (mFilepath + ".xml");
//            mesh_file << (dolfin_mesh);
//        }
//
////        else
////        {
////            // Use the vtk mesh
////            if(!mpVtkMesh)
////            {
////                EXCEPTION("No mesh has been set, cannot do write.");
////            }
////
////            dolfin::MeshEditor editor;
////            dolfin::Mesh dolfin_mesh;
////            editor.open(dolfin_mesh, DIM, DIM);
////
////            // Make sure we only count tetrahedron in 3D, not surface triangles etc.
////            if(DIM==3)
////            {
////                vtkSmartPointer<vtkDataSetTriangleFilter> p_tetrahedralize = vtkSmartPointer<vtkDataSetTriangleFilter>::New();
////                p_tetrahedralize->SetInputData(mpVtkMesh);
////                p_tetrahedralize->TetrahedraOnlyOn();
////                p_tetrahedralize->Update();
////
////                unsigned num_points = p_tetrahedralize->GetOutput()->GetNumberOfPoints();
////                editor.init_vertices(num_points);
////                for(unsigned idx=0; idx<num_points; idx++)
////                {
////                    editor.add_vertex(idx, p_tetrahedralize->GetOutput()->GetPoints()->GetPoint(idx)[0],
////                                      p_tetrahedralize->GetOutput()->GetPoints()->GetPoint(idx)[1],
////                                      p_tetrahedralize->GetOutput()->GetPoints()->GetPoint(idx)[2]);
////                }
////
////                unsigned num_cells = p_tetrahedralize->GetOutput()->GetNumberOfCells();
////                editor.init_cells(num_cells);
////                p_tetrahedralize->GetOutput()->GetCells()->InitTraversal();
////                vtkSmartPointer<vtkIdList> p_seglist = vtkSmartPointer<vtkIdList>::New();
////                for(unsigned idx=0; idx<num_cells; idx++)
////                {
////                    p_tetrahedralize->GetOutput()->GetCells()->GetNextCell(p_seglist);
////                    std::vector<unsigned> point_list;
////                    for(unsigned jdx=0; jdx<p_seglist->GetNumberOfIds(); jdx++)
////                    {
////                        point_list.push_back(unsigned(p_seglist->GetId(jdx)));
////                    }
////                    editor.add_cell(idx, point_list[0], point_list[1], point_list[2], point_list[3]);
////                }
////            }
////            else
////            {
////                unsigned num_points = mpVtkMesh->GetNumberOfPoints();
////                editor.init_vertices(num_points);
////                for(unsigned idx=0; idx<num_points; idx++)
////                {
////                    editor.add_vertex(idx, mpVtkMesh->GetPoints()->GetPoint(idx)[0],
////                                      mpVtkMesh->GetPoints()->GetPoint(idx)[1]);
////                }
////
////                unsigned num_cells = mpVtkMesh->GetNumberOfCells();
////                editor.init_cells(num_cells);
////                mpVtkMesh->GetCells()->InitTraversal();
////                vtkSmartPointer<vtkIdList> p_seglist = vtkSmartPointer<vtkIdList>::New();
////                for(unsigned idx=0; idx<num_cells; idx++)
////                {
////                    mpVtkMesh->GetCells()->GetNextCell(p_seglist);
////                    std::vector<unsigned> point_list;
////                    for(unsigned jdx=0; jdx<p_seglist->GetNumberOfIds(); jdx++)
////                    {
////                        point_list.push_back(unsigned(p_seglist->GetId(jdx)));
////                    }
////                    editor.add_cell(idx, point_list[0], point_list[1], point_list[2]);
////                }
////            }
////
////            editor.close();
////            dolfin_mesh.init();
////
////            // Write the mesh to file
////            dolfin::File mesh_file (mFilepath + ".xml");
////            mesh_file << (dolfin_mesh);
////        }
//    }
}

// Explicit instantiation
template class MultiFormatMeshWriter<2>;
template class MultiFormatMeshWriter<3>;
