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
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkTetra.h>
#include <vtkGenericCell.h>
#include <vtkThreshold.h>
#include <vtkVersion.h>
#include <vtkUnstructuredGrid.h>
#include "Exception.hpp"
#include "Warnings.hpp"
#include "Facet.hpp"
#include "Polygon.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "Element.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractTetrahedralMesh.hpp"
#include "BaseUnits.hpp"
#include <parmetis.h>
#if (PARMETIS_MAJOR_VERSION >= 4) //ParMETIS 4.x and above
//Redefine the index type so that we can still use the old name "idxtype"
#define idxtype idx_t
#else
//Old version of ParMETIS used "float" which may appear elsewhere in, for example, tetgen
#define real_t float
#endif

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::DiscreteContinuumMesh() :
    AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>()
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
std::vector<unsigned> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetElementPartitioning()
{
    if(PetscTools::IsSequential())
    {
        return std::vector<unsigned>(this->GetNumElements(), 0);
    }
    else
    {
        PetscTools::Barrier();

        // Get the mesh dual graph - similar to DistributedTetrahedralMesh method
        const unsigned num_elements = this->GetNumElements();
        const unsigned num_procs = PetscTools::GetNumProcs();
        const unsigned local_proc_index = PetscTools::GetMyRank();

        // Initial assignment of elements to procs
        boost::scoped_array<idxtype> element_distribution(new idxtype[num_procs+1]);
        boost::scoped_array<int> element_counts(new int[num_procs]);
        element_distribution[0] = 0;
        for (unsigned proc_index=1; proc_index<num_procs; proc_index++)
        {
            element_distribution[proc_index] = element_distribution[proc_index-1] + num_elements/num_procs;
            element_counts[proc_index-1] = element_distribution[proc_index] - element_distribution[proc_index-1];
        }
        element_distribution[num_procs] = num_elements;
        element_counts[num_procs-1] = element_distribution[num_procs] - element_distribution[num_procs-1];

        /*
         *  Create distributed mesh data structure
         */
        idxtype first_local_element = element_distribution[local_proc_index];
        idxtype last_plus_one_element = element_distribution[local_proc_index+1];
        idxtype num_local_elements = last_plus_one_element - first_local_element;

        boost::scoped_array<idxtype> eind(new idxtype[num_local_elements*(ELEMENT_DIM+1)]);
        boost::scoped_array<idxtype> eptr(new idxtype[num_local_elements+1]);
        unsigned counter = 0;
        for (idxtype element_index = 0; element_index < num_local_elements; element_index++)
        {
            eptr[element_index] = counter;
            for (unsigned i=0; i<ELEMENT_DIM+1; i++)
            {
                eind[counter++] = this->GetElement(element_index+first_local_element)->GetNode(i)->GetIndex();
            }
        }
        eptr[num_local_elements] = counter;

        // Get the graph
        idxtype numflag = 0; // index from 0
        idxtype ncommonnodes = 2;
        if(ELEMENT_DIM==3)
        {
            ncommonnodes = 3;
        }
        MPI_Comm communicator = PETSC_COMM_WORLD;

        idxtype* xadj;
        idxtype* adjncy;
        ParMETIS_V3_Mesh2Dual(element_distribution.get(), eptr.get(), eind.get(),
                              &numflag, &ncommonnodes, &xadj, &adjncy, &communicator);

        // Get rid of (maybe large) arrays as soon as they're no longer needed, rather than at end of scope
        eind.reset();
        eptr.reset();

        // Get the element distribution
        idxtype weight_flag = 0; // unweighted graph
        idxtype n_constraints = 1; // number of weights that each vertex has (number of balance constraints)
        idxtype n_subdomains = PetscTools::GetNumProcs();
        idxtype options[3]; // extra options
        options[0] = 0; // ignore extra options
        idxtype edgecut;
        boost::scoped_array<real_t> tpwgts(new real_t[n_subdomains]);
        real_t ubvec_value = (real_t)1.05;
        for (unsigned proc=0; proc<PetscTools::GetNumProcs(); proc++)
        {
            tpwgts[proc] = ((real_t)1.0)/n_subdomains;
        }

        boost::scoped_array<idxtype> local_partition(new idxtype[num_local_elements]);

        ParMETIS_V3_PartKway(element_distribution.get(), xadj, adjncy, NULL, NULL, &weight_flag, &numflag,
                             &n_constraints, &n_subdomains, tpwgts.get(), &ubvec_value,
                             options, &edgecut, local_partition.get(), &communicator);
        tpwgts.reset();

        boost::scoped_array<idxtype> global_element_partition(new idxtype[num_elements]);

        //idxtype is normally int (see metis-4.0/Lib/struct.h 17-22) but is 64bit on Windows
        MPI_Datatype mpi_idxtype = MPI_LONG_LONG_INT;
        if (sizeof(idxtype) == sizeof(int))
        {
            mpi_idxtype = MPI_INT;
        }

        boost::scoped_array<int> int_element_distribution(new int[num_procs+1]);
        for (unsigned i=0; i<num_procs+1; ++i)
        {
            int_element_distribution[i] = element_distribution[i];
        }

        MPI_Allgatherv(local_partition.get(), num_local_elements, mpi_idxtype,
                       global_element_partition.get(), element_counts.get(),
                       int_element_distribution.get(), mpi_idxtype, PETSC_COMM_WORLD);
        local_partition.reset();

        // This defeats the original purpose of the scoped array for global element dist, but use it for
        // now.
        std::vector<unsigned> element_partitioning = std::vector<unsigned>(num_elements);
        for (unsigned elem_index=0; elem_index<num_elements; elem_index++)
        {
            element_partitioning[elem_index] = global_element_partition[elem_index];
        }

        free(xadj);
        free(adjncy);
        PetscTools::Barrier();
        return element_partitioning;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::SetUpVtkGrid()
{
    vtkSmartPointer<vtkUnstructuredGrid> p_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();

    unsigned num_nodes = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNumNodes();
    p_vtk_points->SetNumberOfPoints(num_nodes);
    for(unsigned idx=0; idx<num_nodes; idx++)
    {
        c_vector<double, SPACE_DIM> loc = AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>::GetNode(idx)->rGetLocation();
        if(SPACE_DIM==3)
        {
            p_vtk_points->InsertPoint(idx, &loc[0]);
        }
        else
        {
            p_vtk_points->InsertPoint(idx, loc[0], loc[1], 0.0);
        }
    }
    p_grid->SetPoints(p_vtk_points);

    // Add vtk tets or triangles
    std::vector<std::vector<unsigned> > element_connectivity =  GetConnectivity();
    unsigned num_elements = element_connectivity.size();
    p_grid->Allocate(num_elements, num_elements);

    vtkSmartPointer<vtkIntArray> p_element_attributes = vtkSmartPointer<vtkIntArray>::New();
    p_element_attributes->SetName("Region Markers");

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
            p_grid->InsertNextCell(p_vtk_element->GetCellType(), p_vtk_element->GetPointIds());

            if(this->GetElement(idx)->GetNumElementAttributes()>0)
            {
                p_element_attributes->InsertNextTuple1(this->GetElement(idx)->rGetElementAttributes()[0]);
            }
            else
            {
                p_element_attributes->InsertNextTuple1(0.0);
            }
        }
        else
        {
            vtkSmartPointer<vtkTriangle> p_vtk_element = vtkSmartPointer<vtkTriangle>::New();
            unsigned num_nodes = element_connectivity[idx].size();
            for(unsigned jdx=0; jdx<num_nodes; jdx++)
            {
                p_vtk_element->GetPointIds()->SetId(jdx, element_connectivity[idx][jdx]);
            }
            p_grid->InsertNextCell(p_vtk_element->GetCellType(), p_vtk_element->GetPointIds());

            if(this->GetElement(idx)->GetNumElementAttributes()>0)
            {
                p_element_attributes->InsertNextTuple1(this->GetElement(idx)->rGetElementAttributes()[0]);
            }
            else
            {
                p_element_attributes->InsertNextTuple1(0.0);
            }
        }
    }

    // Assign the processor number to the cells
    std::vector<unsigned> element_partitioning = GetElementPartitioning();
    vtkSmartPointer<vtkIntArray> p_cell_data = vtkSmartPointer<vtkIntArray>::New();
    p_cell_data->SetName("Processor Num");

    vtkSmartPointer<vtkIntArray> p_global_num_data = vtkSmartPointer<vtkIntArray>::New();
    p_global_num_data->SetName("Global Num");
    for(unsigned idx=0;idx<element_partitioning.size(); idx++)
    {
        p_global_num_data->InsertNextTuple1(idx);
        p_cell_data->InsertNextTuple1(element_partitioning[idx]);
    }
    p_grid->GetCellData()->AddArray(p_cell_data);
    p_grid->GetCellData()->AddArray(p_global_num_data);
    p_grid->GetCellData()->AddArray(p_element_attributes);

    // Set up the local grid
    if(PetscTools::IsSequential())
    {
        this->mpGlobalVtkGrid = p_grid;
        this->mpVtkGrid = p_grid;
    }
    else
    {
        unsigned local_proc_index = PetscTools::GetMyRank();
        vtkSmartPointer<vtkThreshold> p_threshold = vtkSmartPointer<vtkThreshold>::New();
        #if VTK_MAJOR_VERSION <= 5
        p_threshold->SetInput(p_grid);
        #else
        p_threshold->SetInputData(p_grid);
        #endif
        p_threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "Processor Num");
        p_threshold->ThresholdBetween(local_proc_index, local_proc_index);
        p_threshold->Update();
        this->mpGlobalVtkGrid = p_grid;
        this->mpVtkGrid = p_threshold->GetOutput();
    }

    // Get the local to global map
    this->mLocalGlobalMap.clear();
    for(unsigned idx=0;idx<this->mpVtkGrid->GetNumberOfCells(); idx++)
    {
        this->mLocalGlobalMap.push_back(this->mpVtkGrid->GetCellData()->GetArray("Global Num")->GetTuple1(idx));
        if(this->mpVtkGrid->GetCellData()->GetArray("Global Num")->GetTuple1(idx) >= this->GetNumElements())
        {
            EXCEPTION("Element indexes not in expected order");
        }
    }

    // Update the locations with element centroids
    this->mpCellLocations = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0; idx<this->mpVtkGrid->GetNumberOfCells(); idx++)
    {
        c_vector<double, SPACE_DIM> loc = this->GetElement(this->mLocalGlobalMap[idx])->CalculateCentroid();
        if(SPACE_DIM==3)
        {
            this->mpCellLocations->InsertNextPoint(&loc[0]);
        }
        else
        {
            this->mpCellLocations->InsertNextPoint(loc[0], loc[1], 0.0);
        }
    }

    // Update the node locations
    this->mpPointLocations = vtkSmartPointer<vtkPoints>::New();
    typename DiscreteContinuumMesh<SPACE_DIM, SPACE_DIM>::NodeIterator iter = this->GetNodeIteratorBegin();
    while (iter != this->GetNodeIteratorEnd())
     {
        c_vector<double, SPACE_DIM> loc = (*iter).GetPoint().rGetLocation();
        if(SPACE_DIM==3)
        {
            this->mpPointLocations->InsertNextPoint(&loc[0]);
        }
        else
        {
            this->mpPointLocations->InsertNextPoint(loc[0], loc[1], 0.0);
        }
         ++iter;
     }

    this->mVtkRepresentationUpToDate = true;
    this->SetUpVtkCellLocator();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DimensionalChastePoint<SPACE_DIM> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetGlobalCellLocation(unsigned index)
{
    return DimensionalChastePoint<SPACE_DIM>(this->GetElement(index)->CalculateCentroid(), this->GetReferenceLengthScale());
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
const std::vector<double>& DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::rGetCellVolumes(bool update, bool jiggle)
{
    if(!update)
    {
        return this->mCellVolumes;
    }
    if(this->mCellVolumes.size()!=this->GetNumberOfCells())
    {
        this->mCellVolumes.clear();
        for(unsigned idx=0; idx<this->GetNumberOfCells(); idx++)
        {
            double determinant = 0.0;
            c_matrix<double, ELEMENT_DIM, SPACE_DIM> jacobian;
            this->GetElement(this->mLocalGlobalMap[idx])->CalculateJacobian(jacobian, determinant);
            this->mCellVolumes.push_back(this->GetElement(this->mLocalGlobalMap[idx])->GetVolume(determinant));
        }
    }
    return this->mCellVolumes;
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
vtkSmartPointer<vtkPoints> DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::GetPointLocations()
{
    return this->mpPointLocations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM>::ImportDiscreteContinuumMeshFromTetgen(tetgen::tetgenio& mesherOutput, unsigned numberOfElements,
                                                          int *elementList, unsigned numberOfFaces, int *faceList,
                                                          int *edgeMarkerList, int* triFaceMarkerList,
                                                          unsigned numberoftetrahedronattributes,
                                                          double *tetrahedronattributelist)
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
            if(numberoftetrahedronattributes>0 and tetrahedronattributelist!=NULL)
            {
                p_element->AddElementAttribute(tetrahedronattributelist[numberoftetrahedronattributes*real_element_index]);
            }

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
                if(triFaceMarkerList != NULL)
                {
                    p_b_element->AddElementAttribute(triFaceMarkerList[next_boundary_element_index]);
                }
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

// Explicit instantiation
template class DiscreteContinuumMesh<2> ;
template class DiscreteContinuumMesh<3> ;
