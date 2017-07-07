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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkDataSet.h>
#include <vtkCellLocator.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkArrayData.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkFeatureEdges.h>
#include <vtkGeometryFilter.h>
#include <vtkVersion.h>
#include <vtkMPIController.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include "PetscTools.hpp"
#include "Exception.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumGrid() :
    mCellVolumes(),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
    mpGlobalVtkGrid(),
    mpVtkGrid(),
    mpPointLocations(),
    mpCellLocations(),
    mpVtkCellLocator(vtkSmartPointer<vtkCellLocator>::New()),
    mVtkRepresentationUpToDate(false),
    mLocalGlobalMap(),
    mAttributeKeys()

{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumGrid()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddPointAttributes(const std::vector<unsigned>& rAttributes,
        const std::string& rName)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    vtkSmartPointer<vtkUnsignedIntArray> p_point_data = vtkSmartPointer<vtkUnsignedIntArray>::New();
    p_point_data->SetNumberOfTuples(rAttributes.size());
    p_point_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rAttributes.size(); idx++)
    {
        p_point_data->SetTuple1(idx, rAttributes[idx]);
    }

    mpVtkGrid->GetPointData()->AddArray(p_point_data);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddCellAttributes(const std::vector<unsigned>& rAttributes,
        const std::string& rName)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    vtkSmartPointer<vtkUnsignedIntArray> p_cell_data = vtkSmartPointer<vtkUnsignedIntArray>::New();
    p_cell_data->SetNumberOfTuples(rAttributes.size());
    p_cell_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rAttributes.size(); idx++)
    {
        p_cell_data->SetTuple1(idx, rAttributes[idx]);
    }
    mpVtkGrid->GetCellData()->AddArray(p_cell_data);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddPointData(const std::vector<double>& rNodalValues,
        const std::string& rName)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    vtkSmartPointer<vtkDoubleArray> p_point_data = vtkSmartPointer<vtkDoubleArray>::New();
    p_point_data->SetNumberOfTuples(rNodalValues.size());
    p_point_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rNodalValues.size(); idx++)
    {
        p_point_data->SetTuple1(idx, rNodalValues[idx]);
    }
    mpVtkGrid->GetPointData()->AddArray(p_point_data);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddCellData(const std::vector<double>& rNodalValues,
        const std::string& rName)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    vtkSmartPointer<vtkDoubleArray> p_cell_data = vtkSmartPointer<vtkDoubleArray>::New();
    p_cell_data->SetNumberOfTuples(rNodalValues.size());
    p_cell_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rNodalValues.size(); idx++)
    {
        p_cell_data->SetTuple1(idx, rNodalValues[idx]);
    }
    mpVtkGrid->GetCellData()->AddArray(p_cell_data);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
std::map<unsigned, std::string> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetAttributesKeys()
{
    return mAttributeKeys;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPolyData> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetBoundingGeometry()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
    p_geom_filter->SetInput(mpVtkGrid);
    #else
    p_geom_filter->SetInputData(mpVtkGrid);
    #endif
    p_geom_filter->Update();
    return p_geom_filter->GetOutput();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPointData> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetPointData()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpVtkGrid->GetPointData();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkCellData> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetCellData()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpVtkGrid->GetCellData();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkDataSet> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::CalculateDistanceMap(
        std::shared_ptr<Part<SPACE_DIM> > pSamplePart)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    // Get the outer boundary
    vtkSmartPointer<vtkPolyData> p_bounds = vtkSmartPointer<vtkPolyData>::New();
    if(SPACE_DIM==2)
    {
        vtkSmartPointer<vtkFeatureEdges> p_edges = vtkSmartPointer<vtkFeatureEdges>::New();
		#if VTK_MAJOR_VERSION <= 5
        	p_edges->SetInput(pSamplePart->GetVtk());
		#else
	        p_edges->SetInputData(pSamplePart->GetVtk());
		#endif
        p_edges->Update();
        p_bounds = p_edges->GetOutput();
    }
    else
    {
        vtkSmartPointer<vtkDataSetSurfaceFilter> p_surf = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
		#if VTK_MAJOR_VERSION <= 5
        	p_surf->SetInput(pSamplePart->GetVtk());;
		#else
	        p_surf->SetInputData(pSamplePart->GetVtk());
		#endif
        p_surf->Update();
        p_bounds = p_surf->GetOutput();
    }

    vtkSmartPointer<vtkCellLocator> p_locator = vtkSmartPointer<vtkCellLocator>::New();
    p_locator->SetDataSet(p_bounds);
    p_locator->BuildLocator();

    vtkSmartPointer<vtkDoubleArray> p_distances = vtkSmartPointer<vtkDoubleArray>::New();
    p_distances->SetName("Distance");

    vtkSmartPointer<vtkGenericCell> p_generic_cell = vtkSmartPointer<vtkGenericCell>::New();
    for(unsigned idx=0;idx<mpVtkGrid->GetNumberOfPoints();idx++)
    {
        //Find the closest points to TestPoint
        double closestPoint[3];//the coordinates of the closest point will be returned here
        double closestPointDist2; //the squared distance to the closest point will be returned here
        vtkIdType cellId; //the cell id of the cell containing the closest point will be returned here
        int subId;
        p_locator->FindClosestPoint(mpVtkGrid->GetPoint(idx), closestPoint, p_generic_cell,
                cellId, subId, closestPointDist2);
        p_distances->InsertNextTuple1(std::sqrt(closestPointDist2));
    }
    mpVtkGrid->GetPointData()->AddArray(p_distances);

    return mpVtkGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetGlobalIndex(unsigned localIndex)
{
    if(localIndex>=mLocalGlobalMap.size())
    {
        EXCEPTION("Out of bounds local index requested");
    }
    return mLocalGlobalMap[localIndex];
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
int AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetLocalIndex(unsigned globalIndex)
{
    return globalIndex;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
QLength AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DimensionalChastePoint<SPACE_DIM> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetPoint(unsigned grid_index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    double* loc = mpVtkGrid->GetPoint(grid_index);
    return DimensionalChastePoint<SPACE_DIM>(loc[0], loc[1], loc[2], this->mReferenceLength);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPoints> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetPoints()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpPointLocations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DimensionalChastePoint<SPACE_DIM> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetGlobalPoint(unsigned grid_index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    c_vector<double, 3> loc;
    this->mpGlobalVtkGrid->GetPoint(grid_index, &loc[0]);
    return DimensionalChastePoint<SPACE_DIM>(loc[0], loc[1], loc[2], this->mReferenceLength);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DimensionalChastePoint<SPACE_DIM> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetCellLocation(unsigned grid_index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    double* loc = this->mpCellLocations->GetPoint(grid_index);
    return DimensionalChastePoint<SPACE_DIM>(loc[0], loc[1], loc[2], this->mReferenceLength);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPoints> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetCellLocations()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return this->mpCellLocations;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetNumberOfCells()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return this->mpCellLocations->GetNumberOfPoints();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetNumberOfPoints()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpVtkGrid->GetNumberOfPoints();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AllGatherPointData(const std::string& rName)
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetPointData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetPointData()->GetArray(idx);
        if(p_send_array->GetName()==rName)
        {
            vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
            p_receive_array->SetName(rName.c_str());
            p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
            this->GetGlobalVtkGrid()->GetPointData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AllGatherAllPointData()
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetPointData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetPointData()->GetArray(idx);
        vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
        p_receive_array->SetName(p_send_array->GetName());
        p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
        this->GetGlobalVtkGrid()->GetPointData()->AddArray(p_receive_array);
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GatherPointData(const std::string& rName)
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    for(int idx=0; idx<this->mpVtkGrid->GetPointData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetPointData()->GetArray(idx);
        if(p_send_array->GetName()==rName)
        {
            vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
            p_receive_array->SetName(p_send_array->GetName());
            p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
            this->GetGlobalVtkGrid()->GetPointData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GatherAllPointData()
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetPointData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetPointData()->GetArray(idx);
        vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
        p_receive_array->SetName(p_send_array->GetName());
        p_mpi_controller->GatherV(p_send_array, p_receive_array, PetscTools::MASTER_RANK);
        if(PetscTools::AmMaster())
        {
            this->GetGlobalVtkGrid()->GetPointData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AllGatherCellData(const std::string& rName)
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetCellData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetCellData()->GetArray(idx);
        if(p_send_array->GetName()==rName)
        {
            vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
            p_receive_array->SetName(rName.c_str());
            p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
            this->GetGlobalVtkGrid()->GetCellData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AllGatherAllCellData()
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetCellData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetCellData()->GetArray(idx);
        vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
        p_receive_array->SetName(p_send_array->GetName());
        p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
        this->GetGlobalVtkGrid()->GetCellData()->AddArray(p_receive_array);
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GatherCellData(const std::string& rName)
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    for(int idx=0; idx<this->mpVtkGrid->GetCellData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetCellData()->GetArray(idx);
        if(p_send_array->GetName()==rName)
        {
            vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
            p_receive_array->SetName(p_send_array->GetName());
            p_mpi_controller->AllGatherV(p_send_array, p_receive_array);
            this->GetGlobalVtkGrid()->GetCellData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GatherAllCellData()
{
    vtkSmartPointer<vtkMPIController> p_mpi_controller = vtkSmartPointer<vtkMPIController>::New();

    // Add each local data set to the global vtk before output
    for(int idx=0; idx<this->mpVtkGrid->GetCellData()->GetNumberOfArrays(); idx++)
    {
        vtkDataArray* p_send_array = this->mpVtkGrid->GetPointData()->GetArray(idx);
        vtkSmartPointer<vtkDoubleArray> p_receive_array = vtkSmartPointer<vtkDoubleArray>::New();
        p_receive_array->SetName(p_send_array->GetName());
        p_mpi_controller->GatherV(p_send_array, p_receive_array, PetscTools::MASTER_RANK);
        if(PetscTools::AmMaster())
        {
            this->GetGlobalVtkGrid()->GetCellData()->AddArray(p_receive_array);
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetNearestCellIndex(const DimensionalChastePoint<SPACE_DIM>& rLocation)
{
    c_vector<double, SPACE_DIM> loc = rLocation.GetLocation(mReferenceLength);
    c_vector<double, 3> loc_3d;

    if(SPACE_DIM==3)
    {
        loc_3d = loc;
    }
    else
    {
        loc_3d[0] = loc[0];
        loc_3d[1] = loc[1];
        loc_3d[2] = 0.0;
    }

    int cell_id = GetVtkCellLocator()->FindCell(&loc_3d[0]);
    if(cell_id<0)
    {
        EXCEPTION("Provided location is outside the grid");
    }
    return unsigned(cell_id);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkDataSet> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetGlobalVtkGrid()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpGlobalVtkGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkDataSet> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetVtkGrid()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpVtkGrid;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkCellLocator> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetVtkCellLocator()
{
    if(!mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return mpVtkCellLocator;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::SetAttributesKeys(std::map<unsigned, std::string> attributeKeys)
{
    mAttributeKeys = attributeKeys;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::SetUpVtkCellLocator()
{
    if(!mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    this->mpVtkCellLocator = vtkSmartPointer<vtkCellLocator>::New();
    this->mpVtkCellLocator->SetDataSet(this->mpVtkGrid);
    this->mpVtkCellLocator->BuildLocator();
}

// Explicit instantiation
template class AbstractDiscreteContinuumGrid<2> ;
template class AbstractDiscreteContinuumGrid<3> ;
