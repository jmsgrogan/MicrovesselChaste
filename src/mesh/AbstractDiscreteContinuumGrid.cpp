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

#include <vtkFeatureEdges.h>
#include <vtkGeometryFilter.h>
#include <vtkVersion.h>
#include "Exception.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"
#include "BaseUnits.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumGrid() :
    mAttributes(),
    mVolumes(),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
    mpGlobalVtkGrid(),
    mpVtkGrid(),
    mpVtkCellLocator(vtkSmartPointer<vtkCellLocator>::New()),
    mVtkRepresentationUpToDate(false),
    mPointData(),
    mLocalGlobalMap()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumGrid()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddAttributes(const std::vector<unsigned>& rAttributes,
        const std::string& rName)
{
    vtkSmartPointer<vtkUnsignedIntArray> p_point_data = vtkSmartPointer<vtkUnsignedIntArray>::New();
    p_point_data->SetNumberOfTuples(rAttributes.size());
    p_point_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rAttributes.size(); idx++)
    {
        p_point_data->SetTuple1(idx, rAttributes[idx]);
    }

    mAttributes.push_back(p_point_data);
    mVtkRepresentationUpToDate = false;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::AddPointData(const std::vector<double>& rNodalValues,
        bool clearExisting, const std::string& rName)
{
    vtkSmartPointer<vtkDoubleArray> p_point_data = vtkSmartPointer<vtkDoubleArray>::New();
    p_point_data->SetNumberOfTuples(rNodalValues.size());
    p_point_data->SetName(rName.c_str());
    for(unsigned idx=0; idx<rNodalValues.size(); idx++)
    {
        p_point_data->SetTuple1(idx, rNodalValues[idx]);
    }

    if(clearExisting)
    {
        mPointData.clear();
    }
    mPointData.push_back(p_point_data);
    mVtkRepresentationUpToDate = false;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
const std::vector<vtkSmartPointer<vtkUnsignedIntArray> >& AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::rGetLocationAttributes()
{
    return mAttributes;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPolyData> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetBoundingGeometry()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::length> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
DimensionalChastePoint<SPACE_DIM> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetLocation(unsigned grid_index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    c_vector<double, 3> loc;
    this->mpGridLocations()->GetPoint(grid_index, &loc);
    return DimensionalChastePoint<SPACE_DIM>(loc, this->mReferenceLength);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
vtkSmartPointer<vtkPoints> AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetLocations()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return this->mpGridLocations();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetNumberOfLocations()
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    return this->mpGridLocations()->GetNumberOfPoints();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
unsigned AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::GetNearestLocationIndex(const DimensionalChastePoint<SPACE_DIM>& rLocation)
{
    int cell_id = GetVtkCellLocator()->FindCell(rLocation.GetLocation(mReferenceLength));
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
virtual void AbstractDiscreteContinuumGrid<ELEMENT_DIM, SPACE_DIM>::SetUpVtkCellLocator()
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
