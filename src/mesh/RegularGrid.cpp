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

#include <cmath>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkProbeFilter.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkLine.h>
#include <boost/math/special_functions/round.hpp>
#include "PetscTools.hpp"
#include "Exception.hpp"
#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
RegularGrid<DIM>::RegularGrid() :
        AbstractDiscreteContinuumGrid<DIM, DIM>(),
        mSpacing(BaseUnits::Instance()->GetReferenceLengthScale()),
        mDimensions(scalar_vector<unsigned>(3, 1)),
        mExtents(zero_vector<unsigned>(6)),
        mOrigin(DimensionalChastePoint<DIM>(zero_vector<double>(DIM), BaseUnits::Instance()->GetReferenceLengthScale())),
        mNeighbourData(),
        mpDistributedVectorFactory()
{

}

template<unsigned DIM>
boost::shared_ptr<RegularGrid<DIM> > RegularGrid<DIM>::Create()
{
    typedef RegularGrid<DIM> Reg_Grid_Templated;
    MAKE_PTR(Reg_Grid_Templated, pSelf);
    return pSelf;
}

template<unsigned DIM>
RegularGrid<DIM>::~RegularGrid()
{

}

template<unsigned DIM>
void RegularGrid<DIM>::CalculateNeighbourData()
{
    unsigned num_points = this->GetNumberOfPoints();
    mNeighbourData = std::vector<std::vector<unsigned> >(num_points);

    for (unsigned kdx = mExtents[4]; kdx <= mExtents[5]; kdx++)
    {
        for (unsigned jdx = mExtents[2]; jdx <= mExtents[3]; jdx++)
        {
            for (unsigned idx = mExtents[0]; idx <= mExtents[1]; idx++)
            {
                unsigned index = GetGridIndex(idx, jdx, kdx);
                if (idx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx - 1, jdx, kdx));
                }
                if (idx < mDimensions[0] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx + 1, jdx, kdx));
                }
                if (jdx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx - 1, kdx));
                }
                if (jdx < mDimensions[1] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx + 1, kdx));
                }
                if (kdx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx, kdx - 1));
                }
                if (kdx < mDimensions[2] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx, kdx + 1));
                }
            }
        }
    }
}

template<unsigned DIM>
void RegularGrid<DIM>::CalculateMooreNeighbourData()
{
    unsigned num_points = this->GetNumberOfPoints();
    mNeighbourData = std::vector<std::vector<unsigned> >(num_points);
    for (unsigned kdx = mExtents[4]; kdx <= mExtents[5]; kdx++)
    {
        for (unsigned jdx = mExtents[2]; jdx <= mExtents[3]; jdx++)
        {
            for (unsigned idx = mExtents[0]; idx <= mExtents[1]; idx++)
            {
                unsigned index = GetGridIndex(idx, jdx, kdx);

                // i-1 plane
                if (idx > 0)
                {
                    if(jdx>0)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx-1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx-1, kdx));
                        if(kdx<mDimensions[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx-1, kdx+1));
                        }
                    }
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx, kdx));
                    if(kdx<mDimensions[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx, kdx+1));
                    }
                    if(jdx< mDimensions[1] - 1)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx+1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx+1, kdx));
                        if(kdx<mDimensions[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx-1, jdx+1, kdx+1));
                        }
                    }
                }

                // i plane
                if(jdx>0)
                {
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx-1, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx-1, kdx));
                    if(kdx<mDimensions[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx-1, kdx+1));
                    }
                }
                if(kdx>0)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx, kdx-1));
                }
                if(kdx<mDimensions[2] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx, kdx+1));
                }
                if(jdx< mDimensions[1] - 1)
                {
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx+1, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx+1, kdx));
                    if(kdx<mDimensions[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx, jdx+1, kdx+1));
                    }
                }

                // i+1 plane
                if (idx < mDimensions[0]-1)
                {
                    if(jdx>0)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx-1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx-1, kdx));
                        if(kdx<mDimensions[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx-1, kdx+1));
                        }
                    }
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx, kdx));
                    if(kdx<mDimensions[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx, kdx+1));
                    }
                    if(jdx< mDimensions[1] - 1)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx+1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx+1, kdx));
                        if(kdx<mDimensions[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobalGridIndex(idx+1, jdx+1, kdx+1));
                        }
                    }
                }
            }
        }
    }
}

template<unsigned DIM>
int RegularGrid<DIM>::GetLocalIndex(unsigned globalIndex)
{
    int local_index = -1;
    if(globalIndex >= mpDistributedVectorFactory->GetLow() and globalIndex < mpDistributedVectorFactory->GetHigh())
    {
        return globalIndex-mpDistributedVectorFactory->GetLow();
    }
    return local_index;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetGlobalCellLocation(unsigned index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    double* loc = this->mpGlobalVtkGrid->GetPoint(index);
    return DimensionalChastePoint<DIM>(loc[0], loc[1], loc[2], this->mReferenceLength);
}

template<unsigned DIM>
boost::shared_ptr<DistributedVectorFactory> RegularGrid<DIM>::GetDistributedVectorFactory()
{
    return mpDistributedVectorFactory;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetGlobalGridIndex(unsigned x_index, unsigned y_index, unsigned z_index)
{
    int locs[3];
    locs[0] = x_index;
    locs[1] = y_index;
    locs[2] = z_index;
    return vtkImageData::SafeDownCast(this->mpGlobalVtkGrid)->ComputePointId(locs);
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetGridIndex(unsigned x_index, unsigned y_index, unsigned z_index)
{
    int locs[3];
    locs[0] = x_index;
    locs[1] = y_index;
    locs[2] = z_index;
    return vtkImageData::SafeDownCast(this->mpVtkGrid)->ComputePointId(locs);
}

template<unsigned DIM>
c_vector<unsigned, 3> RegularGrid<DIM>::GetDimensions()
{
    return mDimensions;
}

template<unsigned DIM>
c_vector<unsigned, 6> RegularGrid<DIM>::GetExtents()
{
    return mExtents;
}

template<unsigned DIM>
const std::vector<std::vector<unsigned> >& RegularGrid<DIM>::rGetNeighbourData()
{
    if (mNeighbourData.size() == 0 or mNeighbourData.size() != this->GetNumberOfPoints())
    {
        CalculateNeighbourData();
    }
    return mNeighbourData;
}

template<unsigned DIM>
const std::vector<std::vector<unsigned> >& RegularGrid<DIM>::rGetMooreNeighbourData()
{
    if (mNeighbourData.size() == 0 or mNeighbourData.size() != this->GetNumberOfPoints())
    {
        CalculateMooreNeighbourData();
    }
    return mNeighbourData;
}

template<unsigned DIM>
void RegularGrid<DIM>::GenerateFromPart(boost::shared_ptr<Part<DIM> > pPart, units::quantity<unit::length> gridSize)
{
    mSpacing = gridSize;
    std::vector<units::quantity<unit::length> > spatial_extents = pPart->GetBoundingBox();
    double norm_x = (spatial_extents[1] - spatial_extents[0]) / gridSize;
    double norm_y = (spatial_extents[3] - spatial_extents[2]) / gridSize;
    mDimensions[0] = unsigned(boost::math::iround(norm_x))+1;
    mDimensions[1] = unsigned(boost::math::iround(norm_y))+1;
    if (DIM == 3)
    {
        double norm_z = (spatial_extents[5] - spatial_extents[4]) / gridSize;
        mDimensions[2] = unsigned(boost::math::iround(norm_z))+1;
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/this->mReferenceLength,
                spatial_extents[2]/this->mReferenceLength,
                spatial_extents[4]/this->mReferenceLength, this->mReferenceLength);
    }
    else
    {
        mDimensions[2] = 1;
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/this->mReferenceLength,
                spatial_extents[2]/this->mReferenceLength, 0.0, this->mReferenceLength);
    }
    UpdateExtents();
    SetUpVtkGrid();
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetPoint(unsigned x_index, unsigned y_index, unsigned z_index)
{
    if (!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }
    int locs[3];
    locs[0] = x_index;
    locs[1] = y_index;
    locs[2] = z_index;
    c_vector<double, 3> loc;
    this->mpGlobalVtkGrid->GetPoint(vtkImageData::SafeDownCast(this->mpGlobalVtkGrid)->ComputePointId(locs), &loc[0]);
    return DimensionalChastePoint<DIM>(loc[0], loc[1], loc[2], this->mReferenceLength);
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetOrigin()
{
    return mOrigin;
}

template<unsigned DIM>
units::quantity<unit::length> RegularGrid<DIM>::GetSpacing()
{
    return mSpacing;
}

template<unsigned DIM>
const std::vector<double>& RegularGrid<DIM>::rGetCellVolumes(bool update, bool jiggle)
{
    unsigned num_points = this->GetNumberOfPoints();
    if(update)
    {
        this->mCellVolumes.clear();
        for (unsigned idx=0;idx<num_points; idx++)
        {
            c_vector<double,6> bbox = GetPointBoundingBox(idx, jiggle);
            double area = (bbox[3]-bbox[2])*(bbox[1]-bbox[0]);
            if(DIM==2)
            {
                this->mCellVolumes.push_back(area);
            }
            else
            {
                this->mCellVolumes.push_back(area*(bbox[5]-bbox[4]));
            }
        }
    }
    return this->mCellVolumes;
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsOnBoundary(unsigned gridIndex)
{
    unsigned global_index = this->mLocalGlobalMap[gridIndex];
    unsigned mod_z = global_index % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (global_index - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;
    return IsOnBoundary(x_index, y_index, z_index);
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsOnBoundary(unsigned x_index, unsigned y_index, unsigned z_index)
{
    if (x_index == 0 || x_index == mDimensions[0] - 1)
    {
        return true;
    }
    if (y_index == 0 || y_index == mDimensions[1] - 1)
    {
        return true;
    }
    if (DIM == 3)
    {
        if (z_index == 0 || z_index == mDimensions[2] - 1)
        {
            return true;
        }
    }
    return false;
}

template<unsigned DIM>
c_vector<double,6> RegularGrid<DIM>::GetPointBoundingBox(unsigned gridIndex, bool jiggle)
{
    unsigned global_index = this->mLocalGlobalMap[gridIndex];
    unsigned mod_z = global_index % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (global_index - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;
    return GetPointBoundingBox(x_index, y_index, z_index, jiggle);
}

template<unsigned DIM>
c_vector<double,6> RegularGrid<DIM>::GetPointBoundingBox(unsigned xIndex, unsigned yIndex, unsigned zIndex, bool jiggle)
{
    units::quantity<unit::length> scale_factor = this->GetReferenceLengthScale();
    double dimensionless_spacing = GetSpacing()/scale_factor;
    double jiggle_size = 1.e-3 * dimensionless_spacing;

    c_vector<double, DIM> dimensionless_location = this->GetPoint(xIndex ,yIndex, zIndex).GetLocation(scale_factor);
    c_vector<double, 3> loc;

    if(DIM==2)
    {
        loc[0] = dimensionless_location[0];
        loc[1] = dimensionless_location[1];
        loc[2] = 0.0;
    }
    else
    {
        loc[0] = dimensionless_location[0];
        loc[1] = dimensionless_location[1];
        loc[2] = dimensionless_location[2];
    }

    double x_min_offset = dimensionless_spacing/2.0;
    double x_max_offset = dimensionless_spacing/2.0;
    if (jiggle)
    {
        x_min_offset -= jiggle_size;
        x_max_offset += jiggle_size;
    }

    double y_min_offset = dimensionless_spacing/2.0;
    double y_max_offset = dimensionless_spacing/2.0;
    if (jiggle)
    {
        y_min_offset -= jiggle_size;
        y_max_offset += jiggle_size;
    }

    if (xIndex == 0)
    {
        x_min_offset = 0.0;
    }
    if (xIndex == mDimensions[0] - 1)
    {
        x_max_offset = 0.0;
    }
    if (yIndex == 0)
    {
        y_min_offset = 0.0;
    }
    if (yIndex == mDimensions[1] - 1)
    {
        y_max_offset = 0.0;
    }

    c_vector<double,6> dimensionless_bounds;
    dimensionless_bounds[0] = loc[0] - x_min_offset;
    dimensionless_bounds[1] = loc[0] + x_max_offset;
    dimensionless_bounds[2] = loc[1] - y_min_offset;
    dimensionless_bounds[3] = loc[1] + y_max_offset;
    if(DIM==3)
    {
        double z_min_offset = dimensionless_spacing/2.0;
        double z_max_offset = dimensionless_spacing/2.0;
        if (jiggle)
        {
            z_min_offset -= jiggle_size;
            z_max_offset += jiggle_size;
        }
        if (zIndex == 0)
        {
            z_min_offset = 0.0;
        }
        if (zIndex == mDimensions[2] - 1)
        {
            z_max_offset = 0.0;
        }

        dimensionless_bounds[4] = loc[2] - z_min_offset;
        dimensionless_bounds[5] = loc[2] + z_max_offset;
    }
    else
    {
        dimensionless_bounds[4] = -1.0;
        dimensionless_bounds[5] = 1.0;
    }

    return dimensionless_bounds;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetOrigin(DimensionalChastePoint<DIM> origin)
{
    mOrigin = origin;
    this->mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetSpacing(units::quantity<unit::length> spacing)
{
    mSpacing = spacing;
    UpdateExtents();
    this->mVtkRepresentationUpToDate = false;
    this->rGetCellVolumes(true, false);
}

template<unsigned DIM>
void RegularGrid<DIM>::SetDimensions(c_vector<unsigned, 3> dimensions)
{
    mDimensions = dimensions;
    UpdateExtents();
    this->mVtkRepresentationUpToDate = false;
    this->rGetCellVolumes(true, false);

}

template<unsigned DIM>
void RegularGrid<DIM>::SetUpVtkGrid()
{
    // Set up a VTK grid
    vtkSmartPointer<vtkImageData> p_global_grid = vtkSmartPointer<vtkImageData>::New();
    vtkSmartPointer<vtkImageData> p_local_grid = vtkSmartPointer<vtkImageData>::New();
    p_global_grid->SetDimensions(mDimensions[0], mDimensions[1], mDimensions[2]);
    p_local_grid->SetExtent(mExtents[0], mExtents[1], mExtents[2], mExtents[3], mExtents[4], mExtents[5]);
    double dimensionless_spacing = mSpacing/this->mReferenceLength;
    p_global_grid->SetSpacing(dimensionless_spacing, dimensionless_spacing, dimensionless_spacing);
    p_local_grid->SetSpacing(dimensionless_spacing, dimensionless_spacing, dimensionless_spacing);
    c_vector<double, DIM> dimless_origin = mOrigin.GetLocation(this->mReferenceLength);
    if (DIM == 3)
    {
        p_global_grid->SetOrigin(&dimless_origin[0]);
        p_local_grid->SetOrigin(&dimless_origin[0]);
    }
    else
    {
        p_global_grid->SetOrigin(dimless_origin[0], dimless_origin[1], 0.0);
        p_local_grid->SetOrigin(dimless_origin[0], dimless_origin[1], 0.0);
    }
    this->mpGlobalVtkGrid = p_global_grid;
    this->mpVtkGrid = p_local_grid;
    // Label the grid partitioning
    std::vector<unsigned> global_lows = mpDistributedVectorFactory->rGetGlobalLows();
    vtkSmartPointer<vtkIntArray> p_point_data = vtkSmartPointer<vtkIntArray>::New();
    p_point_data->SetName("Processor Num");
    p_point_data->SetNumberOfTuples(this->mpGlobalVtkGrid->GetNumberOfPoints());
    for(unsigned idx=0;idx<this->mpGlobalVtkGrid->GetNumberOfPoints(); idx++)
    {
        if(global_lows.size()>1)
        {
            for(unsigned jdx=0;jdx<global_lows.size();jdx++)
            {
                if(jdx<global_lows.size()-1)
                {
                    if(idx>=global_lows[jdx] and idx<=global_lows[jdx+1]-1)
                    {
                        p_point_data->SetTuple1(idx, jdx);
                    }
                }
                else
                {
                    if(idx>=global_lows[jdx])
                    {
                        p_point_data->SetTuple1(idx, jdx);
                    }
                }
            }
        }
        else
        {
            p_point_data->SetTuple1(idx, 0);
        }
    }
    this->mpGlobalVtkGrid->GetPointData()->AddArray(p_point_data);

    // Set up the local global map and grid locations
    unsigned lo = mpDistributedVectorFactory->GetLow();
    this->mLocalGlobalMap.clear();
    this->mpCellLocations = vtkSmartPointer<vtkPoints>::New();
    this->mpPointLocations = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0;idx<this->mpVtkGrid->GetNumberOfPoints(); idx++)
    {
        this->mpPointLocations->InsertNextPoint(this->mpVtkGrid->GetPoint(idx));
        this->mpCellLocations->InsertNextPoint(this->mpVtkGrid->GetPoint(idx));
        this->mLocalGlobalMap.push_back(idx+lo);
    }
    this->mVtkRepresentationUpToDate = true;

    // Set up the cell locator
    this->SetUpVtkCellLocator();
}

template<unsigned DIM>
void RegularGrid<DIM>::UpdateExtents()
{
    // Simple geometric partitioning. If Z dim is 1 split over y, otherwise split over z.
    // We need to make sure that all partitions are 'block' shaped for use with the VTK
    // image data structure.
    unsigned num_procs = PetscTools::GetNumProcs();
    unsigned low_index;
    unsigned high_index;
    unsigned num_points = mDimensions[0] * mDimensions[1] * mDimensions[2];

    if(mDimensions[2]==1)
    {
        // Split over Y
        unsigned piece_size = mDimensions[1]/num_procs;
        if(piece_size==0)
        {
            EXCEPTION("2d Grids with Y-dimensions lower than the number of processors are not supported.");
        }

        unsigned local_piece_size = piece_size;
        if (PetscTools::AmTopMost())
        {
            local_piece_size = mDimensions[1] - piece_size*num_procs + piece_size;
        }
        low_index = PetscTools::GetMyRank()*(piece_size*mDimensions[0]);
        high_index = mDimensions[0] * local_piece_size + low_index;
    }
    else
    {
        unsigned piece_size = mDimensions[2]/num_procs;
        if(piece_size==0)
        {
            EXCEPTION("3D Grids with z-dimensions lower than the number of processors are not supported.");
        }

        unsigned local_piece_size = piece_size;
        if (PetscTools::AmTopMost())
        {
            local_piece_size = mDimensions[2] - piece_size*num_procs + piece_size;
        }
        low_index = PetscTools::GetMyRank()*piece_size*mDimensions[0]*mDimensions[1];
        high_index = mDimensions[0]*mDimensions[1] * local_piece_size + low_index;
    }

    // Set up the distributed vector factory and let PETSc decide on splitting
    mpDistributedVectorFactory =
            boost::shared_ptr<DistributedVectorFactory>(new DistributedVectorFactory(low_index, high_index, num_points));

    unsigned lo = mpDistributedVectorFactory->GetLow();
    unsigned hi = mpDistributedVectorFactory->GetHigh();
    unsigned mod_z = lo % (mDimensions[0] * mDimensions[1]);
    mExtents[4] = (lo - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    mExtents[2] = (mod_z - mod_y) / mDimensions[0];
    mExtents[0] = mod_y;

    mod_z = (hi-1) % (mDimensions[0] * mDimensions[1]);
    mExtents[5] = (hi - 1 - mod_z) / (mDimensions[0] * mDimensions[1]);
    mod_y = mod_z % mDimensions[0];
    mExtents[3] = (mod_z - mod_y) / mDimensions[0];
    mExtents[1] = mod_y;

    this->mVtkRepresentationUpToDate = false;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetUpVtkCellLocator()
{
    if(!this->mVtkRepresentationUpToDate)
    {
        SetUpVtkGrid();
    }

    // Put a VTK image data cell over each conventional grid point for easy use of locator methods
    vtkSmartPointer<vtkImageData> p_current_image = vtkImageData::SafeDownCast(this->mpVtkGrid);
    vtkSmartPointer<vtkImageData> p_temp_image = vtkSmartPointer<vtkImageData>::New();
    int* dimensions = p_current_image->GetDimensions();
    if(DIM==3)
    {
        p_temp_image->SetDimensions(dimensions[0]+1, dimensions[1]+1, dimensions[2]+1);
    }
    else
    {
        p_temp_image->SetDimensions(dimensions[0]+1, dimensions[1]+1, 1);
    }
    double spacing = p_current_image->GetSpacing()[0];
    p_temp_image->SetSpacing(p_current_image->GetSpacing());

    // Add a small offset to the origin to act as a 'jiggle' and force borderline cell identification
    // to bias the bottom, front, left cell. Otherwise multiple cells can be identified in maps if
    // vessels or points are equidistant between grid locations.
    double jiggle = 1.e-3*spacing;
    double* origin = p_current_image->GetOrigin();
    if(DIM==3)
    {
        p_temp_image->SetOrigin(origin[0]-spacing/2.0+jiggle,origin[1]-spacing/2.0+jiggle,origin[2]-spacing/2.0+jiggle);
    }
    else
    {
        p_temp_image->SetOrigin(origin[0]-spacing/2.0+jiggle,origin[1]-spacing/2.0+jiggle, 0.0);
    }

    this->mpVtkCellLocator = vtkSmartPointer<vtkCellLocator>::New();
    this->mpVtkCellLocator->SetDataSet(p_temp_image);
    this->mpVtkCellLocator->BuildLocator();
}

template<unsigned DIM>
void RegularGrid<DIM>::Write(boost::shared_ptr<OutputFileHandler> pFileHandler)
{
    // Write the global grid. First everyone adds their point values to the global grid.
    this->GatherAllPointData();

    if(PetscTools::AmMaster())
    {
        RegularGridWriter writer;
        writer.SetFilename(pFileHandler->GetOutputDirectoryFullPath() + "/grid.vti");
        writer.SetImage(vtkImageData::SafeDownCast(this->GetGlobalVtkGrid()));
        writer.Write();
    }
}

// Explicit instantiation
template class RegularGrid<2> ;
template class RegularGrid<3> ;
