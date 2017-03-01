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

#include <cmath>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include <vtkDoubleArray.h>
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
        mSpacing(BaseUnits::Instance()->GetReferenceLengthScale()),
        mDimensions(scalar_vector<unsigned>(3, 1)),
        mExtents(zero_vector<double>(6)),
        mOrigin(DimensionalChastePoint<DIM>(zero_vector<double>(DIM), BaseUnits::Instance()->GetReferenceLengthScale())),
        mpGlobalVtkGrid(),
        mpLocalVtkGrid(),
        mVtkGridIsSetUp(false),
        mNeighbourData(),
        mPointVolumes(),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
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
    mNeighbourData = std::vector<std::vector<unsigned> >(GetNumberOfGlobalPoints());
    for (unsigned kdx = 0; kdx < mDimensions[2]; kdx++)
    {
        for (unsigned jdx = 0; jdx < mDimensions[1]; jdx++)
        {
            for (unsigned idx = 0; idx < mDimensions[0]; idx++)
            {
                unsigned index = GetGlobal1dGridIndex(idx, jdx, kdx);
                if (idx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx - 1, jdx, kdx));
                }
                if (idx < mDimensions[0] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx + 1, jdx, kdx));
                }
                if (jdx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx - 1, kdx));
                }
                if (jdx < mDimensions[1] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx + 1, kdx));
                }
                if (kdx > 0)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx, kdx - 1));
                }
                if (kdx < mDimensions[2] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx, kdx + 1));
                }
            }
        }
    }
}

template<unsigned DIM>
void RegularGrid<DIM>::CalculateMooreNeighbourData()
{
    mNeighbourData = std::vector<std::vector<unsigned> >(GetNumberOfGlobalPoints());
    for (unsigned kdx = 0; kdx < mDimensions[2]; kdx++)
    {
        for (unsigned jdx = 0; jdx < mDimensions[1]; jdx++)
        {
            for (unsigned idx = 0; idx < mDimensions[0]; idx++)
            {
                unsigned index = GetGlobal1dGridIndex(idx, jdx, kdx);

                // i-1 plane
                if (idx > 0)
                {
                    if(jdx>0)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx-1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx-1, kdx));
                        if(kdx<mExtents[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx-1, kdx+1));
                        }
                    }
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx, kdx));
                    if(kdx<mExtents[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx, kdx+1));
                    }
                    if(jdx< mExtents[1] - 1)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx+1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx+1, kdx));
                        if(kdx<mExtents[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx-1, jdx+1, kdx+1));
                        }
                    }
                }

                // i plane
                if(jdx>0)
                {
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx-1, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx-1, kdx));
                    if(kdx<mExtents[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx-1, kdx+1));
                    }
                }
                if(kdx>0)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx, kdx-1));
                }
                if(kdx<mExtents[2] - 1)
                {
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx, kdx+1));
                }
                if(jdx< mExtents[1] - 1)
                {
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx+1, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx+1, kdx));
                    if(kdx<mExtents[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx, jdx+1, kdx+1));
                    }
                }

                // i+1 plane
                if (idx < mExtents[0]-1)
                {
                    if(jdx>0)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx-1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx-1, kdx));
                        if(kdx<mExtents[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx-1, kdx+1));
                        }
                    }
                    if(kdx>0)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx, kdx-1));
                    }
                    mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx, kdx));
                    if(kdx<mExtents[2] - 1)
                    {
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx, kdx+1));
                    }
                    if(jdx< mExtents[1] - 1)
                    {
                        if(kdx>0)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx+1, kdx-1));
                        }
                        mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx+1, kdx));
                        if(kdx<mExtents[2] - 1)
                        {
                            mNeighbourData[index].push_back(GetGlobal1dGridIndex(idx+1, jdx+1, kdx+1));
                        }
                    }
                }
            }
        }
    }
}

template<unsigned DIM>
boost::shared_ptr<DistributedVectorFactory> RegularGrid<DIM>::GetDistributedVectorFactory()
{
    return mpDistributedVectorFactory;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetGlobal1dGridIndex(unsigned x_index, unsigned y_index, unsigned z_index)
{
    return x_index + mDimensions[0] * y_index + mDimensions[0] * mDimensions[1] * z_index;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetLocal1dGridIndex(unsigned x_index, unsigned y_index, unsigned z_index)
{
    if(x_index<mExtents[0] or x_index>=mExtents[1])
    {
        EXCEPTION("Requested local grid index not found on this processor.");
    }

    unsigned slice_width = 1 + mExtents[1] - mExtents[0];
    unsigned slice_depth = 1 + mExtents[3] - mExtents[2];
    return (x_index-mExtents[0]) + slice_width* y_index + slice_width * slice_depth * z_index;
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
const std::vector<std::vector<unsigned> >& RegularGrid<DIM>::GetNeighbourData()
{
    if (mNeighbourData.size() == 0 or mNeighbourData.size() != GetNumberOfGlobalPoints())
    {
        CalculateNeighbourData();
    }
    return mNeighbourData;
}

template<unsigned DIM>
const std::vector<std::vector<unsigned> >& RegularGrid<DIM>::GetMooreNeighbourData()
{
    if (mNeighbourData.size() == 0 or mNeighbourData.size() != GetNumberOfGlobalPoints())
    {
        CalculateMooreNeighbourData();
    }
    return mNeighbourData;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetNearestGlobalGridIndex(const DimensionalChastePoint<DIM>& rLocation)
{
    c_vector<double, DIM> offsets = (rLocation - mOrigin).GetLocation(mReferenceLength);

    double scale_factor = mReferenceLength / mSpacing;
    unsigned x_index = round(offsets[0]*scale_factor);
    unsigned y_index = round(offsets[1]*scale_factor);
    unsigned z_index = 0;
    if (DIM == 3)
    {
        z_index = round(offsets[2]*scale_factor);
    }
    return GetGlobal1dGridIndex(x_index, y_index, z_index);
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
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/mReferenceLength,
                spatial_extents[2]/mReferenceLength,
                spatial_extents[4]/mReferenceLength, mReferenceLength);
    }
    else
    {
        mDimensions[2] = 1;
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/mReferenceLength, spatial_extents[2]/mReferenceLength, 0.0, mReferenceLength);
    }
    UpdateExtents();
}

template<unsigned DIM>
units::quantity<unit::length> RegularGrid<DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned DIM>
std::vector<double> RegularGrid<DIM>::InterpolateGridValues(
        std::vector<DimensionalChastePoint<DIM> > locations, std::vector<double> values, bool useVtk)
{
    std::vector<double> sampled_values(locations.size(), 0.0);

    if (!mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }

    vtkSmartPointer<vtkDoubleArray> pPointData = vtkSmartPointer<vtkDoubleArray>::New();
    pPointData->SetNumberOfComponents(1);
    pPointData->SetNumberOfTuples(GetNumberOfGlobalPoints());
    const std::string ny_name = "test";
    pPointData->SetName(ny_name.c_str());
    for (unsigned idx = 0; idx < GetNumberOfGlobalPoints(); idx++)
    {
        pPointData->SetValue(idx, values[idx]);
    }
    mpGlobalVtkGrid->GetPointData()->AddArray(pPointData);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    p_points->SetNumberOfPoints(locations.size());
    for (unsigned idx = 0; idx < locations.size(); idx++)
    {
        c_vector<double, DIM> scaled_location = locations[idx].GetLocation(mReferenceLength);
        if (DIM == 3)
        {
            p_points->SetPoint(idx, scaled_location[0], scaled_location[1], scaled_location[2]);
        }
        else
        {
            p_points->SetPoint(idx, scaled_location[0], scaled_location[1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(mpVtkGrid);
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(mpGlobalVtkGrid);
    #endif
    p_probe_filter->Update();
    vtkSmartPointer<vtkPointData> p_point_data = p_probe_filter->GetPolyDataOutput()->GetPointData();

    unsigned num_points = p_point_data->GetArray("test")->GetNumberOfTuples();
    for (unsigned idx = 0; idx < num_points; idx++)
    {
        sampled_values[idx] = p_point_data->GetArray("test")->GetTuple1(idx);
    }
    return sampled_values;
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetLocation(unsigned x_index, unsigned y_index, unsigned z_index)
{
    double scale_factor = mSpacing/mReferenceLength;
    c_vector<double, DIM> dimensionless_origin = mOrigin.GetLocation(mReferenceLength);
    if(DIM == 2)
    {
        return DimensionalChastePoint<DIM>(double(x_index) * scale_factor + dimensionless_origin[0],
                                           double(y_index) * scale_factor + dimensionless_origin[1], 0.0, mReferenceLength);
    }
    else
    {
        return DimensionalChastePoint<DIM>(double(x_index) * scale_factor + dimensionless_origin[0],
                                                              double(y_index) * scale_factor + dimensionless_origin[1],
                                                              double(z_index) * scale_factor + dimensionless_origin[2], mReferenceLength);
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetLocationOfGlobal1dIndex(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (grid_index - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;
    unsigned dimless_spacing = mSpacing/mReferenceLength;
    c_vector<double, DIM> dimensionless_origin = mOrigin.GetLocation(mReferenceLength);
    if(DIM == 2)
    {
        return DimensionalChastePoint<DIM>(double(x_index) * dimless_spacing + dimensionless_origin[0],
                                                              double(y_index) * dimless_spacing + dimensionless_origin[1], 0.0, mReferenceLength);
    }
    else
    {
        return DimensionalChastePoint<DIM>(double(x_index) * dimless_spacing + dimensionless_origin[0],
                                                              double(y_index) * dimless_spacing + dimensionless_origin[1],
                                                              double(z_index) * dimless_spacing + dimensionless_origin[2], mReferenceLength);
    }
}

template<unsigned DIM>
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetLocationOfLocal1dIndex(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mExtents[0] * (mExtents[1]-1));
    unsigned z_index = (grid_index - mod_z) / (mExtents[0] * (mExtents[1]-1));
    unsigned mod_y = mod_z % mExtents[0];
    unsigned y_index = (mod_z - mod_y) / mExtents[0];
    unsigned x_index = mod_y;
    unsigned dimless_spacing = mSpacing/mReferenceLength;
    c_vector<double, DIM> dimensionless_origin = mOrigin.GetLocation(mReferenceLength);
    if(DIM == 2)
    {
        return DimensionalChastePoint<DIM>(double(x_index) * dimless_spacing + dimensionless_origin[0],
                                                              double(y_index) * dimless_spacing + dimensionless_origin[1], 0.0, mReferenceLength);
    }
    else
    {
        return DimensionalChastePoint<DIM>(double(x_index) * dimless_spacing + dimensionless_origin[0],
                                                              double(y_index) * dimless_spacing + dimensionless_origin[1],
                                                              double(z_index) * dimless_spacing + dimensionless_origin[2], mReferenceLength);
    }
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > RegularGrid<DIM>::GetGlobalLocations()
{
    std::vector<DimensionalChastePoint<DIM> > locations(GetNumberOfGlobalPoints());
    for (unsigned idx = 0; idx < GetNumberOfGlobalPoints(); idx++)
    {
        locations[idx] = GetLocationOfGlobal1dIndex(idx);
    }
    return locations;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > RegularGrid<DIM>::GetLocalLocations()
{
    std::vector<DimensionalChastePoint<DIM> > locations(GetNumberOfLocalPoints());
    for (unsigned idx = 0; idx < GetNumberOfLocalPoints(); idx++)
    {
        locations[idx] = GetLocationOfLocal1dIndex(idx);
    }
    return locations;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetNumberOfGlobalPoints()
{
    return mDimensions[0] * mDimensions[1] * mDimensions[2];
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetNumberOfLocalPoints()
{
    return (1+ mExtents[1]- mExtents[0])*
            (1+ mExtents[3]- mExtents[2])*
            (1 + mExtents[5]- mExtents[4]);
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
std::vector<double> RegularGrid<DIM>::GetPointVolumes(bool update, bool jiggle)
{
    unsigned num_points = GetNumberOfGlobalPoints();
    if(update)
    {
        mPointVolumes.clear();
        for (unsigned idx=0;idx<num_points; idx++)
        {
            c_vector<double,6> bbox = GetPointBoundingBox(idx, jiggle);
            double area = (bbox[3]-bbox[2])*(bbox[1]-bbox[0]);
            if(DIM==2)
            {
                mPointVolumes.push_back(area);
            }
            else
            {
                mPointVolumes.push_back(area*(bbox[5]-bbox[4]));
            }
        }
    }
    return mPointVolumes;
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsGlobalIndexOnBoundary(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (grid_index - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;
    return IsOnBoundary(x_index, y_index, z_index);
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsLocalIndexOnBoundary(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mExtents[0] * (mExtents[1]-1));
    unsigned z_index = (grid_index - mod_z) / (mExtents[0] * (mExtents[1]-1));
    unsigned mod_y = mod_z % (mExtents[0]-1);
    unsigned y_index = (mod_z - mod_y) / (mExtents[0]-1);
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
    unsigned mod_z = gridIndex % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (gridIndex - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;
    return GetPointBoundingBox(x_index, y_index, z_index, jiggle);
}

template<unsigned DIM>
c_vector<double,6> RegularGrid<DIM>::GetPointBoundingBox(unsigned xIndex, unsigned yIndex, unsigned zIndex, bool jiggle)
{
    units::quantity<unit::length> scale_factor = GetReferenceLengthScale();
    double dimensionless_spacing = GetSpacing()/scale_factor;
    double jiggle_size = 1.e-3 * dimensionless_spacing;

    c_vector<double, 3> dimensionless_location;
    dimensionless_location[0] = GetLocation(xIndex ,yIndex, zIndex).GetLocation(scale_factor)[0];
    dimensionless_location[1] = GetLocation(xIndex ,yIndex, zIndex).GetLocation(scale_factor)[1];
    if(DIM==3)
    {
        dimensionless_location[2] = GetLocation(xIndex ,yIndex, zIndex).GetLocation(scale_factor)[2];
    }
    else
    {
        dimensionless_location[2] = 0.0;
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
    dimensionless_bounds[0] = dimensionless_location[0] - x_min_offset;
    dimensionless_bounds[1] = dimensionless_location[0] + x_max_offset;
    dimensionless_bounds[2] = dimensionless_location[1] - y_min_offset;
    dimensionless_bounds[3] = dimensionless_location[1] + y_max_offset;
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

        dimensionless_bounds[4] = dimensionless_location[2] - z_min_offset;
        dimensionless_bounds[5] = dimensionless_location[2] + z_max_offset;
    }
    else
    {
        dimensionless_bounds[4] = -1.0;
        dimensionless_bounds[5] = 1.0;
    }
    return dimensionless_bounds;
}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> RegularGrid<DIM>::GetGlobalVtkGrid()
{
    if (!mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }
    return mpGlobalVtkGrid;
}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> RegularGrid<DIM>::GetLocalVtkGrid()
{
    if (!mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }
    return mpLocalVtkGrid;
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsLocationInPointVolume(DimensionalChastePoint<DIM> point, unsigned gridIndex)
{
    bool point_in_box = false;
    unsigned mod_z = gridIndex % (mDimensions[0] * mDimensions[1]);
    unsigned z_index = (gridIndex - mod_z) / (mDimensions[0] * mDimensions[1]);
    unsigned mod_y = mod_z % mDimensions[0];
    unsigned y_index = (mod_z - mod_y) / mDimensions[0];
    unsigned x_index = mod_y;

    double dimensionless_spacing = mSpacing/mReferenceLength;
    c_vector<double, DIM> dimensionless_origin = mOrigin.GetLocation(mReferenceLength);

    double loc_x = (double(x_index) * dimensionless_spacing + dimensionless_origin[0]);
    double loc_y = (double(y_index) * dimensionless_spacing + dimensionless_origin[1]);
    double loc_z = 0.0;
    if (DIM == 3)
    {
        loc_z = (double(z_index) * dimensionless_spacing + dimensionless_origin[2]);
    }

    c_vector<double, DIM> dimensionless_point = point.GetLocation(mReferenceLength);
    if (dimensionless_point[0] >= loc_x - dimensionless_spacing / 2.0 && dimensionless_point[0] <= loc_x + dimensionless_spacing / 2.0)
    {
        if (dimensionless_point[1] >= loc_y - dimensionless_spacing / 2.0 && dimensionless_point[1] <= loc_y +dimensionless_spacing / 2.0)
        {
            if (DIM == 3)
            {
                if (dimensionless_point[2] >= loc_z - dimensionless_spacing / 2.0 && dimensionless_point[2] <= loc_z + dimensionless_spacing / 2.0)
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        }
    }
    return point_in_box;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetOrigin(DimensionalChastePoint<DIM> origin)
{
    mOrigin = origin;
    if(mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }
}

template<unsigned DIM>
void RegularGrid<DIM>::SetSpacing(units::quantity<unit::length> spacing)
{
    mSpacing = spacing;
    GetPointVolumes(true, false);

    if(mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }
}

template<unsigned DIM>
void RegularGrid<DIM>::SetDimensions(c_vector<unsigned, 3> dimensions)
{
    mDimensions = dimensions;
    UpdateExtents();

    GetPointVolumes(true, false);
    if(mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }
}

template<unsigned DIM>
void RegularGrid<DIM>::SetUpVtkGrid()
{
    // Set up a VTK grid
    mpGlobalVtkGrid = vtkSmartPointer<vtkImageData>::New();
    mpLocalVtkGrid = vtkSmartPointer<vtkImageData>::New();
    mpGlobalVtkGrid->SetDimensions(mDimensions[0], mDimensions[1], mDimensions[2]);
    mpLocalVtkGrid->SetExtent(mExtents[0], mExtents[1], mExtents[2], mExtents[3], mExtents[4], mExtents[5]);
    mpGlobalVtkGrid->SetSpacing(mSpacing/mReferenceLength, mSpacing/mReferenceLength, mSpacing/mReferenceLength);
    mpLocalVtkGrid->SetSpacing(mSpacing/mReferenceLength, mSpacing/mReferenceLength, mSpacing/mReferenceLength);

    c_vector<double, DIM> dimless_origin = mOrigin.GetLocation(mReferenceLength);
    if (DIM == 3)
    {
        mpGlobalVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], dimless_origin[2]);
        mpLocalVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], dimless_origin[2]);
    }
    else
    {
        mpGlobalVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], 0.0);
        mpLocalVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], 0.0);
    }
    mVtkGridIsSetUp = true;
}

template<unsigned DIM>
void RegularGrid<DIM>::UpdateExtents()
{
    // Set up the distributed vector factory and let PETSc decide on splitting
    mpDistributedVectorFactory =
            boost::shared_ptr<DistributedVectorFactory>(new DistributedVectorFactory(GetNumberOfGlobalPoints()));

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
}

template<unsigned DIM>
void RegularGrid<DIM>::Write(boost::shared_ptr<OutputFileHandler> pFileHandler)
{
    RegularGridWriter writer;
    writer.SetFilename(pFileHandler->GetOutputDirectoryFullPath() + "/grid.vti");
    writer.SetImage(GetGlobalVtkGrid());
    writer.Write();
}

// Explicit instantiation
template class RegularGrid<2> ;
template class RegularGrid<3> ;
