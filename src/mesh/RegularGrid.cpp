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
#include "RegularGrid.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkProbeFilter.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkLine.h>
#include "RegularGridWriter.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
RegularGrid<DIM>::RegularGrid() :
        mSpacing(BaseUnits::Instance()->GetReferenceLengthScale()),
        mExtents(std::vector<unsigned>(3, 10)),
        mOrigin(DimensionalChastePoint<DIM>(zero_vector<double>(DIM), BaseUnits::Instance()->GetReferenceLengthScale())),
        mpNetwork(),
        mpCellPopulation(),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mPointCellMap(),
        mPointNodeMap(),
        mPointSegmentMap(),
        mpVtkGrid(),
        mVtkGridIsSetUp(false),
        mNeighbourData(),
        mHasCellPopulation(false),
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
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
    mNeighbourData = std::vector<std::vector<unsigned> >(GetNumberOfPoints());
    for (unsigned kdx = 0; kdx < mExtents[2]; kdx++)
    {
        for (unsigned jdx = 0; jdx < mExtents[1]; jdx++)
        {
            for (unsigned idx = 0; idx < mExtents[0]; idx++)
            {
                unsigned index = Get1dGridIndex(idx, jdx, kdx);
                if (idx > 0)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx - 1, jdx, kdx));
                }
                if (idx < mExtents[0] - 1)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx + 1, jdx, kdx));
                }
                if (jdx > 0)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx, jdx - 1, kdx));
                }
                if (jdx < mExtents[1] - 1)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx, jdx + 1, kdx));
                }
                if (kdx > 0)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx, jdx, kdx - 1));
                }
                if (kdx < mExtents[2] - 1)
                {
                    mNeighbourData[index].push_back(Get1dGridIndex(idx, jdx, kdx + 1));
                }
            }
        }
    }
}

template<unsigned DIM>
const std::vector<std::vector<unsigned> >& RegularGrid<DIM>::GetNeighbourData()
{
    if (mNeighbourData.size() == 0 or mNeighbourData.size() != GetNumberOfPoints())
    {
        CalculateNeighbourData();
    }
    return mNeighbourData;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetNearestGridIndex(const DimensionalChastePoint<DIM>& rLocation)
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
    return Get1dGridIndex(x_index, y_index, z_index);
}

template<unsigned DIM>
void RegularGrid<DIM>::GenerateFromPart(boost::shared_ptr<Part<DIM> > pPart, units::quantity<unit::length> gridSize)
{
    mSpacing = gridSize;
    std::vector<units::quantity<unit::length> > spatial_extents = pPart->GetBoundingBox();
    mExtents[0] = unsigned((spatial_extents[1] - spatial_extents[0]) / gridSize) + 1u;
    mExtents[1] = unsigned((spatial_extents[3] - spatial_extents[2]) / gridSize) + 1u;
    if (DIM == 3)
    {
        mExtents[2] = unsigned((spatial_extents[5] - spatial_extents[4]) / gridSize) + 1u;
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/mReferenceLength, spatial_extents[2]/mReferenceLength, spatial_extents[4]/mReferenceLength, mReferenceLength);
    }
    else
    {
        mExtents[2] = 1;
        mOrigin = DimensionalChastePoint<DIM>(spatial_extents[0]/mReferenceLength, spatial_extents[2]/mReferenceLength, 0.0, mReferenceLength);
    }
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::Get1dGridIndex(unsigned x_index, unsigned y_index, unsigned z_index)
{
    return x_index + mExtents[0] * y_index + mExtents[0] * mExtents[1] * z_index;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetPointValues(std::vector<double> pointSolution)
{
    mPointSolution = pointSolution;
}

template<unsigned DIM>
units::quantity<unit::length> RegularGrid<DIM>::GetReferenceLengthScale()
{
    return mReferenceLength;
}

template<unsigned DIM>
std::vector<std::vector<unsigned> > RegularGrid<DIM>::GetPointPointMap(
        std::vector<DimensionalChastePoint<DIM> > inputPoints)
{
    double scale_factor = mReferenceLength / mSpacing;
    std::vector<std::vector<unsigned> > point_point_map(GetNumberOfPoints());
    for (unsigned idx = 0; idx < inputPoints.size(); idx++)
    {
        c_vector<double, DIM> offsets = (inputPoints[idx] - mOrigin).GetLocation(mReferenceLength);
        unsigned x_index = round(offsets[0]*scale_factor);
        unsigned y_index = round(offsets[1]*scale_factor);
        unsigned z_index = 0;
        if (DIM == 3)
        {
            z_index = round(offsets[2]*scale_factor);
        }

        if (x_index <= mExtents[0] && y_index <= mExtents[1] && z_index <= mExtents[2])
        {
            unsigned grid_index = x_index + y_index * mExtents[0] + z_index * mExtents[0] * mExtents[1];
            point_point_map[grid_index].push_back(idx);
        }
    }
    return point_point_map;
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
    pPointData->SetNumberOfTuples(GetNumberOfPoints());
    const std::string ny_name = "test";
    pPointData->SetName(ny_name.c_str());
    for (unsigned idx = 0; idx < GetNumberOfPoints(); idx++)
    {
        pPointData->SetValue(idx, values[idx]);
    }
    mpVtkGrid->GetPointData()->AddArray(pPointData);

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
        p_probe_filter->SetSourceData(mpVtkGrid);
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
const std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > >& RegularGrid<DIM>::GetPointNodeMap(bool update)
{
    if (!update)
    {
        return mPointNodeMap;
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a point node map.");
    }

    // Loop over all nodes and associate cells with the points
    mPointNodeMap.clear();
    for (unsigned idx = 0; idx < GetNumberOfPoints(); idx++)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > empty_node_pointers;
        mPointNodeMap.push_back(empty_node_pointers);
    }

    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
    double scale_factor = mReferenceLength / mSpacing;

    for (unsigned idx = 0; idx < nodes.size(); idx++)
    {
        c_vector<double, DIM> offsets = (nodes[idx]->rGetLocation()-mOrigin).GetLocation(mReferenceLength);
        unsigned x_index = round(offsets[0]*scale_factor);
        unsigned y_index = round(offsets[1]*scale_factor);
        unsigned z_index = 0;
        if (DIM == 3)
        {
            z_index = round(offsets[2]*scale_factor);
        }

        if (x_index <= mExtents[0] && y_index <= mExtents[1] && z_index <= mExtents[2])
        {
            unsigned grid_index = x_index + y_index * mExtents[0] + z_index * mExtents[0] * mExtents[1];
            mPointNodeMap[grid_index].push_back(nodes[idx]);
        }
    }
    return mPointNodeMap;
}

template<unsigned DIM>
const std::vector<std::vector<CellPtr> >& RegularGrid<DIM>::GetPointCellMap(bool update)
{
    if (!update)
    {
        return mPointCellMap;
    }

    if (!mpCellPopulation)
    {
        EXCEPTION("A cell population has not been set. Can not create a cell point map.");
    }

    // Loop over all cells and associate cells with the points
    double scale_factor = mReferenceLength / mSpacing;
    double cell_scale_factor = mCellPopulationReferenceLength / mReferenceLength;

    mPointCellMap.clear();
    for (unsigned idx = 0; idx < GetNumberOfPoints(); idx++)
    {
        std::vector<CellPtr> empty_cell_pointers;
        mPointCellMap.push_back(empty_cell_pointers);
    }


    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = mpCellPopulation->Begin();
            cell_iter != mpCellPopulation->End(); ++cell_iter)
    {
        c_vector<double, DIM> location = mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
        c_vector<double, DIM> offsets = location*cell_scale_factor - mOrigin.GetLocation(mReferenceLength);
        unsigned x_index = round(offsets[0]*scale_factor);
        unsigned y_index = round(offsets[1]*scale_factor);
        unsigned z_index = 0;
        if (DIM == 3)
        {
            z_index = round(offsets[2]*scale_factor);
        }

        if (x_index <= mExtents[0] && y_index <= mExtents[1] && z_index <= mExtents[2])
        {
            unsigned grid_index = x_index + y_index * mExtents[0] + z_index * mExtents[0] * mExtents[1];
            mPointCellMap[grid_index].push_back(*cell_iter);
        }

    }
    return mPointCellMap;
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsSegmentAtLatticeSite(unsigned index, bool update)
{
    if(update)
    {
        GetPointSegmentMap(update);
    }

    if(index>=mPointSegmentMap.size())
    {
        EXCEPTION("Out of range index requested for point-segment map");
    }
    return mPointSegmentMap[index].size()>0;
}

template<unsigned DIM>
std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > RegularGrid<DIM>::GetPointSegmentMap(
        bool update, bool useVesselSurface)
{
    if (!update)
    {
        return mPointSegmentMap;
    }

    if (!mpNetwork)
    {
        EXCEPTION("A vessel network has not been set. Can not create a vessel point map.");
    }

    // Loop over all points and segments and associate segments with the points
    mPointSegmentMap.clear();
    for (unsigned idx = 0; idx < GetNumberOfPoints(); idx++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > empty_seg_pointers;
        mPointSegmentMap.push_back(empty_seg_pointers);
    }

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mpNetwork->GetVesselSegments();
    unsigned num_points = GetNumberOfPoints();
    units::quantity<unit::length> cut_off_length = sqrt(1.0 / 2.0) * mSpacing;
    for (unsigned jdx = 0; jdx < segments.size(); jdx++)
    {
        double start_point[3];
        double end_point[3];
        c_vector<double,DIM> start_location = segments[jdx]->GetNode(0)->rGetLocation().GetLocation(mReferenceLength);
        start_point[0] = start_location[0];
        start_point[1] = start_location[1];
        if(DIM==3)
        {
            start_point[2] = start_location[2];
        }
        else
        {
            start_point[2] = 0.0;
        }
        c_vector<double,DIM> end_location = segments[jdx]->GetNode(1)->rGetLocation().GetLocation(mReferenceLength);
        end_point[0] = end_location[0];
        end_point[1] = end_location[1];
        if(DIM==3)
        {
            end_point[2] = end_location[2];
        }
        else
        {
            end_point[2] = 0.0;
        }

        for (unsigned idx = 0; idx < num_points; idx++)
        {
            if (!useVesselSurface)
            {
                double parametric_distance = 0.0;
                double closest_point[3];
                double grid_point[3];
                c_vector<double,DIM> grid_location = GetLocationOf1dIndex(idx).GetLocation(mReferenceLength);
                grid_point[0] = grid_location[0];
                grid_point[1] = grid_location[1];
                if(DIM==3)
                {
                    grid_point[2] = grid_location[2];
                }
                else
                {
                    grid_point[2] = 0.0;
                }

                double distance = vtkLine::DistanceToLine(grid_point,
                                                          start_point,
                                                          end_point,
                                                          parametric_distance, closest_point);
                if (distance*mReferenceLength < cut_off_length)
                {
                    mPointSegmentMap[idx].push_back(segments[jdx]);
                }
            }
            else
            {
                if (segments[jdx]->GetDistance(GetLocationOf1dIndex(idx))< (segments[jdx]->GetRadius() + cut_off_length))
                {
                    mPointSegmentMap[idx].push_back(segments[jdx]);
                }
            }
        }
    }

    return mPointSegmentMap;
}

template<unsigned DIM>
std::vector<unsigned> RegularGrid<DIM>::GetExtents()
{
    return mExtents;
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
DimensionalChastePoint<DIM> RegularGrid<DIM>::GetLocationOf1dIndex(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mExtents[0] * mExtents[1]);
    unsigned z_index = (grid_index - mod_z) / (mExtents[0] * mExtents[1]);
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
std::vector<DimensionalChastePoint<DIM> > RegularGrid<DIM>::GetLocations()
{
    std::vector<DimensionalChastePoint<DIM> > locations(GetNumberOfPoints());
    for (unsigned idx = 0; idx < GetNumberOfPoints(); idx++)
    {
        locations[idx] = GetLocationOf1dIndex(idx);
    }
    return locations;
}

template<unsigned DIM>
unsigned RegularGrid<DIM>::GetNumberOfPoints()
{
    return mExtents[0] * mExtents[1] * mExtents[2];
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
bool RegularGrid<DIM>::IsOnBoundary(unsigned grid_index)
{
    unsigned mod_z = grid_index % (mExtents[0] * mExtents[1]);
    unsigned z_index = (grid_index - mod_z) / (mExtents[0] * mExtents[1]);
    unsigned mod_y = mod_z % mExtents[0];
    unsigned y_index = (mod_z - mod_y) / mExtents[0];
    unsigned x_index = mod_y;
    return IsOnBoundary(x_index, y_index, z_index);
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsOnBoundary(unsigned x_index, unsigned y_index, unsigned z_index)
{
    if (x_index == 0 || x_index == mExtents[0] - 1)
    {
        return true;
    }
    if (y_index == 0 || y_index == mExtents[1] - 1)
    {
        return true;
    }
    if (DIM == 3)
    {
        if (z_index == 0 || z_index == mExtents[2] - 1)
        {
            return true;
        }
    }
    return false;
}

template<unsigned DIM>
bool RegularGrid<DIM>::IsLocationInPointVolume(DimensionalChastePoint<DIM> point, unsigned gridIndex)
{
    bool point_in_box = false;
    unsigned mod_z = gridIndex % (mExtents[0] * mExtents[1]);
    unsigned z_index = (gridIndex - mod_z) / (mExtents[0] * mExtents[1]);
    unsigned mod_y = mod_z % mExtents[0];
    unsigned y_index = (mod_z - mod_y) / mExtents[0];
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
void RegularGrid<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation, units::quantity<unit::length> cellPopulationLengthScale)
{
    mpCellPopulation = &rCellPopulation;
    mCellPopulationReferenceLength = cellPopulationLengthScale;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetExtents(std::vector<unsigned> extents)
{
    if (extents.size() < 3)
    {
        EXCEPTION("The extents should be of dimension 3, regardless of element or space dimension");
    }
    mExtents = extents;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetOrigin(DimensionalChastePoint<DIM> origin)
{
    mOrigin = origin;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetSpacing(units::quantity<unit::length> spacing)
{
    mSpacing = spacing;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetUpVtkGrid()
{
    // Set up a VTK grid
    mpVtkGrid = vtkSmartPointer<vtkImageData>::New();
    if (DIM == 3)
    {
        mpVtkGrid->SetDimensions(mExtents[0], mExtents[1], mExtents[2]);
    }
    else
    {
        mpVtkGrid->SetDimensions(mExtents[0], mExtents[1], 1);
    }
    mpVtkGrid->SetSpacing(mSpacing/mReferenceLength, mSpacing/mReferenceLength, mSpacing/mReferenceLength);

    c_vector<double, DIM> dimless_origin = mOrigin.GetLocation(mReferenceLength);
    if (DIM == 3)
    {
        mpVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], dimless_origin[2]);
    }
    else
    {
        mpVtkGrid->SetOrigin(dimless_origin[0], dimless_origin[1], 0.0);
    }
    mVtkGridIsSetUp = true;
}

template<unsigned DIM>
void RegularGrid<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> RegularGrid<DIM>::GetVtkGrid()
{
    if (!mVtkGridIsSetUp)
    {
        SetUpVtkGrid();
    }

    if (mPointSolution.size() == GetNumberOfPoints())
    {
        vtkSmartPointer<vtkDoubleArray> pPointData = vtkSmartPointer<vtkDoubleArray>::New();
        pPointData->SetNumberOfComponents(1);
        pPointData->SetNumberOfTuples(mPointSolution.size());
        pPointData->SetName("Point Values");
        for (unsigned i = 0; i < mPointSolution.size(); i++)
        {
            pPointData->SetValue(i, mPointSolution[i]);
        }
        mpVtkGrid->GetPointData()->AddArray(pPointData);
    }
    return mpVtkGrid;
}

template<unsigned DIM>
void RegularGrid<DIM>::Write(boost::shared_ptr<OutputFileHandler> pFileHandler)
{
    RegularGridWriter writer;
    writer.SetFilename(pFileHandler->GetOutputDirectoryFullPath() + "/grid.vti");
    writer.SetImage(GetVtkGrid());
    writer.Write();
}

// Explicit instantiation
template class RegularGrid<2> ;
template class RegularGrid<3> ;
