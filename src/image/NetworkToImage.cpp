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

#include "Exception.hpp"
#include "RegularGrid.hpp"
#include "DistanceMap.hpp"
#include "NetworkToImage.hpp"
#include "UnitCollection.hpp"

template<unsigned DIM>
NetworkToImage<DIM>::NetworkToImage()
    : mpImage(vtkSmartPointer<vtkImageData>::New()),
      mpNetwork(),
      mGridSpacing(1.0),
      mPaddingFactorX(0.0),
      mPaddingFactorY(0.0),
      mPaddingFactorZ(0.0),
      mImageDimension(DIM)
{

}

template<unsigned DIM>
boost::shared_ptr<NetworkToImage<DIM> > NetworkToImage<DIM>::Create()
{
    MAKE_PTR(NetworkToImage<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
NetworkToImage<DIM>::~NetworkToImage()
{

}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> NetworkToImage<DIM>::GetOutput()
{
    if(mpImage)
    {
        return(mpImage);
    }
    else
    {
        EXCEPTION("No output set. Did you run 'Update()' ?");
    }
}

template<unsigned DIM>
void NetworkToImage<DIM>::SetNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void NetworkToImage<DIM>::SetGridSpacing(double spacing)
{
    mGridSpacing = spacing;
}

template<unsigned DIM>
void NetworkToImage<DIM>::SetPaddingFactors(double paddingX, double paddingY, double paddingZ)
{
    mPaddingFactorX = paddingX;
    mPaddingFactorY = paddingY;
    mPaddingFactorZ = paddingZ;
}

template<unsigned DIM>
void NetworkToImage<DIM>::SetImageDimension(unsigned dimension)
{
    mImageDimension = dimension;
}

template<unsigned DIM>
void NetworkToImage<DIM>::Update()
{
    if(!mpNetwork)
    {
        EXCEPTION("No input vessel network set.");
    }

    std::vector<std::pair<double, double> > extents = mpNetwork->GetExtents(true);
    double range_x = (extents[0].second - extents[0].first)*(1.0 + 2.0*mPaddingFactorX);
    double range_y = (extents[1].second - extents[1].first)*(1.0 + 2.0*mPaddingFactorY);
    double range_z = 0.0;
    double origin_x = extents[0].first - (extents[0].second - extents[0].first)*mPaddingFactorX;
    double origin_y = extents[1].first - (extents[1].second - extents[1].first)*mPaddingFactorY;
    double origin_z = 0.0;

    if (mImageDimension == 3 and DIM == 3)
    {
        range_z = (extents[2].second - extents[2].first)*(1.0 + 2.0*mPaddingFactorZ);
        origin_z = extents[2].first - (extents[2].second - extents[2].first)*mPaddingFactorZ;
    }

    boost::shared_ptr<RegularGrid<DIM> > p_grid = RegularGrid<DIM>::Create();
    p_grid->SetSpacing(mGridSpacing * 1.e-6 * unit::metres);
    std::vector<unsigned> final_extents;
    final_extents.push_back(unsigned(range_x/mGridSpacing)+1);
    final_extents.push_back(unsigned(range_y/mGridSpacing)+1);
    final_extents.push_back(unsigned(range_z/mGridSpacing)+1);
    p_grid->SetExtents(final_extents);
    c_vector<double, DIM> origin;
    origin[0] = origin_x;
    origin[1] = origin_y;
    if(DIM == 3)
    {
        origin[2] = origin_z;
    }
    p_grid->SetOrigin(origin);

    boost::shared_ptr<DistanceMap<DIM> > p_distance_map = DistanceMap<DIM>::Create();
    p_distance_map->SetVesselNetwork(mpNetwork);
    p_distance_map->SetGrid(p_grid);
    p_distance_map->SetUseSegmentRadii(true);
    p_distance_map->Setup();
    p_distance_map->Solve();

    std::vector<double> point_solution = p_distance_map->GetPointSolution();
    for(unsigned idx=0; idx<point_solution.size();idx++)
    {
        if(point_solution[idx]==0.0)
        {
            point_solution[idx] = 1.0;
        }
        else
        {
            point_solution[idx] = 0.0;
        }
    }

    p_grid->SetUpVtkGrid();
    p_grid->SetPointValues(point_solution);
    mpImage = p_grid->GetVtkGrid();
    mpImage->GetPointData()->SetScalars(mpImage->GetPointData()->GetArray("Point Values"));
}

// Explicit instantiation
template class NetworkToImage<2>;
template class NetworkToImage<3>;
