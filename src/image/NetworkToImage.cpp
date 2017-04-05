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

#include "Exception.hpp"
#include "RegularGrid.hpp"
#include "GridCalculator.hpp"
#include "DistanceMap.hpp"
#include "NetworkToImage.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"

template<unsigned DIM>
NetworkToImage<DIM>::NetworkToImage()
    : mpImage(vtkSmartPointer<vtkImageData>::New()),
      mpNetwork(),
      mGridSpacing(BaseUnits::Instance()->GetReferenceLengthScale()),
      mPaddingFactors(zero_vector<double>(DIM)),
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
void NetworkToImage<DIM>::SetGridSpacing(units::quantity<unit::length> spacing)
{
    mGridSpacing = spacing;
}

template<unsigned DIM>
void NetworkToImage<DIM>::SetPaddingFactors(double paddingX, double paddingY, double paddingZ)
{
    mPaddingFactors[0] = paddingX;
    mPaddingFactors[1] = paddingY;
    if(DIM==3)
    {
        mPaddingFactors[2] = paddingZ;
    }
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

    std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > extents = mpNetwork->GetExtents(true);
    c_vector<double, DIM> range = ublas::element_prod(scalar_vector<double>(DIM, 1.0) + 2.0*mPaddingFactors, (extents.second - extents.first).GetLocation(mGridSpacing));
    c_vector<double, DIM> origin = (extents.first.GetLocation(mGridSpacing) - ublas::element_prod((extents.second - extents.first).GetLocation(mGridSpacing), mPaddingFactors));

    boost::shared_ptr<RegularGrid<DIM> > p_grid = RegularGrid<DIM>::Create();
    p_grid->SetSpacing(mGridSpacing);
    c_vector<double, 3> dimensions;
    dimensions[0] = unsigned(range[0])+1;
    dimensions[1] = unsigned(range[1])+1;
    if(mImageDimension == 3 and DIM==3)
    {
        dimensions[2] = unsigned(range[2])+1;
    }
    else
    {
        dimensions[2] = 1;
    }
    p_grid->SetDimensions(dimensions);
    p_grid->SetOrigin(DimensionalChastePoint<DIM>(origin, mGridSpacing));

    boost::shared_ptr<DistanceMap<DIM> > p_distance_map = DistanceMap<DIM>::Create();
    boost::shared_ptr<DensityMap<DIM> > p_density_map = DensityMap<DIM>::Create();
    p_density_map->SetVesselNetwork(mpNetwork);
    p_density_map->SetGrid(p_grid);
    p_distance_map->SetDensityMap(p_density_map);
    p_distance_map->SetUseSegmentRadii(true);
    p_distance_map->Setup();
    p_distance_map->Solve();

    std::vector<double> point_solution = p_distance_map->GetSolution(p_grid);
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

//    p_grid->SetPointValues(point_solution);
    mpImage = vtkImageData::SafeDownCast(p_grid->GetGlobalVtkGrid());
    mpImage->GetPointData()->SetScalars(mpImage->GetPointData()->GetArray("Point Values"));
}

// Explicit instantiation
template class NetworkToImage<2>;
template class NetworkToImage<3>;
