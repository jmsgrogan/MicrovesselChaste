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

#include "DiscreteSource.hpp"
#include "GeometryTools.hpp"

template<unsigned DIM>
DiscreteSource<DIM>::DiscreteSource()
    :   mPoints(),
        mLabel("Default"),
        mConstantInUValue(0.0*unit::mole_per_metre_cubed_per_second),
        mLinearInUValue(0.0*unit::per_second),
        mpDensityMap()
{

}

template<unsigned DIM>
DiscreteSource<DIM>::~DiscreteSource()
{

}

template<unsigned DIM>
std::shared_ptr<DiscreteSource<DIM> > DiscreteSource<DIM>::Create()
{
    return std::make_shared<DiscreteSource<DIM> >();

}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration_flow_rate> > DiscreteSource<DIM>::GetConstantInUValues()
{
    if(!mpDensityMap)
    {
        EXCEPTION("A density map is required for this type of source");
    }

    if(mPoints.size()==0)
    {
        EXCEPTION("A point is required for this type of source");
    }

    // Loop through all points
    std::vector<units::quantity<unit::concentration_flow_rate> > values(GetNumberOfPoints(),
            0.0*unit::mole_per_metre_cubed_per_second);
    std::vector<std::vector<unsigned> > point_point_map = mpDensityMap->GetGridCalculator()->GetPointMap(mPoints);
    for(unsigned idx=0; idx<point_point_map.size(); idx++)
    {
        values[idx] += mConstantInUValue * double(point_point_map[idx].size());
    }
    return values;
}

template<unsigned DIM>
std::vector<units::quantity<unit::rate> > DiscreteSource<DIM>::GetLinearInUValues()
{
    if(!mpDensityMap)
    {
        EXCEPTION("A density map is required for this type of source");
    }

    if(mPoints.size()==0)
    {
        EXCEPTION("A point is required for this type of source");
    }

    // Loop through all points
    std::vector<units::quantity<unit::rate> > values(GetNumberOfPoints(), 0.0*unit::per_second);
    std::vector<std::vector<unsigned> > point_point_map = mpDensityMap->GetGridCalculator()->GetPointMap(mPoints);
    for(unsigned idx=0; idx<point_point_map.size(); idx++)
    {
        values[idx] += mLinearInUValue * double(point_point_map[idx].size());
    }
    return values;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration_flow_rate> > DiscreteSource<DIM>::GetNonlinearTermValues()
{
    if(!mpDensityMap)
    {
        EXCEPTION("A density map is required for this type of source");
    }

    // Return an empty vector
    std::vector<units::quantity<unit::concentration_flow_rate> > values(GetNumberOfPoints(),
            0.0*unit::mole_per_metre_cubed_per_second);
    return values;
}

template<unsigned DIM>
unsigned DiscreteSource<DIM>::GetNumberOfPoints()
{
    if(!mpDensityMap)
    {
        EXCEPTION("A density map is required for this type of source");
    }
    return mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells();
}

template<unsigned DIM>
void DiscreteSource<DIM>::SetLabelName(const std::string& label)
{
    mLabel = label;
}

template<unsigned DIM>
void DiscreteSource<DIM>::SetPoints(std::vector<DimensionalChastePoint<DIM> > points)
{
    mPoints = points;
}

template<unsigned DIM>
std::shared_ptr<DensityMap<DIM> > DiscreteSource<DIM>::GetDensityMap()
{
    return mpDensityMap;
}

template<unsigned DIM>
void DiscreteSource<DIM>::SetConstantInUValue(units::quantity<unit::concentration_flow_rate> value)
{
    mConstantInUValue = value;
}

template<unsigned DIM>
void DiscreteSource<DIM>::SetLinearInUValue(units::quantity<unit::rate> value)
{
    mLinearInUValue = value;
}

template<unsigned DIM>
void DiscreteSource<DIM>::SetDensityMap(std::shared_ptr<DensityMap<DIM> > pMap)
{
    mpDensityMap = pMap;
}

template<unsigned DIM>
void DiscreteSource<DIM>::UpdateDensityMap()
{
    if(mPoints.size()==0)
    {
        EXCEPTION("A point is required for this type of source");
    }

    mpDensityMap->GetGridCalculator()->GetPointMap(mPoints);
}

// Explicit instantiation
template class DiscreteSource<2>;
template class DiscreteSource<3>;
