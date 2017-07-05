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

#include "CellBasedDiscreteSource.hpp"

template<unsigned DIM>
CellBasedDiscreteSource<DIM>::CellBasedDiscreteSource()
    :   DiscreteSource<DIM>(),
        mCellConstantInUValue(0.0*unit::mole_per_second),
        mCellLinearInUValue(0.0*unit::per_second)
{

}

template<unsigned DIM>
CellBasedDiscreteSource<DIM>::~CellBasedDiscreteSource()
{

}

template<unsigned DIM>
std::shared_ptr<CellBasedDiscreteSource<DIM> > CellBasedDiscreteSource<DIM>::Create()
{
    return std::make_shared<CellBasedDiscreteSource<DIM> >();

}

template<unsigned DIM>
std::vector<QConcentrationFlowRate > CellBasedDiscreteSource<DIM>::GetConstantInUValues()
{
    if(!this->mpDensityMap->GetGridCalculator())
    {
        EXCEPTION("A regular grid is required for this type of source");
    }

    // Get the cell density map
    std::vector<QConcentrationFlowRate > values(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells(),
            0.0*unit::mole_per_metre_cubed_per_second);
    std::vector<double> cell_densities = this->mpDensityMap->rGetCellDensity(true);
    QLength reference_length = this->mpDensityMap->GetGridCalculator()->GetGrid()->GetReferenceLengthScale();
    QVolume ref_volume = reference_length*reference_length*reference_length;
    for(unsigned idx=0;idx<cell_densities.size();idx++)
    {
        values[idx] = values[idx] +  mCellConstantInUValue*cell_densities[idx]/ref_volume;
    }
    return values;
}

template<unsigned DIM>
std::vector<QRate > CellBasedDiscreteSource<DIM>::GetLinearInUValues()
{
    if(!this->mpDensityMap->GetGridCalculator())
    {
        EXCEPTION("A regular grid is required for this type of source");
    }

    std::vector<QRate > values(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells(),
            0.0*unit::per_second);
    std::vector<double> cell_densities = this->mpDensityMap->rGetCellDensity(true);
    for(unsigned idx=0; idx<cell_densities.size(); idx++)
    {
        values[idx] = values[idx] + mCellLinearInUValue * cell_densities[idx];
    }
    return values;
}

template<unsigned DIM>
void CellBasedDiscreteSource<DIM>::SetConstantInUConsumptionRatePerCell(QMolarFlowRate value)
{
    mCellConstantInUValue = -value;
}

template<unsigned DIM>
void CellBasedDiscreteSource<DIM>::SetLinearInUConsumptionRatePerCell(QRate value)
{
    mCellLinearInUValue = -value;
}

template<unsigned DIM>
void CellBasedDiscreteSource<DIM>::UpdateDensityMap()
{
    this->mpDensityMap->rGetCellDensity(true);
}

// Explicit instantiation
template class CellBasedDiscreteSource<2>;
template class CellBasedDiscreteSource<3>;
