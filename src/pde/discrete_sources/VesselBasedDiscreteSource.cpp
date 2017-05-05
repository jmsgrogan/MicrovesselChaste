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

#include "VesselBasedDiscreteSource.hpp"
#include "AbstractCellPopulation.hpp"
#include "VesselNetwork.hpp"
#include "GeometryTools.hpp"
#include "Element.hpp"
#include "Connor17Parameters.hpp"

template<unsigned DIM>
VesselBasedDiscreteSource<DIM>::VesselBasedDiscreteSource()
    :   DiscreteSource<DIM>(),
        mVesselPermeability(0.0*unit::metre_per_second),
        mReferenceConcentration(0.0*unit::mole_per_metre_cubed),
        mHalfMaxUptakeConcentration(0.0*unit::mole_per_metre_cubed),
        mReferenceHaematocrit(1.0),
        mUptakeRatePerCell(-Connor17Parameters::mpReductionInVegfPerCell->GetValue("VesselBasedDiscreteSource")),
        mCellsPerMetre(Connor17Parameters::mpEcsPerLength->GetValue("VesselBasedDiscreteSource"))
{

}

template<unsigned DIM>
VesselBasedDiscreteSource<DIM>::~VesselBasedDiscreteSource()
{

}

template<unsigned DIM>
boost::shared_ptr<VesselBasedDiscreteSource<DIM> > VesselBasedDiscreteSource<DIM>::Create()
{
    MAKE_PTR(VesselBasedDiscreteSource<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration_flow_rate> > VesselBasedDiscreteSource<DIM>::GetConstantInUValues()
{
    if(!this->mpDensityMap)
    {
        EXCEPTION("A density map is required for this type of source");
    }
    std::vector<units::quantity<unit::concentration_flow_rate> > values(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells(),
            0.0*unit::mole_per_metre_cubed_per_second);

    units::quantity<unit::length> reference_length = this->mpDensityMap->GetGridCalculator()->GetGrid()->GetReferenceLengthScale();
    std::vector<double> vessel_densities = this->mpDensityMap->rGetVesselSurfaceAreaDensity(false);
    double haematocrit_ratio = 1.0;
    for(unsigned idx=0;idx<vessel_densities.size();idx++)
    {
        values[idx] += vessel_densities[idx]*mVesselPermeability * (1.0/reference_length) * mReferenceConcentration * haematocrit_ratio;;
    }
    return values;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetUptakeRatePerCell(units::quantity<unit::molar_flow_rate> ratePerCell)
{
    mUptakeRatePerCell = ratePerCell;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetHalfMaxUptakeConcentration(units::quantity<unit::concentration> value)
{

}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetNumberOfCellsPerLength(units::quantity<unit::per_length> cellsPerLength)
{
    mCellsPerMetre = cellsPerLength;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration_flow_rate> > VesselBasedDiscreteSource<DIM>::GetNonlinearTermValues()
{
    std::vector<units::quantity<unit::concentration_flow_rate> > values(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells(),
            0.0*unit::mole_per_metre_cubed_per_second);

    units::quantity<unit::length> reference_length = this->mpDensityMap->GetGridCalculator()->GetGrid()->GetReferenceLengthScale();
    std::vector<double> vessel_densities = this->mpDensityMap->rGetVesselLineDensity(false);
    for(unsigned idx=0;idx<vessel_densities.size();idx++)
    {
        values[idx] += vessel_densities[idx]*mCellsPerMetre*reference_length*mUptakeRatePerCell*(1.0/units::pow<3>(reference_length));
    }
    return values;
}

template<unsigned DIM>
std::vector<units::quantity<unit::rate> > VesselBasedDiscreteSource<DIM>::GetLinearInUValues()
{
    if(!this->mpDensityMap->GetGridCalculator())
    {
        EXCEPTION("A regular grid is required for this type of source");
    }

    std::vector<units::quantity<unit::rate> > values(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfCells(),
            0.0*unit::per_second);
    units::quantity<unit::length> reference_length = this->mpDensityMap->GetGridCalculator()->GetGrid()->GetReferenceLengthScale();
    std::vector<double> vessel_densities = this->mpDensityMap->rGetPerfusedVesselSurfaceAreaDensity(false);
    for(unsigned idx=0;idx<vessel_densities.size();idx++)
    {
        values[idx] += vessel_densities[idx]*mVesselPermeability * (1.0/reference_length);
    }
    return values;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetVesselPermeability(units::quantity<unit::membrane_permeability> value)
{
    mVesselPermeability = value;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetReferenceConcentration(units::quantity<unit::concentration> value)
{
    mReferenceConcentration = value;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetReferenceHaematocrit(units::quantity<unit::dimensionless> value)
{
    mReferenceHaematocrit = value;
}

// Explicit instantiation
template class VesselBasedDiscreteSource<2>;
template class VesselBasedDiscreteSource<3>;
