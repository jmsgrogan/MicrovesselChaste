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
//    if(!this->mpGridCalculator)
//    {
//        EXCEPTION("A regular grid is required for this type of source");
//    }
//
//    if(this->mpGridCalculator->HasStructuredGrid())
//    {
//        std::vector<units::quantity<unit::concentration_flow_rate> > values(this->mpGridCalculator->GetGrid()->GetNumberOfLocations(),
//                0.0*unit::mole_per_metre_cubed_per_second);
//        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > point_segment_map = this->mpGridCalculator->rGetSegmentMap();
//        units::quantity<unit::length> grid_spacing = this->mpGridCalculator->GetGrid()->GetSpacing();
//        units::quantity<unit::volume> grid_volume = units::pow<3>(grid_spacing);
//
//        for(unsigned idx=0; idx<point_segment_map.size(); idx++)
//        {
//            for (unsigned jdx = 0; jdx < point_segment_map[idx].size(); jdx++)
//            {
//                units::quantity<unit::length> length_in_box = LengthOfLineInBox<DIM>(point_segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
//                                                                             point_segment_map[idx][jdx]->GetNode(1)->rGetLocation(),
//                                                                             this->mpGridCalculator->GetGrid()->GetLocationOfGlobalIndex(idx), grid_spacing);
//                units::quantity<unit::area> surface_area = 2.0*M_PI*point_segment_map[idx][jdx]->GetRadius()*length_in_box;
//                double haematocrit_ratio = point_segment_map[idx][jdx]->GetFlowProperties()->GetHaematocrit()/mReferenceHaematocrit;
//                values[idx] += mVesselPermeability * (surface_area/grid_volume) * mReferenceConcentration * haematocrit_ratio;
//            }
//        }
//        return values;
//    }
//    else
//    {
//        std::vector<units::quantity<unit::concentration_flow_rate> > values(this->mpGridCalculator->GetGrid()->GetNumberOfLocations(),
//                0.0*unit::mole_per_metre_cubed_per_second);
//        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > element_segment_map = this->mpGridCalculator->rGetSegmentMap();
//        units::quantity<unit::length> mesh_length = this->mpGridCalculator->GetGrid()->GetReferenceLengthScale();
//        units::quantity<unit::volume> mesh_volume_multiplier = units::pow<3>(mesh_length);
//
//        for(unsigned idx=0; idx<element_segment_map.size(); idx++)
//        {
//            if(element_segment_map[idx].size()>0)
//            {
//                // Get the element nodal locations and element volume
//                Element<DIM, DIM>* p_element = this->mpGridCalculator->GetGrid()->GetElement(idx);
//                std::vector<DimensionalChastePoint<DIM> > element_vertices(4);
//                double determinant = 0.0;
//                c_matrix<double, DIM, DIM> jacobian;
//                p_element->CalculateJacobian(jacobian, determinant);
//                units::quantity<unit::volume> element_volume = p_element->GetVolume(determinant) * mesh_volume_multiplier;
//                if(p_element->GetNumNodes() != 4)
//                {
//                    EXCEPTION("Vessel mesh mapping only supported for linear tetrahedral elements.");
//                }
//                for (unsigned jdx = 0; jdx < 4; jdx++)
//                {
//                    element_vertices[jdx] = DimensionalChastePoint<DIM>(p_element->GetNodeLocation(jdx), mesh_length);
//                }
//
//                for (unsigned jdx = 0; jdx < element_segment_map[idx].size(); jdx++)
//                {
//
//                    units::quantity<unit::length> length_in_box = LengthOfLineInTetra<DIM>(element_segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
//                                                                    element_segment_map[idx][jdx]->GetNode(1)->rGetLocation(), element_vertices);
//                    units::quantity<unit::area> surface_area = 2.0*M_PI*element_segment_map[idx][jdx]->GetRadius()*length_in_box;
//                    double haematocrit_Ratio = element_segment_map[idx][jdx]->GetFlowProperties()->GetHaematocrit()/mReferenceHaematocrit;
//                    values[idx] += mVesselPermeability * (surface_area/element_volume) * mReferenceConcentration * haematocrit_Ratio;
//                }
//            }
//        }
//        return values;
//    }
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetUptakeRatePerCell(units::quantity<unit::molar_flow_rate> ratePerCell)
{
    mUptakeRatePerCell = ratePerCell;
}

template<unsigned DIM>
void VesselBasedDiscreteSource<DIM>::SetNumberOfCellsPerLength(units::quantity<unit::per_length> cellsPerLength)
{
    mCellsPerMetre = cellsPerLength;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration_flow_rate> > VesselBasedDiscreteSource<DIM>::GetNonlinearTermValues()
{
//    std::vector<units::quantity<unit::concentration_flow_rate> > values(this->mpGridCalculator->GetGrid()->GetNumberOfLocations(),
//            0.0*unit::mole_per_metre_cubed_per_second);
//    std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > point_segment_map = this->mpGridCalculator->rGetSegmentMap();
//    units::quantity<unit::length> grid_spacing = this->mpGridCalculator->GetGrid()->GetSpacing();
//    units::quantity<unit::volume> grid_volume = units::pow<3>(grid_spacing);
//
//    for(unsigned idx=0; idx<point_segment_map.size(); idx++)
//    {
//        for (unsigned jdx = 0; jdx < point_segment_map[idx].size(); jdx++)
//        {
//            units::quantity<unit::length> length_in_box = LengthOfLineInBox<DIM>(point_segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
//                                                                         point_segment_map[idx][jdx]->GetNode(1)->rGetLocation(),
//                                                                         this->mpGridCalculator->GetGrid()->GetLocationOfGlobalIndex(idx), grid_spacing);
//            double num_cells = length_in_box*mCellsPerMetre;
//            values[idx] += mUptakeRatePerCell * (num_cells/grid_volume);
//        }
//    }
//    return values;
}

template<unsigned DIM>
std::vector<units::quantity<unit::rate> > VesselBasedDiscreteSource<DIM>::GetLinearInUValues()
{
//    if(!this->mpGridCalculator)
//    {
//        EXCEPTION("A regular grid is required for this type of source");
//    }
//
//    if(this->mpGridCalculator->HasStructuredGrid())
//    {
//        std::vector<units::quantity<unit::rate> > values(this->mpGridCalculator->GetGrid()->GetNumberOfLocations(),
//                0.0*unit::per_second);
//        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > point_segment_map = this->mpGridCalculator->rGetSegmentMap(false);
//        units::quantity<unit::length> grid_spacing = this->mpGridCalculator->GetGrid()->GetSpacing();
//        units::quantity<unit::volume> grid_volume = units::pow<3>(grid_spacing);
//
//        for(unsigned idx=0; idx<point_segment_map.size(); idx++)
//        {
//            for (unsigned jdx = 0; jdx < point_segment_map[idx].size(); jdx++)
//            {
//                units::quantity<unit::length> length_in_box = LengthOfLineInBox<DIM>(point_segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
//                                                                             point_segment_map[idx][jdx]->GetNode(1)->rGetLocation(),
//                                                                             this->mpGridCalculator->GetGrid()->GetLocationOfGlobalIndex(idx), grid_spacing);
//
//                units::quantity<unit::area> surface_area = 2.0*M_PI*point_segment_map[idx][jdx]->GetRadius()*length_in_box;
//                double haematocrit = point_segment_map[idx][jdx]->GetFlowProperties()->GetHaematocrit();
//                if(haematocrit>0.0)
//                {
//                    values[idx] -= mVesselPermeability * (surface_area/grid_volume);
//                }
//            }
//        }
//        return values;
//    }
//    else
//    {
//        std::vector<units::quantity<unit::rate> > values(this->mpGridCalculator->GetNumberOfLocations(), 0.0*unit::per_second);
//        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > element_segment_map = this->mpGridCalculator->GetSegmentMap();
//        units::quantity<unit::length> mesh_length = this->mpGridCalculator->GetMesh()->GetReferenceLengthScale();
//        units::quantity<unit::volume> mesh_volume_multiplier = units::pow<3>(mesh_length);
//
//        for(unsigned idx=0; idx<element_segment_map.size(); idx++)
//        {
//            if(element_segment_map[idx].size()>0)
//            {
//                // Get the element nodal locations and element volume
//                Element<DIM, DIM>* p_element = this->mpGridCalculator->GetMesh()->GetElement(idx);
//                std::vector<DimensionalChastePoint<DIM> > element_vertices(4);
//                double determinant = 0.0;
//                c_matrix<double, DIM, DIM> jacobian;
//                p_element->CalculateJacobian(jacobian, determinant);
//                units::quantity<unit::volume> element_volume = p_element->GetVolume(determinant) * mesh_volume_multiplier;
//                if(p_element->GetNumNodes() != 4)
//                {
//                    EXCEPTION("Vessel mesh mapping only supported for linear tetrahedral elements.");
//                }
//                for (unsigned jdx = 0; jdx < 4; jdx++)
//                {
//                    element_vertices[jdx] = DimensionalChastePoint<DIM>(p_element->GetNodeLocation(jdx), mesh_length);
//                }
//
//                for (unsigned jdx = 0; jdx < element_segment_map[idx].size(); jdx++)
//                {
//                    units::quantity<unit::length> length_in_box = LengthOfLineInTetra<DIM>(element_segment_map[idx][jdx]->GetNode(0)->rGetLocation(),
//                                                                    element_segment_map[idx][jdx]->GetNode(1)->rGetLocation(), element_vertices);
//
//                    units::quantity<unit::area> surface_area = 2.0*M_PI*element_segment_map[idx][jdx]->GetRadius()*length_in_box;
//                    double haematocrit = element_segment_map[idx][jdx]->GetFlowProperties()->GetHaematocrit();
//                    if(haematocrit>0.0)
//                    {
//                        values[idx] += mVesselPermeability * (surface_area/element_volume);
//                    }
//                }
//            }
//        }
//        return values;
//    }
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
