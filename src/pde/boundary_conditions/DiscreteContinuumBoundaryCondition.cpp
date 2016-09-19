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

#include "Facet.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "VesselSegment.hpp"
#include "UnitCollection.hpp"
#include "Debug.hpp"

template<unsigned DIM>
DiscreteContinuumBoundaryCondition<DIM>::DiscreteContinuumBoundaryCondition()
    :   mpDomain(),
        mPoints(),
        mType(BoundaryConditionType::OUTER),
        mSource(BoundaryConditionSource::PRESCRIBED),
        mLabel("Default"),
        mValue(),
        mpRegularGrid(),
        mpMesh(),
        mpNetwork()
{

}

template<unsigned DIM>
DiscreteContinuumBoundaryCondition<DIM>::~DiscreteContinuumBoundaryCondition()
{

}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetNetwork(boost::shared_ptr<VesselNetwork <DIM> > pNetwork)
{
	mpNetwork = pNetwork;
}

template<unsigned DIM>
boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > DiscreteContinuumBoundaryCondition<DIM>::Create()
{
    MAKE_PTR(DiscreteContinuumBoundaryCondition<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
units::quantity<unit::concentration> DiscreteContinuumBoundaryCondition<DIM>::GetValue()
{
    return mValue;
}

template<unsigned DIM>
BoundaryConditionType::Value DiscreteContinuumBoundaryCondition<DIM>::GetType()
{
    return mType;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateBoundaryConditionContainer(boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> > pContainer)
{
    double node_distance_tolerance = 1.e-3;
    bool apply_boundary = true;
    bool use_boundry_nodes = false;

    if(mType == BoundaryConditionType::OUTER)
    {
        pContainer->DefineConstantDirichletOnMeshBoundary(mpMesh.get(), mValue/unit::mole_per_metre_cubed);
        apply_boundary = false;
    }
    else if(mType == BoundaryConditionType::FACET || mType == BoundaryConditionType::VESSEL_VOLUME)
    {
        use_boundry_nodes = true;
    }

    if(apply_boundary)
    {
        if(!use_boundry_nodes)
        {
            typename DiscreteContinuumMesh<DIM, DIM>::NodeIterator iter = mpMesh->GetNodeIteratorBegin();
            while (iter != mpMesh->GetNodeIteratorEnd())
            {
                std::pair<bool,units::quantity<unit::concentration> > result = GetValue((*iter).GetPoint().rGetLocation(), node_distance_tolerance);
                if(result.first)
                {
                    ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result.second/unit::mole_per_metre_cubed);
                    pContainer->AddDirichletBoundaryCondition(&(*iter), p_fixed_boundary_condition, 0, false);
                }
                ++iter;
            }
        }
        else
        {
            typename DiscreteContinuumMesh<DIM, DIM>::BoundaryNodeIterator iter = mpMesh->GetBoundaryNodeIteratorBegin();
            while (iter < mpMesh->GetBoundaryNodeIteratorEnd())
            {
                std::pair<bool,units::quantity<unit::concentration> > result = GetValue((*iter)->GetPoint().rGetLocation(), node_distance_tolerance);
                if(result.first)
                {
                    ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result.second/unit::mole_per_metre_cubed);
                    pContainer->AddDirichletBoundaryCondition(*iter, p_fixed_boundary_condition);
                }
                ++iter;
            }
        }
    }
}

template<unsigned DIM>
std::pair<bool, units::quantity<unit::concentration> > DiscreteContinuumBoundaryCondition<DIM>::GetValue(DimensionalChastePoint<DIM> location, double tolerance)
{
    std::pair<bool, units::quantity<unit::concentration> > result(false, 0.0*unit::mole_per_metre_cubed);
    if(mType == BoundaryConditionType::POINT)
    {
        if(mPoints.size()==0)
        {
            EXCEPTION("A point is required for this type of boundary condition");
        }
        else
        {
            for(unsigned jdx=0; jdx<mPoints.size(); jdx++)
            {
                if(norm_2(location.rGetLocation()-mPoints[jdx].rGetLocation()) < tolerance)
                {
                    return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::FACET)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            std::vector<boost::shared_ptr<Facet> > facets =  mpDomain->GetFacets();
            for(unsigned jdx=0; jdx<facets.size();jdx++)
            {
//                if(facets[jdx]->ContainsPoint(location) && (facets[jdx]->GetData("Boundary")>0.0))
//                {
//                    return std::pair<bool, double>(true, mValue);
//                }
            }
        }
    }

    else if(mType == BoundaryConditionType::VESSEL_LINE)
    {
        if(!mpNetwork)
        {
            EXCEPTION("A vessel network is required for this type of boundary condition");
        }
        else
        {
            std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
            for (unsigned jdx = 0; jdx <  segments.size(); jdx++)
            {
                if (segments[jdx]->GetDistance(location)/segments[jdx]->GetNode(0)->GetReferenceLengthScale()  <= tolerance)
                {
                    if(BoundaryConditionSource::PRESCRIBED)
                    {
                        return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    }
//                    else
//                    {
//                        return std::pair<bool, double>(true, segments[jdx]->template GetData<double>(mLabel));
//                    }
                }
            }
        }
    }

    else if(mType == BoundaryConditionType::VESSEL_VOLUME)
    {
        if(!mpNetwork)
        {
            EXCEPTION("A vessel network is required for this type of boundary condition");
        }
        else
        {
            std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = this->mpNetwork->GetVesselSegments();
            for (unsigned jdx = 0; jdx <  segments.size(); jdx++)
            {
                if (segments[jdx]->GetDistance(location)/segments[jdx]->GetNode(0)->GetReferenceLengthScale()  <= segments[jdx]->GetRadius()/segments[jdx]->GetNode(0)->GetReferenceLengthScale() + tolerance)
                {
                    if(BoundaryConditionSource::PRESCRIBED)
                    {
                        return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    }
                }
            }
        }
    }

    else if(mType == BoundaryConditionType::CELL)
    {
//        if(!mpCellPopulation)
//        {
//            EXCEPTION("A simple cell population is required for this type of boundary condition");
//        }
//
//        std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells = mpCellPopulation->GetCells();
//        for(unsigned idx=0; idx<cells.size(); idx++)
//        {
//            if (norm_2(cells[idx]->rGetLocation()-location)<tolerance)
//            {
//                return std::pair<bool, double>(true, mValue);
//            }
//        }
    }
    else if(mType == BoundaryConditionType::IN_PART)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            if(mpDomain->IsPointInPart(location))
            {
                if(BoundaryConditionSource::PRESCRIBED)
                {
                    return  std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
                else
                {
                    return  std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    return result;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridPointBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    if(mPoints.size()==0)
    {
        EXCEPTION("A point is required for this type of boundary condition");
    }
    else
    {
        std::vector<std::vector<unsigned> > point_point_map = mpRegularGrid->GetPointPointMap(mPoints);
        for(unsigned idx=0; idx<point_point_map.size(); idx++)
        {
            if(point_point_map[idx].size()>0)
            {
                (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
            }
        }
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridFacetBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    if(!mpDomain)
    {
        EXCEPTION("A part is required for this type of boundary condition");
    }
    else
    {
        for(unsigned idx=0; idx<mpRegularGrid->GetNumberOfPoints(); idx++)
        {
            std::vector<boost::shared_ptr<Facet> > facets =  mpDomain->GetFacets();
            for(unsigned jdx=0; jdx<facets.size();jdx++)
            {
                if(facets[jdx]->ContainsPoint(DimensionalChastePoint<3>(mpRegularGrid->GetLocationOf1dIndex(idx).rGetLocation())))
                {
                    if(BoundaryConditionSource::PRESCRIBED)
                    {
                        (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                        break;
                    }
//                    else
//                    {
//                        (*pBoundaryConditions)[idx] = std::pair<bool, double>(true, facets[jdx]->GetData(mLabel));
//                        break;
//                    }
                }
            }
        }
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridSegmentBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > point_segment_map = mpRegularGrid->GetPointSegmentMap(true, !(mType == BoundaryConditionType::VESSEL_LINE));
    for(unsigned idx=0; idx<point_segment_map.size(); idx++)
    {
        if(point_segment_map[idx].size()>0)
        {
            if(BoundaryConditionSource::PRESCRIBED)
            {
                (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
            }
//            else
//            {
//                (*pBoundaryConditions)[idx] = std::pair<bool, double>(true, point_segment_map[idx][0]->template GetData<double>(mLabel));
//            }
        }
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridCellBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
//    std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells = mpCellPopulation->GetCells();
//    std::vector<std::vector<CellPtr> > point_cell_map = mpRegularGrid->GetPointCellMap();
//    for(unsigned idx=0; idx<point_cell_map.size(); idx++)
//    {
//        (*pBoundaryConditions)[idx] = std::pair<bool, double>(true, mValue);
//    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridPartBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    if(!mpDomain)
    {
        EXCEPTION("A part is required for this type of boundary condition");
    }
    else
    {
        for(unsigned idx=0; idx<mpRegularGrid->GetNumberOfPoints(); idx++)
        {
            if(mpDomain->IsPointInPart(mpRegularGrid->GetLocationOf1dIndex(idx)))
            {
                if(BoundaryConditionSource::PRESCRIBED)
                {
                    (*pBoundaryConditions)[idx] =  std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    break;
                }
                else
                {
                    (*pBoundaryConditions)[idx] =  std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    break;
                }
            }
        }
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateRegularGridBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    if(! mpRegularGrid)
    {
        EXCEPTION("A grid has not been set for the determination of boundary condition values. For FE solvers use GetBoundaryConditionContainer()");
    }

    // Check the boundary condition type
    if(mType == BoundaryConditionType::OUTER)
    {
        for(unsigned idx=0; idx<mpRegularGrid->GetNumberOfPoints(); idx++)
        {
            (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> > (mpRegularGrid->IsOnBoundary(idx), mValue);
        }
    }
    else if(mType == BoundaryConditionType::POINT)
    {
        UpdateRegularGridPointBoundaryConditions(pBoundaryConditions);
    }
    else if(mType == BoundaryConditionType::FACET)
    {
        UpdateRegularGridFacetBoundaryConditions(pBoundaryConditions);
    }
    else if(mType == BoundaryConditionType::VESSEL_LINE or mType == BoundaryConditionType::VESSEL_VOLUME)
    {
        UpdateRegularGridSegmentBoundaryConditions(pBoundaryConditions);
    }
    else if(mType == BoundaryConditionType::CELL)
    {
        UpdateRegularGridCellBoundaryConditions(pBoundaryConditions);
    }
    else if(mType == BoundaryConditionType::IN_PART)
    {
        UpdateRegularGridPartBoundaryConditions(pBoundaryConditions);
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetDomain(boost::shared_ptr<Part<DIM> > pDomain)
{
    mpDomain = pDomain;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetPoints(std::vector<DimensionalChastePoint<DIM> > points)
{
    mPoints = points;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetSource(BoundaryConditionSource::Value boundarySource)
{
    mSource = boundarySource;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetType(BoundaryConditionType::Value boundaryType)
{
    mType = boundaryType;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<DIM, DIM> > pRegularGrid)
{
    mpRegularGrid = pRegularGrid;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > pMesh)
{
    mpMesh = pMesh;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetLabelName(const std::string& label)
{
    mLabel = label;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetValue(units::quantity<unit::concentration> value)
{
    mValue = value;
}

// Explicit instantiation
template class DiscreteContinuumBoundaryCondition<2>;
template class DiscreteContinuumBoundaryCondition<3>;
