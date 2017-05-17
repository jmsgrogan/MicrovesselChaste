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

#include "Facet.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "VesselSegment.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "GeometryTools.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "RegularGrid.hpp"

template<unsigned DIM>
DiscreteContinuumBoundaryCondition<DIM>::DiscreteContinuumBoundaryCondition()
    :   mpDomain(),
        mpPoints(),
        mType(BoundaryConditionType::OUTER),
        mSource(BoundaryConditionSource::PRESCRIBED),
        mLabel("Default"),
        mValue(),
        mpGridCalculator(),
        mpNetwork(),
        mReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale()),
        mIsNeumann(false),
        mIsRobin(false)
{

}

template<unsigned DIM>
DiscreteContinuumBoundaryCondition<DIM>::~DiscreteContinuumBoundaryCondition()
{

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
void DiscreteContinuumBoundaryCondition<DIM>::SetIsRobin(bool isRobin)
{
    mIsRobin = isRobin;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetIsNeumann(bool isNeumann)
{
    mIsNeumann = isNeumann;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::UpdateBoundaryConditions(boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> > pContainer)
{
    double node_distance_tolerance = 1.e-3;
    bool apply_boundary = true;
    bool use_boundry_nodes = false;

    // Need a mesh for this rule
    boost::shared_ptr<DiscreteContinuumMesh<DIM> > p_mesh =
            boost::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpGridCalculator->GetGrid());
    if(!p_mesh)
    {
        EXCEPTION("Can't cast to mesh");
    }
    units::quantity<unit::length> length_scale = p_mesh->GetReferenceLengthScale();

    if(mType == BoundaryConditionType::OUTER)
    {
        pContainer->DefineConstantDirichletOnMeshBoundary(p_mesh.get(), mValue/mReferenceConcentration);
        apply_boundary = false;
    }
    else if(mType == BoundaryConditionType::POLYGON || mType == BoundaryConditionType::VESSEL_VOLUME ||
            mIsRobin || mIsNeumann or mType == BoundaryConditionType::EDGE)
    {
        use_boundry_nodes = true;
    }
    if(apply_boundary)
    {
        if(!use_boundry_nodes)
        {
            // Collect the node locations

            if(mType == BoundaryConditionType::IN_PART)
            {
                if(!mpDomain)
                {
                    EXCEPTION("A part is required for this type of boundary condition");
                }
                else
                {
                    std::vector<unsigned> mesh_nodes(p_mesh->GetNumNodes());
                    typename DiscreteContinuumMesh<DIM, DIM>::NodeIterator iter = p_mesh->GetNodeIteratorBegin();
                    unsigned counter=0;
                    while (iter != p_mesh->GetNodeIteratorEnd())
                    {
                         mesh_nodes[counter] = (*iter).GetIndex();
                         counter++;
                         ++iter;
                    }
                    std::vector<bool> inside_flags = mpDomain->IsPointInPart(p_mesh->GetPoints());
                    for(unsigned idx=0; idx<inside_flags.size(); idx++)
                    {
                        if(inside_flags[idx])
                        {
                            ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(mValue/mReferenceConcentration);
                            pContainer->AddDirichletBoundaryCondition(p_mesh->GetNode(mesh_nodes[idx]), p_fixed_boundary_condition, 0, false);
                        }
                    }
                }
            }
            else
            {
                typename DiscreteContinuumMesh<DIM, DIM>::NodeIterator iter = p_mesh->GetNodeIteratorBegin();

                 while (iter != p_mesh->GetNodeIteratorEnd())
                 {
                     DimensionalChastePoint<DIM> probe_location((*iter).GetPoint().rGetLocation(), length_scale);
                     std::pair<bool,units::quantity<unit::concentration> > result = GetValue(probe_location, node_distance_tolerance);
                     if(result.first)
                     {
                         ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result.second/mReferenceConcentration);
                         pContainer->AddDirichletBoundaryCondition(&(*iter), p_fixed_boundary_condition, 0, false);
                     }
                     ++iter;
                 }
            }
        }
        else
        {
            if(!(mIsNeumann or mIsRobin))
            {
                typename DiscreteContinuumMesh<DIM, DIM>::BoundaryNodeIterator iter = p_mesh->GetBoundaryNodeIteratorBegin();
                while (iter < p_mesh->GetBoundaryNodeIteratorEnd())
                {
                    DimensionalChastePoint<DIM> probe_location((*iter)->GetPoint().rGetLocation(), length_scale);
                    std::pair<bool,units::quantity<unit::concentration> > result = GetValue(probe_location, node_distance_tolerance);
                    ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result.second/mReferenceConcentration);
                    if(result.first)
                    {
                        pContainer->AddDirichletBoundaryCondition(*iter, p_fixed_boundary_condition);
                    }
                    ++iter;
                }
            }
            else
            {
                typename TetrahedralMesh<DIM,DIM>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
                while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
                {
                    unsigned node1_index = (*surf_iter)->GetNodeGlobalIndex(0);
                    unsigned node2_index = (*surf_iter)->GetNodeGlobalIndex(1);
                    DimensionalChastePoint<DIM> loc1(p_mesh->GetNode(node1_index)->GetPoint().rGetLocation(), length_scale);
                    DimensionalChastePoint<DIM> loc2(p_mesh->GetNode(node2_index)->GetPoint().rGetLocation(), length_scale);
                    std::pair<bool,units::quantity<unit::concentration> > result1 = GetValue(loc1, node_distance_tolerance);
                    std::pair<bool,units::quantity<unit::concentration> > result2 = GetValue(loc2, node_distance_tolerance);

                    unsigned num_on_feature = 0;
                    num_on_feature += unsigned(result1.first);
                    num_on_feature += unsigned(result2.first);
                    if(DIM==2 and mType == BoundaryConditionType::EDGE)
                    {
                        if(num_on_feature==2)
                        {
                            ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result1.second/mReferenceConcentration);
                            pContainer->AddNeumannBoundaryCondition(*surf_iter, p_fixed_boundary_condition);
                        }
                    }
                    else if(DIM==3)
                    {
                        unsigned node3_index = (*surf_iter)->GetNodeGlobalIndex(2);
                        DimensionalChastePoint<DIM> loc3(p_mesh->GetNode(node3_index)->GetPoint().rGetLocation(), length_scale);
                        std::pair<bool,units::quantity<unit::concentration> > result3 = GetValue(loc3, node_distance_tolerance);
                        num_on_feature += unsigned(result3.first);
                        if(mType == BoundaryConditionType::EDGE)
                        {
                            if(num_on_feature==2)
                            {
                                ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result1.second/mReferenceConcentration);
                                pContainer->AddNeumannBoundaryCondition(*surf_iter, p_fixed_boundary_condition);
                            }
                        }
                        else
                        {
                            if(num_on_feature==3)
                            {
                                ConstBoundaryCondition<DIM>* p_fixed_boundary_condition = new ConstBoundaryCondition<DIM>(result1.second/mReferenceConcentration);
                                pContainer->AddNeumannBoundaryCondition(*surf_iter, p_fixed_boundary_condition);
                            }
                        }
                    }
                    surf_iter++;
                }
            }
        }
    }
}

template<unsigned DIM>
std::pair<bool, units::quantity<unit::concentration> > DiscreteContinuumBoundaryCondition<DIM>::GetValue(DimensionalChastePoint<DIM> location, double tolerance)
{
    units::quantity<unit::length> length_scale = location.GetReferenceLengthScale();
    std::pair<bool, units::quantity<unit::concentration> > result(false, 0.0*unit::mole_per_metre_cubed);
    if(mType == BoundaryConditionType::POINT)
    {
        if(!mpPoints)
        {
            EXCEPTION("A point is required for this type of boundary condition");
        }
        else
        {
            for(unsigned jdx=0; jdx<mpPoints->GetNumberOfPoints(); jdx++)
            {
                c_vector<double, 3> loc3d;
                mpPoints->GetPoint(jdx, &loc3d[0]);
                DimensionalChastePoint<DIM> loc;
                if(DIM==3)
                {
                    loc = DimensionalChastePoint<DIM>(loc3d, length_scale);
                }
                else
                {
                    loc = DimensionalChastePoint<DIM>(loc3d[0], loc3d[1], 0.0, length_scale);
                }
                if(GetDistance<DIM>(location, loc) < tolerance*length_scale)
                {
                    return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::POLYGON)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            std::vector<boost::shared_ptr<Polygon<DIM> > > polygons =  mpDomain->GetPolygons();
            for(unsigned jdx=0; jdx<polygons.size();jdx++)
            {
                if(polygons[jdx]->ContainsPoint(location) && (polygons[jdx]->HasAttribute(mLabel)))
                {
                    return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::EDGE)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            if(mpDomain->EdgeHasAttribute(location, mLabel))
            {
                return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
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
                if (segments[jdx]->GetDistance(location) <= tolerance*length_scale)
                {
                    if(BoundaryConditionSource::PRESCRIBED)
                    {
                        return std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    }
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
                if (segments[jdx]->GetDistance(location) <= segments[jdx]->GetRadius() + tolerance*length_scale)
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
        EXCEPTION("Cell based boundary conditions are not yet supported.");
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
void DiscreteContinuumBoundaryCondition<DIM>::UpdateBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > >pBoundaryConditions)
{
    if(! mpGridCalculator)
    {
        EXCEPTION("A grid calculator has not been set for the determination of boundary condition values. For FE solvers use GetBoundaryConditionContainer()");
    }

    // Need a regular grid for this rule
    boost::shared_ptr<RegularGrid<DIM> > p_regular_grid =
            boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());

    if(!p_regular_grid)
    {
        EXCEPTION("Can't cast to regular grid");
    }

    // Check the boundary condition type
    if(mType == BoundaryConditionType::OUTER)
    {
        for(unsigned idx=0; idx<p_regular_grid->GetNumberOfPoints(); idx++)
        {
            (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> > (p_regular_grid->IsOnBoundary(idx), mValue);
        }
    }
    else if(mType == BoundaryConditionType::POINT)
    {
        if(!mpPoints)
        {
            EXCEPTION("A point is required for this type of boundary condition");
        }
        else
        {
            std::vector<std::vector<unsigned> > point_point_map = mpGridCalculator->GetPointMap(mpPoints);
            for(unsigned idx=0; idx<point_point_map.size(); idx++)
            {
                if(point_point_map[idx].size()>0)
                {
                    (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::POLYGON)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            for(unsigned idx=0; idx<p_regular_grid->GetNumberOfPoints(); idx++)
            {
                std::vector<boost::shared_ptr<Polygon<DIM> > > polygons =  mpDomain->GetPolygons();
                for(unsigned jdx=0; jdx<polygons.size();jdx++)
                {
                    if(polygons[jdx]->ContainsPoint(p_regular_grid->GetPoint(idx)) && polygons[jdx]->HasAttribute(mLabel))
                    {
                        (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                        break;
                    }
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::EDGE)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            for(unsigned idx=0; idx<p_regular_grid->GetNumberOfPoints(); idx++)
            {
                if(mpDomain->EdgeHasAttribute(p_regular_grid->GetPoint(idx), mLabel))
                {
                    (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::VESSEL_LINE or mType == BoundaryConditionType::VESSEL_VOLUME)
    {
        std::vector<std::vector<boost::shared_ptr<VesselSegment<DIM> > > > point_segment_map = mpGridCalculator->rGetSegmentMap(true, !(mType == BoundaryConditionType::VESSEL_LINE));
        for(unsigned idx=0; idx<point_segment_map.size(); idx++)
        {
            if(point_segment_map[idx].size()>0)
            {
                if(BoundaryConditionSource::PRESCRIBED)
                {
                    (*pBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                }
            }
        }
    }
    else if(mType == BoundaryConditionType::IN_PART)
    {
        if(!mpDomain)
        {
            EXCEPTION("A part is required for this type of boundary condition");
        }
        else
        {
            for(unsigned idx=0; idx<p_regular_grid->GetNumberOfPoints(); idx++)
            {
                if(mpDomain->IsPointInPart(p_regular_grid->GetPoint(idx)))
                {
                    (*pBoundaryConditions)[idx] =  std::pair<bool, units::quantity<unit::concentration> >(true, mValue);
                    break;
                }
            }
        }
    }
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetDomain(boost::shared_ptr<Part<DIM> > pDomain)
{
    mpDomain = pDomain;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetPoints(vtkSmartPointer<vtkPoints> pPoints)
{
    mpPoints = pPoints;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetPoints(std::vector<DimensionalChastePoint<DIM> > points)
{
    mpPoints = vtkSmartPointer<vtkPoints>::New();
    for(unsigned idx=0; idx<points.size(); idx++)
    {
        c_vector<double, DIM> loc = points[idx].GetLocation(mpGridCalculator->GetGrid()->GetReferenceLengthScale());
        if(DIM==3)
        {
            mpPoints->InsertNextPoint(&loc[0]);
        }
        else
        {
            mpPoints->InsertNextPoint(loc[0], loc[1], 0.0);
        }
    }
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
void DiscreteContinuumBoundaryCondition<DIM>::SetGridCalculator(boost::shared_ptr<GridCalculator<DIM> > pGridCalculator)
{
    mpGridCalculator = pGridCalculator;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetLabel(const std::string& label)
{
    mLabel = label;
}

template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetNetwork(boost::shared_ptr<VesselNetwork <DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}


template<unsigned DIM>
void DiscreteContinuumBoundaryCondition<DIM>::SetValue(units::quantity<unit::concentration> value)
{
    mValue = value;
}

// Explicit instantiation
template class DiscreteContinuumBoundaryCondition<2>;
template class DiscreteContinuumBoundaryCondition<3>;
