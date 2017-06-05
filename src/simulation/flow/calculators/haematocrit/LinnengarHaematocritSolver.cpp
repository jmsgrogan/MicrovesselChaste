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

#include "LinnengarHaematocritSolver.hpp"
#include "LinearSystem.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
LinnengarHaematocritSolver<DIM>::LinnengarHaematocritSolver() : AbstractHaematocritSolver<DIM>(),
    mTHR(2.5),
    mAlpha(0.5),
    mHaematocrit(0.45),
    mSolveHighConnectivityNetworks(false),
    mTurnOffFungModel(false),
    mUseRandomSplitting(false),
    mExceptionOnFailedConverge(true)
{

}

template<unsigned DIM>
LinnengarHaematocritSolver<DIM>::~LinnengarHaematocritSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<LinnengarHaematocritSolver<DIM> > LinnengarHaematocritSolver<DIM>::Create()
{
    MAKE_PTR(LinnengarHaematocritSolver<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetExceptionOnFailedConverge(bool setException)
{
    mExceptionOnFailedConverge = setException;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetUseRandomSplittingModel(bool useRandomSplittingModel)
{
    mUseRandomSplitting = useRandomSplittingModel;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetTHR(units::quantity<unit::dimensionless> THR)
{
    mTHR = THR;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetAlpha(units::quantity<unit::dimensionless> Alpha)
{
    mAlpha = Alpha;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetUseHigherConnectivityBranches(bool useHighConnectivity)
{
    mSolveHighConnectivityNetworks = useHighConnectivity;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetTurnOffFungModel(bool turnOffFungModel)
{
    mTurnOffFungModel = turnOffFungModel;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::SetHaematocrit(units::quantity<unit::dimensionless> haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void LinnengarHaematocritSolver<DIM>::Calculate()
{
    // Give the vessels unique Ids
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = this->mpNetwork->GetVessels();
    std::vector<double> random_assignment;
    if(mUseRandomSplitting)
    {
        for(unsigned idx=0; idx<vessels.size(); idx++)
        {
            random_assignment.push_back(RandomNumberGenerator::Instance()->ranf());
        }
    }

    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        vessels[idx]->SetId(idx);
    }

    // Set up the linear system
    PetscInt lhsVectorSize = vessels.size();
    unsigned max_vessels_per_branch = 5;
    if(vessels.size() < max_vessels_per_branch)
    {
        max_vessels_per_branch  = unsigned(lhsVectorSize);
    }
    LinearSystem linearSystem(lhsVectorSize, max_vessels_per_branch);
    if(lhsVectorSize > 6)
    {
        linearSystem.SetPcType("lu");
        #ifdef PETSC_HAVE_HYPRE
        linearSystem.SetPcType("hypre");
        #endif //PETSC_HAVE_HYPRE
        linearSystem.SetKspType("preonly");
    }

    std::vector<std::vector<int> > update_indices;
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        // Always have a diagonal entry for system, this sets zero haematocrit by default
        linearSystem.SetMatrixElement(idx, idx, 1);
        if(vessels[idx]->GetStartNode()->GetFlowProperties()->IsInputNode() or
                vessels[idx]->GetEndNode()->GetFlowProperties()->IsInputNode())
        {
            linearSystem.SetRhsVectorElement(idx, mHaematocrit);
        }
        // Set rhs to zero, it should already be zero but this explicitly captures the no flow case
        else if(vessels[idx]->GetFlowProperties()->GetFlowRate()==0.0*unit::metre_cubed_per_second)
        {
            linearSystem.SetRhsVectorElement(idx, 0.0);
        }
        else
        {
            // Identify inflow node
            boost::shared_ptr<VesselNode<DIM> > p_inflow_node;
            units::quantity<unit::flow_rate> flow_rate= vessels[idx]->GetFlowProperties()->GetFlowRate();
            if(flow_rate >0 * unit::metre_cubed_per_second)
            {
                p_inflow_node = vessels[idx]->GetStartNode();
            }
            else
            {
                p_inflow_node = vessels[idx]->GetEndNode();
            }

            // Identify number of inflow and outflow vessels
            if(p_inflow_node->GetNumberOfSegments()>1)
            {
                std::vector<boost::shared_ptr<Vessel<DIM> > > parent_vessels;
                std::vector<boost::shared_ptr<Vessel<DIM> > > competitor_vessels;
                for(unsigned jdx=0; jdx<p_inflow_node->GetSegments().size(); jdx++)
                {
                    // if not this vessel
                    if(p_inflow_node->GetSegment(jdx)->GetVessel()!=vessels[idx])
                    {
                        units::quantity<unit::flow_rate> inflow_rate = p_inflow_node->GetSegment(jdx)->GetVessel()->GetFlowProperties()->GetFlowRate();
                        if(p_inflow_node->GetSegment(jdx)->GetVessel()->GetEndNode()==p_inflow_node)
                        {
                            if(inflow_rate>0.0 * unit::metre_cubed_per_second)
                            {
                                parent_vessels.push_back(p_inflow_node->GetSegment(jdx)->GetVessel());
                            }
                            else if(inflow_rate<0.0 * unit::metre_cubed_per_second)
                            {
                                competitor_vessels.push_back(p_inflow_node->GetSegment(jdx)->GetVessel());
                            }
                        }
                        if(p_inflow_node->GetSegment(jdx)->GetVessel()->GetStartNode()==p_inflow_node)
                        {
                            if(inflow_rate>0.0 * unit::metre_cubed_per_second)
                            {
                                competitor_vessels.push_back(p_inflow_node->GetSegment(jdx)->GetVessel());
                            }
                            else if(inflow_rate<0.0 * unit::metre_cubed_per_second)
                            {
                                parent_vessels.push_back(p_inflow_node->GetSegment(jdx)->GetVessel());
                            }
                        }
                    }
                }

                // If there are no competitor vessels the haematocrit is just the sum of the parent values
                if(competitor_vessels.size()==0 or units::fabs(competitor_vessels[0]->GetFlowProperties()->GetFlowRate()) == 0.0 * unit::metre_cubed_per_second)
                {
                    for(unsigned jdx=0; jdx<parent_vessels.size();jdx++)
                    {
                        linearSystem.SetMatrixElement(idx, parent_vessels[jdx]->GetId(), -fabs(parent_vessels[jdx]->GetFlowProperties()->GetFlowRate()/flow_rate));
                    }
                }
                else
                {
                    bool high_connectivity_node = competitor_vessels.size()>1 or parent_vessels.size()>1;
                    if(high_connectivity_node and !mSolveHighConnectivityNetworks)
                    {
                        EXCEPTION("This solver can only work with branches with connectivity 3");
                    }
                    else if((high_connectivity_node and mSolveHighConnectivityNetworks) or mTurnOffFungModel or mUseRandomSplitting)
                    {
                        // Haematocrit fraction is given by the fractional flow rate. Get the fraction of
                        // the total outflow going into this channel.
                        // Save the indices for later updating
                        std::vector<int> local_update_indics;
                        local_update_indics.push_back(int(idx));

                        if(mUseRandomSplitting)
                        {
                            double my_value = random_assignment[idx];
                            bool my_value_highest = true;
                            for(unsigned comp_index=0; comp_index<competitor_vessels.size(); comp_index++)
                            {
                                local_update_indics.push_back(-1*competitor_vessels[comp_index]->GetId());
                                if(random_assignment[competitor_vessels[comp_index]->GetId()]>my_value)
                                {
                                    my_value_highest = false;
                                    break;
                                }
                            }
                            double fraction = 0.3/double(competitor_vessels.size());
                            if(my_value_highest)
                            {
                                fraction = 0.7;
                            }
                            for(unsigned parent_index=0; parent_index<parent_vessels.size(); parent_index++)
                            {
                                local_update_indics.push_back(parent_vessels[parent_index]->GetId());
                                linearSystem.SetMatrixElement(idx, parent_vessels[parent_index]->GetId(), -fraction);
                            }
                            update_indices.push_back(local_update_indics);
                        }
                        else
                        {
                            units::quantity<unit::flow_rate> outflow_rate = 0.0*unit::metre_cubed_per_second;
                            for(unsigned comp_index=0; comp_index<competitor_vessels.size(); comp_index++)
                            {
                                local_update_indics.push_back(-1*competitor_vessels[comp_index]->GetId());
                                outflow_rate+= units::fabs(competitor_vessels[comp_index]->GetFlowProperties()->GetFlowRate());
                            }
                            double fraction = units::fabs(flow_rate)/(units::fabs(flow_rate)+outflow_rate);
                            for(unsigned parent_index=0; parent_index<parent_vessels.size(); parent_index++)
                            {
                                local_update_indics.push_back(parent_vessels[parent_index]->GetId());
                                linearSystem.SetMatrixElement(idx, parent_vessels[parent_index]->GetId(), -fraction);
                            }
                            update_indices.push_back(local_update_indics);
                        }
                    }
                    else
                    {
                        units::quantity<unit::flow_rate> competitor0_flow_rate = competitor_vessels[0]->GetFlowProperties()->GetFlowRate();
                        units::quantity<unit::flow_rate> parent0_flow_rate = parent_vessels[0]->GetFlowProperties()->GetFlowRate();

                        // There is a bifurcation, apply a haematocrit splitting rule
                        units::quantity<unit::length> my_radius = vessels[idx]->GetRadius();
                        units::quantity<unit::length> competitor_radius = competitor_vessels[0]->GetRadius();
                        units::quantity<unit::velocity> my_velocity = units::fabs(flow_rate)/(M_PI * my_radius * my_radius);
                        units::quantity<unit::velocity> competitor_velocity = fabs(competitor0_flow_rate)/(M_PI * competitor_radius * competitor_radius);

                        units::quantity<unit::dimensionless> alpha = 1.0 - parent_vessels[0]->GetFlowProperties()->GetHaematocrit();
                        units::quantity<unit::dimensionless> flow_ratio_pm = fabs(parent0_flow_rate)/fabs(flow_rate);
                        units::quantity<unit::dimensionless> flow_ratio_cm = fabs(competitor0_flow_rate)/fabs(flow_rate);
                        double numer = flow_ratio_pm;

                        // Apply fungs rule to faster vessel
                        if(my_velocity >= competitor_velocity)
                        {
                            units::quantity<unit::dimensionless> term = alpha * (my_velocity/competitor_velocity-1.0);
                            double denom = 1.0+flow_ratio_cm*(1.0/(1.0+term));
                            linearSystem.SetMatrixElement(idx, parent_vessels[0]->GetId(), -numer/denom);
                        }
                        else
                        {
                            units::quantity<unit::dimensionless> term = alpha * (competitor_velocity/my_velocity-1.0);
                            double denom = 1.0+flow_ratio_cm*(1.0+term);
                            linearSystem.SetMatrixElement(idx, parent_vessels[0]->GetId(), -numer/denom);
                        }

                        // Save the indices for later updating
                        std::vector<int> local_update_indics = std::vector<int>(3);
                        local_update_indics[0] = idx;
                        local_update_indics[1] = parent_vessels[0]->GetId();
                        local_update_indics[2] = competitor_vessels[0]->GetId();
                        update_indices.push_back(local_update_indics);
                    }
                }
            }
        }
    }

    // Set the parameters for iteration
    double tolerance = 1e-3;
    double max_iterations = 1000;
    double residual = DBL_MAX;
    int iterations = 0;

    while(residual > tolerance && iterations < max_iterations)
    {
        if(iterations>0 and update_indices.size()>0)
        {
            // Update the system
            linearSystem.SwitchWriteModeLhsMatrix();
            for(unsigned idx=0; idx<update_indices.size();idx++)
            {
                if(update_indices[idx].size()>3 or mTurnOffFungModel or mUseRandomSplitting)
                {
                    if(mUseRandomSplitting)
                     {
                         double my_value = random_assignment[update_indices[idx][0]];
                         bool my_value_highest = true;
                         unsigned counter = 0;
                         for(unsigned local_update_index=1; local_update_index<update_indices[idx].size(); local_update_index++)
                         {
                             if(update_indices[idx][local_update_index]<0)
                             {
                                 if(random_assignment[update_indices[idx][abs(local_update_index)]]>my_value)
                                 {
                                     my_value_highest = false;
                                 }
                                 counter++;
                             }
                         }
                         double fraction = 0.3/double(counter);
                         if(my_value_highest)
                         {
                             fraction = 0.7;
                         }
                         for(unsigned local_update_index=1;local_update_index<update_indices[idx].size();local_update_index++)
                         {
                             if(update_indices[idx][local_update_index]>=0)
                             {
                                 linearSystem.SetMatrixElement(update_indices[idx][0], update_indices[idx][local_update_index], -fraction);
                             }
                         }
                     }
                    else
                    {
                        units::quantity<unit::flow_rate> self_flow_rate = vessels[update_indices[idx][0]]->GetFlowProperties()->GetFlowRate();
                        units::quantity<unit::flow_rate> outflow_rate = 0.0*unit::metre_cubed_per_second;
                        for(unsigned local_update_index=1;local_update_index<update_indices[idx].size();local_update_index++)
                        {
                            if(update_indices[idx][local_update_index]<0)
                            {
                                outflow_rate+= units::fabs(vessels[abs(update_indices[idx][local_update_index])]->GetFlowProperties()->GetFlowRate());
                            }
                        }
                        double fraction = units::fabs(self_flow_rate)/(units::fabs(self_flow_rate)+outflow_rate);
                        for(unsigned local_update_index=1;local_update_index<update_indices[idx].size();local_update_index++)
                        {
                            if(update_indices[idx][local_update_index]>=0)
                            {
                                linearSystem.SetMatrixElement(update_indices[idx][0], update_indices[idx][local_update_index], -fraction);
                            }
                        }
                    }
                }
                else
                {
                    units::quantity<unit::flow_rate> self_flow_rate = vessels[update_indices[idx][0]]->GetFlowProperties()->GetFlowRate();
                    units::quantity<unit::flow_rate> competitor0_flow_rate = vessels[update_indices[idx][2]]->GetFlowProperties()->GetFlowRate();
                    units::quantity<unit::flow_rate> parent0_flow_rate = vessels[update_indices[idx][1]]->GetFlowProperties()->GetFlowRate();

                    units::quantity<unit::length> my_radius = vessels[update_indices[idx][0]]->GetRadius();
                    units::quantity<unit::length> competitor_radius = vessels[update_indices[idx][2]]->GetRadius();
                    units::quantity<unit::velocity> my_velocity = fabs(self_flow_rate)/(M_PI * my_radius * my_radius);
                    units::quantity<unit::velocity> competitor_velocity = fabs(competitor0_flow_rate)/(M_PI * competitor_radius * competitor_radius);
                    units::quantity<unit::dimensionless> alpha = 1.0 - vessels[update_indices[idx][1]]->GetFlowProperties()->GetHaematocrit();

                    double flow_ratio_pm = fabs(parent0_flow_rate/self_flow_rate);
                    double flow_ratio_cm = fabs(competitor0_flow_rate/self_flow_rate);
                    double numer = flow_ratio_pm;

                    // Apply fungs rule to faster vessel
                    if(my_velocity >= competitor_velocity)
                    {
                        double term = alpha * (my_velocity/competitor_velocity-1.0);
                        double denom = 1.0+flow_ratio_cm*(1.0/(1.0+term));
                        linearSystem.SetMatrixElement(update_indices[idx][0], update_indices[idx][1], -numer/denom);
                    }
                    else
                    {
                        double term = alpha * (competitor_velocity/my_velocity-1.0);
                        double denom = 1.0+flow_ratio_cm*(1.0+term);
                        linearSystem.SetMatrixElement(update_indices[idx][0], update_indices[idx][1], -numer/denom);
                    }
                }
            }
        }

        Vec solution = PetscTools::CreateVec(vessels.size());
        linearSystem.AssembleFinalLinearSystem();
        solution = linearSystem.Solve();
        ReplicatableVector a(solution);

        // Get the residual
        residual = 0.0;
        for (unsigned i = 0; i < vessels.size(); i++)
        {
            if(fabs(vessels[i]->GetFlowProperties()->GetHaematocrit() - a[i]) > residual)
            {
                residual = fabs(vessels[i]->GetFlowProperties()->GetHaematocrit() - a[i]);
            }
        }

        // assign haematocrit levels to vessels
        for (unsigned idx = 0; idx < vessels.size(); idx++)
        {
            for (unsigned jdx = 0; jdx < vessels[idx]->GetNumberOfSegments(); jdx++)
            {
                vessels[idx]->GetSegments()[jdx]->GetFlowProperties()->SetHaematocrit(a[idx]);
            }
        }

        iterations++;
        if(iterations == max_iterations)
        {
            if(mExceptionOnFailedConverge)
            {
                EXCEPTION("Haematocrit calculation failed to converge.");
            }
            else
            {
                std::cout << "Warning: haematocrit calculation failed to converge" << std::endl;
            }

        }

        PetscTools::Destroy(solution);
    }
}
// Explicit instantiation
template class LinnengarHaematocritSolver<2>;
template class LinnengarHaematocritSolver<3>;
