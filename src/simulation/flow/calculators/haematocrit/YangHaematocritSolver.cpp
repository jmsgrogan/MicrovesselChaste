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

#include "YangHaematocritSolver.hpp"
#include "LinearSystem.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
YangHaematocritSolver<DIM>::YangHaematocritSolver() : AbstractHaematocritSolver<DIM>(),
    mTHR(2.5),
    mAlpha(0.5),
    mHaematocrit(0.45),
    mYangM0(10.0),
    mYangk(4.0),
    mSolveHighConnectivityNetworks(false),
    mTurnOffFungModel(false),
    mUseRandomSplitting(false),
    mExceptionOnFailedConverge(true)
{

}

template<unsigned DIM>
YangHaematocritSolver<DIM>::~YangHaematocritSolver()
{

}

template <unsigned DIM>
std::shared_ptr<YangHaematocritSolver<DIM> > YangHaematocritSolver<DIM>::Create()
{
    return std::make_shared<YangHaematocritSolver<DIM> >();
}

template <unsigned DIM>
void YangHaematocritSolver<DIM>::SetExceptionOnFailedConverge(bool setException)
{
    mExceptionOnFailedConverge = setException;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetUseRandomSplittingModel(bool useRandomSplittingModel)
{
    mUseRandomSplitting = useRandomSplittingModel;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetTHR(QDimensionless THR)
{
    mTHR = THR;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetAlpha(QDimensionless Alpha)
{
    mAlpha = Alpha;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetUseHigherConnectivityBranches(bool useHighConnectivity)
{
    mSolveHighConnectivityNetworks = useHighConnectivity;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetTurnOffFungModel(bool turnOffFungModel)
{
    mTurnOffFungModel = turnOffFungModel;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetHaematocrit(QDimensionless haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetYangM0(QDimensionless YangM0)
{
    mYangM0 = YangM0;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::SetYangk(QDimensionless Yangk)
{
    mYangk = Yangk;
}

template<unsigned DIM>
void YangHaematocritSolver<DIM>::Calculate()
{
    // Give the vessels unique Ids
    double localM = 1.0;
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = this->mpNetwork->GetVessels();
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
    //if(lhsVectorSize > 6)
    //{
    //   linearSystem.SetPcType("lu");
    //    #ifdef PETSC_HAVE_HYPRE
    //    linearSystem.SetPcType("hypre");
    //    #endif //PETSC_HAVE_HYPRE
    //    linearSystem.SetKspType("preonly");
   //}

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
            std::shared_ptr<VesselNode<DIM> > p_inflow_node;
            QFlowRate flow_rate= vessels[idx]->GetFlowProperties()->GetFlowRate();
            if(flow_rate >(0.0 * unit::metre_cubed_per_second))
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
                std::vector<std::shared_ptr<Vessel<DIM> > > parent_vessels;
                std::vector<std::shared_ptr<Vessel<DIM> > > competitor_vessels;
                for(unsigned jdx=0; jdx<p_inflow_node->GetSegments().size(); jdx++)
                {
                    // if not this vessel
                    if(p_inflow_node->GetSegment(jdx)->GetVessel()!=vessels[idx])
                    {
                        QFlowRate inflow_rate = p_inflow_node->GetSegment(jdx)->GetVessel()->GetFlowProperties()->GetFlowRate();
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
                if(competitor_vessels.size()==0 or Qabs(competitor_vessels[0]->GetFlowProperties()->GetFlowRate()) == 0.0 * unit::metre_cubed_per_second)
                {
                    for(unsigned jdx=0; jdx<parent_vessels.size();jdx++)
                    {
                        linearSystem.SetMatrixElement(idx, parent_vessels[jdx]->GetId(), -Qabs(parent_vessels[jdx]->GetFlowProperties()->GetFlowRate()/flow_rate));
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
std::cout << "Blaaaa";
                        

                        if(mUseRandomSplitting)
                        {
                            double my_value = random_assignment[idx];
                            bool my_value_highest = true;
                            for(unsigned comp_index=0; comp_index<competitor_vessels.size(); comp_index++)
                            {
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

                                linearSystem.SetMatrixElement(idx, parent_vessels[parent_index]->GetId(), -fraction);
                            }
                        }
                        else
                        {
                            QFlowRate outflow_rate = 0.0*unit::metre_cubed_per_second;
                            for(unsigned comp_index=0; comp_index<competitor_vessels.size(); comp_index++)
                            {

                                outflow_rate = outflow_rate + Qabs(competitor_vessels[comp_index]->GetFlowProperties()->GetFlowRate());
                            }
                            double fraction = Qabs(flow_rate)/(Qabs(flow_rate)+outflow_rate);
                            for(unsigned parent_index=0; parent_index<parent_vessels.size(); parent_index++)
                            {

                                linearSystem.SetMatrixElement(idx, parent_vessels[parent_index]->GetId(), -fraction);
                            }
                        }
                    }
                    else
                    {
                        QFlowRate competitor0_flow_rate = competitor_vessels[0]->GetFlowProperties()->GetFlowRate();
                        QFlowRate parent0_flow_rate = parent_vessels[0]->GetFlowProperties()->GetFlowRate();

                        // There is a bifurcation, apply a haematocrit splitting rule
                        QLength my_radius = vessels[idx]->GetRadius();
                        QLength competitor_radius = competitor_vessels[0]->GetRadius();                      

                        // Apply Yang's rule using vessel with greater radius
			if(my_radius >= competitor_radius)
			{
				localM = mYangM0*exp(-mYangk*flow_rate/parent0_flow_rate);	
			}
			else
			{
				localM = mYangM0*exp(-mYangk*competitor0_flow_rate/parent0_flow_rate);
			}
                        QDimensionless term =  pow(competitor_radius/my_radius,2.0/localM);
std::cout << "The local M is:" << localM << "  and the local term is:" << term << "\n";
//std::cout << -Qabs(parent0_flow_rate)/(Qabs(flow_rate)+Qabs(competitor0_flow_rate)*term) << "\n";
                        linearSystem.SetMatrixElement(idx, parent_vessels[0]->GetId(), -Qabs(parent0_flow_rate)/(Qabs(flow_rate)+Qabs(competitor0_flow_rate)*term));
                        
             
                    }
                }
            }
        }
    }

Vec solution = PetscTools::CreateVec(vessels.size());
linearSystem.AssembleFinalLinearSystem();
solution = linearSystem.Solve();
ReplicatableVector a(solution);

// assign haematocrit levels to vessels
for (unsigned idx = 0; idx < vessels.size(); idx++)
   {
        for (unsigned jdx = 0; jdx < vessels[idx]->GetNumberOfSegments(); jdx++)
        {
                vessels[idx]->GetSegments()[jdx]->GetFlowProperties()->SetHaematocrit(a[idx]);
        }
   }

PetscTools::Destroy(solution);

}
// Explicit instantiation
template class YangHaematocritSolver<2>;
template class YangHaematocritSolver<3>;
