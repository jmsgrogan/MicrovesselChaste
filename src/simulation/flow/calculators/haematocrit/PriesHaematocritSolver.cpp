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

#include "PriesHaematocritSolver.hpp"
#include "LinearSystem.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
PriesHaematocritSolver<DIM>::PriesHaematocritSolver() : AbstractHaematocritSolver<DIM>(),
    mHaematocrit(0.45),
    mSolveHighConnectivityNetworks(false),
    mTurnOffFungModel(false),
    mUseRandomSplitting(false),
    mExceptionOnFailedConverge(true)
{

}

template<unsigned DIM>
PriesHaematocritSolver<DIM>::~PriesHaematocritSolver()
{

}

template <unsigned DIM>
std::shared_ptr<PriesHaematocritSolver<DIM> > PriesHaematocritSolver<DIM>::Create()
{
    return std::make_shared<PriesHaematocritSolver<DIM> >();
}

template <unsigned DIM>
void PriesHaematocritSolver<DIM>::SetExceptionOnFailedConverge(bool setException)
{
    mExceptionOnFailedConverge = setException;
}

template<unsigned DIM>
void PriesHaematocritSolver<DIM>::SetUseRandomSplittingModel(bool useRandomSplittingModel)
{
    mUseRandomSplitting = useRandomSplittingModel;
}

template<unsigned DIM>
void PriesHaematocritSolver<DIM>::SetUseHigherConnectivityBranches(bool useHighConnectivity)
{
    mSolveHighConnectivityNetworks = useHighConnectivity;
}

template<unsigned DIM>
void PriesHaematocritSolver<DIM>::SetTurnOffFungModel(bool turnOffFungModel)
{
    mTurnOffFungModel = turnOffFungModel;
}

template<unsigned DIM>
void PriesHaematocritSolver<DIM>::SetHaematocrit(QDimensionless haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void PriesHaematocritSolver<DIM>::Calculate()
{
    // Give the vessels unique Ids
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
    if(lhsVectorSize > 6)
    {
        linearSystem.SetPcType("lu");
        #ifdef PETSC_HAVE_HYPRE
        linearSystem.SetPcType("hypre");
        //std::cout << "We know about HYPRE\n";
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
                            QFlowRate outflow_rate = 0.0*unit::metre_cubed_per_second;
                            for(unsigned comp_index=0; comp_index<competitor_vessels.size(); comp_index++)
                            {
                                local_update_indics.push_back(-1*competitor_vessels[comp_index]->GetId());
                                outflow_rate = outflow_rate + Qabs(competitor_vessels[comp_index]->GetFlowProperties()->GetFlowRate());
                            }
                            double fraction = Qabs(flow_rate)/(Qabs(flow_rate)+outflow_rate);
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
                        QFlowRate competitor0_flow_rate = competitor_vessels[0]->GetFlowProperties()->GetFlowRate();
                        QFlowRate parent0_flow_rate = parent_vessels[0]->GetFlowProperties()->GetFlowRate();

		
                        QDimensionless flow_ratio_pm = Qabs(parent0_flow_rate)/Qabs(flow_rate);

                        // There is a bifurcation, apply a haematocrit splitting rule from Pries1989
                        QLength my_radius = vessels[idx]->GetRadius();
                        QLength competitor_radius = competitor_vessels[0]->GetRadius();
                        QLength parent_radius = parent_vessels[0]->GetRadius();

        		double micron_my_radius = (my_radius/unit::metres)*1.e6;
        		double micron_competitor_radius = (competitor_radius/unit::metres)*1.e6;
        		double micron_parent_radius = (parent_radius/unit::metres)*1.e6;

			QDimensionless diameter_ratio = micron_my_radius/micron_competitor_radius;

			// Assign A, B and X0 from Pries1989 model
			// Here we assume that the fractional flow rate will not fall below X0 for either branch; this assumption is easily satisfied with our networks and with the splitting model without memory effects

			double X0 = 0.964*(1-parent_vessels[0]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);
        		double B = 1.0 + 6.98*(1.0-parent_vessels[0]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);

                        QDimensionless modified_flow_ratio_mc;

			modified_flow_ratio_mc = (Qabs(flow_rate)-X0*Qabs(parent0_flow_rate))/(Qabs(competitor0_flow_rate)-X0*Qabs(parent0_flow_rate));
			
        		double A = -13.29*((1.0-parent_vessels[0]->GetFlowProperties()->GetHaematocrit())*(diameter_ratio*diameter_ratio-1.0))/(2.0*micron_parent_radius*(diameter_ratio*diameter_ratio+1.0));
                        
			double term1 = pow(modified_flow_ratio_mc,B);
			double term2 = exp(A);
						
			double numer = term2*term1*flow_ratio_pm;                
                        double denom = 1.0+term2*term1;
			// Apply Pries1989 rule
                        linearSystem.SetMatrixElement(idx, parent_vessels[0]->GetId(), -numer/denom);
                     
                     

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

    // Set the parameters for iteration; we will be solving a nonlinear system
    double tolerance = 1e-10;
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
                        QFlowRate self_flow_rate = vessels[update_indices[idx][0]]->GetFlowProperties()->GetFlowRate();
                        QFlowRate outflow_rate = 0.0*unit::metre_cubed_per_second;
                        for(unsigned local_update_index=1;local_update_index<update_indices[idx].size();local_update_index++)
                        {
                            if(update_indices[idx][local_update_index]<0)
                            {
                                outflow_rate = outflow_rate + Qabs(vessels[abs(update_indices[idx][local_update_index])]->GetFlowProperties()->GetFlowRate());
                            }
                        }
                        double fraction = Qabs(self_flow_rate)/(Qabs(self_flow_rate)+outflow_rate);
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

                        QFlowRate self_flow_rate = vessels[update_indices[idx][0]]->GetFlowProperties()->GetFlowRate();
                        QFlowRate competitor0_flow_rate = vessels[update_indices[idx][2]]->GetFlowProperties()->GetFlowRate();
                        QFlowRate parent0_flow_rate = vessels[update_indices[idx][1]]->GetFlowProperties()->GetFlowRate();

			QDimensionless flow_ratio_pm = Qabs(parent0_flow_rate)/Qabs(self_flow_rate);

			QLength my_radius = vessels[update_indices[idx][0]]->GetRadius();
                   	QLength competitor_radius = vessels[update_indices[idx][2]]->GetRadius();
                        QLength parent_radius = vessels[update_indices[idx][1]]->GetRadius();
			
                

        		double micron_my_radius = (my_radius/unit::metres)*1.e6;
        		double micron_competitor_radius = (competitor_radius/unit::metres)*1.e6;
        		double micron_parent_radius = (parent_radius/unit::metres)*1.e6;

			QDimensionless diameter_ratio = micron_my_radius/micron_competitor_radius;



			double X0;
			X0 = 0.964*(1-vessels[update_indices[idx][1]]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);
        		double B = 1.0 + 6.98*(1.0-vessels[update_indices[idx][1]]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);

                        QDimensionless modified_flow_ratio_mc;
			modified_flow_ratio_mc = (Qabs(self_flow_rate)-X0*Qabs(parent0_flow_rate))/(Qabs(competitor0_flow_rate)-X0*Qabs(parent0_flow_rate));

        		double A = -13.29*((1.0-vessels[update_indices[idx][1]]->GetFlowProperties()->GetHaematocrit())*(diameter_ratio*diameter_ratio-1.0))/(2.0*micron_parent_radius*(diameter_ratio*diameter_ratio+1.0));
                        
			double term1 = pow(modified_flow_ratio_mc,B);
			double term2 = exp(A);
						
			double numer = term2*term1*flow_ratio_pm;                
                        double denom = 1.0+term2*term1;
			// Apply Pries1989 rule
                        linearSystem.SetMatrixElement(update_indices[idx][0], update_indices[idx][1], -numer/denom);
		

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
            if(std::abs(vessels[i]->GetFlowProperties()->GetHaematocrit() - a[i]) > residual)
            {
                residual = std::abs(vessels[i]->GetFlowProperties()->GetHaematocrit() - a[i]);
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
template class PriesHaematocritSolver<2>;
template class PriesHaematocritSolver<3>;