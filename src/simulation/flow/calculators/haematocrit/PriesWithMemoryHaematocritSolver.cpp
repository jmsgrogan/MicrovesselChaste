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

#include "PriesWithMemoryHaematocritSolver.hpp"
#include "LinearSystem.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
PriesWithMemoryHaematocritSolver<DIM>::PriesWithMemoryHaematocritSolver() : AbstractHaematocritSolver<DIM>(),
    mHaematocrit(0.45),
    mExceptionOnFailedConverge(true)
{

}

template<unsigned DIM>
PriesWithMemoryHaematocritSolver<DIM>::~PriesWithMemoryHaematocritSolver()
{

}

template <unsigned DIM>
std::shared_ptr<PriesWithMemoryHaematocritSolver<DIM> > PriesWithMemoryHaematocritSolver<DIM>::Create()
{
    return std::make_shared<PriesWithMemoryHaematocritSolver<DIM> >();
}

template <unsigned DIM>
void PriesWithMemoryHaematocritSolver<DIM>::SetExceptionOnFailedConverge(bool setException)
{
    mExceptionOnFailedConverge = setException;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolver<DIM>::SetHaematocrit(QDimensionless haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolver<DIM>::Calculate()
{
    // Give the vessels unique Ids
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = this->mpNetwork->GetVessels();

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
    LinearSystem linear_system(lhsVectorSize, max_vessels_per_branch);
    if(lhsVectorSize > 6)
    {
        linear_system.SetPcType("lu");
        #ifdef PETSC_HAVE_HYPRE
        linear_system.SetPcType("hypre");
        //std::cout << "We know about HYPRE\n";
        #endif //PETSC_HAVE_HYPRE
        linear_system.SetKspType("preonly");
    }

    std::vector<std::vector<int> > update_indices;
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        // Always have a diagonal entry for system, this sets zero haematocrit by default
        linear_system.SetMatrixElement(idx, idx, 1);
        if(vessels[idx]->GetStartNode()->GetFlowProperties()->IsInputNode() or
                vessels[idx]->GetEndNode()->GetFlowProperties()->IsInputNode())
        {
            linear_system.SetRhsVectorElement(idx, mHaematocrit);
        }
        // Set rhs to zero, it should already be zero but this explicitly captures the no flow case
        else if(vessels[idx]->GetFlowProperties()->GetFlowRate()==0.0*unit::metre_cubed_per_second)
        {
            linear_system.SetRhsVectorElement(idx, 0.0);
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

                // Convergence
                // If there are no competitor vessels the haematocrit is just the sum of the parent values
                if(competitor_vessels.size()==0 or Qabs(competitor_vessels[0]->GetFlowProperties()->GetFlowRate()) == 0.0 * unit::metre_cubed_per_second)
                {
                    for(unsigned jdx=0; jdx<parent_vessels.size();jdx++)
                    {
                        linear_system.SetMatrixElement(idx, parent_vessels[jdx]->GetId(), -Qabs(parent_vessels[jdx]->GetFlowProperties()->GetFlowRate()/flow_rate));
                    }
                }
                else if (competitor_vessels.size()>1 or parent_vessels.size()>1)
                {
                    EXCEPTION("This solver can only work with branches with connectivity 3");
                }
                else // Divergence (1 parent, 1 competitor)
                {
                    auto me=vessels[idx];
                    auto comp=competitor_vessels[0];
                    auto parent=parent_vessels[0];
                    UpdateBifurcation(me, comp, parent, linear_system);

                    // Save the indices for later updating
                    std::vector<int> local_update_indices = std::vector<int>(3);
                    local_update_indices[0] = me->GetId();
                    local_update_indices[1] = parent->GetId();
                    local_update_indices[2] = comp->GetId();
                    update_indices.push_back(local_update_indices);
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
            linear_system.SwitchWriteModeLhsMatrix();
            for(unsigned idx=0; idx<update_indices.size();idx++)
            {
                // same as in the initialisation step
                auto me=vessels[update_indices[idx][0]];
                auto comp=vessels[update_indices[idx][2]];
                auto parent=vessels[update_indices[idx][1]];
                UpdateBifurcation(me, comp, parent, linear_system);
            }
        }

        Vec solution = PetscTools::CreateVec(vessels.size());
        linear_system.AssembleFinalLinearSystem();
        solution = linear_system.Solve();
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
        std::cout<< Reflect() <<": iteration = "<<iterations<<"\t";
        std::cout<<"residual = "<<residual<<"\t";
        std::cout<<"max unconserved RBCs = "<<this->CheckSolution()<<"\n";
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

template<unsigned DIM>
void PriesWithMemoryHaematocritSolver<DIM>::UpdateBifurcation(std::shared_ptr<Vessel<DIM> > me, std::shared_ptr<Vessel<DIM> > comp,
                                                    std::shared_ptr<Vessel<DIM> > parent, LinearSystem& rLinearSystem)
{
    QFlowRate my_flow_rate = me->GetFlowProperties()->GetFlowRate();
    QFlowRate competitor_flow_rate = comp->GetFlowProperties()->GetFlowRate();
    QFlowRate parent_flow_rate = parent->GetFlowProperties()->GetFlowRate();
    QDimensionless flow_ratio_pm = Qabs(parent_flow_rate)/Qabs(my_flow_rate);

    // There is a bifurcation, apply a haematocrit splitting rule from Pries1989
    double micron_my_radius = (me->GetRadius()/unit::metres)*1.e6;
    double micron_competitor_radius = (comp->GetRadius()/unit::metres)*1.e6;
    double micron_parent_radius = (parent->GetRadius()/unit::metres)*1.e6;
    QDimensionless diameter_ratio = micron_my_radius/micron_competitor_radius;

    QDimensionless parent_haematocrit = parent->GetFlowProperties()->GetHaematocrit();

    // new bits with memory effects for dichotomous networks follow

    double cfl_term = 1000.0;   // this corresponds to A^{shift} \times f(l;D_P) in our paper, and should be changed to something reasonable for all vessels in this process

    // get distance to previous bifurcation (also get the value in microns)
    QLength dist_to_prev_bif = me->GetDistToPrevBif();
    double micron_distTPB = (dist_to_prev_bif/unit::metres)*1.e6;
    double A_shift = 0.5;

    // X0 and its values for favourable and unfavourable branches

    double X0 = 0.964*(1-parent_haematocrit)/(2.0*micron_parent_radius);
    double X0_favor = X0*(1.0-exp(-micron_distTPB/(8*micron_parent_radius)));
    double X0_unfavor = X0*(1.0+exp(-micron_distTPB/(8*micron_parent_radius)));
    double B = 1.0 + 6.98*(1.0-parent_haematocrit)/(2.0*micron_parent_radius);

    QDimensionless modified_flow_ratio_mc;

    if(parent->GetStartNode()->GetFlowProperties()->IsInputNode() or
            parent->GetEndNode()->GetFlowProperties()->IsInputNode())
    {
        cfl_term = 0.0;
    }
    else if(me->GetPreference() == 1)
    {
        cfl_term = A_shift*exp(-micron_distTPB/(8*micron_parent_radius)); // omega is approximately 4; 2*radius = diameter
    }
    else
    {
        cfl_term = -A_shift*exp(-micron_distTPB/(8*micron_parent_radius));
    }

    double A = -13.29*((1.0-parent_haematocrit)*(diameter_ratio*diameter_ratio-1.0))/(2.0*micron_parent_radius*(diameter_ratio*diameter_ratio+1.0))+cfl_term;

    double term2 = exp(A);
    // Apply extended Pries rule

    if(parent->GetStartNode()->GetFlowProperties()->IsInputNode() or
                parent->GetEndNode()->GetFlowProperties()->IsInputNode())
    {
        if(Qabs(my_flow_rate)/Qabs(parent_flow_rate) < X0)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), 0.0);
        }
        else if(Qabs(competitor_flow_rate)/Qabs(parent_flow_rate) < X0)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -Qabs(parent_flow_rate)/Qabs(my_flow_rate));
        }
        else
        {
            modified_flow_ratio_mc = (Qabs(my_flow_rate)-X0*Qabs(parent_flow_rate))/(Qabs(competitor_flow_rate)-X0*Qabs(parent_flow_rate));
            double term1 = pow(modified_flow_ratio_mc,B);
            double numer = term2*term1*flow_ratio_pm;
            double denom = 1.0+term2*term1;
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -numer/denom);
        }
    }
    else if(me->GetPreference() == 1)
    {
        if(Qabs(my_flow_rate)/Qabs(parent_flow_rate) < X0_favor)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), 0.0);
        }
        else if(Qabs(competitor_flow_rate)/Qabs(parent_flow_rate) < X0_unfavor)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -Qabs(parent_flow_rate)/Qabs(my_flow_rate));
        }
        else
        {
            modified_flow_ratio_mc = (Qabs(my_flow_rate)-X0_favor*Qabs(parent_flow_rate))/(Qabs(competitor_flow_rate)-X0_unfavor*Qabs(parent_flow_rate));
            double term1 = pow(modified_flow_ratio_mc,B);
            double numer = term2*term1*flow_ratio_pm;
            double denom = 1.0+term2*term1;
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -numer/denom);
        }
    }
    else
    {
        if(Qabs(my_flow_rate)/Qabs(parent_flow_rate) < X0_unfavor)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), 0.0);
        }
        else if(Qabs(competitor_flow_rate)/Qabs(parent_flow_rate) < X0_favor)
        {
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -Qabs(parent_flow_rate)/Qabs(my_flow_rate));
        }
        else
        {
            modified_flow_ratio_mc = (Qabs(my_flow_rate)-X0_unfavor*Qabs(parent_flow_rate))/(Qabs(competitor_flow_rate)-X0_favor*Qabs(parent_flow_rate));
            double term1 = pow(modified_flow_ratio_mc,B);
            double numer = term2*term1*flow_ratio_pm;
            double denom = 1.0+term2*term1;
            rLinearSystem.SetMatrixElement(me->GetId(), parent->GetId(), -numer/denom);
        }
    }
}

// Explicit instantiation
template class PriesWithMemoryHaematocritSolver<2>;
template class PriesWithMemoryHaematocritSolver<3>;
