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

#include "PriesWithMemoryHaematocritSolverNonLinear.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "RandomNumberGenerator.hpp"
#include <iostream>

template<unsigned DIM>
PriesWithMemoryHaematocritSolverNonLinear<DIM>::PriesWithMemoryHaematocritSolverNonLinear() : AbstractHaematocritSolver<DIM>(),
    mHaematocrit(0.45),
    mSolveHighConnectivityNetworks(false),
    mTurnOffFungModel(false),
    mUseRandomSplitting(false),
    mExceptionOnFailedConverge(true)
{

}

template<unsigned DIM>
PriesWithMemoryHaematocritSolverNonLinear<DIM>::~PriesWithMemoryHaematocritSolverNonLinear()
{

}

template <unsigned DIM>
std::shared_ptr<PriesWithMemoryHaematocritSolverNonLinear<DIM> > PriesWithMemoryHaematocritSolverNonLinear<DIM>::Create()
{
    return std::make_shared<PriesWithMemoryHaematocritSolverNonLinear<DIM> >();
}

template <unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::SetExceptionOnFailedConverge(bool setException)
{
    mExceptionOnFailedConverge = setException;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::SetUseRandomSplittingModel(bool useRandomSplittingModel)
{
    mUseRandomSplitting = useRandomSplittingModel;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::SetUseHigherConnectivityBranches(bool useHighConnectivity)
{
    mSolveHighConnectivityNetworks = useHighConnectivity;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::SetTurnOffFungModel(bool turnOffFungModel)
{
    mTurnOffFungModel = turnOffFungModel;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::SetHaematocrit(QDimensionless haematocrit)
{
    mHaematocrit = haematocrit;
}

template<unsigned DIM>
void PriesWithMemoryHaematocritSolverNonLinear<DIM>::Calculate()
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

    //Initialise the vessel network by setting the haematocrit for input vessels to the artery haematocrit
    //Identify the inlet vessels and add them to the covered set of indices
    std::vector<unsigned> completed_indices;
    std::vector<unsigned> leading_indices;
    std::vector<unsigned> new_leading_indices;
    std::vector<bool> vessel_covered(vessels.size(), false);
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        // Always have a diagonal entry for system, this sets zero haematocrit by default
        if(vessels[idx]->GetStartNode()->GetFlowProperties()->IsInputNode() or
                vessels[idx]->GetEndNode()->GetFlowProperties()->IsInputNode())
        {
            vessels[idx]->GetFlowProperties()->SetHaematocrit(mHaematocrit);
            leading_indices.push_back(idx);
            completed_indices.push_back(idx);
            vessel_covered[idx] = true;
        }
    }
    QDimensionless NewHaematocrit;
    // While the set of complete indices does not equal the full set of indices
    while(completed_indices.size() < vessels.size())
    {
        // loop through all the previously calculate haematocrits to solve for the next set of vessels
        for(auto t=leading_indices.begin(); t!=leading_indices.end(); ++t)
        {
          // Identify inflow node
          std::shared_ptr<VesselNode<DIM> > p_outflow_node;
          if(vessels[*t]->GetStartNode()->GetFlowProperties()->GetPressure() < vessels[*t]->GetEndNode()->GetFlowProperties()->GetPressure())
          {
              p_outflow_node = vessels[*t]->GetStartNode();
          }
          else
          {
              p_outflow_node = vessels[*t]->GetEndNode();
          }
          if(p_outflow_node->GetNumberOfSegments() == 3)
          {
            // Vector to store which vessel have been resolved
            std::vector<bool> vessel_resolution;
            // Vector to identify which way the blood flows at this junction
            // True means they flow away from the junction, False means towards
            std::vector<bool> vessel_flow_prop;
            // Vector to store the indices of the vessels being looked at
            std::vector<unsigned> vessel_indxs;

            // Identifying these properties for the vessels which have not yet been covered
            for(unsigned jdx=0; jdx<p_outflow_node->GetNumberOfSegments(); jdx++)
            {
              if(p_outflow_node->GetSegment(jdx)->GetVessel()->GetId() != *t)
              {
                vessel_indxs.push_back(p_outflow_node->GetSegment(jdx)->GetVessel()->GetId());
                vessel_resolution.push_back(vessel_covered[p_outflow_node->GetSegment(jdx)->GetVessel()->GetId()]);
                if(p_outflow_node->GetSegment(jdx)->GetVessel()->GetStartNode() == p_outflow_node)
                {
                  if(p_outflow_node->GetSegment(jdx)->GetVessel()->GetEndNode()->GetFlowProperties()->GetPressure() > p_outflow_node->GetFlowProperties()->GetPressure())
                  {
                    vessel_flow_prop.push_back(false);
                  }
                  else
                  {
                    vessel_flow_prop.push_back(true);
                  }
                }
                else
                {
                  if(p_outflow_node->GetSegment(jdx)->GetVessel()->GetStartNode()->GetFlowProperties()->GetPressure() > p_outflow_node->GetFlowProperties()->GetPressure())
                  {
                    vessel_flow_prop.push_back(false);
                  }
                  else
                  {
                    vessel_flow_prop.push_back(true);
                  }
                }

              }
            }

            // Identifying the flow rates of the three vessels
            QFlowRate split_vessel0_flow_rate = vessels[vessel_indxs[0]]->GetFlowProperties()->GetFlowRate();
            QFlowRate split_vessel1_flow_rate = vessels[vessel_indxs[1]]->GetFlowProperties()->GetFlowRate();
            QFlowRate current_vessel_flow_rate = vessels[*t]->GetFlowProperties()->GetFlowRate();
            // Indentifying the different cases
            /* First Case:
             * Both of the vessels connecting to the leading vessels outlet node have already been resolved.
             * No action needs to be taken.
             */
            if(vessel_resolution[0] && vessel_resolution[1])
            {
            }
            /* Second Case:
             * Neither vessel has been resolved and the junction of the three vessels is a bifurcation.
             * The ratio of haematocrit needs to be solved then appropriate haematocrit set.
             */
            else if(vessel_flow_prop[0] && vessel_flow_prop[1])
            {
              // There is a bifurcation, apply a haematocrit splitting rule
              QLength parent_radius = vessels[*t]->GetRadius();
              double micron_parent_radius = (parent_radius/unit::metres)*1.e6;

              QLength dist_to_prev_bif;
              double micron_distTPB;

              unsigned pref_indx, nonpref_indx;
              double X0, X0_favor, X0_unfavor, A_shift;
              QFlowRate pref_vessel_flow, nonpref_vessel_flow;
              QLength pref_vessel_radius, nonpref_vessel_radius;

              X0 = 0.964*(1-vessels[*t]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);
              double B = 1.0 + 6.98*(1.0-vessels[*t]->GetFlowProperties()->GetHaematocrit())/(2.0*micron_parent_radius);
              // Identify the preference vessel if any and set the appropriate constants.
              if(vessels[*t]->GetStartNode()->GetFlowProperties()->IsInputNode() or
                                vessels[*t]->GetEndNode()->GetFlowProperties()->IsInputNode())
              {
                pref_indx = vessel_indxs[0];
                nonpref_indx = vessel_indxs[1];

                pref_vessel_flow = split_vessel0_flow_rate;
                nonpref_vessel_flow = split_vessel1_flow_rate;

                dist_to_prev_bif = vessels[pref_indx]->GetDistToPrevBif();
                // dist_to_prev_bif = vessels[*t]->GetDistToPrevBif();
                micron_distTPB = (dist_to_prev_bif/unit::metres)*1.e6;

                X0_favor = X0;
                X0_unfavor = X0;
                A_shift = 0.0;
              }
              else if(vessels[vessel_indxs[0]]->GetPreference() == 1)
              {
                pref_indx = vessel_indxs[0];
                nonpref_indx = vessel_indxs[1];

                pref_vessel_flow = split_vessel0_flow_rate;
                nonpref_vessel_flow = split_vessel1_flow_rate;

                dist_to_prev_bif = vessels[pref_indx]->GetDistToPrevBif();
                // dist_to_prev_bif = vessels[*t]->GetDistToPrevBif();
                micron_distTPB = (dist_to_prev_bif/unit::metres)*1.e6;

                X0_favor = X0*(1.0-exp(-micron_distTPB/(8*micron_parent_radius)));
                X0_unfavor = X0*(1.0+exp(-micron_distTPB/(8*micron_parent_radius)));
                A_shift = 0.5;
              }
              else if(vessels[vessel_indxs[1]]->GetPreference() == 1)
              {
                pref_indx = vessel_indxs[1];
                nonpref_indx = vessel_indxs[0];

                pref_vessel_flow = split_vessel1_flow_rate;
                nonpref_vessel_flow = split_vessel0_flow_rate;

                dist_to_prev_bif = vessels[pref_indx]->GetDistToPrevBif();
                // dist_to_prev_bif = vessels[*t]->GetDistToPrevBif();
                micron_distTPB = (dist_to_prev_bif/unit::metres)*1.e6;

                X0_favor = X0*(1.0-exp(-micron_distTPB/(8*micron_parent_radius)));
                X0_unfavor = X0*(1.0+exp(-micron_distTPB/(8*micron_parent_radius)));
                A_shift = 0.5;
              }
              else
              {
                pref_indx = vessel_indxs[1];
                nonpref_indx = vessel_indxs[0];

                pref_vessel_flow = split_vessel1_flow_rate;
                nonpref_vessel_flow = split_vessel0_flow_rate;

                dist_to_prev_bif = vessels[pref_indx]->GetDistToPrevBif();
                // dist_to_prev_bif = vessels[*t]->GetDistToPrevBif();
                micron_distTPB = (dist_to_prev_bif/unit::metres)*1.e6;

                X0_favor = X0;
                X0_unfavor = X0;
                A_shift = 0.0;
              }

              pref_vessel_radius = vessels[pref_indx]->GetRadius();
              nonpref_vessel_radius = vessels[nonpref_indx]->GetRadius();

              QLength micron_pref_vessel_radius = (pref_vessel_radius/unit::metres)*1.e6;
              QLength micron_nonpref_vessel_radius = (nonpref_vessel_radius/unit::metres)*1.e6;

              QDimensionless diameter_ratio = micron_pref_vessel_radius/micron_nonpref_vessel_radius;

              // Identify if the flow rate for the vessel is enough for RBCs to enter the vessel.
              if(Qabs(pref_vessel_flow) < X0_favor*Qabs(current_vessel_flow_rate))
              {
                // All the RBCs enter the non-preferential vessel
                vessels[pref_indx]->GetFlowProperties()->SetHaematocrit(0.0);
                vessels[nonpref_indx]->GetFlowProperties()
                ->SetHaematocrit(vessels[*t]->GetFlowProperties()->GetHaematocrit()*Qabs(current_vessel_flow_rate)/Qabs(nonpref_vessel_flow));
              }
              else if(Qabs(nonpref_vessel_flow) < X0_unfavor*Qabs(current_vessel_flow_rate))
              {
                // All the RBCs enter the preferential vessel
                vessels[nonpref_indx]->GetFlowProperties()->SetHaematocrit(0.0);
                NewHaematocrit = vessels[*t]->GetFlowProperties()->GetHaematocrit()*Qabs(current_vessel_flow_rate)/Qabs(pref_vessel_flow);
                if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
                {
                  EXCEPTION("Haematocrit value out of bounds");
                }
                vessels[pref_indx]->GetFlowProperties()->SetHaematocrit(NewHaematocrit);
              }
              else
              {
                // The RBCs are split according to the formula for the ratio
                double A = -13.29*((1.0-vessels[*t]->GetFlowProperties()->GetHaematocrit())*(diameter_ratio*diameter_ratio-1.0))/(2.0*micron_parent_radius*(diameter_ratio*diameter_ratio+1.0))+A_shift*exp(-micron_distTPB/(8*micron_parent_radius));
                double modified_flow_ratio_mc = (Qabs(pref_vessel_flow) - X0_favor*Qabs(current_vessel_flow_rate))/
                (Qabs(nonpref_vessel_flow) - X0_unfavor*Qabs(current_vessel_flow_rate));
                double term2 = exp(A);
                double term1 = pow(modified_flow_ratio_mc,B);
                QDimensionless haematocrit_ratio = term1*term2;

                NewHaematocrit = vessels[*t]->GetFlowProperties()->GetHaematocrit()*current_vessel_flow_rate/(haematocrit_ratio*nonpref_vessel_flow + nonpref_vessel_flow);
                if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
                {
                  EXCEPTION("Haematocrit value out of bounds");
                }
                if(isnan(NewHaematocrit))
                {
                  std::cout << "Haematocrit is nan for vessel " << nonpref_indx << std::endl;
                  EXCEPTION("Haematocrit is nan");
                }
                vessels[nonpref_indx]->GetFlowProperties()->SetHaematocrit(NewHaematocrit);

                NewHaematocrit = (vessels[*t]->GetFlowProperties()->GetHaematocrit()*current_vessel_flow_rate - NewHaematocrit*nonpref_vessel_flow)/pref_vessel_flow;
                if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
                {
                  EXCEPTION("Haematocrit value out of bounds");
                }
                if(isnan(NewHaematocrit))
                {
                  std::cout << "Haematocrit is nan for vessel " << pref_indx << std::endl;
                  EXCEPTION("Haematocrit is nan");
                }
                vessels[pref_indx]->GetFlowProperties()->SetHaematocrit(NewHaematocrit);
                }

                // The set of completed indices is updated.
                completed_indices.push_back(vessel_indxs[0]);
                completed_indices.push_back(vessel_indxs[1]);
                // The vessels are set to covered so that the values are not re-calculated.
                vessel_covered[vessel_indxs[0]] = true;
                vessel_covered[vessel_indxs[1]] = true;
                // The newly calculated vessels are added to the next set of leading vessels.
                new_leading_indices.push_back(vessel_indxs[0]);
                new_leading_indices.push_back(vessel_indxs[1]);
            }
            /* Third Case:
             * Two resolved vessles are flowing into and unresolved vessel.
             * The unresolved vessel's haematocrit can be calculated using the conservation laws.
             */
            else if(vessel_flow_prop[0] && vessel_resolution[1] && !vessel_resolution[0])
            {
              NewHaematocrit = (current_vessel_flow_rate*vessels[*t]->GetFlowProperties()->GetHaematocrit()+split_vessel1_flow_rate*vessels[vessel_indxs[1]]->GetFlowProperties()->GetHaematocrit())/split_vessel0_flow_rate;
              if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
              {
                EXCEPTION("Haematocrit value out of bounds");
              }
              if(isnan(NewHaematocrit))
              {
                std::cout << "Haematocrit is nan for vessel " << vessel_indxs[0] << std::endl;
                EXCEPTION("Haematocrit is nan");
              }
              vessels[vessel_indxs[0]]->GetFlowProperties()->SetHaematocrit(NewHaematocrit);
              // The set of completed indices is updated.
              completed_indices.push_back(vessel_indxs[0]);
              // The vessel is then set to covered so the value is not re-calculated.
              vessel_covered[vessel_indxs[0]] = true;
              // The newly calculated vessel is then add to the next set of leading vessels.
              new_leading_indices.push_back(vessel_indxs[0]);
            }
            else if(vessel_flow_prop[1] && vessel_resolution[0] && !vessel_resolution[1])
            {
              NewHaematocrit = (current_vessel_flow_rate*vessels[*t]->GetFlowProperties()->GetHaematocrit()+split_vessel0_flow_rate*vessels[vessel_indxs[0]]->GetFlowProperties()->GetHaematocrit())/split_vessel1_flow_rate;
              if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
              {
                EXCEPTION("Haematocrit value out of bounds");
              }
              if(isnan(NewHaematocrit))
              {
                std::cout << "Haematocrit is nan for vessel " << vessel_indxs[1] << std::endl;
                EXCEPTION("Haematocrit is nan");
              }
              vessels[vessel_indxs[1]]->GetFlowProperties()->SetHaematocrit(NewHaematocrit);
              // The set of completed indices is updated.
              completed_indices.push_back(vessel_indxs[1]);
              // The vessel is then set to covered so the value is not re-calculated.
              vessel_covered[vessel_indxs[1]] = true;
              // The newly calculated vessel is then add to the next set of leading vessels.
              new_leading_indices.push_back(vessel_indxs[1]);
            }
            /* Fourth Case:
             * One unresolved vessel is flowing into another unresolved vessel.
             * No action to calculate either of these can be taken.
             * The leading vessel will remain a leading vessel until the unresolved vessel
             * which is flowing into the next unresolved vessel has been resolved
             */
            else if(vessel_flow_prop[0] || vessel_flow_prop[1])
            {
              new_leading_indices.push_back(*t);
            }
            // Clearing vector quanties to be used in the next iteration.
            vessel_indxs.clear();
            vessel_resolution.clear();
            vessel_flow_prop.clear();
          }
          // Continuation of the previous vessel, possibly a change in the diameter
          else if(p_outflow_node->GetNumberOfSegments() == 2)
          {
            for(unsigned jdx = 0; jdx < p_outflow_node->GetNumberOfSegments(); jdx++)
            {
              if(p_outflow_node->GetSegment(jdx)->GetVessel()->GetId() != *t)
              {
                NewHaematocrit = Qabs(vessels[*t]->GetFlowProperties()
                ->GetHaematocrit()*vessels[*t]->GetFlowProperties()
                ->GetFlowRate()/p_outflow_node->GetSegment(jdx)->GetVessel()
                ->GetFlowProperties()->GetFlowRate());
                if(NewHaematocrit < 0.0 || NewHaematocrit > 1)
                {
                  EXCEPTION("Haematocrit value out of bounds");
                }
                if(isnan(NewHaematocrit))
                {
                  std::cout << "Haematocrit is nan for vessel " << p_outflow_node->GetSegment(jdx)->GetVessel()->GetId() << std::endl;
                  EXCEPTION("Haematocrit is nan");
                }
                p_outflow_node->GetSegment(jdx)->GetVessel()->GetFlowProperties()
                ->SetHaematocrit(NewHaematocrit);

                completed_indices.push_back(p_outflow_node->GetSegment(jdx)->GetVessel()->GetId());
                vessel_covered[p_outflow_node->GetSegment(jdx)->GetVessel()->GetId()] = true;
                new_leading_indices.push_back(p_outflow_node->GetSegment(jdx)->GetVessel()->GetId());
              }
            }
          }
          // The case with 4 or more vessels can not be calculated with this method
          else if(p_outflow_node->GetNumberOfSegments() > 3)
          {
            EXCEPTION("This solver can only work with branches with connectivity 3");
          }
        }
        // The new leading indices are then set for the next iteration.
        leading_indices.clear();
        leading_indices = new_leading_indices;
        new_leading_indices.clear();
    }
}

// Explicit instantiation
template class PriesWithMemoryHaematocritSolverNonLinear<2>;
template class PriesWithMemoryHaematocritSolverNonLinear<3>;
