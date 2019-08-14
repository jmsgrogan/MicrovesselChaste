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

#include "AbstractHaematocritSolver.hpp"

template<unsigned DIM>
AbstractHaematocritSolver<DIM>::AbstractHaematocritSolver() : AbstractVesselNetworkCalculator<DIM>()
{

}

template<unsigned DIM>
AbstractHaematocritSolver<DIM>::~AbstractHaematocritSolver()
{

}

template<unsigned DIM>
QDimensionless AbstractHaematocritSolver<DIM>::CheckSolution()
{
  QDimensionless sup_rbc = 0.0;
  QDimensionless node_rbc;
  QDimensionless abs_node_rbc;
  std::vector<std::shared_ptr<VesselSegment<DIM> >> current_vessels;
  std::shared_ptr<Vessel<DIM> > current_vessel;
  std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = this->mpNetwork->GetNodes();
  for(unsigned i = 0; i < nodes.size(); i++)
  {
    node_rbc = 0.0;
    abs_node_rbc = 0.0;
    double number_of_vessels = nodes[i]->GetNumberOfSegments();
    if(!nodes[i]->GetFlowProperties()->IsInputNode() && !nodes[i]->GetFlowProperties()->IsOutputNode())
    {
      current_vessels = nodes[i]->GetSegments();
      for(unsigned j = 0; j < current_vessels.size(); j++)
      {
        current_vessel = current_vessels[j]->GetVessel();
        abs_node_rbc = abs_node_rbc + Qabs(current_vessel->GetFlowProperties()->GetFlowRate()*current_vessel->GetFlowProperties()->GetHaematocrit());
        if(current_vessel->GetStartNode() == nodes[i])
        {
          if(current_vessel->GetEndNode()->GetFlowProperties()->GetPressure() > nodes[i]->GetFlowProperties()->GetPressure())
          {
            node_rbc = node_rbc + Qabs(current_vessel->GetFlowProperties()->GetFlowRate()*current_vessel->GetFlowProperties()->GetHaematocrit());
          }
          else
          {
            node_rbc = node_rbc - Qabs(current_vessel->GetFlowProperties()->GetFlowRate()*current_vessel->GetFlowProperties()->GetHaematocrit());
          }
        }
        else
        {
          if(current_vessel->GetStartNode()->GetFlowProperties()->GetPressure() > nodes[i]->GetFlowProperties()->GetPressure())
          {
            node_rbc = node_rbc + Qabs(current_vessel->GetFlowProperties()->GetFlowRate()*current_vessel->GetFlowProperties()->GetHaematocrit());
          }
          else
          {
            node_rbc = node_rbc - Qabs(current_vessel->GetFlowProperties()->GetFlowRate()*current_vessel->GetFlowProperties()->GetHaematocrit());
          }
        }
      }
      if(sup_rbc < number_of_vessels*Qabs(node_rbc)/abs_node_rbc)
      {
        sup_rbc = number_of_vessels*Qabs(node_rbc)/abs_node_rbc;
      }
    }
  }
  return sup_rbc;
}

// Explicit instantiation
template class AbstractHaematocritSolver<2>;
template class AbstractHaematocritSolver<3>;
