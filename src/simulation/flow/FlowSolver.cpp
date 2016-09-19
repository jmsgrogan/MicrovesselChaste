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

#include <algorithm>
#include "Exception.hpp"
#include "ReplicatableVector.hpp"
#include "PetscTools.hpp"
#include "VesselSegment.hpp"
#include "FlowSolver.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkGraphCalculator.hpp"

template<unsigned DIM>
FlowSolver<DIM>::FlowSolver()
    :   mNodes(),
        mVessels(),
        mpVesselNetwork(),
        mNodeVesselConnectivity(),
        mNodeNodeConnectivity(),
        mBoundaryConditionNodeIndices(),
        mUnconnectedNodeIndices(),
        mpLinearSystem(),
        mUseDirectSolver(true),
        mIsSetUp(false)
{

}

template<unsigned DIM>
FlowSolver<DIM>::~FlowSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<FlowSolver<DIM> > FlowSolver<DIM>::Create()
{
    MAKE_PTR(FlowSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void FlowSolver<DIM>::SetUp()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required before calling SetUp");
    }
    mVessels = mpVesselNetwork->GetVessels();
    mNodes = mpVesselNetwork->GetVesselEndNodes();

    unsigned num_nodes = mNodes.size();
    unsigned max_branches = mpVesselNetwork->GetMaxBranchesOnNode();

    // Set up the system
    mpLinearSystem = boost::shared_ptr<LinearSystem>(new LinearSystem(num_nodes, max_branches + 1));

    // If the network is small the preconditioner is turned off in LinearSystem,
    // so an iterative solver is used instead.
    if (num_nodes >= 6 && mUseDirectSolver)
    {
        mpLinearSystem->SetPcType("lu");
        mpLinearSystem->SetKspType("preonly");
    }

    // Get the node-vessel and node-node connectivity
    boost::shared_ptr<VesselNetworkGraphCalculator<DIM> > p_graph_calculator = VesselNetworkGraphCalculator<DIM>::Create();
    p_graph_calculator->SetVesselNetwork(mpVesselNetwork);
    mNodeVesselConnectivity = p_graph_calculator->GetNodeVesselConnectivity();
    mNodeNodeConnectivity = p_graph_calculator->GetNodeNodeConnectivity();

    // Get the boundary condition nodes
    std::vector<boost::shared_ptr<VesselNode<DIM> > > boundary_condition_nodes;
    for (unsigned node_index = 0; node_index < num_nodes; node_index++)
    {
        if (mNodes[node_index]->GetFlowProperties()->IsInputNode()
                || mNodes[node_index]->GetFlowProperties()->IsOutputNode())
        {
            boundary_condition_nodes.push_back(mNodes[node_index]);
            mBoundaryConditionNodeIndices.push_back(node_index);
        }
    }

    // Get the nodes that correspond to segments that are not connected to the rest of the network
    std::vector<bool> connected = p_graph_calculator->IsConnected(boundary_condition_nodes, mNodes);
    std::vector<unsigned> mUnconnectedNodeIndices;
    for (unsigned node_index = 0; node_index < num_nodes; node_index++)
    {
        if (!connected[node_index])
        {
            mUnconnectedNodeIndices.push_back(node_index);
        }
    }

    mIsSetUp = true;
    Update(false);
}

template<unsigned DIM>
void FlowSolver<DIM>::SetUseDirectSolver(bool useDirectSolver)
{
    mUseDirectSolver = useDirectSolver;
}

template<unsigned DIM>
void FlowSolver<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    mpVesselNetwork = pVesselNetwork;
}

template<unsigned DIM>
void FlowSolver<DIM>::Update(bool runSetup)
{
    if(!mIsSetUp or runSetup)
    {
        SetUp();
    }

    mpLinearSystem->SwitchWriteModeLhsMatrix();
    mpLinearSystem->ZeroLhsMatrix();

    // Get the impedances, scale them by the maximum impedance to remove small values from the system matrix
    std::vector<units::quantity<unit::flow_impedance> > scaled_impedances;
    units::quantity<unit::flow_impedance> max_impedance = 0.0 * unit::pascal_second_per_metre_cubed;
    units::quantity<unit::flow_impedance> min_impedance = DBL_MAX * unit::pascal_second_per_metre_cubed;
    for (unsigned vessel_index = 0; vessel_index < mVessels.size(); vessel_index++)
    {
        units::quantity<unit::flow_impedance> impedance = mVessels[vessel_index]->GetFlowProperties()->GetImpedance();
        if (impedance <= 0.0 * unit::pascal_second_per_metre_cubed)
        {
            EXCEPTION("Impedance should be a positive number.");
        }
        if(impedance > max_impedance)
        {
            max_impedance = impedance;
        }
        if(impedance < min_impedance)
        {
            min_impedance = impedance;
        }
        scaled_impedances.push_back(impedance);
    }
    units::quantity<unit::flow_impedance> multipler = (max_impedance + min_impedance) / 2.0; //scale impedances to avoid floating point problems in PETSC solvers.

    // Set up the system matrix
    for (unsigned node_index = 0; node_index < mNodes.size(); node_index++)
    {
        bool is_bc_node = (std::find(mBoundaryConditionNodeIndices.begin(), mBoundaryConditionNodeIndices.end(),
                                     node_index) != mBoundaryConditionNodeIndices.end());
        bool is_unconnected_node = (std::find(mUnconnectedNodeIndices.begin(), mUnconnectedNodeIndices.end(),
                                              node_index) != mUnconnectedNodeIndices.end());

        if (is_bc_node or is_unconnected_node)
        {
            mpLinearSystem->AddToMatrixElement(node_index, node_index, 1.0);
            if(mNodes[node_index]->GetFlowProperties()->UseVelocityBoundaryCondition())
            {
                // Velocity BC: Assumes only single vessel at inlets
                mpLinearSystem->AddToMatrixElement(node_index, mNodeNodeConnectivity[node_index][0], -1.0);
            }

        }
        else
        {
            for (unsigned vessel_index = 0; vessel_index < mNodeVesselConnectivity[node_index].size(); vessel_index++)
            {
                units::quantity<unit::flow_impedance> impedance = scaled_impedances[mNodeVesselConnectivity[node_index][vessel_index]];
                // Add the inverse impedances to the linear system
                mpLinearSystem->AddToMatrixElement(node_index, node_index, -multipler / impedance); // Aii
                mpLinearSystem->AddToMatrixElement(node_index, mNodeNodeConnectivity[node_index][vessel_index], multipler / impedance); // Aij
            }
        }
    }

    mpLinearSystem->AssembleIntermediateLinearSystem();
    // Update the RHS
    for (unsigned bc_index = 0; bc_index < mBoundaryConditionNodeIndices.size(); bc_index++)
    {
        if(mNodes[mBoundaryConditionNodeIndices[bc_index]]->GetFlowProperties()->UseVelocityBoundaryCondition())
        {
            boost::shared_ptr<Vessel<DIM> > p_vessel = mNodes[mBoundaryConditionNodeIndices[bc_index]]->GetSegment(0)->GetVessel();
            units::quantity<unit::flow_rate> flow_rate = boost::units::fabs(p_vessel->GetFlowProperties()->GetFlowRate());
            units::quantity<unit::flow_impedance> impedance = p_vessel->GetFlowProperties()->GetImpedance();
            double pressure_drop = flow_rate * impedance/ unit::pascals;
            mpLinearSystem->SetRhsVectorElement(mBoundaryConditionNodeIndices[bc_index], pressure_drop);
        }
        else
        {
            mpLinearSystem->SetRhsVectorElement(
                    mBoundaryConditionNodeIndices[bc_index],
                    mNodes[mBoundaryConditionNodeIndices[bc_index]]->GetFlowProperties()->GetPressure()/unit::pascals);
        }
    }
}

template<unsigned DIM>
void FlowSolver<DIM>::Solve()
{
    if (!mIsSetUp)
    {
        SetUp();
    }

    // Assemble and solve the final system
    mpLinearSystem->AssembleFinalLinearSystem();
    Vec solution = PetscTools::CreateVec(mNodes.size());
    solution = mpLinearSystem->Solve();

    // Recover the pressure of the vessel nodes
    ReplicatableVector a(solution);
    for (unsigned node_index = 0; node_index < mNodes.size(); node_index++)
    {
        mNodes[node_index]->GetFlowProperties()->SetPressure(a[node_index] * unit::pascals);
    }

    // Set the segment flow rates and nodal pressures
    for (unsigned vessel_index = 0; vessel_index < mVessels.size(); vessel_index++)
    {
        units::quantity<unit::pressure> start_node_pressure = mVessels[vessel_index]->GetStartNode()->GetFlowProperties()->GetPressure();
        units::quantity<unit::pressure> end_node_pressure = mVessels[vessel_index]->GetEndNode()->GetFlowProperties()->GetPressure();
        units::quantity<unit::flow_rate> flow_rate = (start_node_pressure - end_node_pressure) / mVessels[vessel_index]->GetFlowProperties()->GetImpedance();

        // Clean up small values as some structural adaptation calculators are sensitive to them.
        if (fabs(flow_rate) < pow(10, -20)*unit::metre_cubed_per_second)
        {
            flow_rate = 0.0 * unit::metre_cubed_per_second;
        }

        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = mVessels[vessel_index]->GetSegments();
        units::quantity<unit::pressure> pressure = start_node_pressure;
        for (unsigned segment_index = 0; segment_index < segments.size() - 1; segment_index++)
        {
            pressure -= segments[segment_index]->GetFlowProperties()->GetImpedance() * flow_rate;
            segments[segment_index]->GetNode(1)->GetFlowProperties()->SetPressure(pressure);
            segments[segment_index]->GetFlowProperties()->SetFlowRate(flow_rate);
        }
        segments[segments.size() - 1]->GetFlowProperties()->SetFlowRate(flow_rate);
    }

    // Clean up
    PetscTools::Destroy(solution);
}

// Explicit instantiation
template class FlowSolver<2> ;
template class FlowSolver<3> ;

