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

#ifndef FLOWSOLVER_HPP_
#define FLOWSOLVER_HPP_

#include <vector>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "Vessel.hpp"
#include "VesselNode.hpp"
#include "LinearSystem.hpp"

/**
 * This solver calculates the pressures at nodes in a vessel network and flow rates in
 * vessels by assuming mass conservation over inflows and outflows at nodes and based
 * on prescribed pressures at inlet and outlet nodes.
 */
template<unsigned DIM>
class FlowSolver
{

private:

    /**
     * Nodes in the vessel network. Stored in the flow solver to avoid
     * recalculation by the vessel network class.
     */
    std::vector<boost::shared_ptr<VesselNode<DIM> > > mNodes;

    /**
     * Vessels in the vessel network. Stored in the flow solver to avoid
     * recalculation by the vessel network class.
     */
    std::vector<boost::shared_ptr<Vessel<DIM> > > mVessels;

    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * A node-vessel connectivity map
     */
    std::vector<std::vector<unsigned> > mNodeVesselConnectivity;

    /**
     * A node-node connectivity map
     */
    std::vector<std::vector<unsigned> > mNodeNodeConnectivity;

    /**
     * Indices of nodes on the network boundary
     */
    std::vector<unsigned> mBoundaryConditionNodeIndices;

    /**
     * Indices of nodes that are not connected to the rest of the network
     */
    std::vector<unsigned> mUnconnectedNodeIndices;

    /**
     * The linear system to be solved for the nodal pressures
     */
    boost::shared_ptr<LinearSystem> mpLinearSystem;

    /**
     * Whether to use a direct or iterative solver, the former is recommended.
     */
    bool mUseDirectSolver;

    /**
     * Has the solver been set up.
     */
    bool mIsSetUp;

public:

    /**
     * Constructor.
     */
    FlowSolver();

    /**
     * Destructor.
     */
    ~FlowSolver();

    /*
     * Factor constructor. Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class.
     */
    static boost::shared_ptr<FlowSolver<DIM> > Create();

    /**
     * Set whether to use a direct solver, an iterative one is used if false (not recommended).
     * @param useDirectSolver whether to use a direct solver
     */
    void SetUseDirectSolver(bool useDirectSolver);

    /**
     * Set up the flow solver. Called the first time the solver is run.
     */
    void SetUp();

    /**
     * Set the vessel network to use in the solver.
     * @param pVesselNetwork the vessel network.
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Run the flow solver and update the pressure and flow rate data in the vessel network
     */
    void Solve();

    /**
     * Update the solver prior to each run.
     * @param runSetup whether to do a full SetUp or just update the impedances. The former is needed for
     * angiogenesis simulations.
     */
    void Update(bool runSetup=false);
};

#endif /* FLOWSOLVER_HPP_ */
