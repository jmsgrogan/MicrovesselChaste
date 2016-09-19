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

#include "GeometryTools.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "Node.hpp"
#include "NodesOnlyMesh.hpp"
#include "OffLatticeSimulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "CellsGenerator.hpp"
#include "SimulationTime.hpp"
#include "RandomNumberGenerator.hpp"
#include "OffLatticeSimulation.hpp"
#include "SimpleCell.hpp"
#include "SimpleCellCollectionModifier.hpp"

#include "NodeBasedSimulationWrapper.hpp"

NodeBasedSimulationWrapper::NodeBasedSimulationWrapper() :
    mTimeStepSize(1.0),
    mNumberOfTimeSteps(100),
    mpInputPopulation(),
    mOutputPopulations()
{

}

NodeBasedSimulationWrapper::~NodeBasedSimulationWrapper()
{

}

void NodeBasedSimulationWrapper::SetTimeStepSize(double stepSize)
{
    mTimeStepSize = stepSize;
}

void NodeBasedSimulationWrapper::SetNumberOfTimeSteps(unsigned numberOfTimeSteps)
{
    mNumberOfTimeSteps = numberOfTimeSteps;
}

void NodeBasedSimulationWrapper::SetInputPopulation(boost::shared_ptr<SimpleCellPopulation<3> > pInputPopulation)
{
    mpInputPopulation = pInputPopulation;
}

std::vector<boost::shared_ptr<SimpleCellPopulation<3> > > NodeBasedSimulationWrapper::GetOutputPopulations()
{
    return mOutputPopulations;
}

void NodeBasedSimulationWrapper::Run()
{
//    // Set up the necessary singletons
//    SimulationTime::Instance()->SetStartTime(0.0);
//    RandomNumberGenerator::Instance()->Reseed(0);
//    CellId::ResetMaxCellId();
//
//    // Create the Chaste cells using the simple cells
//    if(!mpInputPopulation)
//    {
//        EXCEPTION("An input simple cell population is required to run this simulation.");
//    }
//
//    std::vector<boost::shared_ptr<SimpleCell<3> > > simple_cells = mpInputPopulation->GetCells();
//
//    std::vector<Node<3>*> nodes;
//    for(unsigned idx=0; idx<simple_cells.size(); idx++)
//    {
//        nodes.push_back(new Node<3>(idx, simple_cells[idx]->rGetLocation(), false));
//    }
//
//    double spacing = 10.0;
//    NodesOnlyMesh<3> mesh;
//    mesh.ConstructNodesWithoutMesh(nodes, 1.5 * spacing);
//    std::vector<CellPtr> cells;
//
//    CellsGenerator<SinglePhaseCellCycleModel, 3> cells_generator;
//    cells_generator.GenerateBasic(cells, nodes.size());
//
//    // Create cell population
//    NodeBasedCellPopulation<3> cell_population(mesh, cells);
//    cell_population.SetAbsoluteMovementThreshold(2.0 * spacing);
//
//
//    OffLatticeSimulation<3> simulator(cell_population);
//    simulator.SetOutputDirectory("TemporaryWrapperDirectory");
//    simulator.SetDt(mTimeStepSize);
//    simulator.SetEndTime(mNumberOfTimeSteps * mTimeStepSize);
//    boost::shared_ptr<SimpleCellCollectionModifier<3> > p_modifier = boost::shared_ptr<SimpleCellCollectionModifier<3> >(new SimpleCellCollectionModifier<3> ());
//    simulator.AddSimulationModifier(p_modifier);
//    simulator.SetSamplingTimestepMultiple(1000);
//
//    MAKE_PTR(GeneralisedLinearSpringForce<3>, p_force);
//    simulator.AddForce(p_force);
//    simulator.Solve();
//
//    // Get the output populations
//    mOutputPopulations = p_modifier->GetSimpleCellPopulations();
//
//    // Tidy up
//    for (unsigned i=0; i<nodes.size(); i++)
//    {
//        delete nodes[i];
//    }
//
//    SimulationTime::Destroy();
//    RandomNumberGenerator::Destroy();
//    CellPropertyRegistry::Instance()->Clear();
}
