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

#include "SimulationTime.hpp"
#include "OnLatticeSimulation.hpp"
#include "PetscSetupUtils.hpp"
#include "SimpleCellCollectionModifier.hpp"
#include "CellLabelWriter.hpp"
#include "CellProliferativePhasesCountWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "CellProliferativeTypesWriter.hpp"
#include "CellProliferativePhasesWriter.hpp"
#include "ApoptoticCellKiller.hpp"
#include "StalkCellMutationState.hpp"
#include "UnitCollection.hpp"

#include "OnLatticeSimulationWrapper.hpp"

OnLatticeSimulationWrapper::OnLatticeSimulationWrapper() :
    mDt(1.0),
    mEndTime(100.0),
    mSamplingMultiple(1.0),
    mWorkingDirectory(),
    mpInputCellPopulation(),
    mOutputCellPopulations(),
    mUseRadiotherapyKiller(),
    mpNetwork(),
    mVesselDistanceTolerance(0.75 * unit::microns),
    mRadiotherapyHitTimes(),
    mRadiotherapyDose(2.0),
    mOerAlphaMax(1.75),
    mOerAlphaMin(1.0),
    mOerBetaMax(3.25),
    mOerBetaMin(1.0),
    mKOer(3.28),
    mAlphaMax(0.3),
    mBetaMax(0.03),
    mUseOer(false)
{

}

OnLatticeSimulationWrapper::~OnLatticeSimulationWrapper()
{

}

void OnLatticeSimulationWrapper::SetRadiotherapyHitTimes(std::vector<double> hitTimes)
{
    mRadiotherapyHitTimes = hitTimes;
}

void OnLatticeSimulationWrapper::SetRadiotherapyDose(double dose)
{
    mRadiotherapyDose = dose;
}

void OnLatticeSimulationWrapper::SetOerAlphaMax(double value)
{
    mOerAlphaMax = value;
}

void OnLatticeSimulationWrapper::SetOerAlphaMin(double value)
{
    mOerAlphaMin = value;
}

void OnLatticeSimulationWrapper::SetOerBetaMax(double value)
{
    mOerBetaMax = value;
}

void OnLatticeSimulationWrapper::SetOerBetaMin(double value)
{
    mOerBetaMin = value;
}

void OnLatticeSimulationWrapper::SetOerConstant(double value)
{
    mKOer = value;
}

void OnLatticeSimulationWrapper::SetAlphaMax(double value)
{
    mAlphaMax = value;
}

void OnLatticeSimulationWrapper::SetBetaMax(double value)
{
    mBetaMax = value;
}

void OnLatticeSimulationWrapper::UseOer(bool useOer)
{
    mUseOer = useOer;
}

void OnLatticeSimulationWrapper::SetVesselDistanceTolerance(units::quantity<unit::length> tolerance)
{
    mVesselDistanceTolerance = tolerance;
}

void OnLatticeSimulationWrapper::SetOutputDirectory(const std::string& rDirectory)
{
    mWorkingDirectory = rDirectory;
}

void OnLatticeSimulationWrapper::SetDt(double timeStepSize)
{
    mDt = timeStepSize;
}

void OnLatticeSimulationWrapper::SetSamplingTimestepMultiple(unsigned samplingMultiple)
{
    mSamplingMultiple = samplingMultiple;
}

void OnLatticeSimulationWrapper::SetEndTime(double endTime)
{
    mEndTime = endTime;
}

void OnLatticeSimulationWrapper::SetCellPopulation(boost::shared_ptr<CaBasedCellPopulation<3> > pInputPopulation)
{
    mpInputCellPopulation = pInputPopulation;
}

std::vector<boost::shared_ptr<SimpleCellPopulation<3> > > OnLatticeSimulationWrapper::GetOutputPopulations()
{
    return mOutputCellPopulations;
}

void OnLatticeSimulationWrapper::SetUseRadiotherapyCellKiller(bool UseKiller)
{
    mUseRadiotherapyKiller = UseKiller;
}

void OnLatticeSimulationWrapper::SetNetwork(boost::shared_ptr<VesselNetwork<3> > pNetwork)
{
    mpNetwork = pNetwork;
}

void OnLatticeSimulationWrapper::Solve(boost::shared_ptr<MicrovesselSimulationModifier<3> > pVtModifier)
{
    // Create the simulation
    mpInputCellPopulation->SetOutputResultsForChasteVisualizer(false);
    mpInputCellPopulation->AddCellWriter<CellLabelWriter>();
    mpInputCellPopulation->AddCellPopulationCountWriter<CellProliferativePhasesCountWriter>();
    mpInputCellPopulation->AddCellWriter<CellMutationStatesWriter>();
    mpInputCellPopulation->AddCellWriter<CellProliferativeTypesWriter>();
    mpInputCellPopulation->AddCellWriter<CellProliferativePhasesWriter>();
    MAKE_PTR(StalkCellMutationState, p_ec_state);

    if(mpNetwork)
    {
        for (unsigned index=0; index < mpInputCellPopulation->rGetMesh().GetNumNodes(); index++)
        {
            c_vector<double, 3> grid_location = mpInputCellPopulation->rGetMesh().GetNode(index)->rGetLocation();
            std::pair<boost::shared_ptr<VesselSegment<3> >, units::quantity<unit::length> > segment_distance_pair = mpNetwork->GetNearestSegment(grid_location);
            if (segment_distance_pair.second < mVesselDistanceTolerance || mpNetwork->GetDistanceToNearestNode(grid_location) < mVesselDistanceTolerance)
            {
                mpInputCellPopulation->GetCellUsingLocationIndex(index)->SetMutationState(p_ec_state);
            }
        }
    }

    OnLatticeSimulation<3> simulator(*mpInputCellPopulation);
    simulator.SetOutputDirectory(mWorkingDirectory);
    simulator.SetDt(mDt);
    simulator.SetEndTime(mEndTime);
    boost::shared_ptr<SimpleCellCollectionModifier<3> > p_modifier = boost::shared_ptr<SimpleCellCollectionModifier<3> >(new SimpleCellCollectionModifier<3> ());
    simulator.AddSimulationModifier(p_modifier);
    simulator.AddSimulationModifier(pVtModifier);
    simulator.SetSamplingTimestepMultiple(mSamplingMultiple);

    if (mUseRadiotherapyKiller)
    {
        MAKE_PTR_ARGS(LQRadiotherapyCellKiller<3>, p_killer, (mpInputCellPopulation.get()));
        p_killer->SetDoseInjected(mRadiotherapyDose);
        p_killer->SetTimeOfRadiation(mRadiotherapyHitTimes);
        p_killer->SetOerAlphaMax(mOerAlphaMax);
        p_killer->SetOerAlphaMin(mOerAlphaMin);
        p_killer->SetOerBetaMax(mOerBetaMax);
        p_killer->SetOerBetaMin(mOerBetaMin);
        p_killer->SetOerConstant(mKOer);
        p_killer->SetAlphaMax(mAlphaMax);
        p_killer->SetBetaMax(mBetaMax);
        p_killer->UseOer(mUseOer);
        simulator.AddCellKiller(p_killer);
    }
    MAKE_PTR_ARGS(ApoptoticCellKiller<3>, p_apop_killer, (mpInputCellPopulation.get()));
    simulator.AddCellKiller(p_apop_killer);

    simulator.Solve();

    // Get the output populations
    mOutputCellPopulations = p_modifier->GetSimpleCellPopulations();

    SimulationTime::Destroy();
    RandomNumberGenerator::Destroy();
    CellPropertyRegistry::Instance()->Clear();
}
