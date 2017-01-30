/*

Copyright (c) 2005-2016, University of Oxford.
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

#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "ApoptoticCellProperty.hpp"
#include "TipCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "CellPropertyRegistry.hpp"
#include "CellLabel.hpp"
#include "Exception.hpp"
#include "SimulationTime.hpp"
#include "CellCycleModelOdeHandler.hpp"
#include "CvodeAdaptor.hpp"
#include "RandomNumberGenerator.hpp"
#include "AbstractPhaseBasedCellCycleModel.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "Owen11Parameters.hpp"
#include "BaseUnits.hpp"
#include "BackwardEulerIvpOdeSolver.hpp"

Owen2011OxygenBasedCellCycleModel::Owen2011OxygenBasedCellCycleModel(boost::shared_ptr<AbstractCellCycleModelOdeSolver> pOdeSolver)
: AbstractOdeBasedPhaseBasedCellCycleModel(SimulationTime::Instance()->GetTime(), pOdeSolver),
  mOdeIntegrationTimeStep(30.0*unit::minutes),
  sOnset(0.4),
  g2Onset(0.6),
  mOnset(0.9),
  mReferenceTimeScale(BaseUnits::Instance()->GetReferenceTimeScale()),
  mReferenceConcentrationScale(BaseUnits::Instance()->GetReferenceConcentrationScale()),
  mMaxRandInitialPhase(0.99),
  mCurrentQuiescentDuration(0.0*unit::seconds),
  mCurrentQuiescenceOnsetTime(0.0*unit::seconds),
  mEnterQuiescenceOxygenConcentration(Owen11Parameters::mpOxygenPartialPressureAtQuiescence->GetValue("Owen2011OxygenBasedCellCycleModel")),
  mLeaveQuiescenceOxygenConcentration(Owen11Parameters::mpOxygenPartialPressureLeaveQuiescence->GetValue("Owen2011OxygenBasedCellCycleModel")),
  mCriticalQuiescentDuration(Owen11Parameters::mpTimeToDeathDueToQuiescence->GetValue("Owen2011OxygenBasedCellCycleModel")),
  mp53ThresholdForApoptosisOfNormalCellsInHealthyMicroenvironment(0.8),
  mp53ThresholdForApoptosisOfNormalCellsInTumourMicroenvironment(0.08),
  mthresholdFractionOfNormalCellNeighbours(0.75)
{
    if (!mpOdeSolver)
    {
        #ifdef CHASTE_CVODE
            mpOdeSolver = CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, CvodeAdaptor>::Instance();
        #else
            mpOdeSolver = CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, BackwardEulerIvpOdeSolver>::Instance();
        #endif
        mpOdeSolver->Initialise();
    }

    SetDt(mOdeIntegrationTimeStep/mReferenceTimeScale);

    mReferenceSolubility =
            Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("Owen2011OxygenBasedCellCycleOdeSystem") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("Owen2011OxygenBasedCellCycleOdeSystem");
}

void Owen2011OxygenBasedCellCycleModel::AdjustOdeParameters(double currentTime)
{
    // Pass this time step's oxygen concentration into the solver as a constant over this timestep
    mpOdeSystem->rGetStateVariables()[3] = mpCell->GetCellData()->GetItem("oxygen");

    // Use the cell's current mutation status as another input
    static_cast<Owen2011OxygenBasedCellCycleOdeSystem*>(mpOdeSystem)->SetMutationState(mpCell->GetMutationState());
}

void Owen2011OxygenBasedCellCycleModel::CheckAndLabelCell()
{
    assert(mpCell->GetMutationState()->IsType<CancerCellMutationState>() || mpCell->GetMutationState()->IsType<WildTypeCellMutationState>());

    // Get cell's oxygen concentration
    units::quantity<unit::concentration> oxygen_concentration = mpCell->GetCellData()->GetItem("oxygen")*mReferenceConcentrationScale;
    if (mpCell->GetMutationState()->IsType<CancerCellMutationState>())
    {
        if(oxygen_concentration/mReferenceSolubility <= mEnterQuiescenceOxygenConcentration)
        {
            mpCell->SetMutationState(CellPropertyRegistry::Instance()->Get<QuiescentCancerCellMutationState>());
            assert(mpCell->GetMutationState()->IsType<QuiescentCancerCellMutationState>());
            assert(!(mpCell->GetMutationState()->IsType<CancerCellMutationState>()));
            mCurrentQuiescenceOnsetTime = SimulationTime::Instance()->GetTime()*mReferenceTimeScale;
            mCurrentCellCyclePhase = G_ZERO_PHASE;
        }
    }
    else
    {
        double p53_concentration = mpCell->GetCellData()->GetItem("p53");
        double p53threshold = 0.0;
        unsigned number_of_normal_neighbours = mpCell->GetCellData()->GetItem("Number_of_normal_neighbours");
        unsigned number_of_cancerous_neighbours = mpCell->GetCellData()->GetItem("Number_of_cancerous_neighbours");
        double normal_neighbour_fraction = double(number_of_normal_neighbours)/double(number_of_cancerous_neighbours+number_of_normal_neighbours);
        if (normal_neighbour_fraction > mthresholdFractionOfNormalCellNeighbours)
        {
            p53threshold = mp53ThresholdForApoptosisOfNormalCellsInHealthyMicroenvironment;
        }
        else
        {
            p53threshold = mp53ThresholdForApoptosisOfNormalCellsInTumourMicroenvironment;
        }
        if(p53_concentration > p53threshold)
        {
            assert(mpCell->GetMutationState()->IsType<WildTypeCellMutationState>());
            mpCell->AddCellProperty(CellPropertyRegistry::Instance()->Get<ApoptoticCellProperty>());
        }
    }
}

AbstractCellCycleModel* Owen2011OxygenBasedCellCycleModel::CreateCellCycleModel()
{
    // Create a new cell-cycle model
    Owen2011OxygenBasedCellCycleModel* p_model = new Owen2011OxygenBasedCellCycleModel(mpOdeSolver);
    /*
     * Set each member variable of the new cell-cycle model that inherits
     * its value from the parent.
     *
     * Note 1: some of the new cell-cycle model's member variables (namely
     * mBirthTime, mCurrentCellCyclePhase, mReadyToDivide, mDt, mpOdeSolver)
     * will already have been correctly initialized in its constructor.
     *
     * Note 2: one or more of the new cell-cycle model's member variables
     * may be set/overwritten as soon as InitialiseDaughterCell() is called on
     * the new cell-cycle model.
     */
    p_model->SetBirthTime(mBirthTime);
    p_model->SetDimension(mDimension);
    p_model->SetMinimumGapDuration(mMinimumGapDuration);
    p_model->SetStemCellG1Duration(mStemCellG1Duration);
    p_model->SetTransitCellG1Duration(mTransitCellG1Duration);
    p_model->SetSDuration(mSDuration);
    p_model->SetG2Duration(mG2Duration);
    p_model->SetMDuration(mMDuration);
    p_model->SetLastTime(mLastTime);
    p_model->SetEnterQuiescenceOxygenConcentration(mEnterQuiescenceOxygenConcentration);
    p_model->SetLeaveQuiescenceOxygenConcentration(mLeaveQuiescenceOxygenConcentration);
    p_model->SetCriticalQuiescentDuration(mCriticalQuiescentDuration);
    p_model->SetCurrentQuiescenceOnsetTime(mCurrentQuiescenceOnsetTime);
    p_model->SetReferenceTimeScale(mReferenceTimeScale);
    p_model->SetReferenceConcentrationScale(mReferenceConcentrationScale);

    /*
     * Create the new cell-cycle model's ODE system and use the current values
     * of the state variables in mpOdeSystem as an initial condition.
     */
    assert(mpOdeSystem);
    // note should the second argument here not be a flag ...
    p_model->SetOdeSystem(new Owen2011OxygenBasedCellCycleOdeSystem(mpCell->GetCellData()->GetItem("oxygen")*mReferenceConcentrationScale,
                                                                    mpCell->GetMutationState()));
    p_model->SetStateVariables(mpOdeSystem->rGetStateVariables());

    return p_model;
}

double Owen2011OxygenBasedCellCycleModel::GetSDuration() const
{
    return 0.0;
}

double Owen2011OxygenBasedCellCycleModel::GetG2Duration() const
{
    return 0.0;
}

double Owen2011OxygenBasedCellCycleModel::GetMDuration() const
{
    return 0.0;
}

double Owen2011OxygenBasedCellCycleModel::GetPhi()
{
    assert(mpOdeSystem != NULL);
    double phi = mpOdeSystem->rGetStateVariables()[0];
    return phi;
}

double Owen2011OxygenBasedCellCycleModel::GetVEGF()
{
    assert(mpOdeSystem != NULL);
    double VEGF = mpOdeSystem->rGetStateVariables()[2];
    return VEGF;
}

double Owen2011OxygenBasedCellCycleModel::GetP53()
{
    assert(mpOdeSystem != NULL);
    double p53 = mpOdeSystem->rGetStateVariables()[1];
    return p53;
}

units::quantity<unit::time> Owen2011OxygenBasedCellCycleModel::GetCurrentQuiescentDuration()
{
    return mCurrentQuiescentDuration;
}

units::quantity<unit::time> Owen2011OxygenBasedCellCycleModel::GetCurrentQuiescenceOnsetTime()
{
    return mCurrentQuiescenceOnsetTime;
}

units::quantity<unit::pressure> Owen2011OxygenBasedCellCycleModel::GetEnterQuiescenceOxygenConcentration()
{
    return mEnterQuiescenceOxygenConcentration;
}

units::quantity<unit::pressure> Owen2011OxygenBasedCellCycleModel::GetLeaveQuiescenceOxygenConcentration()
{
    return mLeaveQuiescenceOxygenConcentration;
}

units::quantity<unit::time> Owen2011OxygenBasedCellCycleModel::GetCriticalQuiescentDuration()
{
    return mCriticalQuiescentDuration;
}

void Owen2011OxygenBasedCellCycleModel::Initialise()
{
    assert(mpOdeSystem == NULL);
    assert(mpCell != NULL);

    mpOdeSystem = new Owen2011OxygenBasedCellCycleOdeSystem(mpCell->GetCellData()->GetItem("oxygen")*mReferenceConcentrationScale,
                                                            mpCell->GetMutationState());

    mpCell->SetBirthTime(SimulationTime::Instance()->GetTime());
    std::vector<double> init_conds = mpOdeSystem->GetInitialConditions();
    init_conds[0] = mMaxRandInitialPhase*RandomNumberGenerator::Instance()->ranf(); // phi is random initially
    init_conds[3] = mpCell->GetCellData()->GetItem("oxygen");

    mpOdeSystem->SetStateVariables(init_conds);
}

void Owen2011OxygenBasedCellCycleModel::InitialiseDaughterCell()
{
    mpOdeSystem->rGetStateVariables()[0] = mpOdeSystem->GetInitialConditions()[0];
    mCurrentCellCyclePhase = G_ONE_PHASE;
}

void Owen2011OxygenBasedCellCycleModel::OutputCellCycleModelParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<EnterQuiescenceOxygenConcentration>" << mEnterQuiescenceOxygenConcentration << "</EnterQuiescenceOxygenConcentration>\n";
    *rParamsFile << "\t\t\t<LeaveQuiescenceOxygenConcentration>" << mLeaveQuiescenceOxygenConcentration << "</LeaveQuiescenceOxygenConcentration>\n";
    *rParamsFile << "\t\t\t<CriticalQuiescentDuration>" << mCriticalQuiescentDuration << "</CriticalQuiescentDuration>\n";

    // Call method on direct parent class
    AbstractOdeBasedPhaseBasedCellCycleModel::OutputCellCycleModelParameters(rParamsFile);
}

void Owen2011OxygenBasedCellCycleModel::ResetForDivision()
{
    assert(mpOdeSystem != NULL);
    assert(mFinishedRunningOdes);
    assert(mReadyToDivide);
    mReadyToDivide = false;
    mFinishedRunningOdes = false;

    // Keep the oxygen concentration the same but reset everything else
    mpOdeSystem->rGetStateVariables()[0] = mpOdeSystem->GetInitialConditions()[0];
    mpOdeSystem->rGetStateVariables()[1] = mpOdeSystem->GetInitialConditions()[1];
    mpOdeSystem->rGetStateVariables()[2] = mpOdeSystem->GetInitialConditions()[2];

    assert(!mpCell->GetMutationState()->IsType<QuiescentCancerCellMutationState>());
    mpCell->SetMutationState(mpCell->GetMutationState());
    mCurrentCellCyclePhase = G_ONE_PHASE;
}

bool Owen2011OxygenBasedCellCycleModel::ReadyToDivide()
{
    if (mpCell->GetMutationState()->IsType<TipCellMutationState>() || mpCell->GetMutationState()->IsType<StalkCellMutationState>())
    {
        mReadyToDivide = false;
    }
    else
    {
        if(!mpCell->HasApoptosisBegun() && !mpCell->IsDead())
        {
            UpdateCellCyclePhase();
            if(mFinishedRunningOdes)
            {
                mReadyToDivide = true;
            }
            else
            {
                mReadyToDivide = false;
            }
        }
        else
        {
            mReadyToDivide = false;
        }
    }
    return mReadyToDivide;
}

void Owen2011OxygenBasedCellCycleModel::SetSOnset(units::quantity<unit::dimensionless> value)
{
    sOnset = value;
}

void Owen2011OxygenBasedCellCycleModel::SetG2Onset(units::quantity<unit::dimensionless> value)
{
    g2Onset = value;
}

void Owen2011OxygenBasedCellCycleModel::SetMOnset(units::quantity<unit::dimensionless> value)
{
    mOnset = value;
}

void Owen2011OxygenBasedCellCycleModel::SetOdeSolverTimeStep(units::quantity<unit::time> timeStep)
{
    mOdeIntegrationTimeStep = timeStep;
    SetDt(mOdeIntegrationTimeStep/mReferenceTimeScale);
}

void Owen2011OxygenBasedCellCycleModel::SetReferenceTimeScale(units::quantity<unit::time> referenceTimeScale)
{
    mReferenceTimeScale = referenceTimeScale;
}

void Owen2011OxygenBasedCellCycleModel::SetReferenceConcentrationScale(units::quantity<unit::concentration> referenceConcentrationScale)
{
    mReferenceConcentrationScale = referenceConcentrationScale;
}

void Owen2011OxygenBasedCellCycleModel::SetMaxRandInitialPhase(units::quantity<unit::dimensionless> rand_max_phase)
{
    assert(rand_max_phase >= 0.0);
    mMaxRandInitialPhase = rand_max_phase;
}

void Owen2011OxygenBasedCellCycleModel::SetEnterQuiescenceOxygenConcentration(units::quantity<unit::pressure> enterQuiescenceOxygenConcentration)
{
    assert(enterQuiescenceOxygenConcentration>=0.0*unit::pascals);
    mEnterQuiescenceOxygenConcentration = enterQuiescenceOxygenConcentration;
}

void Owen2011OxygenBasedCellCycleModel::SetLeaveQuiescenceOxygenConcentration(units::quantity<unit::pressure> leaveQuiescenceOxygenConcentration)
{
    assert(leaveQuiescenceOxygenConcentration >= 0.0*unit::pascals);
    mLeaveQuiescenceOxygenConcentration = leaveQuiescenceOxygenConcentration;
}

void Owen2011OxygenBasedCellCycleModel::SetCriticalQuiescentDuration(units::quantity<unit::time> criticalQuiescentDuration)
{
    assert(criticalQuiescentDuration >= 0.0*unit::seconds);
    mCriticalQuiescentDuration = criticalQuiescentDuration;
}

void Owen2011OxygenBasedCellCycleModel::SetCurrentQuiescenceOnsetTime(units::quantity<unit::time> currentQuiescenceOnsetTime)
{
    assert(currentQuiescenceOnsetTime >= 0.0*unit::seconds);
    mCurrentQuiescenceOnsetTime = currentQuiescenceOnsetTime;
}

void Owen2011OxygenBasedCellCycleModel::SetThresholdFractionOfNormalCellNeighbours(double value)
{
    mthresholdFractionOfNormalCellNeighbours = value;
}

void Owen2011OxygenBasedCellCycleModel::UpdateCellCyclePhase()
{
    assert(!mpCell->HasApoptosisBegun());
    assert(!mpCell->IsDead());

    // if cell is still marked for division then the cell has failed to divide and we need to reset the cell here
    // most notably this happens in on-lattice simulations where there is no space available for a cell to divide
    if(mReadyToDivide)
    {
        ResetForDivision();
    }

    if(mpCell->GetMutationState()->IsType<CancerCellMutationState>() || mpCell->GetMutationState()->IsType<WildTypeCellMutationState>())
    {
        CheckAndLabelCell();
    }

    if(mpCell->GetMutationState()->IsType<QuiescentCancerCellMutationState>())
    {
        UpdateQuiescentDuration();
    }

    #ifdef CHASTE_CVODE
    // must do this because we are using CVode and by
    // default stopping events are not found
    mpOdeSolver->CheckForStoppingEvents();
    #endif

    assert(mpOdeSystem != NULL);
    double current_time = SimulationTime::Instance()->GetTime();

    // adjust ode parameters is called in here ... updates mutation state and oxygen concentration
    mFinishedRunningOdes = SolveOdeToTime(current_time);

    // Check no concentrations have gone negative
    for (unsigned i=0; i<mpOdeSystem->GetNumberOfStateVariables(); i++)
    {
        if (mpOdeSystem->rGetStateVariables()[i] < -DBL_EPSILON)
        {
            EXCEPTION("A protein concentration " << i << " has gone negative (" << mpOdeSystem->rGetStateVariables()[i] << ")\n"
                    << "A CellCycleModel numerical method is probably unstable.");
        }
    }

    if(mCurrentCellCyclePhase != G_ZERO_PHASE)
    {
        if(mpOdeSystem->rGetStateVariables()[0]<sOnset)
        {
            mCurrentCellCyclePhase = G_ONE_PHASE;
        }
        else if(mpOdeSystem->rGetStateVariables()[0]<g2Onset)
        {
            mCurrentCellCyclePhase = S_PHASE;
        }
        else if(mpOdeSystem->rGetStateVariables()[0]<mOnset)
        {
            mCurrentCellCyclePhase = G_TWO_PHASE;
        }
        else
        {
            mCurrentCellCyclePhase = M_PHASE;
        }
    }
}

void Owen2011OxygenBasedCellCycleModel::UpdateQuiescentDuration()
{
    assert(mpCell->GetMutationState()->IsType<QuiescentCancerCellMutationState>());
    assert(!(mpCell->HasCellProperty<ApoptoticCellProperty>()));
    assert(!mpCell->HasApoptosisBegun());

    // Get cell's oxygen concentration
    units::quantity<unit::concentration> oxygen_concentration = mpCell->GetCellData()->GetItem("oxygen")*mReferenceConcentrationScale;
    if (oxygen_concentration/mReferenceSolubility <= mLeaveQuiescenceOxygenConcentration)
    {
        // Update the duration of the current period of hypoxia
        mCurrentQuiescentDuration = SimulationTime::Instance()->GetTime()*mReferenceTimeScale - mCurrentQuiescenceOnsetTime;
        if (mCurrentQuiescentDuration >= mCriticalQuiescentDuration)
        {
            mpCell->AddCellProperty(CellPropertyRegistry::Instance()->Get<ApoptoticCellProperty>());
        }
    }
    else
    {
        // Reset the cell's quiescent duration.
        mCurrentQuiescentDuration = 0.0 * unit::seconds;
        mCurrentQuiescenceOnsetTime = 0.0 * unit::seconds;
        mCurrentCellCyclePhase = G_ONE_PHASE;
        mpCell->SetMutationState(CellPropertyRegistry::Instance()->Get<CancerCellMutationState>());
    }
}

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(Owen2011OxygenBasedCellCycleModel)
#include "CellCycleModelOdeSolverExportWrapper.hpp"
EXPORT_CELL_CYCLE_MODEL_ODE_SOLVER(Owen2011OxygenBasedCellCycleModel)
