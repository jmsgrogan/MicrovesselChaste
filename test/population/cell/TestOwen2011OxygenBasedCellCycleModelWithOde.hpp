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

#ifndef TESTODEBASEDCELLCYCLEMODELS_HPP_
#define TESTODEBASEDCELLCYCLEMODELS_HPP_

#include <cxxtest/TestSuite.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "AbstractCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "StemCellProliferativeType.hpp"
#include "ApcTwoHitCellMutationState.hpp"
#include "BetaCateninOneHitCellMutationState.hpp"
#include "OutputFileHandler.hpp"
#include "CheckReadyToDivideAndPhaseIsUpdated.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "SmartPointers.hpp"
#include "CvodeAdaptor.hpp"

/**
 * This class contains tests for methods on classes
 * inheriting from AbstractOdeBasedCellCycleModel.
 */
class TestOwen2011OxygenBasedCellCycleModelWithOde : public AbstractCellBasedTestSuite
{
public:

    void TestOwen2011OxygenBasedCellCycleModelWithOdeForNormalCells() throw(Exception)
    {
        // Set up SimulationTime
        SimulationTime* p_simulation_time = SimulationTime::Instance();
        p_simulation_time->SetEndTimeAndNumberOfTimeSteps(250.0, 500);

        // Set up oxygen_concentration
        double oxygen_concentration = 1.0;

        // Create cell-cycle models
        Owen2011OxygenBasedCellCycleModel* p_model_1d = new Owen2011OxygenBasedCellCycleModel();
        p_model_1d->SetDimension(1);
        p_model_1d->SetMaxRandInitialPhase(0);

        Owen2011OxygenBasedCellCycleModel* p_model_2d = new Owen2011OxygenBasedCellCycleModel();
        p_model_2d->SetDimension(2);
        p_model_2d->SetMaxRandInitialPhase(0);

        Owen2011OxygenBasedCellCycleModel* p_model_3d = new Owen2011OxygenBasedCellCycleModel();
        p_model_3d->SetDimension(3);
        p_model_3d->SetMaxRandInitialPhase(0);

        // Create cells
        MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(StemCellProliferativeType, p_stem_type);

        CellPtr p_cell_1d(new Cell(p_state, p_model_1d));
        p_cell_1d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_1d->SetCellProliferativeType(p_stem_type);
        p_cell_1d->InitialiseCellCycleModel();

        CellPtr p_cell_2d(new Cell(p_state, p_model_2d));
        p_cell_2d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_2d->SetCellProliferativeType(p_stem_type);
        p_cell_2d->InitialiseCellCycleModel();

        CellPtr p_cell_3d(new Cell(p_state, p_model_3d));
        p_cell_3d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_3d->SetCellProliferativeType(p_stem_type);
        p_cell_3d->InitialiseCellCycleModel();

        // For coverage, we create another cell-cycle model that is identical to p_model_2d except for the ODE solver
        boost::shared_ptr<CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, CvodeAdaptor> >
        p_solver(CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, CvodeAdaptor>::Instance());
        p_solver->Initialise();

        Owen2011OxygenBasedCellCycleModel* p_other_model_2d = new Owen2011OxygenBasedCellCycleModel(p_solver);
        p_other_model_2d->SetDimension(2);
        p_other_model_2d->SetMaxRandInitialPhase(0);

        CellPtr p_other_cell_2d(new Cell(p_state, p_other_model_2d));
        p_other_cell_2d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_other_cell_2d->SetCellProliferativeType(p_stem_type);
        p_other_cell_2d->InitialiseCellCycleModel();

        // Check oxygen concentration is correct in cell-cycle model
        TS_ASSERT_DELTA(p_model_2d->GetProteinConcentrations()[3], 1.0, 1e-5);
        double this_phi = p_model_2d->GetPhi();
        double this_p53 = p_model_2d->GetP53();
        double this_VEGF = p_model_2d->GetVEGF();
        p_cell_2d->GetCellData()->SetItem("Phi", this_phi);
        p_cell_2d->GetCellData()->SetItem("p53", this_p53);
        p_cell_2d->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell_2d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell_2d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);

        TS_ASSERT_DELTA(p_other_model_2d->GetProteinConcentrations()[3], 1.0, 1e-5);
        this_phi = p_other_model_2d->GetPhi();
        this_p53 = p_other_model_2d->GetP53();
        this_VEGF = p_other_model_2d->GetVEGF();
        p_other_cell_2d->GetCellData()->SetItem("Phi", this_phi);
        p_other_cell_2d->GetCellData()->SetItem("p53", this_p53);
        p_other_cell_2d->GetCellData()->SetItem("VEGF", this_VEGF);
        p_other_cell_2d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_other_cell_2d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        TS_ASSERT_EQUALS(p_other_model_2d->ReadyToDivide(), false);

        // Divide the cells
        Owen2011OxygenBasedCellCycleModel* p_model_1d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_1d->CreateCellCycleModel());
        CellPtr p_cell_1d_2(new Cell(p_state, p_model_1d_2));
        p_cell_1d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_1d_2->SetCellProliferativeType(p_stem_type);

        Owen2011OxygenBasedCellCycleModel* p_model_2d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_2d->CreateCellCycleModel());
        CellPtr p_cell_2d_2(new Cell(p_state, p_model_2d_2));
        p_cell_2d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);;
        p_cell_2d_2->SetCellProliferativeType(p_stem_type);

        Owen2011OxygenBasedCellCycleModel* p_model_3d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_3d->CreateCellCycleModel());
        CellPtr p_cell_3d_2(new Cell(p_state, p_model_3d_2));
        p_cell_3d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_3d_2->SetCellProliferativeType(p_stem_type);

        for (unsigned i=0; i<400; i++)
        {
            p_simulation_time->IncrementTimeOneStep();
            this_phi = p_model_1d->GetPhi();
            this_p53 = p_model_1d->GetP53();
            this_VEGF = p_model_1d->GetVEGF();
            p_cell_1d->GetCellData()->SetItem("Phi", this_phi);
            p_cell_1d->GetCellData()->SetItem("p53", this_p53);
            p_cell_1d->GetCellData()->SetItem("VEGF", this_VEGF);
            p_cell_1d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
            p_cell_1d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
            this_phi = p_model_2d->GetPhi();
            this_p53 = p_model_2d->GetP53();
            this_VEGF = p_model_2d->GetVEGF();
            p_cell_2d->GetCellData()->SetItem("Phi", this_phi);
            p_cell_2d->GetCellData()->SetItem("p53", this_p53);
            p_cell_2d->GetCellData()->SetItem("VEGF", this_VEGF);
            p_cell_2d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
            p_cell_2d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
            this_phi = p_model_3d->GetPhi();
            this_p53 = p_model_3d->GetP53();
            this_VEGF = p_model_3d->GetVEGF();
            p_cell_3d->GetCellData()->SetItem("Phi", this_phi);
            p_cell_3d->GetCellData()->SetItem("p53", this_p53);
            p_cell_3d->GetCellData()->SetItem("VEGF", this_VEGF);
            p_cell_3d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
            p_cell_3d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
            TS_ASSERT_EQUALS(p_model_1d->ReadyToDivide(), false);
            TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);
            TS_ASSERT_EQUALS(p_model_3d->ReadyToDivide(), false);
        }

        p_simulation_time->IncrementTimeOneStep();
        this_phi = p_model_1d->GetPhi();
        this_p53 = p_model_1d->GetP53();
        this_VEGF = p_model_1d->GetVEGF();
        p_cell_1d->GetCellData()->SetItem("Phi", this_phi);
        p_cell_1d->GetCellData()->SetItem("p53", this_p53);
        p_cell_1d->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell_1d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell_1d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        this_phi = p_model_2d->GetPhi();
        this_p53 = p_model_2d->GetP53();
        this_VEGF = p_model_2d->GetVEGF();
        p_cell_2d->GetCellData()->SetItem("Phi", this_phi);
        p_cell_2d->GetCellData()->SetItem("p53", this_p53);
        p_cell_2d->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell_2d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell_2d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        this_phi = p_model_3d->GetPhi();
        this_p53 = p_model_3d->GetP53();
        this_VEGF = p_model_3d->GetVEGF();
        p_cell_3d->GetCellData()->SetItem("Phi", this_phi);
        p_cell_3d->GetCellData()->SetItem("p53", this_p53);
        p_cell_3d->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell_3d->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell_3d->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        TS_ASSERT_EQUALS(p_model_1d->ReadyToDivide(), true);
        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), true);
        TS_ASSERT_EQUALS(p_model_3d->ReadyToDivide(), true);


        TS_ASSERT_THROWS_NOTHING(p_model_2d->ResetForDivision());

        // For coverage, create a 1D model
        Owen2011OxygenBasedCellCycleModel* p_cell_model3 = new Owen2011OxygenBasedCellCycleModel();
        p_cell_model3->SetDimension(1);
        p_cell_model3->SetMaxRandInitialPhase(0);

        CellPtr p_cell3(new Cell(p_state, p_cell_model3));
        p_cell3->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell3->SetCellProliferativeType(p_stem_type);
        p_cell3->InitialiseCellCycleModel();

        TS_ASSERT_DELTA(p_cell_model3->GetProteinConcentrations()[3], 1.0, 1e-5);
        p_simulation_time->IncrementTimeOneStep();
        this_phi = p_cell_model3->GetPhi();
        this_p53 = p_cell_model3->GetP53();
        this_VEGF = p_cell_model3->GetVEGF();
        p_cell3->GetCellData()->SetItem("Phi", this_phi);
        p_cell3->GetCellData()->SetItem("p53", this_p53);
        p_cell3->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell3->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell3->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        TS_ASSERT_EQUALS(p_cell_model3->ReadyToDivide(), false);
        p_simulation_time->IncrementTimeOneStep();
        this_phi = p_cell_model3->GetPhi();
        this_p53 = p_cell_model3->GetP53();
        this_VEGF = p_cell_model3->GetVEGF();
        p_cell3->GetCellData()->SetItem("Phi", this_phi);
        p_cell3->GetCellData()->SetItem("p53", this_p53);
        p_cell3->GetCellData()->SetItem("VEGF", this_VEGF);
        p_cell3->GetCellData()->SetItem("Number_of_normal_neighbours", 0);
        p_cell3->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0);
        TS_ASSERT_EQUALS(p_cell_model3->ReadyToDivide(), false);
    }

    void TestOwen2011OxygenBasedCellCycleModelWithOdeForCancerCells() throw(Exception)
            {
        // Set up SimulationTime
        SimulationTime* p_simulation_time = SimulationTime::Instance();
        p_simulation_time->SetEndTimeAndNumberOfTimeSteps(75.0, 150);

        // Set up oxygen_concentration
        double oxygen_concentration = 10.0;

        // Create cell-cycle models
        Owen2011OxygenBasedCellCycleModel* p_model_1d = new Owen2011OxygenBasedCellCycleModel();
        p_model_1d->SetDimension(1);
        p_model_1d->SetMaxRandInitialPhase(0);

        Owen2011OxygenBasedCellCycleModel* p_model_2d = new Owen2011OxygenBasedCellCycleModel();
        p_model_2d->SetDimension(2);
        p_model_2d->SetMaxRandInitialPhase(0);

        Owen2011OxygenBasedCellCycleModel* p_model_3d = new Owen2011OxygenBasedCellCycleModel();
        p_model_3d->SetDimension(3);
        p_model_3d->SetMaxRandInitialPhase(0);

        // Create cells
        MAKE_PTR(CancerCellMutationState, p_state);
        MAKE_PTR(StemCellProliferativeType, p_stem_type);

        CellPtr p_cell_1d(new Cell(p_state, p_model_1d));
        p_cell_1d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_1d->SetCellProliferativeType(p_stem_type);
        p_cell_1d->InitialiseCellCycleModel();
        TS_ASSERT_EQUALS(p_state->IsType<CancerCellMutationState>(), true);

        CellPtr p_cell_2d(new Cell(p_state, p_model_2d));
        p_cell_2d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_2d->SetCellProliferativeType(p_stem_type);
        p_cell_2d->InitialiseCellCycleModel();
        TS_ASSERT_EQUALS(p_state->IsType<CancerCellMutationState>(), true);

        CellPtr p_cell_3d(new Cell(p_state, p_model_3d));
        p_cell_3d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_3d->SetCellProliferativeType(p_stem_type);
        p_cell_3d->InitialiseCellCycleModel();
        TS_ASSERT_EQUALS(p_state->IsType<CancerCellMutationState>(), true);

        // For coverage, we create another cell-cycle model that is identical to p_model_2d except for the ODE solver
        boost::shared_ptr<CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, CvodeAdaptor> >
        p_solver(CellCycleModelOdeSolver<Owen2011OxygenBasedCellCycleModel, CvodeAdaptor>::Instance());
        p_solver->Initialise();

        Owen2011OxygenBasedCellCycleModel* p_other_model_2d = new Owen2011OxygenBasedCellCycleModel(p_solver);
        p_other_model_2d->SetDimension(2);
        p_other_model_2d->SetMaxRandInitialPhase(0);

        CellPtr p_other_cell_2d(new Cell(p_state, p_other_model_2d));
        p_other_cell_2d->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_other_cell_2d->SetCellProliferativeType(p_stem_type);
        p_other_cell_2d->InitialiseCellCycleModel();

        // Check oxygen concentration is correct in cell-cycle model
        TS_ASSERT_DELTA(p_model_2d->GetProteinConcentrations()[3], 10.0, 1e-5);
        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);

        TS_ASSERT_DELTA(p_other_model_2d->GetProteinConcentrations()[3], 10.0, 1e-5);
        TS_ASSERT_EQUALS(p_other_model_2d->ReadyToDivide(), false);

        // Divide the cells
        Owen2011OxygenBasedCellCycleModel* p_model_1d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_1d->CreateCellCycleModel());
        CellPtr p_cell_1d_2(new Cell(p_state, p_model_1d_2));
        p_cell_1d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_1d_2->SetCellProliferativeType(p_stem_type);

        Owen2011OxygenBasedCellCycleModel* p_model_2d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_2d->CreateCellCycleModel());
        CellPtr p_cell_2d_2(new Cell(p_state, p_model_2d_2));
        p_cell_2d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_2d_2->SetCellProliferativeType(p_stem_type);

        Owen2011OxygenBasedCellCycleModel* p_model_3d_2 = static_cast<Owen2011OxygenBasedCellCycleModel*> (p_model_3d->CreateCellCycleModel());
        CellPtr p_cell_3d_2(new Cell(p_state, p_model_3d_2));
        p_cell_3d_2->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell_3d_2->SetCellProliferativeType(p_stem_type);

        for (unsigned i=0; i<60; i++)
        {
            p_simulation_time->IncrementTimeOneStep();
            TS_ASSERT_EQUALS(p_model_1d->ReadyToDivide(), false);
            TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);
            TS_ASSERT_EQUALS(p_model_3d->ReadyToDivide(), false);
        }

        p_simulation_time->IncrementTimeOneStep();

        TS_ASSERT_EQUALS(p_model_1d->ReadyToDivide(), true);
        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), true);
        TS_ASSERT_EQUALS(p_model_3d->ReadyToDivide(), true);

        TS_ASSERT_THROWS_NOTHING(p_model_2d->ResetForDivision());
        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);

        for (unsigned i=0; i<60; i++)
        {
            p_simulation_time->IncrementTimeOneStep();
            TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), false);
        }

        p_simulation_time->IncrementTimeOneStep();

        TS_ASSERT_EQUALS(p_model_2d->ReadyToDivide(), true);

        // For coverage, create a 1D model
        Owen2011OxygenBasedCellCycleModel* p_cell_model3 = new Owen2011OxygenBasedCellCycleModel();
        p_cell_model3->SetDimension(1);
        p_cell_model3->SetMaxRandInitialPhase(0);

        CellPtr p_cell3(new Cell(p_state, p_cell_model3));
        p_cell3->GetCellData()->SetItem("oxygen", oxygen_concentration);
        p_cell3->InitialiseCellCycleModel();
        p_cell3->SetCellProliferativeType(p_stem_type);

        TS_ASSERT_DELTA(p_cell_model3->GetProteinConcentrations()[3], 10.0, 1e-5);
        TS_ASSERT_EQUALS(p_cell_model3->ReadyToDivide(), false);
        p_simulation_time->IncrementTimeOneStep();
        TS_ASSERT_EQUALS(p_cell_model3->ReadyToDivide(), false);
            }

    void TestOwen2011OxygenBasedCellCycleModelWithOdeForQuiescentCancerCells() throw(Exception)
            {
        //Check that oxygen concentration is set up correctly
        SimulationTime* p_simulation_time = SimulationTime::Instance();
        p_simulation_time->SetEndTimeAndNumberOfTimeSteps(3.0, 3);

        Owen2011OxygenBasedCellCycleModel* p_model1 = new Owen2011OxygenBasedCellCycleModel();
        p_model1->SetDimension(2);
        p_model1->SetMaxRandInitialPhase(0);

        MAKE_PTR(CancerCellMutationState, p_state);
        MAKE_PTR(StemCellProliferativeType, p_stem_type);
        CellPtr p_cell1(new Cell(p_state, p_model1));

        double lo_oxygen = 1.0;
        double hi_oxygen = 10.0;

        p_cell1->GetCellData()->SetItem("oxygen", lo_oxygen);
        p_cell1->SetCellProliferativeType(p_stem_type);
        p_cell1->InitialiseCellCycleModel();

        p_model1->ReadyToDivide();
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescentDuration(), 0.0, 1e-12);
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescenceOnsetTime(), 0.0, 1e-12);

        p_simulation_time->IncrementTimeOneStep(); // t=1.0
        p_model1->ReadyToDivide();
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescentDuration(), 1.0, 1e-12);
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescenceOnsetTime(), 0.0, 1e-12);

        p_cell1->GetCellData()->SetItem("oxygen", hi_oxygen);
        p_simulation_time->IncrementTimeOneStep(); // t=2.0
        p_model1->ReadyToDivide();
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescentDuration(), 0.0, 1e-12);
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescenceOnsetTime(), 0.0, 1e-12);

        p_cell1->GetCellData()->SetItem("oxygen", lo_oxygen);
        p_simulation_time->IncrementTimeOneStep(); // t=3.0
        p_model1->ReadyToDivide();
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescentDuration(), 0.0, 1e-12);
        TS_ASSERT_DELTA(p_model1->GetCurrentQuiescenceOnsetTime(), 3.0, 1e-12);

        // Set up SimulationTime
        SimulationTime::Destroy();
        p_simulation_time = SimulationTime::Instance();
        p_simulation_time->SetStartTime(0.0);
        p_simulation_time->SetEndTimeAndNumberOfTimeSteps(75.0, 150);

        // Create cell-cycle models and cells
        Owen2011OxygenBasedCellCycleModel* p_model = new Owen2011OxygenBasedCellCycleModel();
        p_model->SetDimension(2);
        p_model->SetMaxRandInitialPhase(0);
        p_model->SetBirthTime(0.0);

        CellPtr p_cell(new Cell(p_state, p_model));
        p_cell->GetCellData()->SetItem("oxygen", hi_oxygen);
        p_cell->SetCellProliferativeType(p_stem_type);
        p_cell->InitialiseCellCycleModel();

        // Check that the cell cycle phase and ready to divide are updated correctly
        TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(),G_ONE_PHASE);

        for (unsigned i=0; i<60; i++)
        {
            p_simulation_time->IncrementTimeOneStep();
            TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        }

        TS_ASSERT_DELTA(p_model->GetAge(), p_simulation_time->GetTime(), 1e-9);

        // Check that cell division correctly resets the cell cycle phase
        p_simulation_time->IncrementTimeOneStep();
        TS_ASSERT_EQUALS(p_cell->ReadyToDivide(), true);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(), M_PHASE);
        CellPtr p_cell2 = p_cell->Divide();
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(), G_ONE_PHASE);
        Owen2011OxygenBasedCellCycleModel* p_model2 = static_cast <Owen2011OxygenBasedCellCycleModel*>(p_cell2->GetCellCycleModel());
        p_model2->SetMaxRandInitialPhase(0);

        TS_ASSERT_EQUALS(p_model2->GetCurrentCellCyclePhase(), G_ONE_PHASE);
        TS_ASSERT_EQUALS(p_model2->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model2->GetCurrentCellCyclePhase(), G_ONE_PHASE);

        p_cell->GetCellData()->SetItem("oxygen", lo_oxygen);

        TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(), G_ZERO_PHASE);

        p_simulation_time->IncrementTimeOneStep();
        TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(),G_ZERO_PHASE);

        p_simulation_time->IncrementTimeOneStep();
        TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(),G_ZERO_PHASE);

        p_cell->GetCellData()->SetItem("oxygen", hi_oxygen);

        TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model->GetCurrentCellCyclePhase(),G_ONE_PHASE);

        for (unsigned i=0; i<60; i++)
        {
            p_simulation_time->IncrementTimeOneStep();
            TS_ASSERT_EQUALS(p_model->ReadyToDivide(), false);
        }

        p_simulation_time->IncrementTimeOneStep();

        // Check that cell division correctly resets the cell cycle phase
        TS_ASSERT_EQUALS(p_cell->ReadyToDivide(), true);
        CellPtr p_cell3 = p_cell->Divide();
        Owen2011OxygenBasedCellCycleModel* p_model3 = static_cast <Owen2011OxygenBasedCellCycleModel*>(p_cell3->GetCellCycleModel());
        p_model3->SetMaxRandInitialPhase(0);

        TS_ASSERT_EQUALS(p_model3->ReadyToDivide(), false);
        TS_ASSERT_EQUALS(p_model3->GetCurrentCellCyclePhase(), G_ONE_PHASE);

        // For coverage, create a 1D model

        Owen2011OxygenBasedCellCycleModel* p_cell_model1d = new Owen2011OxygenBasedCellCycleModel();
        p_cell_model1d->SetDimension(1);
        p_cell_model1d->SetMaxRandInitialPhase(0);


        CellPtr p_cell1d(new Cell(p_state, p_cell_model1d));
        p_cell1d->GetCellData()->SetItem("oxygen", hi_oxygen);
        p_cell1d->SetCellProliferativeType(p_stem_type);
        p_cell1d->InitialiseCellCycleModel();

        TS_ASSERT_EQUALS(p_cell_model1d->ReadyToDivide(), false);

        // For coverage, create a 3D model

        Owen2011OxygenBasedCellCycleModel* p_cell_model3d = new Owen2011OxygenBasedCellCycleModel();
        p_cell_model3d->SetDimension(3);
        p_cell_model3d->SetMaxRandInitialPhase(0);

        CellPtr p_cell3d(new Cell(p_state, p_cell_model3d));
        p_cell3d->GetCellData()->SetItem("oxygen", hi_oxygen);
        p_cell3d->SetCellProliferativeType(p_stem_type);
        p_cell3d->InitialiseCellCycleModel();

        TS_ASSERT_EQUALS(p_cell_model3d->ReadyToDivide(), false);
            }
};

#endif /*TESTODEBASEDCELLCYCLEMODELS_HPP_*/
