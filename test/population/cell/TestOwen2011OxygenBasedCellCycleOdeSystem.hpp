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

#ifndef TESTOWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_
#define TESTOWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_

#include <cxxtest/TestSuite.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <stdio.h>
#include <ctime>
#include <vector>
#include <iostream>
#include "OutputFileHandler.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "Owen2011OxygenBasedCellCycleOdeSystem.hpp"
#include "RungeKutta4IvpOdeSolver.hpp"
#include "RungeKuttaFehlbergIvpOdeSolver.hpp"
#include "CvodeAdaptor.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"

/**
 * This class contains tests for Owen2011OxygenBasedCellCycleOdeSystem,
 * a system of ODEs that are used by the cell-cycle model
 * Owen2011OxygenBasedCellCycleModel to determine when a cell is ready
 * to divide.
 */
class TestOwen2011OxygenBasedCellCycleOdeSystem : public CxxTest::TestSuite
{
public:

    /**
     * Test derivative calculations (correct values calculated using Matlab).
     */
    void TestOwen2011EquationsForNormalCells()
    {
        BaseUnits::SharedInstance()->SetReferenceConcentrationScale(1.e-3*unit::mole_per_metre_cubed);
        BaseUnits::SharedInstance()->SetReferenceTimeScale(1.0*unit::seconds);
        units::quantity<unit::pressure> low_oxygen_partial_pressure(0.1*unit::mmHg);
        units::quantity<unit::pressure> hi_oxygen_partial_pressure(1.0*unit::mmHg);
        units::quantity<unit::solubility> oxygen_solubility = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("Test") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("Test");

        units::quantity<unit::concentration> low_oxygen_concentration = low_oxygen_partial_pressure*oxygen_solubility;
        units::quantity<unit::concentration> hi_oxygen_concentration = hi_oxygen_partial_pressure*oxygen_solubility;

        // Set up
        double time = 0.0;
        boost::shared_ptr<WildTypeCellMutationState> mutation_state(new WildTypeCellMutationState);
        Owen2011OxygenBasedCellCycleOdeSystem normal_system(hi_oxygen_concentration, mutation_state);

        std::vector<double> initial_conditions = normal_system.GetInitialConditions();
        TS_ASSERT_DELTA(initial_conditions[0],0.0,1e-5);
        TS_ASSERT_DELTA(initial_conditions[1],0.0,1e-5);
        TS_ASSERT_DELTA(initial_conditions[2],0.0,1e-5);
        TS_ASSERT_DELTA(initial_conditions[3],double(hi_oxygen_concentration/(1.e-3*unit::mole_per_metre_cubed)),1e-5);

        std::vector<double> normal_derivs(initial_conditions.size());
        normal_system.EvaluateYDerivatives(time, initial_conditions, normal_derivs);

        /**
         * Test derivatives are correct initially
         */
        TS_ASSERT_DELTA(normal_derivs[0], 1.38889e-06, 1e-8);
        TS_ASSERT_DELTA(normal_derivs[1], 3.33333e-05, 1e-6);
        TS_ASSERT_DELTA(normal_derivs[2], 3.33333e-05, 1e-6);
        TS_ASSERT_DELTA(normal_derivs[3], 0.0, 1e-5);

        /**
         * Again test derivatives are correct initially, but for
         * different initial conditions (corresponding to a low
         * oxygen concentration). The usual initial condition for
         * z is zero, so we need to change it to see any difference.
         */
        Owen2011OxygenBasedCellCycleOdeSystem normal_system2(low_oxygen_concentration, mutation_state);
        std::vector<double> normal_derivs2(initial_conditions.size());
        normal_system2.SetDefaultInitialCondition(1, 0.1);
        std::vector<double> initial_conditions2 = normal_system2.GetInitialConditions();
        normal_system2.EvaluateYDerivatives(time, initial_conditions2, normal_derivs2);

        // Normal cell
        TS_ASSERT_DELTA(normal_derivs2[0], 1.79211e-07, 1e-8);
        TS_ASSERT_DELTA(normal_derivs2[1], 3.29662e-05, 1e-7);
        TS_ASSERT_DELTA(normal_derivs2[2], 3.33333e-05, 1e-7);
        TS_ASSERT_DELTA(normal_derivs2[3], 0.0, 1e-5);
    }

    /**
     * Test derivative calculations (correct values calculated using Matlab).
     */
    void TestOwen2011EquationsForCancerCells()
    {
        // Set up
        double time = 0.0;
        units::quantity<unit::concentration> oxygen_concentration = 1.0 * unit::mole_per_metre_cubed;

        boost::shared_ptr<CancerCellMutationState> mutation_state(new CancerCellMutationState);
        Owen2011OxygenBasedCellCycleOdeSystem cancer_system(oxygen_concentration, mutation_state);
        std::vector<double> initial_conditions = cancer_system.GetInitialConditions();
        std::vector<double> cancer_derivs(initial_conditions.size());
        cancer_system.EvaluateYDerivatives(time, initial_conditions, cancer_derivs);

        /**
         * Test derivatives are correct initially
         * (correct values calculated using Matlab code)
         */
        // Cancer cell
//        TS_ASSERT_DELTA(cancer_derivs[0], 1.38889e-06, 1e-7);
        TS_ASSERT_DELTA(cancer_derivs[1], 3.33333e-05, 1e-6);
        TS_ASSERT_DELTA(cancer_derivs[2], 3.33333e-05, 1e-6);
        TS_ASSERT_DELTA(cancer_derivs[3], 0.0, 1e-5);

        /**
         * Again test derivatives are correct initially, but for
         * different initial conditions (corresponding to a low
         * oxygen concentration). The usual initial condition for
         * z is zero, so we need to change it to see any difference.
         */
        oxygen_concentration = 0.1* unit::mole_per_metre_cubed;
        Owen2011OxygenBasedCellCycleOdeSystem cancer_system2(oxygen_concentration, mutation_state);

        std::vector<double> cancer_derivs2(initial_conditions.size());
        cancer_system2.SetDefaultInitialCondition(2, 0.1);
        std::vector<double> initial_conditions2 = cancer_system2.GetInitialConditions();
        cancer_system2.EvaluateYDerivatives(time, initial_conditions2, cancer_derivs2);

        // Cancer cell
        TS_ASSERT_DELTA(cancer_derivs2[0], 1.02187e-05, 1e-6);
        TS_ASSERT_DELTA(cancer_derivs2[1], 3.33333e-05, 1e-6);
        TS_ASSERT_DELTA(cancer_derivs2[2], 1.76315e-05, 1e-6);
        TS_ASSERT_DELTA(cancer_derivs2[3], 0.0, 1e-5);
    }

    /**
     * Test two ODE solvers with this ODE system (correct values calculated using the Matlab solver ode15s).
     *
     */
    void DontTestOwen2011Solver() throw(Exception)
    {
        // Set up
        units::quantity<unit::concentration> oxygen_concentration = 1.0 * unit::mole_per_metre_cubed;
        boost::shared_ptr<WildTypeCellMutationState> mutation_state(new WildTypeCellMutationState);

        Owen2011OxygenBasedCellCycleOdeSystem owen_system(oxygen_concentration, mutation_state);

        // Create ODE solvers
        RungeKutta4IvpOdeSolver rk4_solver;
        RungeKuttaFehlbergIvpOdeSolver rkf_solver;
        CvodeAdaptor cvode_solver;
        cvode_solver.CheckForStoppingEvents();

        // Set up for solver
        OdeSolution solutions1;
        OdeSolution solutions2;
        OdeSolution solutions3;
        std::vector<double> initial_conditions = owen_system.GetInitialConditions();
        double start_time = 0.0;
        double end_time = 0.0;
        double elapsed_time = 0.0;
        double h_value = 1e-4; // maximum tolerance

        // Solve the ODE system using a Runge Kutta fourth order solver
        start_time = std::clock();
        solutions1 = rk4_solver.Solve(&owen_system, initial_conditions, 0.0, 210.0, h_value, h_value);
        end_time = std::clock();
        elapsed_time = (end_time - start_time)/(CLOCKS_PER_SEC);
        std::cout << "1. Runge-Kutta Elapsed time = " << elapsed_time << "\n";

        // Solve the ODE system using a Runge Kutta Fehlber solver
        initial_conditions = owen_system.GetInitialConditions();
        start_time = std::clock();
        solutions2 = rkf_solver.Solve(&owen_system, initial_conditions, 0.0, 210.0, h_value, h_value);
        end_time = std::clock();
        elapsed_time = (end_time - start_time)/(CLOCKS_PER_SEC);
        std::cout << "2. Runge-Kutta-Fehlberg Elapsed time = " << elapsed_time << "\n";

        // Solve the ODE system using the Cvode solver
        initial_conditions = owen_system.GetInitialConditions();
        start_time = std::clock();
        solutions3 = cvode_solver.Solve(&owen_system, initial_conditions, 0.0, 210.0, 10.0, 10);
        end_time = std::clock();
        elapsed_time = (end_time - start_time)/(CLOCKS_PER_SEC);
        std::cout << "3. Cvode Solver Elapsed time = " << elapsed_time << "\n";

        // Test that solutions are accurate for a small time increase
        int end1 = solutions1.rGetSolutions().size() - 1;
        int end2 = solutions2.rGetSolutions().size() - 1;
        int end3 = solutions3.rGetSolutions().size() - 1;

        // Test that the solver stops at the right time
        // Rate of increase in phase is 0.005 hr^-1 (as in Line 97). THis remains
        // constant in time so end/division time is 1/0.005 = 200
        TS_ASSERT_DELTA(solutions1.rGetTimes()[end1], 210.00, 1e-2);
        TS_ASSERT_DELTA(solutions2.rGetTimes()[end2], 210.00, 1e-2);
        TS_ASSERT_DELTA(solutions3.rGetTimes()[end3], 210.00, 1e-2);


        // Test solution - note the high tolerances
        TS_ASSERT_DELTA(solutions1.rGetSolutions()[end1][0], 1.000, 1e-3);
        TS_ASSERT_DELTA(solutions1.rGetSolutions()[end1][3], 1.000, 1e-3);
        TS_ASSERT_DELTA(solutions2.rGetSolutions()[end2][0], 1.000, 1e-3);
        TS_ASSERT_DELTA(solutions2.rGetSolutions()[end2][3], 1.000, 1e-3);
        TS_ASSERT_DELTA(solutions3.rGetSolutions()[end3][0], 1.000, 1e-3);
        TS_ASSERT_DELTA(solutions3.rGetSolutions()[end3][3], 1.000, 1e-3);

    }

    void DontTestArchiving()
    {
        OutputFileHandler handler("archive", false);
        std::string archive_filename = handler.GetOutputDirectoryFullPath() + "owen_ode.arch";

        {
            units::quantity<unit::concentration> oxygen_concentration = 0.0007 * unit::mole_per_metre_cubed;
            boost::shared_ptr<WildTypeCellMutationState> mutation_state(new WildTypeCellMutationState);

            Owen2011OxygenBasedCellCycleOdeSystem ode_system(oxygen_concentration, mutation_state);

            TS_ASSERT_DELTA(ode_system.GetOxygenConcentration().value(), 0.0007, 1e-6);
            TS_ASSERT(ode_system.GetMutationState()->IsType<WildTypeCellMutationState>());

            std::vector<double> initial_conditions = ode_system.GetInitialConditions();
            TS_ASSERT_EQUALS(initial_conditions.size(), 4u);
            TS_ASSERT_DELTA(initial_conditions[0], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[1], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[2], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[3],700.0, 1e-4);

            // Create an output archive
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);

            // Archive ODE system
            AbstractOdeSystem* const p_const_ode_system = &ode_system;
            output_arch << p_const_ode_system;
        }

        {
            AbstractOdeSystem* p_ode_system;

            // Create an input archive
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // Restore from the archive
            input_arch >> p_ode_system;

            // Check that archiving worked correctly
            TS_ASSERT_DELTA(static_cast<Owen2011OxygenBasedCellCycleOdeSystem*>(p_ode_system)->GetOxygenConcentration().value(), 0.0007, 1e-6);
            TS_ASSERT(static_cast<Owen2011OxygenBasedCellCycleOdeSystem*>(p_ode_system)->GetMutationState()->IsType<WildTypeCellMutationState>());

            std::vector<double> initial_conditions = p_ode_system->GetInitialConditions();
            TS_ASSERT_EQUALS(initial_conditions.size(), 4u);
            TS_ASSERT_DELTA(initial_conditions[0], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[1], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[2], 0.00, 1e-6);
            TS_ASSERT_DELTA(initial_conditions[3], 700.0, 1e4);

            // Tidy up
            delete p_ode_system;
        }
    }
};

#endif /*TESTOWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_*/
