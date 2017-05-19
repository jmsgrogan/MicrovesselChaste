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
};

#endif /*TESTOWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_*/
