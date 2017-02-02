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

#ifndef TESTLQRADIOTHERAPYCELLKILLER_HPP_
#define TESTLQRADIOTHERAPYCELLKILLER_HPP_

#include <cxxtest/TestSuite.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <vector>
#include <iostream>
#include "AbstractCellBasedTestSuite.hpp"
#include "OutputFileHandler.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "PottsMeshGenerator.hpp"
#include "PottsMesh.hpp"
#include "CaBasedCellPopulation.hpp"
#include "CellsGenerator.hpp"
#include "SimpleOxygenBasedCellCycleModel.hpp"
#include "LQRadiotherapyCellKiller.hpp"
#include "OnLatticeSimulation.hpp"
#include "ApoptoticCellKiller.hpp"

#include "PetscSetupAndFinalize.hpp"

/**
 * Test RT cell killer
 */
class TestLQRadiotherapyCellKiller : public AbstractCellBasedTestSuite
{
public:

    void TestSimulationWithOer()
    {
        PottsMeshGenerator<2> generator(5, 0, 0, 5, 0, 0);
        PottsMesh<2>* p_mesh = generator.GetMesh();

        // Fill the domain with tumour cells
        std::vector<unsigned> location_indices;
        for(unsigned idx=0; idx<25; idx++)
        {
            location_indices.push_back(idx);
        }

        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, location_indices.size());

        units::quantity<unit::concentration> reference_concentration = 1.e-3*unit::mole_per_metre_cubed;
        units::quantity<unit::time> reference_time = 3600.0*unit::seconds;
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);
        units::quantity<unit::solubility> oxygen_solubility_at_stp =
                Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        units::quantity<unit::concentration> oxygen_concentration = 40.0*unit::pascals*oxygen_solubility_at_stp;
        for(unsigned idx=0;idx<cells.size(); idx++)
        {
            cells[idx]->GetCellData()->SetItem("oxygen", oxygen_concentration/reference_concentration);
        }

        // Create cell population
        boost::shared_ptr<CaBasedCellPopulation<2> > p_cell_population =
                boost::shared_ptr<CaBasedCellPopulation<2> >(new CaBasedCellPopulation<2>(*p_mesh, cells, location_indices));

        boost::shared_ptr<LQRadiotherapyCellKiller<2> > p_rt_killer =
                boost::shared_ptr<LQRadiotherapyCellKiller<2> >(new LQRadiotherapyCellKiller<2> (p_cell_population.get()));
        p_rt_killer->SetAlphaMax(0.3*unit::per_gray);
        p_rt_killer->SetBetaMax(0.03*unit::per_gray_squared);
        p_rt_killer->SetDoseInjected(2.0*unit::gray);
        p_rt_killer->SetCancerousRadiosensitivity(0.3 * unit::per_gray, 0.03 * unit::per_gray_squared);
        p_rt_killer->SetNormalRadiosensitivity(0.15 * unit::per_gray, 0.05 * unit::per_gray_squared);
        p_rt_killer->SetOerAlphaMax(1.75);
        p_rt_killer->SetOerAlphaMin(1.0);
        p_rt_killer->SetOerBetaMax(3.25);
        p_rt_killer->SetOerBetaMin(1.0);
        p_rt_killer->SetOerConstant(oxygen_solubility_at_stp * 3.28 * unit::pascals);
        p_rt_killer->SetAlphaMax(0.3 * unit::per_gray);
        p_rt_killer->SetBetaMax(0.03 * unit::per_gray_squared);
        p_rt_killer->UseOer(true);

        // Set Up the Radiation Times
        std::vector<units::quantity<unit::time> > rt_times;
        rt_times.push_back(3600.0*24.0*unit::seconds);
        rt_times.push_back(3600.0*48.0*unit::seconds);
        rt_times.push_back(3600.0*72.0*unit::seconds);
        p_rt_killer->SetTimeOfRadiation(rt_times);
        p_rt_killer->AddTimeOfRadiation(3600.0*96.0*unit::seconds);

        OnLatticeSimulation<2> simulator(*p_cell_population);
        /*
         * Add a killer for RT and to remove apoptotic cells
         */
        boost::shared_ptr<ApoptoticCellKiller<2> > p_apoptotic_cell_killer(new ApoptoticCellKiller<2>(p_cell_population.get()));
        simulator.AddCellKiller(p_apoptotic_cell_killer);
        simulator.AddCellKiller(p_rt_killer);
        simulator.SetOutputDirectory("TestLQRadiotherapyCellKiller/WithOer");
        simulator.SetSamplingTimestepMultiple(1);
        simulator.SetDt(2.0);
        simulator.SetEndTime(96.0); // hours
        simulator.Solve();
    }

    void TestSimulationWithoutOer()
    {
        PottsMeshGenerator<2> generator(5, 0, 0, 5, 0, 0);
        PottsMesh<2>* p_mesh = generator.GetMesh();

        // Fill the domain with tumour cells
        std::vector<unsigned> location_indices;
        for(unsigned idx=0; idx<25; idx++)
        {
            location_indices.push_back(idx);
        }

        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasic(cells, location_indices.size());

        units::quantity<unit::concentration> reference_concentration = 1.e-3*unit::mole_per_metre_cubed;
        units::quantity<unit::time> reference_time = 3600.0*unit::seconds;
        BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
        BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);
        units::quantity<unit::solubility> oxygen_solubility_at_stp =
                Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("User") *
                GenericParameters::mpGasConcentrationAtStp->GetValue("User");
        units::quantity<unit::concentration> oxygen_concentration = 40.0*unit::pascals*oxygen_solubility_at_stp;
        for(unsigned idx=0;idx<cells.size(); idx++)
        {
            cells[idx]->GetCellData()->SetItem("oxygen", oxygen_concentration/reference_concentration);
        }

        // Create cell population
        boost::shared_ptr<CaBasedCellPopulation<2> > p_cell_population =
                boost::shared_ptr<CaBasedCellPopulation<2> >(new CaBasedCellPopulation<2>(*p_mesh, cells, location_indices));

        boost::shared_ptr<LQRadiotherapyCellKiller<2> > p_rt_killer =
                boost::shared_ptr<LQRadiotherapyCellKiller<2> >(new LQRadiotherapyCellKiller<2> (p_cell_population.get()));
        p_rt_killer->SetAlphaMax(0.3*unit::per_gray);
        p_rt_killer->SetBetaMax(0.03*unit::per_gray_squared);
        p_rt_killer->SetDoseInjected(2.0*unit::gray);
        p_rt_killer->SetCancerousRadiosensitivity(0.3 * unit::per_gray, 0.03 * unit::per_gray_squared);
        p_rt_killer->SetNormalRadiosensitivity(0.15 * unit::per_gray, 0.05 * unit::per_gray_squared);
        p_rt_killer->SetOerAlphaMax(1.75);
        p_rt_killer->SetOerAlphaMin(1.0);
        p_rt_killer->SetOerBetaMax(3.25);
        p_rt_killer->SetOerBetaMin(1.0);
        p_rt_killer->SetOerConstant(oxygen_solubility_at_stp * 3.28 * unit::pascals);
        p_rt_killer->SetAlphaMax(0.3 * unit::per_gray);
        p_rt_killer->SetBetaMax(0.03 * unit::per_gray_squared);
        p_rt_killer->UseOer(false);

        // Set Up the Radiation Times
        std::vector<units::quantity<unit::time> > rt_times;
        rt_times.push_back(3600.0*24.0*unit::seconds);
        rt_times.push_back(3600.0*48.0*unit::seconds);
        rt_times.push_back(3600.0*72.0*unit::seconds);
        rt_times.push_back(3600.0*96.0*unit::seconds);
        p_rt_killer->SetTimeOfRadiation(rt_times);

        OnLatticeSimulation<2> simulator(*p_cell_population);
        /*
         * Add a killer for RT and to remove apoptotic cells
         */
        boost::shared_ptr<ApoptoticCellKiller<2> > p_apoptotic_cell_killer(new ApoptoticCellKiller<2>(p_cell_population.get()));
        simulator.AddCellKiller(p_apoptotic_cell_killer);
        simulator.AddCellKiller(p_rt_killer);
        simulator.SetOutputDirectory("TestLQRadiotherapyCellKiller/WithoutOer");
        simulator.SetSamplingTimestepMultiple(1);
        simulator.SetDt(2.0);
        simulator.SetEndTime(96.0); // hours
        simulator.Solve();
    }
};

#endif /*TESTLQRADIOTHERAPYCELLKILLER_HPP_*/
