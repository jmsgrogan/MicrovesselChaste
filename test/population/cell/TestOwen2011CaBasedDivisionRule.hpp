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

#ifndef TESTOWEN2011CABASEDDIVISIONRULE_HPP_
#define TESTOWEN2011CABASEDDIVISIONRULE_HPP_

#include <cxxtest/TestSuite.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <stdio.h>
#include <ctime>
#include <vector>
#include <iostream>
#include "AbstractCellBasedTestSuite.hpp"
#include "OutputFileHandler.hpp"
#include "WildTypeCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "Owen11CaBasedDivisionRule.hpp"
#include "PottsMeshGenerator.hpp"
#include "PottsMesh.hpp"
#include "CaBasedCellPopulation.hpp"
#include "CellsGenerator.hpp"
#include "SimpleOxygenBasedCellCycleModel.hpp"

/**
 * Test the CaDivision rules
 */
class TestOwen2011CaBasedDivisionRule : public AbstractCellBasedTestSuite
{
public:

    void TestSimpleMethods()
    {
        Owen11CaBasedDivisionRule<3> division_rule;

        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        std::vector<boost::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<81; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10.0, 50.0, 100.0, 1.e-6*unit::metres));
        }
        boost::shared_ptr<Vessel<3> > p_vessel_1 = Vessel<3>::Create(bottom_nodes);
        p_network->AddVessel(p_vessel_1);
        division_rule.SetVesselNetwork(p_network);

        PottsMeshGenerator<3> generator(10, 0, 0, 10, 0, 0, 1, 0, 0);
        PottsMesh<3>* p_mesh = generator.GetMesh();

        // Create a tumour cells in a cylinder in the middle of the domain
        std::vector<unsigned> location_indices;
        for(unsigned idx=0; idx<100; idx++)
        {
            location_indices.push_back(idx);
        }

        std::vector<CellPtr> cells;
        CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
        cells_generator.GenerateBasic(cells, location_indices.size());

        // Create cell population
        CaBasedCellPopulation<3> cell_population(*p_mesh, cells, location_indices);

        TS_ASSERT_THROWS_THIS(division_rule.IsRoomToDivide(cell_population.GetCellUsingLocationIndex(0), cell_population),
                              "A regular grid is required for determining vessel based lattice occupancy");
    }

    void TestArchiving()
    {
        OutputFileHandler handler("archive", false);
        std::string archive_filename = handler.GetOutputDirectoryFullPath() + "TestOwen2011CaBasedDivisionRule.arch";
        {
            Owen11CaBasedDivisionRule<3> division_rule;

            // Create an output archive
            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);

            // Archive ODE system
            AbstractCaBasedDivisionRule<3>* const p_division_rule = &division_rule;
            output_arch << p_division_rule;
        }

        {
            AbstractCaBasedDivisionRule<3>* p_division_rule;

            // Create an input archive
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // Restore from the archive
            input_arch >> p_division_rule;

            // No archived members, check that we can cast.
            bool can_cast = static_cast<Owen11CaBasedDivisionRule<3> *>(p_division_rule);
            TS_ASSERT(can_cast);

            // Tidy up
            delete p_division_rule;
        }
    }
};

#endif /*TESTOWEN2011CABASEDDIVISIONRULE_HPP_*/
