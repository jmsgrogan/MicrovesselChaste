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

#ifndef TESTDENSITYMAP_HPP_
#define TESTDENSITYMAP_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "Part.hpp"
#include "DensityMap.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "BaseUnits.hpp"
#include "PetscTools.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestDensityMap : public CxxTest::TestSuite
{

public:

    void TestAllInsideBoxAndAllOutsideBox2d() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Set up the vessel network
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        units::quantity<unit::length> vessel_length = 0.15 * unit::metres;
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(
                vessel_length, DimensionalChastePoint<2>(0.25, 0.05, 0.0, 1.0*unit::metres));

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2.0 * unit::metres, 2.0 * unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 1.0 * unit::metres);

        // Get the map
        DensityMap<2> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetGrid(p_grid);
        std::vector<double> line_density = solver.rGetVesselLineDensity(true);

        // Check the density
        TS_ASSERT_DELTA(line_density[0], 0.15/(0.501*0.501), 1e-6);
        TS_ASSERT_DELTA(line_density[1], 0.0, 1e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestCrossingBoxes2d() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Set up the vessel network
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        units::quantity<unit::length> vessel_length = 0.5 * unit::metres;
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(
                vessel_length, DimensionalChastePoint<2>(0.25, 0.25, 0.0, 1.0*unit::metres));

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2.0 * unit::metres, 2.0 * unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 1.0 * unit::metres);

        // Get the map
        DensityMap<2> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetGrid(p_grid);
        std::vector<double> line_density = solver.rGetVesselLineDensity(true);

        // Check the density
        TS_ASSERT_DELTA(line_density[0], 0.251/(0.501*0.501), 1e-6);
        TS_ASSERT_DELTA(line_density[3], 0.249/(0.501*1.000), 1e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestAllInsideBoxAndAllOutsideBox3d() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Set up the vessel network
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        units::quantity<unit::length> vessel_length = 0.15 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(
                vessel_length, DimensionalChastePoint<3>(0.25, 0.05, 0.05, 1.0*unit::metres));

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(2.0 * unit::metres, 2.0 * unit::metres, 2.0 * unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 1.0 * unit::metres);

        // Get the map
        DensityMap<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetGrid(p_grid);
        std::vector<double> line_density = solver.rGetVesselLineDensity(true);

        // Check the density
        TS_ASSERT_DELTA(line_density[0], 0.15/(0.501*0.501*0.501), 1e-6);
        TS_ASSERT_DELTA(line_density[1], 0.0, 1e-6);
        TS_ASSERT_DELTA(line_density[2], 0.0, 1e-6);
        TS_ASSERT_DELTA(line_density[3], 0.0, 1e-6);
        TS_ASSERT_DELTA(line_density[4], 0.0, 1e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestCrossingBoxes3d() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Set up the vessel network
        BaseUnits::Instance()->SetReferenceLengthScale(1.0 * unit::metres);
        units::quantity<unit::length> vessel_length = 0.5 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(
                vessel_length, DimensionalChastePoint<3>(0.25, 0.25, 0.0, 1.0*unit::metres), 1, 1);

        // Set up the grid
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(2.0 * unit::metres, 2.0 * unit::metres, 2.0 * unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 1.0 * unit::metres);

        // Get the map
        DensityMap<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetGrid(p_grid);
        std::vector<double> line_density = solver.rGetVesselLineDensity(true);

        // Check the density
        TS_ASSERT_DELTA(line_density[0], 0.251/(0.501*0.501*0.501), 1e-2);
        TS_ASSERT_DELTA(line_density[3], 0.249/(0.501*0.501*1.0), 1e-3);
        TS_ASSERT_DELTA(line_density[4], 0.0, 1e-6);
        BaseUnits::Instance()->Destroy();
    }

    void TestBifurcationNetwork() throw(Exception)
    {
        std::string output_path = "TestDensityMap/Bifurcation";
        if(PetscTools::IsParallel())
        {
            output_path += "Parallel";
        }
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, (output_path, true));

        // Set up the vessel network
        units::quantity<unit::length> vessel_length = 100 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<3> generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateBifurcationUnit(vessel_length,
                                                                                           DimensionalChastePoint<3>(0.0, vessel_length/(1.e-6*unit::metres), 0.0));
        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"/network.vtp");

        // Set up the tissue domain
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(4.0 * vessel_length,
                            2.0 * vessel_length,
                            2.0 * vessel_length,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 20.0e-6 * unit::metres);

        // Get the map
        DensityMap<3> calculator;
        calculator.SetVesselNetwork(p_network);
        calculator.SetGrid(p_grid);
        std::vector<double> line_density = calculator.rGetVesselLineDensity(true);
        p_grid->AddPointData(line_density);
        p_grid->Write(p_output_file_handler);
    }

    void TestBifurcationNetwork2d() throw(Exception)
    {

        std::string output_path = "TestDensityMap/Bifurcation2d";
        if(PetscTools::IsParallel())
        {
            output_path += "Parallel";
        }
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, (output_path, true));

        // Set up the vessel network
        units::quantity<unit::length> vessel_length = 100 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateBifurcationUnit(vessel_length,
                                                                                           DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"/network.vtp");

        // Set up the tissue domain
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(4.0 * vessel_length,
                            2.0 * vessel_length,
                            DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 20.0e-6 * unit::metres);

        // Get the map
        DensityMap<2> calculator;
        calculator.SetVesselNetwork(p_network);
        calculator.SetGrid(p_grid);
        std::vector<double> line_density = calculator.rGetVesselLineDensity(true);
        p_grid->AddPointData(line_density);
        p_grid->Write(p_output_file_handler);
    }

    void TestConservationOverBoxSize() throw(Exception)
    {
        EXIT_IF_PARALLEL;

        // Set up the vessel network
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestDensityMap/TestConservationOverBoxSize", false));
        units::quantity<unit::length> vessel_length = 100 * 1.e-6 * unit::metres;
        VesselNetworkGenerator<2> generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateBifurcationUnit(vessel_length,
                                                                                           DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        p_network->Write(p_output_file_handler->GetOutputDirectoryFullPath()+"/network.vtp");
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(4.0 * vessel_length, 2.0 * vessel_length, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        std::vector<double> densities;
        double grid_size = 5.0;
        for(unsigned idx=0; idx<5; idx++)
        {
            // Set up the tissue domain
            boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
            p_grid->GenerateFromPart(p_domain, grid_size*1.e-6* unit::metres);

            // Get the map
            DensityMap<2> calculator;
            calculator.SetVesselNetwork(p_network);
            calculator.SetGrid(p_grid);
            std::vector<double> line_density = calculator.rGetVesselLineDensity(true);
            p_grid->AddPointData(line_density);
            p_grid->Write(p_output_file_handler);

            double running_total = 0.0;
            for(unsigned jdx=0; jdx<line_density.size(); jdx++)
            {
                c_vector<double, 6> bbox = p_grid->GetPointBoundingBox(jdx);
                running_total+= line_density[jdx]*(bbox[1]-bbox[0])*(bbox[3]-bbox[2]);
            }

            densities.push_back(running_total);
            grid_size*=2.0;
        }

        for(unsigned idx=1; idx<5; idx++)
        {
            TS_ASSERT_DELTA(densities[0]/densities[idx], 1.0, 1.e-3);
        }
    }
};

#endif /*TESTDENSITYMAP_HPP_*/
