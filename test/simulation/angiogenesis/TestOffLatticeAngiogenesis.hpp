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

#ifndef TESTOFFLATTICEANGIOGENESISSOLVER_HPP
#define TESTOFFLATTICEANGIOGENESISSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "RandomNumberGenerator.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "LatticeBasedMigrationRule.hpp"
#include "LatticeBasedSproutingRule.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "AngiogenesisSolver.hpp"
#include "GridCalculator.hpp"
#include "CornealMicropocketSimulation.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestOffLatticeAngiogenesisSolver : public CxxTest::TestSuite
{

public:

    void Test2dPlane() throw(Exception)
    {
        CornealMicropocketSimulation<2> simulation;
        simulation.SetDomainType(DomainType::PLANAR_2D);
        simulation.SetWorkDir("TestOffLatticeAngiogenesisSolver/Plane_2D_FG");
        simulation.SetIncludeVesselSink(false);
        simulation.SetUptakeRatePerCell((4.e-12/3600.0)*unit::mole_per_second);
        simulation.SetUseFixedGradient(true);
        simulation.SetPelletConcentration(1.e-10*unit::mole_per_metre_cubed);
        simulation.SetTotalTime(96.0*3600.0*unit::seconds);
        simulation.SetPersistenceAngle(0.0);
        simulation.SetChemotacticStrength(0.0);
        simulation.SetOnlyPerfusedSprout(true);

        simulation.Run();
    }

    void Test2dCircle() throw(Exception)
    {
        CornealMicropocketSimulation<2> simulation;
        simulation.SetDomainType(DomainType::CIRCLE_2D);
        simulation.SetWorkDir("TestOffLatticeAngiogenesisSolver/Circle_2D_FG");
        simulation.SetIncludeVesselSink(false);
        simulation.SetUptakeRatePerCell((4.e-12/3600.0)*unit::mole_per_second);
        simulation.SetUseFixedGradient(true);
        simulation.SetPelletConcentration(8.e-10*unit::mole_per_metre_cubed);
        simulation.SetTotalTime(72.0*3600.0*unit::seconds);
        simulation.SetPelletHeight(200.0e-6*unit::metres);
        simulation.SetPersistenceAngle(0.0);
        simulation.SetChemotacticStrength(0.0);
        simulation.SetOnlyPerfusedSprout(true);
        simulation.Run();
    }

    void Test3dPlane() throw(Exception)
    {
        CornealMicropocketSimulation<3> simulation;
        simulation.SetDomainType(DomainType::PLANAR_3D);
        simulation.SetWorkDir("TestOffLatticeAngiogenesisSolver/Plane_3D_FG");
        simulation.SetIncludeVesselSink(false);
        simulation.SetUptakeRatePerCell((4.e-12/3600.0)*unit::mole_per_second);
        simulation.SetUseFixedGradient(true);
        simulation.SetPelletConcentration(1.e-10*unit::mole_per_metre_cubed);
        simulation.SetTotalTime(96.0*3600.0*unit::seconds);
        simulation.SetPersistenceAngle(0.0);
        simulation.SetChemotacticStrength(0.0);
        //simulation.SetOnlyPerfusedSprout(true);
        simulation.SetGridSpacing(20.0e-6*unit::metres);
        simulation.Run();
    }

    void Test3dCircle() throw(Exception)
    {
        CornealMicropocketSimulation<3> simulation;
        simulation.SetDomainType(DomainType::CIRCLE_3D);
        simulation.SetWorkDir("TestOffLatticeAngiogenesisSolver/Circle_3D_FG");
        simulation.SetIncludeVesselSink(false);
        simulation.SetUptakeRatePerCell((4.e-12/3600.0)*unit::mole_per_second);
        simulation.SetUseFixedGradient(true);
        simulation.SetPelletConcentration(1.e-10*unit::mole_per_metre_cubed);
        simulation.SetTotalTime(96.0*3600.0*unit::seconds);
        simulation.SetPersistenceAngle(5.0);
        simulation.SetChemotacticStrength(0.5);
        simulation.SetPelletHeight(200.0e-6*unit::metres);
        simulation.Run();
    }

    void xTestHemisphere() throw(Exception)
    {
        CornealMicropocketSimulation<3> simulation;
        simulation.SetDomainType(DomainType::HEMISPHERE);
        simulation.SetWorkDir("TestOffLatticeAngiogenesisSolver/Hemisphere_FG");
        simulation.SetIncludeVesselSink(false);
        simulation.SetUptakeRatePerCell((4.e-12/3600.0)*unit::mole_per_second);
        simulation.SetUseFixedGradient(true);
        simulation.SetPelletConcentration(1.e-10*unit::mole_per_metre_cubed);
        simulation.SetTotalTime(96.0*3600.0*unit::seconds);
        simulation.SetPersistenceAngle(0.0);
        simulation.SetChemotacticStrength(0.0);
        simulation.SetOnlyPerfusedSprout(true);
        simulation.Run();
    }
};

#endif // TESTOFFLATTICEANGIOGENESISSOLVER_HPP
