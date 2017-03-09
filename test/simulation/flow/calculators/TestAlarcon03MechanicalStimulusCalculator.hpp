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

#ifndef TESTALARCON03MECHANICALSTIMULUSCALCULATOR_HPP
#define TESTALARCON03MECHANICALSTIMULUSCALCULATOR_HPP

#include <cxxtest/TestSuite.h>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include "VesselNetwork.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "UnitCollection.hpp"
#include "OutputFileHandler.hpp"

class TestMechanicalStimulusCalculator : public CxxTest::TestSuite
{

public:

    void TestSingleVessel()
    {
        // Make a network
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0));
        nodes.push_back(VesselNode<3>::Create(100));
        double pressure = 3933.0;
        nodes[0]->GetFlowProperties()->SetPressure(pressure*unit::pascals);
        nodes[1]->GetFlowProperties()->SetPressure(pressure*unit::pascals);

        boost::shared_ptr<Vessel<3> > p_vessel(Vessel<3>::Create(VesselSegment<3>::Create(nodes[0], nodes[1])));
        boost::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel);
        double wall_shear_stress = 25.0;
        p_vessel->GetSegments()[0]->GetFlowProperties()->SetWallShearStress(wall_shear_stress*unit::pascals);

        boost::shared_ptr<MechanicalStimulusCalculator<3> > calculator(new MechanicalStimulusCalculator<3>());
        calculator->SetVesselNetwork(p_vascular_network);
        calculator->Calculate();

        // convert pressure to mmhg
        double converted_pressure = pressure * (760.0 / (101325.0));
        double Tau_P = (100.0 - 86.0 * exp(-5000.0*pow(log10(log10(converted_pressure)), 5.4)));
        double expected_mechanical_stimulus = log10(10.0*wall_shear_stress + 1.e-5) - 0.5 * log10(Tau_P);
        TS_ASSERT_DELTA(p_vessel->GetSegments()[0]->GetFlowProperties()->GetGrowthStimulus().value(), expected_mechanical_stimulus, 1e-3);
    }

    void TestMechanicalStimulusVsPressure()
    {
        // Make a network
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0));
        nodes.push_back(VesselNode<3>::Create(100));
        double pressure = 3933.0;
        nodes[0]->GetFlowProperties()->SetPressure(pressure*unit::pascals);
        nodes[1]->GetFlowProperties()->SetPressure(pressure*unit::pascals);

        boost::shared_ptr<Vessel<3> > p_vessel(Vessel<3>::Create(VesselSegment<3>::Create(nodes[0], nodes[1])));
        boost::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel);
        double wall_shear_stress = 25.0;
        p_vessel->GetSegments()[0]->GetFlowProperties()->SetWallShearStress(wall_shear_stress*unit::pascals);

        boost::shared_ptr<MechanicalStimulusCalculator<3> > calculator(new MechanicalStimulusCalculator<3>());
        calculator->SetVesselNetwork(p_vascular_network);

        OutputFileHandler output_file(
                "Vasculature/StructuralAdaptationAlgorithm/TestAlarcon03MechanicalStimulusCalculator", true);

        std::string filename = output_file.GetOutputDirectoryFullPath() + "ShearStressVsPressure.dat";
        std::ofstream out(filename.c_str());
        out << "# Pressure (mmHg) # TauP (dyne/cm^2)\n";

        for (int mmHgPressure = 1; mmHgPressure < 101; mmHgPressure++)
        {
            nodes[0]->GetFlowProperties()->SetPressure(double(mmHgPressure) * (1.01 * pow(10.0, 5) / 760)*unit::pascals);
            nodes[1]->GetFlowProperties()->SetPressure(double(mmHgPressure) * (1.01 * pow(10.0, 5) / 760)*unit::pascals);
            calculator->Calculate();
            out << mmHgPressure << " " << (calculator->GetTauP()/unit::pascals) / (0.1) << "\n";
        }
        out.close();
    }

};

#endif // TESTALARCON03MECHANICALSTIMULUSCALCULATOR_HPP
