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

#ifndef OnLatticeSimulationWrapper_HPP_
#define OnLatticeSimulationWrapper_HPP_

#include <string>
#include <vector>
#include "UblasVectorInclude.hpp"
#include "SmartPointers.hpp"
#include "SimpleCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "LQRadiotherapyCellKiller.hpp"
#include "VesselNetwork.hpp"

class OnLatticeSimulationWrapper
{
    double mDt;

    double mEndTime;

    unsigned mSamplingMultiple;

    std::string mWorkingDirectory;

    boost::shared_ptr<CaBasedCellPopulation<3> > mpInputCellPopulation;

    std::vector<boost::shared_ptr<SimpleCellPopulation<3> > > mOutputCellPopulations;

    bool mUseRadiotherapyKiller;

    boost::shared_ptr<VesselNetwork<3> > mpNetwork;

    units::quantity<unit::length> mVesselDistanceTolerance;

    std::vector<double> mRadiotherapyHitTimes;

    double mRadiotherapyDose;

    double mOerAlphaMax;

    double mOerAlphaMin;

    double mOerBetaMax;

    double mOerBetaMin;

    double mKOer;

    double mAlphaMax;

    double mBetaMax;

    bool mUseOer;


public:

    /* Constructor
     */
    OnLatticeSimulationWrapper();

    /* Desctructor
     */
    ~OnLatticeSimulationWrapper();

    void SetCellPopulation(boost::shared_ptr<CaBasedCellPopulation<3> > pInputPopulation);

    void SetOutputDirectory(const std::string& rDirectory);

    void SetRadiotherapyHitTimes(std::vector<double> hitTimes);

    void SetRadiotherapyDose(double dose);

    void SetOerAlphaMax(double value);

    void SetOerAlphaMin(double value);

    void SetOerBetaMax(double value);

    void SetOerBetaMin(double value);

    void SetOerConstant(double value);

    void SetAlphaMax(double value);

    void SetBetaMax(double value);

    void UseOer(bool useOer);

    void SetDt(double timeStepSize);

    void SetSamplingTimestepMultiple(unsigned samplingMultiple);

    void SetUseRadiotherapyCellKiller(bool UseKiller);

    void SetEndTime(double endTime);

    void SetNetwork(boost::shared_ptr<VesselNetwork<3> > pNetwork);

    void Solve(boost::shared_ptr<MicrovesselSimulationModifier<3> > pVtModifier);

    void SetVesselDistanceTolerance(units::quantity<unit::length> tolerance);

    std::vector<boost::shared_ptr<SimpleCellPopulation<3> > > GetOutputPopulations();

};



#endif /* OnLatticeSimulationWrapper_HPP_*/
