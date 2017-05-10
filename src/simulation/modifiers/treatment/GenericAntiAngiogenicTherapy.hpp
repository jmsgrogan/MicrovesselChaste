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

#ifndef GENERICANTIANGIOGENICTHERAPY_HPP_
#define GENERICANTIANGIOGENICTHERAPY_HPP_

#include "RandomNumberGenerator.hpp"
#include "AbstractTherapy.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"
#include "AbstractSproutingRule.hpp"
#include "UnitCollection.hpp"

/**
 * A generic anti-angiogenic therapy which reduces sensitivity to VEGF and
 * decreases time until regression.
 */
template<unsigned DIM>
class GenericAntiAngiogenicTherapy : public AbstractTherapy<DIM>
{
    units::quantity<unit::concentration> mCurrentConcentration;

    units::quantity<unit::concentration> mConcentrationAtMaxEffect;

    units::quantity<unit::rate> mRemovalRate;

    boost::shared_ptr<WallShearStressBasedRegressionSolver<DIM> > mpRegressionSolver;

    boost::shared_ptr<AbstractSproutingRule<DIM> > mpSproutingRule;

    units::quantity<unit::time> mPreviousTime;

    units::quantity<unit::pressure> mInitialWSS;

    units::quantity<unit::time> mInitialRegressionTime;

    units::quantity<unit::rate> mInitialSproutingProbability;

public:

    /**
     * Default constructor.
     */
    GenericAntiAngiogenicTherapy();

    virtual ~GenericAntiAngiogenicTherapy();

    void SetRegressionSolver(boost::shared_ptr<WallShearStressBasedRegressionSolver<DIM> > regressionSolver);

    void SetSproutingRule(boost::shared_ptr<AbstractSproutingRule<DIM> > sproutingRule);

    /**
     * Overridden SetupSolve() method.
     * Specify what to do in the simulation before the start of the time loop.
     *
     * @param outputDirectory the output directory, relative to where Chaste output is stored
     */
    virtual void SetupSolve(std::string outputDirectory);

    /**
     * Overridden UpdateAtEndOfTimeStep() method.
     * Specify what to do in the simulation at the end of each time step.
     */
    virtual void UpdateAtEndOfTimeStep();
};

#endif
