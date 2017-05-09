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

#ifndef WALLSHEARSTRESSBASEDREGRESSIONSOLVER_HPP_
#define WALLSHEARSTRESSBASEDREGRESSIONSOLVER_HPP_

#include "UnitCollection.hpp"
#include "RegressionSolver.hpp"

/**
 * This class is for simulating modifications to the vessel network due to regression.
 */
template<unsigned DIM>
class WallShearStressBasedRegressionSolver : public RegressionSolver<DIM>
{
    /**
     * Threshold wall shear stress level, below which vessels will be removed.
     * This threshold should be prescribed in units of pascals.
     */
    units::quantity<unit::pressure> mThresholdWss;

    /**
     *  Maximum time that a vessel may exist with low wall shear stress.
     *  After this amount of time a vessel is removed completely from
     *  the vessel network.
     *  This time should be prescribed in units of hours.
     */
    units::quantity<unit::time> mMaxTimeWithLowWss;

public:

    /**
     * Constructor.
     */
    WallShearStressBasedRegressionSolver();

    /**
     * Destructor.
     */
    virtual ~WallShearStressBasedRegressionSolver();

    /**
     * Factor constructor. Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class.
     */
    static boost::shared_ptr<WallShearStressBasedRegressionSolver<DIM> > Create();

    /**
     *  Setter for mMaxTimeWithLowWss parameter.
     *  @param time the max time for low WSS
     */
    void SetMaximumTimeWithLowWallShearStress(units::quantity<unit::time> time);

    /**
     *  Setter for mThresholdWss parameter.
     *  @param threshold the value of WSS below which it is considered too low for the vessel
     */
    void SetLowWallShearStressThreshold(units::quantity<unit::pressure> threshold);

    /**
     * Increment one step in time
     */
    virtual void Increment();

};

#endif /* WALLSHEARSTRESSBASEDREGRESSIONSOLVER_HPP_ */
