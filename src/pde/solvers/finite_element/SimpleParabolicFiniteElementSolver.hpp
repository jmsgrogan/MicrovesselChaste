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

#ifndef SIMPLEPARABOLICFINITEELEMENTSOLVER_HPP_
#define SIMPLEPARABOLICFINITEELEMENTSOLVER_HPP_

#include "SmartPointers.hpp"
#include "UnitCollection.hpp"
#include "AbstractFiniteElementSolverBase.hpp"

/**
 * A finite element solver for parabolic PDEs with multiple discrete sinks or sources.
 */
template<unsigned DIM>
class SimpleParabolicFiniteElementSolver : public AbstractFiniteElementSolverBase<DIM>
{

    /**
     * Storage for intermediate solutions. Useful for debugging.
     */
    std::vector<std::pair<std::vector<double>, double> > mIntermediateSolutionCollection;

    /**
     * How often to store intermediate solutions.
     */
    unsigned mIntermediateSolutionFrequency;

    /**
     * Whether to store intermediate solutions.
     */
    bool mStoreIntermediate;

    /**
     * Whether to write intermediate solutions.
     */
    bool mWriteIntermediate;

    /**
     * The target time increment
     */
    double mTimeIncrement;

    /**
     * The solve time
     */
    double mSolveStartTime;

    /**
     * The end time
     */
    double mSolveEndTime;

    /**
     * The initial guess
     */
    std::vector<double> mInitialGuess;


public:

    /**
     * Constructor
     */
    SimpleParabolicFiniteElementSolver();

    /**
     * Destructor
     */
    virtual ~SimpleParabolicFiniteElementSolver();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to a class instance.
     */
    static std::shared_ptr<SimpleParabolicFiniteElementSolver<DIM> > Create();

    /**
     * Return the intermediate solutions. Empty if not stored.
     * @return the intermediate solutions. Empty if not stored.
     */
    const std::vector<std::pair<std::vector<double>, double> >& rGetIntermediateSolutions();

    /**
     * Set the target time increment for the solver
     * @param targetIncrement the target time increment for the solver
     */
    void SetTargetTimeIncrement(double targetIncrement);

    /**
     * Set the solver start time
     * @param startTime the solver start time
     */
    void SetStartTime(double startTime);

    /**
     * Set the solver end time
     * @param endTime the solver end time
     */
    void SetEndTime(double endTime);

    /**
     * Set the initial guess
     * @param rInitialGuess the initial guess
     */
    void SetInitialGuess(const std::vector<double>& rInitialGuess);

    /**
     * Whether to store intermediate solutions, useful for debugging. Default (off).
     * @param store store intermediate solutions
     * @param frequency the frequency to store solutions at
     */
    void SetStoreIntermediateSolutions(bool store, unsigned frequency=1);

    /**
     * Whether to write intermediate solutions, useful for debugging. Default (off).
     * @param write write intermediate solutions
     * @param frequency the frequency to store solutions at
     */
    void SetWriteIntermediateSolutions(bool write, unsigned frequency=1);

    /**
     * Overridden solve method
     */
    void Solve();

};

#endif /* SIMPLEPARABOLICFINITEELEMENTSOLVER_HPP_ */
