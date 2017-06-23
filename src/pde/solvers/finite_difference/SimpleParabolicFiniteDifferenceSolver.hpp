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

#ifndef SIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_
#define SIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_

#include <petscts.h>
#include "SmartPointers.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"
#include "UnitCollection.hpp"
#include "PetscTools.hpp"

/**
 * A finite difference solver for parabolic PDEs of the form:
 * du/dt = Grad.(D(x)*Grad(u))+LinearSourceTerm(x)+NonlinearSourceTerm(x, u)
 *
 * Solve or matrix assembly methods can be over-ridden to customize solution
 * options.
 */
template<unsigned DIM>
class SimpleParabolicFiniteDifferenceSolver : public AbstractFiniteDifferenceSolverBase<DIM>
{

protected:

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

public:

    /**
     * Constructor
     */
    SimpleParabolicFiniteDifferenceSolver();

    /**
     * Destructor
     */
    virtual ~SimpleParabolicFiniteDifferenceSolver();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to a class instance.
     */
    static std::shared_ptr<SimpleParabolicFiniteDifferenceSolver<DIM> > Create();

    /**
     * Assemble the system matrix
     */
    virtual void AssembleMatrix();

    /**
     * Assemble the system vector
     */
    virtual void AssembleVector();

    /**
     * Compute the residual vector given the current solution guess.
     *
     * @param currentGuess The solution guess for the current nonlinear solve iteration.
     * @param residualVector We fill this with the residual vector.
     *
     * NOTE: this method is called indirectly by the PETSc iterative
     * solvers, so must be public.
     */
    void ComputeRHSFunction(const Vec currentGuess, Vec dUdt, TS ts);

    /**
     * Compute the Jacobian matrix given a current guess at the solution.
     * Choose whether to use a numerical or analytical method based on a flag
     * provided by the user (in Solve()).
     *
     * @param currentGuess The solution guess for the current iteration.
     * @param pJacobian Pointer to object to fill with the Jacobian matrix.
     *
     * NOTE: this method is called indirectly by the PETSc iterative
     * solvers, so must be public.
     */
    void ComputeJacobian(const Vec currentGuess, Mat* pJacobian, TS ts);

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
    virtual void Solve();
};

#endif /* SIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_ */
