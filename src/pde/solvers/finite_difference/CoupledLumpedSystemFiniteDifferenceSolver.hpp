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

#ifndef COUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_
#define COUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_

#include "SmartPointers.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"
#include "UnitCollection.hpp"
#include "PetscTools.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"

/**
 * Base class for finite difference solvers.
 */
template<unsigned DIM>
class CoupledLumpedSystemFiniteDifferenceSolver : public SimpleParabolicFiniteDifferenceSolver<DIM>
{

public:

    /**
     * Constructor
     */
    CoupledLumpedSystemFiniteDifferenceSolver();

    /**
     * Destructor
     */
    virtual ~CoupledLumpedSystemFiniteDifferenceSolver();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to a class instance.
     */
    static boost::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver<DIM> > Create();

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
    void ComputeRHSFunction(const Vec currentGuess, Vec dUdt);

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
    void ComputeJacobian(const Vec currentGuess, Mat* pJacobian);

    /**
     * Overridden solve method
     */
    virtual void Solve();
};

#endif /* COUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_ */
