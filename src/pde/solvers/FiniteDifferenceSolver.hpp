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

#ifndef FINITEDIFFERENCESOLVER_HPP_
#define FINITEDIFFERENCESOLVER_HPP_

#include "SmartPointers.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"
#include "UnitCollection.hpp"
#include "PetscTools.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"

/**
 * Finite difference solver for concentration based reaction diffusion PDEs. It can include
 * discrete representations of cells and vessels.
 */
template<unsigned DIM>
class FiniteDifferenceSolver : public AbstractFiniteDifferenceSolverBase<DIM>
{
    /**
     * Time increment for the parabolic solver
     */
    double mParabolicSolverTimeIncrement;

public:

    /**
     * Constructor
     */
    FiniteDifferenceSolver();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<FiniteDifferenceSolver<DIM> > Create();

    /**
     * Destructor
     */
    virtual ~FiniteDifferenceSolver();

    /**
     * Overridden solve method
     */
    virtual void Solve();

    /**
     * Overridden setup method
     */
    void Setup();

    /**
     * Dimensionless time increment for the parabolic solver
     * @param timeIncrement Dimensionless time increment for the parabolic solver
     */
    void SetParabolicSolverTimeIncrement(double timeIncrement);

private:

    /**
     * Do a linear PDE solve
     */
    void DoLinearSolve(boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_pde);

    /**
     * Do a nonlinear PDE solve
     */
    void DoNonLinearSolve(boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > p_pde);

    /**
     * Do a parabolic PDE solve
     */
    void DoParabolicSolve(boost::shared_ptr<AbstractDiscreteContinuumParabolicPde<DIM, DIM> > p_pde);
};

#endif /* FINITEDIFFERENCESOLVER_HPP_ */
