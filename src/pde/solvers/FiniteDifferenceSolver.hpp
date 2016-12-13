/*

Copyright (c) 2005-2016, University of Oxford.
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
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "UnitCollection.hpp"

/**
 * Finite difference solver for concentration based reaction diffusion PDEs. It can include
 * discrete representations of cells and vessels.
 */
template<unsigned DIM>
class FiniteDifferenceSolver : public AbstractRegularGridDiscreteContinuumSolver<DIM>
{
    /**
     * Solver specific copy of boundary condition information, used for efficiency. It is a
     * vector of pairs of whether to apply-concentration values ordered by grid index.
     */
    boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > mpBoundaryConditions;

    /**
     * Do the boundary conditions need to be udpated each solve.
     */
    bool mUpdateBoundaryConditionsEachSolve;

    /**
     * Do the boundary conditions need to be udpated each solve.
     */
    bool mBoundaryConditionsSet;

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
     * Get the boundary conditions in the finite difference representation
     * @return pointer to the vector of boundary conditions, which is pairs of whether to apply-concentration values ordered by grid index.
     */
    boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > GetRGBoundaryConditions();

    /**
     * Overridden solve method
     */
    virtual void Solve();

    /**
     * Overridden setup method
     */
    void Setup();

    /**
     * Overridden update method
     */
    void Update();

    /**
     * Whether to update the boundary conditions on each solve
     * @param doUpdate update the boundary conditions on each solve
     */
    void UpdateBoundaryConditionsEachSolve(bool doUpdate);

private:

    /**
     * Do a linear PDE solve
     */
    void DoLinearSolve();

    /**
     * Do a parabolic PDE solve
     */
    void DoParabolicSolve();
};

#endif /* FINITEDIFFERENCESOLVER_HPP_ */
