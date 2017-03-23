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



#ifndef AbstractFiniteElementSolverBase_HPP_
#define AbstractFiniteElementSolverBase_HPP_

#include "SmartPointers.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"
#include "DiscreteContinuumMesh.hpp"

/**
 * A finite element solver base class for use with multiple discrete sinks or sources.
 */
template<unsigned DIM>
class AbstractFiniteElementSolverBase : public AbstractUnstructuredGridDiscreteContinuumSolver<DIM>
{

protected:

    /**
     * Over-ride the base class solve method
     */
    using AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Solve;

    /**
     * Use the chaste newton solver, if appropriate
     */
    bool mUseNewton;

    /**
     * An initial guess, if needed
     */
    std::vector<double> mGuess;

public:

    /**
     * Constructor
     */
    AbstractFiniteElementSolverBase();

    /**
     * Destructor
     */
    virtual ~AbstractFiniteElementSolverBase();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to a class instance.
     */
    static boost::shared_ptr<AbstractFiniteElementSolverBase<DIM> > Create();

    /**
     * Overridden solve method
     */
    void Solve();

    /**
     * Set the initial dimensionless guess
     * @param guess the guess.
     */
    void SetGuess(const std::vector<double>& guess);

    /**
     * Use Chaste's simple newton solve
     * @param useNewton use Chaste's simple newton solve
     */
    void SetUseSimpleNetonSolver(bool useNewton);

    /**
     * Overridden setup method
     */
    void Setup();

    /**
     * Overridden update method
     */
    void Update();
};

#endif /* AbstractFiniteElementSolverBase_HPP_ */
