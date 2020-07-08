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

#ifndef _PriesWithMemoryHaematocritSolver_hpp
#define _PriesWithMemoryHaematocritSolver_hpp

#include "SmartPointers.hpp"
#include "LinearSystem.hpp"
#include "AbstractHaematocritSolver.hpp"
#include "UnitCollection.hpp"

/**
 * This solver calculates the distribution of haematocrit in branching vessel networks according to
 * our new model extending the model from Pries1989 by adding memory (CFL disruption and recovery) effects
 */
template<unsigned DIM>
class PriesWithMemoryHaematocritSolver : public AbstractHaematocritSolver<DIM>
{

private:


    /**
     * The arterial haematocrit level
     */
    QDimensionless mHaematocrit;

    /**
     * Throw an exception if convergence fails
     */
    bool mExceptionOnFailedConverge;

public:

    /**
     * Constructor.
     */
    PriesWithMemoryHaematocritSolver();

    /**
     *  destructor.
     */
    ~PriesWithMemoryHaematocritSolver();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new class instance
     */
    static std::shared_ptr<PriesWithMemoryHaematocritSolver<DIM> > Create();

    /**
     *  Do the solve
     */
    void Calculate();

    void SetExceptionOnFailedConverge(bool setException);


    /**
     * Set the artial haematocrit
     * @param haematocrit the arterial haematocrit
     */
    void SetHaematocrit(QDimensionless haematocrit);

    /**
     * Temporary for debugging
     * @return class name of this object
     */
    std::string Reflect()
    {
        return "PriesWithMemoryHaematocritSolver";
    }

    /**
     * Writes one entry in the matrix of the system assuming that the diagonal
     * entry is 1.  That is calculate the fraction numer/denom such that (-numer/denom)*H_parent + H_me = 0,
     * so numer*H_parent = denom*H_me.
     * @param me - the vessel we are calculating on
     * @param comp - competitor/sibling vessel
     * @param parent - parent/feeding vessel
     * @param rLinearSystem - the linear system
     */
    void UpdateBifurcation(std::shared_ptr<Vessel<DIM> > me, std::shared_ptr<Vessel<DIM> > comp,
                           std::shared_ptr<Vessel<DIM> > parent, LinearSystem& rLinearSystem);
};

#endif
