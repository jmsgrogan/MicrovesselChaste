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

#include "SimpleLinearParabolicSolver.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "SimpleParabolicFiniteElementSolver.hpp"
#include "Exception.hpp"

template<unsigned DIM>
SimpleParabolicFiniteElementSolver<DIM>::SimpleParabolicFiniteElementSolver()
    : AbstractFiniteElementSolverBase<DIM>()
{

}

template<unsigned DIM>
SimpleParabolicFiniteElementSolver<DIM>::~SimpleParabolicFiniteElementSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<SimpleParabolicFiniteElementSolver<DIM> > SimpleParabolicFiniteElementSolver<DIM>::Create()
{
    MAKE_PTR(SimpleParabolicFiniteElementSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void SimpleParabolicFiniteElementSolver<DIM>::Solve()
{
    AbstractFiniteElementSolverBase<DIM>::Solve();

    // Check the type of pde
    if(boost::shared_ptr<AbstractDiscreteContinuumParabolicPde<DIM, DIM> > p_parabolic_pde =
            boost::dynamic_pointer_cast<AbstractDiscreteContinuumParabolicPde<DIM, DIM> >(this->mpPde))
    {

    }
    else
    {
        EXCEPTION("PDE Type could not be identified, did you set a PDE?");
    }

    if(this->mWriteSolution)
    {
        this->Write();
    }
}

// Explicit instantiation
template class SimpleParabolicFiniteElementSolver<2>;
template class SimpleParabolicFiniteElementSolver<3>;
