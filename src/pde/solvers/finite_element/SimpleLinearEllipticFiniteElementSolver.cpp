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

#include "SimpleLinearEllipticSolver.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "Exception.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "BoundaryConditionsContainer.hpp"

template<unsigned DIM>
SimpleLinearEllipticFiniteElementSolver<DIM>::SimpleLinearEllipticFiniteElementSolver()
    : AbstractFiniteElementSolverBase<DIM>()
{

}

template<unsigned DIM>
SimpleLinearEllipticFiniteElementSolver<DIM>::~SimpleLinearEllipticFiniteElementSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<SimpleLinearEllipticFiniteElementSolver<DIM> > SimpleLinearEllipticFiniteElementSolver<DIM>::Create()
{
    MAKE_PTR(SimpleLinearEllipticFiniteElementSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteElementSolver<DIM>::Solve()
{
    AbstractFiniteElementSolverBase<DIM>::Solve();

    // Set up the boundary conditions in the Chaste format
    boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> > p_bcc =
            boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> >(new BoundaryConditionsContainer<DIM, DIM, 1> );

    for(unsigned idx=0; idx<this->mBoundaryConditions.size(); idx++)
    {
        this->mBoundaryConditions[idx]->SetGridCalculator(this->mpDensityMap->GetGridCalculator());
        this->mBoundaryConditions[idx]->UpdateBoundaryConditions(p_bcc);
    }

    // Check the type of pde
    if(boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
            boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde))
    {
        SimpleLinearEllipticSolver<DIM, DIM> static_solver(this->mpMesh.get(), p_linear_pde.get(), p_bcc.get());
        ReplicatableVector solution_repl(static_solver.Solve());

        this->mSolution = std::vector<double>(solution_repl.GetSize());
        this->mConcentrations = std::vector<units::quantity<unit::concentration> >(solution_repl.GetSize());
        for(unsigned idx = 0; idx < solution_repl.GetSize(); idx++)
        {
            this->mSolution[idx] = solution_repl[idx];
            this->mConcentrations[idx] = solution_repl[idx]*this->mReferenceConcentration;
        }
        this->UpdateSolution(this->mSolution);
    }
    else
    {
        EXCEPTION("PDE of wrong type for this solver.");
    }

    if(this->mWriteSolution)
    {
        this->Write();
    }
}

// Explicit instantiation
template class SimpleLinearEllipticFiniteElementSolver<2> ;
template class SimpleLinearEllipticFiniteElementSolver<3> ;
