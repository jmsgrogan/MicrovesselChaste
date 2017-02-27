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

#include <petscts.h>
#include <petscdmda.h>
#include "LinearSystem.hpp"
#include "ReplicatableVector.hpp"
#include "VesselSegment.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "SimplePetscNonlinearSolver.hpp"
#include "BaseUnits.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"

template<unsigned DIM>
FiniteDifferenceSolver<DIM>::FiniteDifferenceSolver()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>(),
        mpSolver()

{

}

template<unsigned DIM>
boost::shared_ptr<FiniteDifferenceSolver<DIM> > FiniteDifferenceSolver<DIM>::Create()
{
    MAKE_PTR(FiniteDifferenceSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
FiniteDifferenceSolver<DIM>::~FiniteDifferenceSolver()
{

}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::SetSolver(boost::shared_ptr<AbstractFiniteDifferenceSolverBase<DIM> > pSolver)
{
    mpSolver = pSolver;
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::Setup()
{
    if(mpSolver)
    {
        mpSolver->Setup();
    }
    else
    {
        // Try to create a new solver based on the PDE type
        if(boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
                boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde))
        {
            mpSolver = boost::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<DIM> >
            (new SimpleLinearEllipticFiniteDifferenceSolver<DIM> );
            mpSolver->Setup();
        }
        else if(boost::shared_ptr<AbstractDiscreteContinuumParabolicPde<DIM, DIM> > p_parabolic_pde =
                boost::dynamic_pointer_cast<AbstractDiscreteContinuumParabolicPde<DIM, DIM> >(this->mpPde))
        {
            mpSolver = boost::shared_ptr<SimpleParabolicFiniteDifferenceSolver<DIM> >
            (new SimpleParabolicFiniteDifferenceSolver<DIM> );
            mpSolver->Setup();
        }
        else if(boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > p_nonlinear_pde =
                boost::dynamic_pointer_cast<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> >(this->mpPde))
        {
            mpSolver = boost::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver<DIM> >
            (new SimpleNonLinearEllipticFiniteDifferenceSolver<DIM> );
            mpSolver->Setup();
        }
        else
        {
            EXCEPTION("PDE Type could not be identified, did you set a PDE?");
        }
    }
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        this->Setup();
    }
    mpSolver->Solve();
}

// Explicit instantiation
template class FiniteDifferenceSolver<2>;
template class FiniteDifferenceSolver<3>;
