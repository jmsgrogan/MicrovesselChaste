/*

Copyright (c) 2005-2015, University of Oxford.
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

#ifndef COUPLEDODEPDEPARABOLICTERMASSEMBLER_HPP
#define COUPLEDODEPDEPARABOLICTERMASSEMBLER_HPP

#include "AbstractFeSurfaceIntegralAssemblerWithMatrix.hpp"
#include "AbstractFeVolumeIntegralAssembler.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "PdeSimulationTime.hpp"
#include "AbstractLinearParabolicPde.hpp"

/**
 * Assemble the parabolic parts of the coupled ODE-Parabolic PDE system
 */
template<unsigned DIM>
class CoupledOdePdeParabolicTermAssembler
    : public AbstractFeVolumeIntegralAssembler<DIM,DIM,1 ,true ,true, NORMAL>
{

protected:

    AbstractLinearParabolicPde<DIM,DIM>* mpParabolicPde;

private:

    /* Even when a class isn't being written for a very general dimensions sometimes it is a good idea
     * to define the following, and then use `ELEMENT_DIM` etc in the below, as it can make the code a
     * bit easier to understand.
     */
    static const unsigned ELEMENT_DIM = DIM;
    static const unsigned SPACE_DIM = DIM;
    static const unsigned PROBLEM_DIM = 1;

    /* We are assembling a matrix, we means we need to provide a `ComputeMatrixTerm()` method, to return the
     * elemental contribution to the RHS matrix. Note that `ELEMENT_DIM+1` is the number of
     * nodes in the element (=number of basis functions).
     */
    c_matrix<double,PROBLEM_DIM*(ELEMENT_DIM+1),PROBLEM_DIM*(ELEMENT_DIM+1)> ComputeMatrixTerm(
                                                                                c_vector<double, ELEMENT_DIM+1> &rPhi,
                                                                                c_matrix<double, SPACE_DIM, ELEMENT_DIM+1> &rGradPhi,
                                                                                ChastePoint<SPACE_DIM> &rX,
                                                                                c_vector<double,PROBLEM_DIM> &rU,
                                                                                c_matrix<double, PROBLEM_DIM, SPACE_DIM> &rGradU /* not used */,
                                                                                Element<ELEMENT_DIM,SPACE_DIM>* pElement)
    {
        c_matrix<double, SPACE_DIM, SPACE_DIM> pde_diffusion_term = mpParabolicPde->ComputeDiffusionTerm(rX, pElement);

        return    prod( trans(rGradPhi), c_matrix<double, SPACE_DIM, ELEMENT_DIM+1>(prod(pde_diffusion_term, rGradPhi)) )
                + PdeSimulationTime::GetPdeTimeStepInverse() * mpParabolicPde->ComputeDuDtCoefficientFunction(rX) * outer_prod(rPhi, rPhi);

    }

    /**
     * @return the term to be added to the element stiffness vector - see AbstractFeVolumeIntegralAssembler
     *
     * @param rPhi The basis functions, rPhi(i) = phi_i, i=1..numBases
     * @param rGradPhi Basis gradients, rGradPhi(i,j) = d(phi_j)/d(X_i)
     * @param rX The point in space
     * @param rU The unknown as a vector, u(i) = u_i
     * @param rGradU The gradient of the unknown as a matrix, rGradU(i,j) = d(u_i)/d(X_j)
     * @param pElement Pointer to the element
     */
    virtual c_vector<double,1*(ELEMENT_DIM+1)> ComputeVectorTerm(
        c_vector<double, ELEMENT_DIM+1>& rPhi,
        c_matrix<double, SPACE_DIM, ELEMENT_DIM+1>& rGradPhi,
        ChastePoint<SPACE_DIM>& rX,
        c_vector<double,1>& rU,
        c_matrix<double,1,SPACE_DIM>& rGradU,
        Element<ELEMENT_DIM,SPACE_DIM>* pElement)

        {
            return (mpParabolicPde->ComputeSourceTerm(rX, rU(0), pElement)
                    + PdeSimulationTime::GetPdeTimeStepInverse() * mpParabolicPde->ComputeDuDtCoefficientFunction(rX) * rU(0)) * rPhi;
        }

public:
    CoupledOdePdeParabolicTermAssembler(AbstractTetrahedralMesh<DIM,DIM>* pMesh,
            AbstractLinearParabolicPde<DIM,DIM>* pPde)
        : AbstractFeVolumeIntegralAssembler<DIM,DIM,1,true,true,NORMAL>(pMesh),
          mpParabolicPde(pPde)
    {
    }
};

#endif /* COUPLEDODEPDEPARABOLICTERMASSEMBLER_HPP */
