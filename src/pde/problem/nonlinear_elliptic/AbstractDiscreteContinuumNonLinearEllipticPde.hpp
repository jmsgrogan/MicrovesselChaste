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

#ifndef ABSTRACTDISCRETECONTINUUMNONLINEARELLIPTICPDE_HPP_
#define ABSTRACTDISCRETECONTINUUMNONLINEARELLIPTICPDE_HPP_

#include <string>
#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "DiscreteSource.hpp"
#include "GeometryTools.hpp"
#include "RegularGrid.hpp"
#include "TetrahedralMesh.hpp"
#include "AbstractNonlinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumPde.hpp"

/**
 * Non-Linear Elliptic PDE with both continuum and discrete source terms. There is repition with
 * DiscreteContinuumLinearEllipticPde to avoid multiple inheritance.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class AbstractDiscreteContinuumNonLinearEllipticPde : public AbstractNonlinearEllipticPde<SPACE_DIM>,
    public AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>
{
protected:

    /**
     * The discrete source strengths
     */
    std::vector<QConcentrationFlowRate > mDiscreteSourceStrengths;

public:

    using AbstractNonlinearEllipticPde<SPACE_DIM>::ComputeNonlinearSourceTermPrime;
    using AbstractNonlinearEllipticPde<SPACE_DIM>::ComputeNonlinearSourceTerm;
    using AbstractNonlinearEllipticPde<SPACE_DIM>::ComputeLinearSourceTerm;

    /**
     * Constructor
     */
    AbstractDiscreteContinuumNonLinearEllipticPde();

    /**
     * Desctructor
     */
    virtual ~AbstractDiscreteContinuumNonLinearEllipticPde();

    /**
     * Over-ridden method to return the diffusion term to the Chaste FE solver
     * @param rX the grid location
     * @param u the solution value
     * @return the diffusion matrix
     */
    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>& rX, double u);

    /**
     * Return the derivative of the diffusion term
     * @param rX the grid location
     * @param u the solution value
     * @return the derivative of the diffusion term
     */
    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTermPrime(const ChastePoint<SPACE_DIM>& rX, double u);

    /**
     * Abstract method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual QConcentrationFlowRate ComputeLinearSourceTerm(unsigned gridIndex=0);

    /**
     * Abstract method to return the non linear contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual QConcentrationFlowRate ComputeNonlinearSourceTerm(unsigned gridIndex, QConcentration u)=0;

    /**
     * Abstract method to return the non linear prime contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual QRate ComputeNonlinearSourceTermPrime(unsigned gridIndex, QConcentration u)=0;

    /**
     * Update the discrete source strengths
     */
    virtual void UpdateDiscreteSourceStrengths();
};

#endif /*ABSTRACTDISCRETECONTINUUMNONLINEARELLIPTICPDE_HPP_*/
