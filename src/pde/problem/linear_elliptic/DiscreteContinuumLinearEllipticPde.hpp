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

#ifndef DISCRETECONTINUUMLINEARELLIPTICPDE_HPP_
#define DISCRETECONTINUUMLINEARELLIPTICPDE_HPP_

#include <string>
#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "DiscreteSource.hpp"
#include "GeometryTools.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumPde.hpp"

/**
 * Base PDE class for managing discrete entities on the computational grid. Child classes need to
 * implement methods for calculating the 'LinearInU' source terms, which will have units related to
 * the field quantity being solved for.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class DiscreteContinuumLinearEllipticPde : public AbstractDiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>
{

public:

    using AbstractLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeConstantInUSourceTerm;
    using AbstractLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearInUCoeffInSourceTerm;

    /**
     * Constructor
     */
    DiscreteContinuumLinearEllipticPde();

    /**
     * Destructor
     */
    virtual ~DiscreteContinuumLinearEllipticPde();

    /**
     * Factory Constructor
     * @return a pointer to an instance of the pde
     */
    static std::shared_ptr<DiscreteContinuumLinearEllipticPde<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * Overwritten method to return the constant in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeConstantInUSourceTerm(const ChastePoint<SPACE_DIM>& rX, Element<ELEMENT_DIM, SPACE_DIM>* pElement);

    /**
     * Overwritten method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    QConcentrationFlowRate ComputeConstantInUSourceTerm(unsigned gridIndex=0);

    /**
     * Overwritten method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    QConcentrationFlowRate ComputeDiscreteConstantInUSourceTerm(unsigned gridIndex=0);

    /**
     * Overwritten method to return the linear in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeLinearInUCoeffInSourceTerm(const ChastePoint<SPACE_DIM>& rX, Element<ELEMENT_DIM, SPACE_DIM>* pElement);

    /**
     * Overwritten method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    QRate ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

    /**
     * Overwritten method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    QRate ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

};

#endif /*DISCRETECONTINUUMLINEARELLIPTICPDE_HPP_*/
