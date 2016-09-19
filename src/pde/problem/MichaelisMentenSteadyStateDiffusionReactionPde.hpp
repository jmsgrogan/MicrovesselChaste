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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#ifndef MICHAELISMENTENSTEADYSTATEDIFFUSIONREACTIONPDE_HPP_
#define MICHAELISMENTENSTEADYSTATEDIFFUSIONREACTIONPDE_HPP_

#include <string>
#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"

/**
 * A steady state concentration based reaction-diffusion pde with michaelis menten type consumption terms.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class MichaelisMentenSteadyStateDiffusionReactionPde : public AbstractDiscreteContinuumNonLinearEllipticPde<SPACE_DIM>
{
    /**
     * The concentration value a half maximum reaction rate in the michaelis menten model.
     */
    units::quantity<unit::concentration> mMichaelisMentenThreshold;

public:

    /**
     * Constructor
     */
    MichaelisMentenSteadyStateDiffusionReactionPde();

    /**
     * Destructor
     */
    virtual ~MichaelisMentenSteadyStateDiffusionReactionPde();

    /**
     * Factory Constructor
     * @return a pointer to an instance of the pde
     */
    static boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * Over-ridden method to return the constant in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeConstantInUSourceTerm(const ChastePoint<SPACE_DIM>& rX, Element<ELEMENT_DIM, SPACE_DIM>* pElement);

    /**
     * Over-ridden method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    units::quantity<unit::concentration_flow_rate> ComputeConstantInUSourceTerm(unsigned gridIndex=0);

    /**
     * Over-ridden method to return the diffusion term to the Chaste FE solver
     * @return the diffusion matrix
     */
    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>&, double u);

    /**
     * Return the derivative of the diffusion term
     * @return the derivative of the diffusion term
     */
    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTermPrime(const ChastePoint<SPACE_DIM>& rX, double u);

    /**
     * Over-ridden method to return the linear in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeLinearSourceTerm(const ChastePoint<SPACE_DIM>& rX);

    /**
     * Over-ridden method to return the linear in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeLinearInUCoeffInSourceTerm(const ChastePoint<SPACE_DIM>& rX, Element<ELEMENT_DIM, SPACE_DIM>* pElement);

    /**
     * Over-ridden method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

    /**
     * Over-ridden method to return the nonlinear in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeNonlinearSourceTerm(const ChastePoint<SPACE_DIM>& rX, double u);

    /**
     * Over-ridden method to return the nonlinear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    units::quantity<unit::concentration_flow_rate> ComputeNonlinearSourceTerm(unsigned gridIndex, units::quantity<unit::concentration> u);

    /**
     * Over-ridden method to return the nonlinear in U contribution prime to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeNonlinearSourceTermPrime(const ChastePoint<SPACE_DIM>& rX, double u);

    /**
     * Over-ridden method to return the nonlinear in U contribution prime to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    units::quantity<unit::rate> ComputeNonlinearSourceTermPrime(unsigned gridIndex, units::quantity<unit::concentration> u);

    /**
     * Get the concentration at half max consumption in the michaelis menten model
     * @return threshold the concentration at half max consumption in the michaelis menten model
     */
    units::quantity<unit::concentration> GetMichaelisMentenThreshold();

    /**
     * Set the concentration at half max consumption in the michaelis menten model
     * @param threshold the concentration at half max consumption in the michaelis menten model
     */
    void SetMichaelisMentenThreshold(units::quantity<unit::concentration> threshold);
};

#endif /*MICHAELISMENTENSTEADYSTATEDIFFUSIONREACTIONPDE_HPP_*/
