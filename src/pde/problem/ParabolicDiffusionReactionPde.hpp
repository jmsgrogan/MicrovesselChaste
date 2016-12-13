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

#ifndef PARABOLICDIFFUSIONREACTIONPDE_HPP_
#define PARABOLICDIFFUSIONREACTIONPDE_HPP_

#include "ChastePoint.hpp"
#include "Element.hpp"
#include "SmartPointers.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"

/**
 * Linear reaction diffusion PDE
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class ParabolicDiffusionReactionPde : public AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, SPACE_DIM>
{
    /**
     * The continuum linear in U term, discrete terms are added to this.
     */
    units::quantity<unit::rate> mLinearInUTerm;

    /**
     * The non-linear source strengths for each point on the grid or mesh
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > mDiscreteNonLinearSourceStrengths;

    /**
     * The linear source strengths for each point on the grid or mesh
     */
    std::vector<units::quantity<unit::rate> > mDiscreteLinearSourceStrengths;

public:

    /**
     * Constructor
     */
    ParabolicDiffusionReactionPde();

    /**
     * Destructor
     */
    virtual ~ParabolicDiffusionReactionPde();

    /**
     * Factory Constructor
     * @return a pointer to an instance of the pde
     */
    static boost::shared_ptr<ParabolicDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > Create();

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
    units::quantity<unit::rate> ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

    /**
     * Abstract method to return the non linear contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual units::quantity<unit::concentration_flow_rate> ComputeNonlinearSourceTerm(unsigned gridIndex, units::quantity<unit::concentration> u);

    /**
     * Abstract method to return the non linear prime contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeNonlinearSourceTermPrime(unsigned gridIndex, units::quantity<unit::concentration> u);

    /**
     * Set the linear constant in U term
     * @param linearInUTerm the linear constant in U term
     */
    void SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm);

    /**
     * Update the discrete source strengths
     */
    void UpdateDiscreteSourceStrengths();

};

#endif /*ParabolicDiffusionReactionPde_HPP_*/
