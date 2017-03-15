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

#ifndef ABSTRACTDISCRETECONTINUUMLINEARELLIPTICPDE_HPP_
#define ABSTRACTDISCRETECONTINUUMLINEARELLIPTICPDE_HPP_

#include <string>
#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "AbstractLinearEllipticPde.hpp"
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
class AbstractDiscreteContinuumLinearEllipticPde : public AbstractLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>,
    public AbstractDiscreteContinuumPde<ELEMENT_DIM, SPACE_DIM>
{

public:

    using AbstractLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeConstantInUSourceTerm;
    using AbstractLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearInUCoeffInSourceTerm;

protected:

    /**
     * The continuum constant in U term, discrete terms are added to this.
     */
    units::quantity<unit::concentration_flow_rate> mConstantInUTerm;

    /**
     * The continuum linear in U term, discrete terms are added to this.
     */
    units::quantity<unit::rate> mLinearInUTerm;

    /**
     * The constant in U term source strengths for each point on the grid or element in the mesh
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > mDiscreteConstantSourceStrengths;

    /**
     * The linear in U term source strengths for each point on the grid or mesh
     */
    std::vector<units::quantity<unit::rate> > mDiscreteLinearSourceStrengths;

public:

    /**
     * Constructor
     */
    AbstractDiscreteContinuumLinearEllipticPde();

    /**
     * Destructor
     */
    virtual ~AbstractDiscreteContinuumLinearEllipticPde();

    /**
     * Overwritten method to return the diffusion term to the Chaste FE solver
     * @return the diffusion matrix
     */
    c_matrix<double, SPACE_DIM, SPACE_DIM> ComputeDiffusionTerm(const ChastePoint<SPACE_DIM>&);

    /**
     * Virtual method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::concentration_flow_rate> ComputeConstantInUSourceTerm(unsigned gridIndex=0) = 0;

    /**
     * Virtual method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::concentration_flow_rate> ComputeDiscreteConstantInUSourceTerm(unsigned gridIndex=0);

    /**
     * Overwritten method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0) = 0;

    /**
     * Overwritten method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

    /**
     * Set the continuum constant in U term
     * @param constantInUTerm the continuum constant in U term
     */
    void SetContinuumConstantInUTerm(units::quantity<unit::concentration_flow_rate> constantInUTerm);

    /**
     * Set the linear constant in U term
     * @param linearInUTerm the linear constant in U term
     */
    void SetContinuumLinearInUTerm(units::quantity<unit::rate> linearInUTerm);

    /**
     * Update the discrete source strengths
     */
    virtual void UpdateDiscreteSourceStrengths();

};

#endif /*ABSTRACTDISCRETECONTINUUMLINEARELLIPTICPDE_HPP_*/
