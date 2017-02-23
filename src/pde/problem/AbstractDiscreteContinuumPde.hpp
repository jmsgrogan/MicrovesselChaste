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

#ifndef ABSTRACTDISCRETECONTINUUMPDE_HPP_
#define ABSTRACTDISCRETECONTINUUMPDE_HPP_

#include <string>
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "DiscreteSource.hpp"
#include "GeometryTools.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"

/**
 * This class specifies interfaces that PDEs being used in Discrete Continuum solvers should
 * implement, mostly related to interaction with discrete sinks and sources.
 * It can be combined with a range of abstract PDEs in Chaste or used to build
 * more general PDE descriptions.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class AbstractDiscreteContinuumPde
{

protected:

    /**
     * The diffusion constant for isotropic diffusion
     */
    units::quantity<unit::diffusivity> mDiffusivity;

    /**
     * The continuum constant in U term, discrete terms are added to this.
     */
    units::quantity<unit::concentration_flow_rate> mConstantInUTerm;

    /**
     * The continuum linear in U term, discrete terms are added to this.
     */
    units::quantity<unit::rate> mLinearInUTerm;

    /**
     * The collection of discrete sources for addition to the continuum terms
     */
    std::vector<boost::shared_ptr<DiscreteSource<SPACE_DIM> > > mDiscreteSources;

    /**
     * The grid for solvers using regular grids
     */
    boost::shared_ptr<RegularGrid<SPACE_DIM> > mpRegularGrid;

    /**
     * The mesh for solvers using finite element meshes
     */
    boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > mpMesh;

    /**
     * Whether to use a regular grid or mesh for discrete source calculations
     */
    bool mUseRegularGrid;

    /**
     * The constant source strengths for each point on the grid or element in the mesh
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > mDiscreteConstantSourceStrengths;

    /**
     * The linear source strengths for each point on the grid or mesh
     */
    std::vector<units::quantity<unit::rate> > mDiscreteLinearSourceStrengths;

    /**
     * This is used internally to scale concentrations before and after linear system solves, reads and writes.
     * Since those functions don't use Boost Units. It should not affect the solution, but can be judiciously chosen
     * to avoid precision problems.
     */
    units::quantity<unit::concentration> mReferenceConcentration;

public:

    /**
     * Constructor
     */
    AbstractDiscreteContinuumPde();

    /**
     * Desctructor
     */
    virtual ~AbstractDiscreteContinuumPde();

    /**
     * Add a discrete source to the pde
     * @param pDiscreteSource a pointer the discrete source
     */
    void AddDiscreteSource(boost::shared_ptr<DiscreteSource<SPACE_DIM> > pDiscreteSource);

    /**
     * Virtual method to return the constant in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::concentration_flow_rate> ComputeConstantInUSourceTerm(unsigned gridIndex=0) = 0;

    /**
     * Return the diffusion constant for isotropic diffusion
     * @return the diffusion constant
     */
    units::quantity<unit::diffusivity> ComputeIsotropicDiffusionTerm();

    /**
     * Virtual method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0) = 0;

    /**
     * Return the collection of discrete sources
     * @return vector of pointers to the discrete sources
     */
    std::vector<boost::shared_ptr<DiscreteSource<SPACE_DIM> > > GetDiscreteSources();

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
     * Set the isotropic diffusion constant
     * @param diffusivity the isotropic diffusion constant
     */
    void SetIsotropicDiffusionConstant(units::quantity<unit::diffusivity> diffusivity);

    /**
     * Set the regular grid
     * @param pRegularGrid the regular grid
     */
    void SetRegularGrid(boost::shared_ptr<RegularGrid<SPACE_DIM> > pRegularGrid);

    /**
     * Set the finite element mesh
     * @param pMesh the finite element mesh
     */
    void SetMesh(boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > pMesh);

    /**
     * Set the reference concentration
     * @param referenceConcentration the reference concentration
     */
    void SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration);

    /**
     * Set whether to use a regular grid
     * @param useRegularGrid whether to use a regular grid
     */
    void SetUseRegularGrid(bool useRegularGrid);

    /**
     * Update the discrete source strengths
     */
    virtual void UpdateDiscreteSourceStrengths();

};

#endif /*ABSTRACTDISCRETECONTINUUMPDE_HPP_*/
