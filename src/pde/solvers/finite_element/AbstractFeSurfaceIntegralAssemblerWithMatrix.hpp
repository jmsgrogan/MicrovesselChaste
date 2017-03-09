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

#ifndef ABSTRACTFESURFACENTEGRALASSEMBLERWITHMATRIX_HPP_
#define ABSTRACTFESURFACENTEGRALASSEMBLERWITHMATRIX_HPP_

#include "AbstractFeAssemblerCommon.hpp"
#include "GaussianQuadratureRule.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "PetscVecTools.hpp"
#include "PetscMatTools.hpp"


/**
 *  Similar to AbstractFeVolumeIntegralAssembler but is used for constructing finite element objects
 *  that are based on SURFACE INTEGRALS, as opposed to volume integrals.
 *
 *  The interface is the same the volume assemblers.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM, unsigned PROBLEM_DIM>
class AbstractFeSurfaceIntegralAssemblerWithMatrix : public AbstractFeAssemblerCommon<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM,true,true,NORMAL>
{
protected:
    /** Mesh to be solved on. */
    AbstractTetrahedralMesh<ELEMENT_DIM, SPACE_DIM>* mpMesh;

    /** Boundary conditions container */
    BoundaryConditionsContainer<ELEMENT_DIM, SPACE_DIM, PROBLEM_DIM>* mpBoundaryConditions;

    /** Quadrature rule for use on boundary elements. */
    GaussianQuadratureRule<ELEMENT_DIM-1>* mpSurfaceQuadRule;

    /** Basis function for use with boundary elements. */
    typedef LinearBasisFunction<ELEMENT_DIM-1> SurfaceBasisFunction;

    /**
     * @return the matrix to be added to element stiffness matrix for surface integrals.
     * This is used for implementing Robin boundary conditions.
     */
    virtual c_matrix<double,PROBLEM_DIM*(ELEMENT_DIM),PROBLEM_DIM*(ELEMENT_DIM)> ComputeMatrixSurfaceTerm(
            const BoundaryElement<ELEMENT_DIM-1,SPACE_DIM>& rSurfaceElement,
            c_vector<double, ELEMENT_DIM>& rPhi,
            ChastePoint<SPACE_DIM>& rX)
    {
        // If this line is reached this means this method probably hasn't been over-ridden correctly in
        // the concrete class
        NEVER_REACHED;
        return zero_matrix<double>(ELEMENT_DIM*PROBLEM_DIM, ELEMENT_DIM*PROBLEM_DIM);
    }

    /**
     * @return the vector to be added to full vector
     * for a given Gauss point in BoundaryElement, ie, essentially the
     * INTEGRAND in the boundary integral part of the definition of the vector.
     * The arguments are the bases, x and current solution computed at the
     * Gauss point.
     *
     *  ** This method needs to be overloaded in the concrete class **
     *
     * @param rSurfaceElement the element which is being considered.
     * @param rPhi The basis functions, rPhi(i) = phi_i, i=1..numBases
     * @param rX The point in space
     */
    virtual c_vector<double, PROBLEM_DIM*ELEMENT_DIM> ComputeVectorSurfaceTerm(
        const BoundaryElement<ELEMENT_DIM-1,SPACE_DIM>& rSurfaceElement,
        c_vector<double, ELEMENT_DIM>& rPhi,
        ChastePoint<SPACE_DIM>& rX)
    {
        // If this line is reached this means this method probably hasn't been over-ridden correctly in
        // the concrete class
        NEVER_REACHED;
        return zero_vector<double>(ELEMENT_DIM*PROBLEM_DIM);
    }

    /**
     * Calculate the contribution of a single surface element with Neumann
     * boundary condition to the linear system.
     *
     * @param rSurfaceElement The element to assemble on.
     * @param rBSurfElem The element's contribution to the RHS vector is returned in this
     *     vector of length n, the no. of nodes in this element. There is no
     *     need to zero this vector before calling.
     */
    virtual void AssembleOnSurfaceElement(const BoundaryElement<ELEMENT_DIM-1,SPACE_DIM>& rSurfaceElement,
                                          c_matrix<double, PROBLEM_DIM*(ELEMENT_DIM), PROBLEM_DIM*(ELEMENT_DIM) >& rASurfElem,
                                          c_vector<double, PROBLEM_DIM*ELEMENT_DIM>& rBSurfElem);


    /**
     * Main assemble method. Users should call Assemble() however
     */
    virtual void DoAssemble();


public:
    /**
     * Constructor
     *
     * @param pMesh The mesh
     * @param pBoundaryConditions The boundary conditions container
     */
    AbstractFeSurfaceIntegralAssemblerWithMatrix(AbstractTetrahedralMesh<ELEMENT_DIM,SPACE_DIM>* pMesh,
                                       BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM>* pBoundaryConditions);

    /**
     * Destructor
     */
    virtual ~AbstractFeSurfaceIntegralAssemblerWithMatrix();

    /**
     * Reset the internal boundary conditions container pointer
     * @param pBoundaryConditions
     */
    void ResetBoundaryConditionsContainer(BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM>* pBoundaryConditions)
    {
        assert(pBoundaryConditions);
        this->mpBoundaryConditions = pBoundaryConditions;
    }
};


template <unsigned ELEMENT_DIM, unsigned SPACE_DIM, unsigned PROBLEM_DIM>
AbstractFeSurfaceIntegralAssemblerWithMatrix<ELEMENT_DIM, SPACE_DIM, PROBLEM_DIM>::AbstractFeSurfaceIntegralAssemblerWithMatrix(
            AbstractTetrahedralMesh<ELEMENT_DIM,SPACE_DIM>* pMesh,
            BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM>* pBoundaryConditions)
    : AbstractFeAssemblerCommon<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM,true,true,NORMAL>(),
      mpMesh(pMesh),
      mpBoundaryConditions(pBoundaryConditions)
{
    assert(pMesh);
    assert(pBoundaryConditions);
    // Default to 2nd order quadrature.  Our default basis functions are piecewise linear
    // which means that we are integrating functions which in the worst case (mass matrix)
    // are quadratic.
    mpSurfaceQuadRule = new GaussianQuadratureRule<ELEMENT_DIM-1>(2);
}

template <unsigned ELEMENT_DIM, unsigned SPACE_DIM, unsigned PROBLEM_DIM>
AbstractFeSurfaceIntegralAssemblerWithMatrix<ELEMENT_DIM, SPACE_DIM, PROBLEM_DIM>::~AbstractFeSurfaceIntegralAssemblerWithMatrix()
{
    delete mpSurfaceQuadRule;
}


template <unsigned ELEMENT_DIM, unsigned SPACE_DIM, unsigned PROBLEM_DIM>
void AbstractFeSurfaceIntegralAssemblerWithMatrix<ELEMENT_DIM, SPACE_DIM, PROBLEM_DIM>::DoAssemble()
{
    assert(this->mAssembleMatrix || this->mAssembleVector);

    // Zero the matrix/vector if it is to be assembled
    if (this->mAssembleVector && this->mZeroVectorBeforeAssembly)
    {
        PetscVecTools::Zero(this->mVectorToAssemble);
    }
    if (this->mAssembleMatrix && this->mZeroMatrixBeforeAssembly)
    {
        PetscMatTools::Zero(this->mMatrixToAssemble);
    }

    const size_t STENCIL_SIZE=PROBLEM_DIM*(ELEMENT_DIM);
    c_matrix<double, STENCIL_SIZE, STENCIL_SIZE> a_surf_elem;
    c_vector<double, STENCIL_SIZE> b_surf_elem;

    // Loop over surface elements
    typename BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,PROBLEM_DIM>::NeumannMapIterator
        neumann_iterator = mpBoundaryConditions->BeginNeumann();

    // Iterate over defined conditions
    while (neumann_iterator != mpBoundaryConditions->EndNeumann())
    {
        const BoundaryElement<ELEMENT_DIM-1,SPACE_DIM>& r_surf_element = *(neumann_iterator->first);
        AssembleOnSurfaceElement(r_surf_element, a_surf_elem, b_surf_elem);

        unsigned p_indices[STENCIL_SIZE];
        r_surf_element.GetStiffnessMatrixGlobalIndices(PROBLEM_DIM, p_indices);

        if (this->mAssembleMatrix)
        {
            PetscMatTools::AddMultipleValues<STENCIL_SIZE>(this->mMatrixToAssemble, p_indices, a_surf_elem);
        }

        if (this->mAssembleVector)
        {
            PetscVecTools::AddMultipleValues<STENCIL_SIZE>(this->mVectorToAssemble, p_indices, b_surf_elem);
        }

        //PetscVecTools::AddMultipleValues<STENCIL_SIZE>(this->mVectorToAssemble, p_indices, b_surf_elem);
        ++neumann_iterator;
    }
}


template <unsigned ELEMENT_DIM, unsigned SPACE_DIM, unsigned PROBLEM_DIM>
void AbstractFeSurfaceIntegralAssemblerWithMatrix<ELEMENT_DIM, SPACE_DIM, PROBLEM_DIM>::AssembleOnSurfaceElement(
            const BoundaryElement<ELEMENT_DIM-1,SPACE_DIM>& rSurfaceElement,
            c_matrix<double, PROBLEM_DIM*(ELEMENT_DIM), PROBLEM_DIM*(ELEMENT_DIM) >& rASurfElem,
            c_vector<double, PROBLEM_DIM*ELEMENT_DIM>& rBSurfElem)
{
    c_vector<double, SPACE_DIM> weighted_direction;
    double jacobian_determinant;
    mpMesh->GetWeightedDirectionForBoundaryElement(rSurfaceElement.GetIndex(), weighted_direction, jacobian_determinant);

    if (this->mAssembleMatrix)
    {
        rASurfElem.clear();
    }

    if (this->mAssembleVector)
    {
        rBSurfElem.clear();
    }

    // Allocate memory for the basis function values
    c_vector<double, ELEMENT_DIM>  phi;

    // Loop over Gauss points
    for (unsigned quad_index=0; quad_index<mpSurfaceQuadRule->GetNumQuadPoints(); quad_index++)
    {
        const ChastePoint<ELEMENT_DIM-1>& quad_point = mpSurfaceQuadRule->rGetQuadPoint(quad_index);

        SurfaceBasisFunction::ComputeBasisFunctions(quad_point, phi);

        // The location of the Gauss point in the original element will be stored in x
        ChastePoint<SPACE_DIM> x(0,0,0);

        this->ResetInterpolatedQuantities();
        for (unsigned i=0; i<rSurfaceElement.GetNumNodes(); i++)
        {
            const c_vector<double, SPACE_DIM> node_loc = rSurfaceElement.GetNode(i)->rGetLocation();
            x.rGetLocation() += phi(i)*node_loc;

            // Allow the concrete version of the assembler to interpolate any desired quantities
            this->IncrementInterpolatedQuantities(phi(i), rSurfaceElement.GetNode(i));
        }

        double wJ = jacobian_determinant * mpSurfaceQuadRule->GetWeight(quad_index);

        // Create rAElem and rBElem
        if (this->mAssembleMatrix)
        {
            noalias(rASurfElem) += ComputeMatrixSurfaceTerm(rSurfaceElement, phi, x) * wJ;
        }

        if (this->mAssembleVector)
        {
            noalias(rBSurfElem) += ComputeVectorSurfaceTerm(rSurfaceElement, phi, x) * wJ;
        }
    }
}

#endif // ABSTRACTFESURFACENTEGRALASSEMBLERWITHMATRIX_HPP_
