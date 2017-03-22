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

#ifndef NULLSURFACEINTEGRALCALCULATOR_HPP
#define NULLSURFACEINTEGRALCALCULATOR_HPP

#include "AbstractFeSurfaceIntegralAssemblerWithMatrix.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "PdeSimulationTime.hpp"

/**
 * Helper class for calculating the surface area on the mesh.
 * Useful for 'surface area' averaging
 */
template<unsigned DIM>
class NullSurfaceIntegralCalculator
    : public AbstractFeSurfaceIntegralAssemblerWithMatrix<DIM,DIM,1>
{

public:

    double CalculateSurfaceIntegral(bool useSolution=true)
    {
        double integral_total = 0.0;
        typename BoundaryConditionsContainer<DIM,DIM,1>::NeumannMapIterator
            neumann_iterator = this->mpBoundaryConditions->BeginNeumann();

        // Iterate over defined conditions
        while (neumann_iterator != this->mpBoundaryConditions->EndNeumann())
        {
            const BoundaryElement<DIM-1,DIM>& r_surf_element = *(neumann_iterator->first);

            c_vector<double, DIM> weighted_direction;
            double jacobian_determinant;
            this->mpMesh->GetWeightedDirectionForBoundaryElement(r_surf_element.GetIndex(), weighted_direction,
                    jacobian_determinant);

            // Allocate memory for the basis function values
            c_vector<double, DIM> phi;

            // Loop over Gauss points
            double element_integral = 0.0;
            for (unsigned quad_index=0; quad_index<this->mpSurfaceQuadRule->GetNumQuadPoints(); quad_index++)
            {
                const ChastePoint<DIM-1>& quad_point = this->mpSurfaceQuadRule->rGetQuadPoint(quad_index);

                LinearBasisFunction<DIM-1>::ComputeBasisFunctions(quad_point, phi);
                double node_contribution = 0.0;
                for (unsigned i=0; i<r_surf_element.GetNumNodes(); i++)
                {
                    double u_at_node;
                    if(useSolution)
                    {
                        u_at_node=this->GetCurrentSolutionOrGuessValue(r_surf_element.GetNode(i)->GetIndex(), 0);
                    }
                    else
                    {
                        u_at_node=1.0;
                    }
                    node_contribution += phi(i) * u_at_node;
                }
                double wJ = jacobian_determinant * this->mpSurfaceQuadRule->GetWeight(quad_index);
                element_integral += wJ*node_contribution;
            }
            integral_total += element_integral;
            ++neumann_iterator;
        }
        return integral_total;
    }

public:

    NullSurfaceIntegralCalculator(AbstractTetrahedralMesh<DIM,DIM>* pMesh,
            BoundaryConditionsContainer<DIM,DIM,1>* pBoundaryConditions)
        : AbstractFeSurfaceIntegralAssemblerWithMatrix<DIM,DIM,1>(pMesh, pBoundaryConditions)
    {

    }
};

#endif /* NULLSURFACEINTEGRALCALCULATOR_HPP */
