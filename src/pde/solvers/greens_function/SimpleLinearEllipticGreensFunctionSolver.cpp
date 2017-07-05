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

#include "Vessel.hpp"
#include "VesselSegment.hpp"
#include "ChastePoint.hpp"
#include "LinearSystem.hpp"
#include "ReplicatableVector.hpp"
#include "UblasMatrixInclude.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"

#include "SimpleLinearEllipticGreensFunctionSolver.hpp"

template<unsigned DIM>
SimpleLinearEllipticGreensFunctionSolver<DIM>::SimpleLinearEllipticGreensFunctionSolver()
    : AbstractGreensFunctionSolverBase<DIM>()
{

}

template<unsigned DIM>
SimpleLinearEllipticGreensFunctionSolver<DIM>::~SimpleLinearEllipticGreensFunctionSolver()
{

}

template<unsigned DIM>
void SimpleLinearEllipticGreensFunctionSolver<DIM>::Solve()
{
    // Set up the sub-segment and tissue point co-ordinates
    this->GenerateSubSegments();
    this->GenerateTissuePoints();

    // Generate the greens function matrices
    this->UpdateGreensFunctionMatrices(1, 1, 1, 1);

    // Get the sink rates
    std::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_elliptic_pde =
                std::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde);
    if(!p_elliptic_pde)
    {
        EXCEPTION("PDE type not recognized in Green's function solver.");
    }

    unsigned number_of_sinks = this->mSinkCoordinates.size();
    QConcentrationFlowRate sink_rate = p_elliptic_pde->ComputeConstantInUSourceTerm();

    QLength spacing = this->mpRegularGrid->GetSpacing();
    QVolume sink_volume = spacing*spacing*spacing;
    this->mSinkRates = std::vector<QMolarFlowRate >(number_of_sinks, sink_rate * sink_volume);
    QMolarFlowRate total_sink_rate = std::accumulate(this->mSinkRates.begin(), this->mSinkRates.end(), 0.0*unit::mole_per_second);

    // Get the sink substance demand on each vessel subsegment
    unsigned number_of_subsegments = this->mSubSegmentCoordinates.size();
    QDiffusivity diffusivity = p_elliptic_pde->ComputeIsotropicDiffusionTerm();
    std::vector<QConcentration > sink_demand_per_subsegment(number_of_subsegments, 0.0*this->mReferenceConcentration);
    for (unsigned idx = 0; idx < number_of_subsegments; idx++)
    {
        for (unsigned jdx = 0; jdx < number_of_sinks; jdx++)
        {
            sink_demand_per_subsegment[idx] = sink_demand_per_subsegment[idx] + ((*this->mGvt)[idx][jdx] / diffusivity) * this->mSinkRates[jdx];
        }
    }

    this->mSegmentConcentration = std::vector<QConcentration >(number_of_subsegments, 1.0*this->mReferenceConcentration);
    this->mConcentrations = std::vector<QConcentration >(number_of_sinks, 0.0*this->mReferenceConcentration);

    // Solve for the subsegment source rates required to meet the sink substance demand
    double tolerance = 1.e-10;
    QConcentration g0 = 0.0 * this->mReferenceConcentration;
    this->mSourceRates = std::vector<QMolarFlowRate >(number_of_subsegments, 0.0*unit::mole_per_second);

    QTime reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    QConcentration reference_concentration = this->mReferenceConcentration;
    QAmount reference_amount(1.0*unit::moles);
    LinearSystem linear_system(number_of_subsegments + 1, number_of_subsegments + 1);
    linear_system.SetKspType("bcgs");

    for (unsigned iteration = 0; iteration < 10; iteration++)
    {
        linear_system.AssembleIntermediateLinearSystem();
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            linear_system.SetRhsVectorElement(i, (this->mSegmentConcentration[i] - sink_demand_per_subsegment[i])/reference_concentration);
        }

        linear_system.SetRhsVectorElement(number_of_subsegments, -total_sink_rate*(reference_time/reference_amount));

        // Set up Linear system matrix
        for (unsigned iter = 0; iter < number_of_subsegments; iter++)
        {
            for (unsigned jter = 0; jter < number_of_subsegments; jter++)
            {
                linear_system.SetMatrixElement(iter, jter, ((*this->mGvv)[iter][jter] / diffusivity)*(reference_amount/(reference_time*reference_concentration)));
            }
            linear_system.SetMatrixElement(number_of_subsegments, iter, 1.0);
            linear_system.SetMatrixElement(iter, number_of_subsegments, 1.0);
        }
        linear_system.SetMatrixElement(number_of_subsegments, number_of_subsegments, 0.0);

        // Solve the linear system
        linear_system.AssembleFinalLinearSystem();
        ReplicatableVector soln_repl(linear_system.Solve());

        // Populate the solution vector
        std::vector<QMolarFlowRate > solution_vector(number_of_subsegments + 1);
        for (unsigned row = 0; row < number_of_subsegments + 1; row++)
        {
            (solution_vector)[row] = soln_repl[row]*(reference_amount/reference_time);
        }

        // Check convergence
        bool all_in_tolerance = true;
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            QMolarFlowRate diff = Qabs(this->mSourceRates[i] - solution_vector[i]);
            if (diff > tolerance*unit::mole_per_second)
            {
                all_in_tolerance = false;
                break;
            }
        }
        // Retrieve the solution
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            this->mSourceRates[i] = solution_vector[i];
        }
        g0 = solution_vector[number_of_subsegments]*reference_concentration*(reference_time/reference_amount);

        if (all_in_tolerance)
        {
            break;
        }
        else
        {
            if (iteration == 9)
            {
                std::cout << "Did not converge\n";
            }
        }
    }

    // Get the tissue concentration and write the solution
    for (unsigned i = 0; i < number_of_sinks; i++)
    {
        this->mConcentrations[i] = 0.0 * this->mReferenceConcentration;
        for (unsigned j = 0; j < number_of_sinks; j++)
        {
            this->mConcentrations[i] = this->mConcentrations[i] + (*this->mGtt)[i][j] * this->mSinkRates[j] / diffusivity;
        }

        for (unsigned j = 0; j < number_of_subsegments; j++)
        {
            this->mConcentrations[i] = this->mConcentrations[i] +(*this->mGtv)[i][j] * this->mSourceRates[j] / diffusivity;
        }
        this->mConcentrations[i] = this->mConcentrations[i] + g0;
    }

    std::map<std::string, std::vector<QConcentration > > segmentPointData;
    segmentPointData[this->mLabel] = this->mSegmentConcentration;

    this->UpdateSolution(this->mConcentrations);
    if(this->mWriteSolution)
    {
        this->WriteSolution(segmentPointData);
    }
}

// Explicit instantiation
template class SimpleLinearEllipticGreensFunctionSolver<2>;
template class SimpleLinearEllipticGreensFunctionSolver<3>;
