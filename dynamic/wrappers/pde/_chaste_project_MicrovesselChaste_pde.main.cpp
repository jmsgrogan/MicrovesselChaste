// This file has been generated by Py++.

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


#include "boost/python.hpp"

#include "indexing_suite/value_traits.hpp"

#include "indexing_suite/container_suite.hpp"

#include "indexing_suite/vector.hpp"

#include "indexing_suite/map.hpp"

#include "wrapper_header_collection.hpp"

#include "AbstractDiscreteContinuumLinearEllipticPde2_2.pypp.hpp"

#include "AbstractDiscreteContinuumLinearEllipticPde3_3.pypp.hpp"

#include "AbstractDiscreteContinuumNonLinearEllipticPde2_2.pypp.hpp"

#include "AbstractDiscreteContinuumNonLinearEllipticPde3_3.pypp.hpp"

#include "AbstractDiscreteContinuumParabolicPde2_2.pypp.hpp"

#include "AbstractDiscreteContinuumParabolicPde3_3.pypp.hpp"

#include "AbstractDiscreteContinuumPde2_2.pypp.hpp"

#include "AbstractDiscreteContinuumPde3_3.pypp.hpp"

#include "AbstractDiscreteContinuumSolver2.pypp.hpp"

#include "AbstractDiscreteContinuumSolver3.pypp.hpp"

#include "AbstractFiniteDifferenceSolverBase2.pypp.hpp"

#include "AbstractFiniteDifferenceSolverBase3.pypp.hpp"

#include "AbstractFiniteElementSolverBase2.pypp.hpp"

#include "AbstractFiniteElementSolverBase3.pypp.hpp"

#include "AbstractMixedGridDiscreteContinuumSolver2.pypp.hpp"

#include "AbstractMixedGridDiscreteContinuumSolver3.pypp.hpp"

#include "AbstractRegularGridDiscreteContinuumSolver2.pypp.hpp"

#include "AbstractRegularGridDiscreteContinuumSolver3.pypp.hpp"

#include "AbstractUnstructuredGridDiscreteContinuumSolver2.pypp.hpp"

#include "AbstractUnstructuredGridDiscreteContinuumSolver3.pypp.hpp"

#include "BoundaryConditionSource.pypp.hpp"

#include "BoundaryConditionType.pypp.hpp"

#include "CellBasedDiscreteSource2.pypp.hpp"

#include "CellBasedDiscreteSource3.pypp.hpp"

#include "CellStateDependentDiscreteSource2.pypp.hpp"

#include "CellStateDependentDiscreteSource3.pypp.hpp"

#include "CoupledLumpedSystemFiniteDifferenceSolver2.pypp.hpp"

#include "CoupledLumpedSystemFiniteDifferenceSolver3.pypp.hpp"

#include "CoupledLumpedSystemFiniteElementSolver2.pypp.hpp"

#include "CoupledLumpedSystemFiniteElementSolver3.pypp.hpp"

#include "CoupledVegfPelletDiffusionReactionPde2_2.pypp.hpp"

#include "CoupledVegfPelletDiffusionReactionPde3_3.pypp.hpp"

#include "DensityMap2.pypp.hpp"

#include "DensityMap3.pypp.hpp"

#include "DiscreteContinuumBoundaryCondition2.pypp.hpp"

#include "DiscreteContinuumBoundaryCondition3.pypp.hpp"

#include "DiscreteContinuumLinearEllipticPde2_2.pypp.hpp"

#include "DiscreteContinuumLinearEllipticPde3_3.pypp.hpp"

#include "DiscreteSource2.pypp.hpp"

#include "DiscreteSource3.pypp.hpp"

#include "FunctionMap2.pypp.hpp"

#include "FunctionMap3.pypp.hpp"

#include "MapUnsigned_Concentration.pypp.hpp"

#include "MapUnsigned_ConcentrationFlowRate.pypp.hpp"

#include "MichaelisMentenSteadyStateDiffusionReactionPde2_2.pypp.hpp"

#include "MichaelisMentenSteadyStateDiffusionReactionPde3_3.pypp.hpp"

#include "ParabolicDiffusionReactionPde2_2.pypp.hpp"

#include "ParabolicDiffusionReactionPde3_3.pypp.hpp"

#include "SimpleLinearEllipticFiniteDifferenceSolver2.pypp.hpp"

#include "SimpleLinearEllipticFiniteDifferenceSolver3.pypp.hpp"

#include "SimpleLinearEllipticFiniteElementSolver2.pypp.hpp"

#include "SimpleLinearEllipticFiniteElementSolver3.pypp.hpp"

#include "SimpleNonLinearEllipticFiniteDifferenceSolver2.pypp.hpp"

#include "SimpleNonLinearEllipticFiniteDifferenceSolver3.pypp.hpp"

#include "SimpleNonLinearEllipticFiniteElementSolver2.pypp.hpp"

#include "SimpleNonLinearEllipticFiniteElementSolver3.pypp.hpp"

#include "SimpleParabolicFiniteDifferenceSolver2.pypp.hpp"

#include "SimpleParabolicFiniteDifferenceSolver3.pypp.hpp"

#include "SimpleParabolicFiniteElementSolver2.pypp.hpp"

#include "SimpleParabolicFiniteElementSolver3.pypp.hpp"

#include "SolutionDependentDiscreteSource2.pypp.hpp"

#include "SolutionDependentDiscreteSource3.pypp.hpp"

#include "VectorCVectorDouble3.pypp.hpp"

#include "VectorConcentrationFlowRateQuantity.pypp.hpp"

#include "VectorConcentrationQuantity.pypp.hpp"

#include "VectorDimensionalChastePoint2.pypp.hpp"

#include "VectorDimensionalChastePoint3.pypp.hpp"

#include "VectorDouble.pypp.hpp"

#include "VectorPairVectorDoubleDouble.pypp.hpp"

#include "VectorRateQuantity.pypp.hpp"

#include "VectorSharedPtrDiscreteSource2.pypp.hpp"

#include "VectorSharedPtrDiscreteSource3.pypp.hpp"

#include "VesselBasedDiscreteSource2.pypp.hpp"

#include "VesselBasedDiscreteSource3.pypp.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_chaste_project_MicrovesselChaste_pde){
    register_VectorPairVectorDoubleDouble_class();

    register_VectorDouble_class();

    register_VectorRateQuantity_class();

    register_VectorConcentrationFlowRateQuantity_class();

    register_VectorConcentrationQuantity_class();

    register_VectorSharedPtrDiscreteSource3_class();

    register_VectorSharedPtrDiscreteSource2_class();

    register_VectorCVectorDouble3_class();

    register_VectorDimensionalChastePoint3_class();

    register_VectorDimensionalChastePoint2_class();

    register_MapUnsigned_ConcentrationFlowRate_class();

    register_MapUnsigned_Concentration_class();

    register_AbstractDiscreteContinuumPde2_2_class();

    register_AbstractDiscreteContinuumLinearEllipticPde2_2_class();

    register_AbstractDiscreteContinuumPde3_3_class();

    register_AbstractDiscreteContinuumLinearEllipticPde3_3_class();

    register_AbstractDiscreteContinuumNonLinearEllipticPde2_2_class();

    register_AbstractDiscreteContinuumNonLinearEllipticPde3_3_class();

    register_AbstractDiscreteContinuumParabolicPde2_2_class();

    register_AbstractDiscreteContinuumParabolicPde3_3_class();

    register_AbstractDiscreteContinuumSolver2_class();

    register_AbstractDiscreteContinuumSolver3_class();

    register_AbstractRegularGridDiscreteContinuumSolver2_class();

    register_AbstractFiniteDifferenceSolverBase2_class();

    register_AbstractRegularGridDiscreteContinuumSolver3_class();

    register_AbstractFiniteDifferenceSolverBase3_class();

    register_AbstractUnstructuredGridDiscreteContinuumSolver2_class();

    register_AbstractFiniteElementSolverBase2_class();

    register_AbstractUnstructuredGridDiscreteContinuumSolver3_class();

    register_AbstractFiniteElementSolverBase3_class();

    register_AbstractMixedGridDiscreteContinuumSolver2_class();

    register_AbstractMixedGridDiscreteContinuumSolver3_class();

    register_BoundaryConditionSource_class();

    register_BoundaryConditionType_class();

    register_DiscreteSource2_class();

    register_CellBasedDiscreteSource2_class();

    register_DiscreteSource3_class();

    register_CellBasedDiscreteSource3_class();

    register_CellStateDependentDiscreteSource2_class();

    register_CellStateDependentDiscreteSource3_class();

    register_SimpleParabolicFiniteDifferenceSolver2_class();

    register_CoupledLumpedSystemFiniteDifferenceSolver2_class();

    register_SimpleParabolicFiniteDifferenceSolver3_class();

    register_CoupledLumpedSystemFiniteDifferenceSolver3_class();

    register_CoupledLumpedSystemFiniteElementSolver2_class();

    register_CoupledLumpedSystemFiniteElementSolver3_class();

    register_CoupledVegfPelletDiffusionReactionPde2_2_class();

    register_CoupledVegfPelletDiffusionReactionPde3_3_class();

    register_DensityMap2_class();

    register_DensityMap3_class();

    register_DiscreteContinuumBoundaryCondition2_class();

    register_DiscreteContinuumBoundaryCondition3_class();

    register_DiscreteContinuumLinearEllipticPde2_2_class();

    register_DiscreteContinuumLinearEllipticPde3_3_class();

    register_FunctionMap2_class();

    register_FunctionMap3_class();

    register_MichaelisMentenSteadyStateDiffusionReactionPde2_2_class();

    register_MichaelisMentenSteadyStateDiffusionReactionPde3_3_class();

    register_ParabolicDiffusionReactionPde2_2_class();

    register_ParabolicDiffusionReactionPde3_3_class();

    register_SimpleLinearEllipticFiniteDifferenceSolver2_class();

    register_SimpleLinearEllipticFiniteDifferenceSolver3_class();

    register_SimpleLinearEllipticFiniteElementSolver2_class();

    register_SimpleLinearEllipticFiniteElementSolver3_class();

    register_SimpleNonLinearEllipticFiniteDifferenceSolver2_class();

    register_SimpleNonLinearEllipticFiniteDifferenceSolver3_class();

    register_SimpleNonLinearEllipticFiniteElementSolver2_class();

    register_SimpleNonLinearEllipticFiniteElementSolver3_class();

    register_SimpleParabolicFiniteElementSolver2_class();

    register_SimpleParabolicFiniteElementSolver3_class();

    register_SolutionDependentDiscreteSource2_class();

    register_SolutionDependentDiscreteSource3_class();

    register_VesselBasedDiscreteSource2_class();

    register_VesselBasedDiscreteSource3_class();
}

