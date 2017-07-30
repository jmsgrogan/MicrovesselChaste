#include <pybind11/pybind11.h>
#include "DiscreteContinuumBoundaryCondition2.cppwg.hpp"
#include "DiscreteContinuumBoundaryCondition3.cppwg.hpp"
#include "BoundaryConditionType.cppwg.hpp"
#include "BoundaryConditionSource.cppwg.hpp"
#include "DiscreteSource2.cppwg.hpp"
#include "DiscreteSource3.cppwg.hpp"
#include "CellStateDependentDiscreteSource2.cppwg.hpp"
#include "CellStateDependentDiscreteSource3.cppwg.hpp"
#include "CellBasedDiscreteSource2.cppwg.hpp"
#include "CellBasedDiscreteSource3.cppwg.hpp"
#include "SolutionDependentDiscreteSource2.cppwg.hpp"
#include "SolutionDependentDiscreteSource3.cppwg.hpp"
#include "VesselBasedDiscreteSource2.cppwg.hpp"
#include "VesselBasedDiscreteSource3.cppwg.hpp"
#include "AbstractDiscreteContinuumPde2_2.cppwg.hpp"
#include "AbstractDiscreteContinuumPde3_3.cppwg.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde2_2.cppwg.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde3_3.cppwg.hpp"
#include "DiscreteContinuumLinearEllipticPde2_2.cppwg.hpp"
#include "DiscreteContinuumLinearEllipticPde3_3.cppwg.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde2_2.cppwg.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde3_3.cppwg.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde2_2.cppwg.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde3_3.cppwg.hpp"
#include "AbstractDiscreteContinuumParabolicPde2_2.cppwg.hpp"
#include "AbstractDiscreteContinuumParabolicPde3_3.cppwg.hpp"
#include "ParabolicDiffusionReactionPde2_2.cppwg.hpp"
#include "ParabolicDiffusionReactionPde3_3.cppwg.hpp"
#include "CoupledVegfPelletDiffusionReactionPde2_2.cppwg.hpp"
#include "CoupledVegfPelletDiffusionReactionPde3_3.cppwg.hpp"
#include "AbstractDiscreteContinuumSolver2.cppwg.hpp"
#include "AbstractDiscreteContinuumSolver3.cppwg.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver2.cppwg.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver3.cppwg.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver2.cppwg.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver3.cppwg.hpp"
#include "AbstractMixedGridDiscreteContinuumSolver2.cppwg.hpp"
#include "AbstractMixedGridDiscreteContinuumSolver3.cppwg.hpp"
#include "AbstractFiniteDifferenceSolverBase2.cppwg.hpp"
#include "AbstractFiniteDifferenceSolverBase3.cppwg.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver2.cppwg.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver3.cppwg.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver2.cppwg.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver3.cppwg.hpp"
#include "AbstractFiniteElementSolverBase2.cppwg.hpp"
#include "AbstractFiniteElementSolverBase3.cppwg.hpp"
#include "SimpleLinearEllipticFiniteElementSolver2.cppwg.hpp"
#include "SimpleLinearEllipticFiniteElementSolver3.cppwg.hpp"
#include "SimpleNonLinearEllipticFiniteElementSolver2.cppwg.hpp"
#include "SimpleNonLinearEllipticFiniteElementSolver3.cppwg.hpp"
#include "SimpleParabolicFiniteElementSolver2.cppwg.hpp"
#include "SimpleParabolicFiniteElementSolver3.cppwg.hpp"
#include "SimpleParabolicFiniteDifferenceSolver2.cppwg.hpp"
#include "SimpleParabolicFiniteDifferenceSolver3.cppwg.hpp"
#include "CoupledLumpedSystemFiniteElementSolver2.cppwg.hpp"
#include "CoupledLumpedSystemFiniteElementSolver3.cppwg.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver2.cppwg.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver3.cppwg.hpp"
#include "FunctionMap2.cppwg.hpp"
#include "FunctionMap3.cppwg.hpp"
#include "DensityMap2.cppwg.hpp"
#include "DensityMap3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_pde, m)
{
    register_DiscreteContinuumBoundaryCondition2_class(m);
    register_DiscreteContinuumBoundaryCondition3_class(m);
    register_BoundaryConditionType_class(m);
    register_BoundaryConditionSource_class(m);
    register_DiscreteSource2_class(m);
    register_DiscreteSource3_class(m);
    register_CellStateDependentDiscreteSource2_class(m);
    register_CellStateDependentDiscreteSource3_class(m);
    register_CellBasedDiscreteSource2_class(m);
    register_CellBasedDiscreteSource3_class(m);
    register_SolutionDependentDiscreteSource2_class(m);
    register_SolutionDependentDiscreteSource3_class(m);
    register_VesselBasedDiscreteSource2_class(m);
    register_VesselBasedDiscreteSource3_class(m);
    register_AbstractDiscreteContinuumPde2_2_class(m);
    register_AbstractDiscreteContinuumPde3_3_class(m);
    register_AbstractDiscreteContinuumLinearEllipticPde2_2_class(m);
    register_AbstractDiscreteContinuumLinearEllipticPde3_3_class(m);
    register_DiscreteContinuumLinearEllipticPde2_2_class(m);
    register_DiscreteContinuumLinearEllipticPde3_3_class(m);
    register_AbstractDiscreteContinuumNonLinearEllipticPde2_2_class(m);
    register_AbstractDiscreteContinuumNonLinearEllipticPde3_3_class(m);
    register_MichaelisMentenSteadyStateDiffusionReactionPde2_2_class(m);
    register_MichaelisMentenSteadyStateDiffusionReactionPde3_3_class(m);
    register_AbstractDiscreteContinuumParabolicPde2_2_class(m);
    register_AbstractDiscreteContinuumParabolicPde3_3_class(m);
    register_ParabolicDiffusionReactionPde2_2_class(m);
    register_ParabolicDiffusionReactionPde3_3_class(m);
    register_CoupledVegfPelletDiffusionReactionPde2_2_class(m);
    register_CoupledVegfPelletDiffusionReactionPde3_3_class(m);
    register_AbstractDiscreteContinuumSolver2_class(m);
    register_AbstractDiscreteContinuumSolver3_class(m);
    register_AbstractRegularGridDiscreteContinuumSolver2_class(m);
    register_AbstractRegularGridDiscreteContinuumSolver3_class(m);
    register_AbstractUnstructuredGridDiscreteContinuumSolver2_class(m);
    register_AbstractUnstructuredGridDiscreteContinuumSolver3_class(m);
    register_AbstractMixedGridDiscreteContinuumSolver2_class(m);
    register_AbstractMixedGridDiscreteContinuumSolver3_class(m);
    register_AbstractFiniteDifferenceSolverBase2_class(m);
    register_AbstractFiniteDifferenceSolverBase3_class(m);
    register_SimpleLinearEllipticFiniteDifferenceSolver2_class(m);
    register_SimpleLinearEllipticFiniteDifferenceSolver3_class(m);
    register_SimpleNonLinearEllipticFiniteDifferenceSolver2_class(m);
    register_SimpleNonLinearEllipticFiniteDifferenceSolver3_class(m);
    register_AbstractFiniteElementSolverBase2_class(m);
    register_AbstractFiniteElementSolverBase3_class(m);
    register_SimpleLinearEllipticFiniteElementSolver2_class(m);
    register_SimpleLinearEllipticFiniteElementSolver3_class(m);
    register_SimpleNonLinearEllipticFiniteElementSolver2_class(m);
    register_SimpleNonLinearEllipticFiniteElementSolver3_class(m);
    register_SimpleParabolicFiniteElementSolver2_class(m);
    register_SimpleParabolicFiniteElementSolver3_class(m);
    register_SimpleParabolicFiniteDifferenceSolver2_class(m);
    register_SimpleParabolicFiniteDifferenceSolver3_class(m);
    register_CoupledLumpedSystemFiniteElementSolver2_class(m);
    register_CoupledLumpedSystemFiniteElementSolver3_class(m);
    register_CoupledLumpedSystemFiniteDifferenceSolver2_class(m);
    register_CoupledLumpedSystemFiniteDifferenceSolver3_class(m);
    register_FunctionMap2_class(m);
    register_FunctionMap3_class(m);
    register_DensityMap2_class(m);
    register_DensityMap3_class(m);
}
