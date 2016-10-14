#include "DiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "SolutionDependentDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "FiniteElementSolver.hpp"
#include "FunctionMap.hpp"
#include "GreensFunctionSolver.hpp"

namespace chaste
{
    namespace pde
    {
        inline int Instantiation()
        {
            return  sizeof(DiscreteSource<3>) +
                    sizeof(CellStateDependentDiscreteSource<3>) +
                    sizeof(CellBasedDiscreteSource<3>) +
                    sizeof(SolutionDependentDiscreteSource<3>) +
                    sizeof(VesselBasedDiscreteSource<3>) +
                    sizeof(DiscreteContinuumBoundaryCondition<3>) +
                    sizeof(AbstractDiscreteContinuumLinearEllipticPde<3, 3>) +
                    sizeof(AbstractDiscreteContinuumNonLinearEllipticPde<3, 3>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(MichaelisMentenSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<3, 3>) +
                    sizeof(AbstractDiscreteContinuumSolver<3>) +
                    sizeof(AbstractRegularGridDiscreteContinuumSolver<3>) +
                    sizeof(AbstractUnstructuredGridDiscreteContinuumSolver<3>) +
                    sizeof(FiniteDifferenceSolver<3>) +
                    sizeof(FiniteElementSolver<3>) +
                    sizeof(FunctionMap<3>) +
                    sizeof(GreensFunctionSolver<3>) +
                    sizeof(DiscreteSource<2>) +
                    sizeof(CellStateDependentDiscreteSource<2>) +
                    sizeof(CellBasedDiscreteSource<2>) +
                    sizeof(SolutionDependentDiscreteSource<2>) +
                    sizeof(VesselBasedDiscreteSource<2>) +
                    sizeof(DiscreteContinuumBoundaryCondition<2>) +
                    sizeof(AbstractDiscreteContinuumLinearEllipticPde<2, 2>) +
                    sizeof(AbstractDiscreteContinuumNonLinearEllipticPde<2, 2>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(MichaelisMentenSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(LinearSteadyStateDiffusionReactionPde<2, 2>) +
                    sizeof(AbstractDiscreteContinuumSolver<2>) +
                    sizeof(AbstractRegularGridDiscreteContinuumSolver<2>) +
                    sizeof(AbstractUnstructuredGridDiscreteContinuumSolver<2>) +
                    sizeof(FiniteDifferenceSolver<2>) +
                    sizeof(FiniteElementSolver<2>) +
                    sizeof(FunctionMap<2>) +
                    sizeof(GreensFunctionSolver<2>);
        }
    }
}
