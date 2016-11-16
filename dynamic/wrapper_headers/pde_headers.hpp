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

//// Typdef in this namespace so that pyplusplus uses the nicer typedef'd name for the class
namespace pyplusplus{
namespace aliases{
typedef std::map<unsigned, units::quantity<unit::concentration_flow_rate> >  MapUnsigned_ConcentrationFlowRate;
typedef std::map<unsigned, units::quantity<unit::concentration> >  MapUnsigned_Concentration;
}
}

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
                    sizeof(GreensFunctionSolver<2>) +
                    sizeof(pyplusplus::aliases::MapUnsigned_ConcentrationFlowRate) +
                                sizeof(pyplusplus::aliases::MapUnsigned_Concentration);
        }
    }
}
