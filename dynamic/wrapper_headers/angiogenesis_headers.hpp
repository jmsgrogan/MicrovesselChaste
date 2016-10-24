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

#include "AngiogenesisSolver.hpp"
#include "Owen2011MigrationRule.hpp"
#include "Owen2011SproutingRule.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "AbstractMigrationRule.hpp"
#include "AbstractSproutingRule.hpp"
#include "LatticeBasedMigrationRule.hpp"
#include "CellPopulationMigrationRule.hpp"
#include "RegressionSolver.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"

template class AngiogenesisSolver<3>;
template class Owen2011MigrationRule<3>;
template class Owen2011SproutingRule<3>;
template class OffLatticeMigrationRule<3>;
template class OffLatticeSproutingRule<3>;
template class AbstractMigrationRule<3>;
template class AbstractSproutingRule<3>;
template class RegressionSolver<3>;
template class WallShearStressBasedRegressionSolver<3>;
template class CellPopulationMigrationRule<3>;
template class AngiogenesisSolver<2>;
template class Owen2011MigrationRule<2>;
template class Owen2011SproutingRule<2>;
template class OffLatticeMigrationRule<2>;
template class OffLatticeSproutingRule<2>;
template class AbstractMigrationRule<2>;
template class AbstractSproutingRule<2>;
template class LatticeBasedMigrationRule<2>;
template class CellPopulationMigrationRule<2>;
template class RegressionSolver<2>;
template class WallShearStressBasedRegressionSolver<2>;

