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
template class AngiogenesisSolver<2>;
template class Owen2011MigrationRule<2>;
template class Owen2011SproutingRule<2>;
template class OffLatticeMigrationRule<3>;
template class OffLatticeSproutingRule<2>;
template class AbstractMigrationRule<2>;
template class AbstractSproutingRule<2>;
template class LatticeBasedMigrationRule<2>;
template class AbstractMigrationRule<2>;
template class CellPopulationMigrationRule<2>;
template class RegressionSolver<2>;
template class WallShearStressBasedRegressionSolver<2>;

