#include "AngiogenesisSolver.hpp"
#include "Owen2011MigrationRule.hpp"
#include "Owen2011SproutingRule.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "AbstractMigrationRule.hpp"
#include "AbstractSproutingRule.hpp"

template class AngiogenesisSolver<3>;
template class Owen2011MigrationRule<3>;
template class Owen2011SproutingRule<3>;
template class OffLatticeMigrationRule<3>;
template class OffLatticeSproutingRule<3>;
template class AbstractMigrationRule<3>;
template class AbstractSproutingRule<3>;
