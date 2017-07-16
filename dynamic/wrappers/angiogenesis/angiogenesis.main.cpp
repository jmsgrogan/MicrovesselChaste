#include <pybind11/pybind11.h>
#include "AbstractMigrationRule2.cppwg.hpp"
#include "AbstractMigrationRule3.cppwg.hpp"
#include "AbstractSproutingRule2.cppwg.hpp"
#include "AbstractSproutingRule3.cppwg.hpp"
#include "AngiogenesisSolver2.cppwg.hpp"
#include "AngiogenesisSolver3.cppwg.hpp"
#include "OffLatticeMigrationRule2.cppwg.hpp"
#include "OffLatticeMigrationRule3.cppwg.hpp"
#include "OffLatticeSproutingRule2.cppwg.hpp"
#include "OffLatticeSproutingRule3.cppwg.hpp"
#include "LatticeBasedMigrationRule2.cppwg.hpp"
#include "LatticeBasedMigrationRule3.cppwg.hpp"
#include "LatticeBasedSproutingRule2.cppwg.hpp"
#include "LatticeBasedSproutingRule3.cppwg.hpp"
#include "TipAttractionLatticeBasedMigrationRule2.cppwg.hpp"
#include "TipAttractionLatticeBasedMigrationRule3.cppwg.hpp"
#include "Owen2011MigrationRule2.cppwg.hpp"
#include "Owen2011MigrationRule3.cppwg.hpp"
#include "Owen2011SproutingRule2.cppwg.hpp"
#include "Owen2011SproutingRule3.cppwg.hpp"
#include "CellPopulationMigrationRule2.cppwg.hpp"
#include "CellPopulationMigrationRule3.cppwg.hpp"
#include "RegressionSolver2.cppwg.hpp"
#include "RegressionSolver3.cppwg.hpp"
#include "WallShearStressBasedRegressionSolver2.cppwg.hpp"
#include "WallShearStressBasedRegressionSolver3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_angiogenesis, m)
{
    register_AbstractMigrationRule2_class(m);
    register_AbstractMigrationRule3_class(m);
    register_AbstractSproutingRule2_class(m);
    register_AbstractSproutingRule3_class(m);
    register_AngiogenesisSolver2_class(m);
    register_AngiogenesisSolver3_class(m);
    register_OffLatticeMigrationRule2_class(m);
    register_OffLatticeMigrationRule3_class(m);
    register_OffLatticeSproutingRule2_class(m);
    register_OffLatticeSproutingRule3_class(m);
    register_LatticeBasedMigrationRule2_class(m);
    register_LatticeBasedMigrationRule3_class(m);
    register_LatticeBasedSproutingRule2_class(m);
    register_LatticeBasedSproutingRule3_class(m);
    register_TipAttractionLatticeBasedMigrationRule2_class(m);
    register_TipAttractionLatticeBasedMigrationRule3_class(m);
    register_Owen2011MigrationRule2_class(m);
    register_Owen2011MigrationRule3_class(m);
    register_Owen2011SproutingRule2_class(m);
    register_Owen2011SproutingRule3_class(m);
    register_CellPopulationMigrationRule2_class(m);
    register_CellPopulationMigrationRule3_class(m);
    register_RegressionSolver2_class(m);
    register_RegressionSolver3_class(m);
    register_WallShearStressBasedRegressionSolver2_class(m);
    register_WallShearStressBasedRegressionSolver3_class(m);
}
