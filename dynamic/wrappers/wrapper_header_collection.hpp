#ifndef MICROVESSELCHASTEHEADERS_HPP_ 
#define MICROVESSELCHASTEHEADERS_HPP_ 

// Includes 
#include <vector>
#include <set>
#include <map>
#include "Polygon.hpp"
#include "Facet.hpp"
#include "Part.hpp"
#include "MappableGridGenerator.hpp"
#include "NetworkToSurface.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "BoundaryExtractor.hpp"
#include "SurfaceCleaner.hpp"
#include "GeometryWriter.hpp"
#include "AbstractMesh.hpp"
#include "AbstractTetrahedralMesh.hpp"
#include "TetrahedralMesh.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"
#include "DimensionalChastePoint.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "GridCalculator.hpp"
#include "MeshReader.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "AbstractCellKiller.hpp"
#include "AbstractCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "VesselCellMutationState.hpp"
#include "MacrophageMutationState.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "Owen11CellPopulationGenerator.hpp"
#include "CaBasedCellPopulation.hpp"
#include "LQRadiotherapyCellKiller.hpp"
#include "Owen11CaBasedDivisionRule.hpp"
#include "Owen11CaUpdateRule.hpp"
#include "TipAttractionLatticeBasedMigrationRule.hpp"
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
#include "LatticeBasedSproutingRule.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteSource.hpp"
#include "CellStateDependentDiscreteSource.hpp"
#include "CellBasedDiscreteSource.hpp"
#include "SolutionDependentDiscreteSource.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "AbstractDiscreteContinuumPde.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "ParabolicDiffusionReactionPde.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"
#include "AbstractMixedGridDiscreteContinuumSolver.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "AbstractFiniteElementSolverBase.hpp"
#include "CoupledLumpedSystemFiniteElementSolver.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "SimpleNonLinearEllipticFiniteElementSolver.hpp"
#include "SimpleParabolicFiniteElementSolver.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"
#include "FunctionMap.hpp"
#include "DensityMap.hpp"
#include "FlowSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "BetteridgeHaematocritSolver.hpp"
#include "AbstractVesselNetworkCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "MetabolicStimulusCalculator.hpp"
#include "RadiusCalculator.hpp"
#include "ShrinkingStimulusCalculator.hpp"
#include "ViscosityCalculator.hpp"
#include "WallShearStressCalculator.hpp"
#include "AbstractStructuralAdaptationSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "AbstractHaematocritSolver.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "AbstractMicrovesselModifier.hpp"
#include "VtkSceneMicrovesselModifier.hpp"
#include "Owen2011TrackingModifier.hpp"
#include "AbstractCellBasedSimulationModifier.hpp"
#include "VesselNetworkReader.hpp"
#include "NodeFlowProperties.hpp"
#include "SegmentFlowProperties.hpp"
#include "VesselFlowProperties.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "AbstractVesselNetworkComponent.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"
#include "VesselNetworkWriter.hpp"
#include "DistanceMap.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "VesselNetworkGraphCalculator.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"
#include "AbstractVesselNetworkComponentFlowProperties.hpp"
#include "AbstractVesselNetworkComponentChemicalProperties.hpp"
#include "MicrovesselVtkScene.hpp"
#include "CellPopulationActorGenerator.hpp"
#include "DiscreteContinuumMeshActorGenerator.hpp"
#include "PartActorGenerator.hpp"
#include "RegularGridActorGenerator.hpp"
#include "VesselNetworkActorGenerator.hpp"
#include "AbstractActorGenerator.hpp"
#include "AbstractMicrovesselModifier.hpp"
#include "VtkSceneMicrovesselModifier.hpp"
#include "ParameterInstance.hpp"
#include "BaseUnits.hpp"
#include "UnitCollection.hpp"
#include "ParameterCollection.hpp"
#include "BaseParameterInstance.hpp"
#include "Owen11Parameters.hpp"
#include "Connor17Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "ImageReader.hpp"
#include "ImageToMesh.hpp"
#include "ImageToSurface.hpp"
#include "NetworkToImage.hpp"

// Instantiate Template Classes 
template class Polygon<2 >;
template class Polygon<3 >;
template class Facet<2 >;
template class Facet<3 >;
template class Part<2 >;
template class Part<3 >;
template class NetworkToSurface<2 >;
template class NetworkToSurface<3 >;
template class VesselSurfaceGenerator<2 >;
template class VesselSurfaceGenerator<3 >;
template class AbstractMesh<2,2 >;
template class AbstractMesh<3,3 >;
template class AbstractTetrahedralMesh<2,2 >;
template class AbstractTetrahedralMesh<3,3 >;
template class TetrahedralMesh<2,2 >;
template class TetrahedralMesh<3,3 >;
template class AbstractDiscreteContinuumGrid<2,2 >;
template class AbstractDiscreteContinuumGrid<3,3 >;
template class DimensionalChastePoint<2 >;
template class DimensionalChastePoint<3 >;
template class DiscreteContinuumMesh<2,2 >;
template class DiscreteContinuumMesh<3,3 >;
template class DiscreteContinuumMeshGenerator<2,2 >;
template class DiscreteContinuumMeshGenerator<3,3 >;
template class GridCalculator<2 >;
template class GridCalculator<3 >;
template class MultiFormatMeshWriter<2 >;
template class MultiFormatMeshWriter<3 >;
template class RegularGrid<2 >;
template class RegularGrid<3 >;
template class AbstractCellKiller<2 >;
template class AbstractCellKiller<3 >;
template class Owen11CellPopulationGenerator<2 >;
template class Owen11CellPopulationGenerator<3 >;
template class CaBasedCellPopulation<2 >;
template class CaBasedCellPopulation<3 >;
template class LQRadiotherapyCellKiller<2 >;
template class LQRadiotherapyCellKiller<3 >;
template class Owen11CaBasedDivisionRule<2 >;
template class Owen11CaBasedDivisionRule<3 >;
template class Owen11CaUpdateRule<2 >;
template class Owen11CaUpdateRule<3 >;
template class TipAttractionLatticeBasedMigrationRule<2 >;
template class TipAttractionLatticeBasedMigrationRule<3 >;
template class AngiogenesisSolver<2 >;
template class AngiogenesisSolver<3 >;
template class Owen2011MigrationRule<2 >;
template class Owen2011MigrationRule<3 >;
template class Owen2011SproutingRule<2 >;
template class Owen2011SproutingRule<3 >;
template class OffLatticeMigrationRule<2 >;
template class OffLatticeMigrationRule<3 >;
template class OffLatticeSproutingRule<2 >;
template class OffLatticeSproutingRule<3 >;
template class AbstractMigrationRule<2 >;
template class AbstractMigrationRule<3 >;
template class AbstractSproutingRule<2 >;
template class AbstractSproutingRule<3 >;
template class LatticeBasedMigrationRule<2 >;
template class LatticeBasedMigrationRule<3 >;
template class CellPopulationMigrationRule<2 >;
template class CellPopulationMigrationRule<3 >;
template class RegressionSolver<2 >;
template class RegressionSolver<3 >;
template class WallShearStressBasedRegressionSolver<2 >;
template class WallShearStressBasedRegressionSolver<3 >;
template class LatticeBasedSproutingRule<2 >;
template class LatticeBasedSproutingRule<3 >;
template class DiscreteContinuumBoundaryCondition<2 >;
template class DiscreteContinuumBoundaryCondition<3 >;
template class DiscreteSource<2 >;
template class DiscreteSource<3 >;
template class CellStateDependentDiscreteSource<2 >;
template class CellStateDependentDiscreteSource<3 >;
template class CellBasedDiscreteSource<2 >;
template class CellBasedDiscreteSource<3 >;
template class SolutionDependentDiscreteSource<2 >;
template class SolutionDependentDiscreteSource<3 >;
template class VesselBasedDiscreteSource<2 >;
template class VesselBasedDiscreteSource<3 >;
template class AbstractDiscreteContinuumPde<2,2 >;
template class AbstractDiscreteContinuumPde<3,3 >;
template class AbstractDiscreteContinuumLinearEllipticPde<2,2 >;
template class AbstractDiscreteContinuumLinearEllipticPde<3,3 >;
template class DiscreteContinuumLinearEllipticPde<2,2 >;
template class DiscreteContinuumLinearEllipticPde<3,3 >;
template class AbstractDiscreteContinuumNonLinearEllipticPde<2,2 >;
template class AbstractDiscreteContinuumNonLinearEllipticPde<3,3 >;
template class MichaelisMentenSteadyStateDiffusionReactionPde<2,2 >;
template class MichaelisMentenSteadyStateDiffusionReactionPde<3,3 >;
template class AbstractDiscreteContinuumParabolicPde<2,2 >;
template class AbstractDiscreteContinuumParabolicPde<3,3 >;
template class ParabolicDiffusionReactionPde<2,2 >;
template class ParabolicDiffusionReactionPde<3,3 >;
template class CoupledVegfPelletDiffusionReactionPde<2,2 >;
template class CoupledVegfPelletDiffusionReactionPde<3,3 >;
template class AbstractDiscreteContinuumSolver<2 >;
template class AbstractDiscreteContinuumSolver<3 >;
template class AbstractRegularGridDiscreteContinuumSolver<2 >;
template class AbstractRegularGridDiscreteContinuumSolver<3 >;
template class AbstractUnstructuredGridDiscreteContinuumSolver<2 >;
template class AbstractUnstructuredGridDiscreteContinuumSolver<3 >;
template class AbstractMixedGridDiscreteContinuumSolver<2 >;
template class AbstractMixedGridDiscreteContinuumSolver<3 >;
template class AbstractFiniteDifferenceSolverBase<2 >;
template class AbstractFiniteDifferenceSolverBase<3 >;
template class CoupledLumpedSystemFiniteDifferenceSolver<2 >;
template class CoupledLumpedSystemFiniteDifferenceSolver<3 >;
template class SimpleLinearEllipticFiniteDifferenceSolver<2 >;
template class SimpleLinearEllipticFiniteDifferenceSolver<3 >;
template class SimpleNonLinearEllipticFiniteDifferenceSolver<2 >;
template class SimpleNonLinearEllipticFiniteDifferenceSolver<3 >;
template class AbstractFiniteElementSolverBase<2 >;
template class AbstractFiniteElementSolverBase<3 >;
template class CoupledLumpedSystemFiniteElementSolver<2 >;
template class CoupledLumpedSystemFiniteElementSolver<3 >;
template class SimpleLinearEllipticFiniteElementSolver<2 >;
template class SimpleLinearEllipticFiniteElementSolver<3 >;
template class SimpleNonLinearEllipticFiniteElementSolver<2 >;
template class SimpleNonLinearEllipticFiniteElementSolver<3 >;
template class SimpleParabolicFiniteElementSolver<2 >;
template class SimpleParabolicFiniteElementSolver<3 >;
template class SimpleParabolicFiniteDifferenceSolver<2 >;
template class SimpleParabolicFiniteDifferenceSolver<3 >;
template class FunctionMap<2 >;
template class FunctionMap<3 >;
template class DensityMap<2 >;
template class DensityMap<3 >;
template class FlowSolver<2 >;
template class FlowSolver<3 >;
template class VesselImpedanceCalculator<2 >;
template class VesselImpedanceCalculator<3 >;
template class BetteridgeHaematocritSolver<2 >;
template class BetteridgeHaematocritSolver<3 >;
template class AbstractVesselNetworkCalculator<2 >;
template class AbstractVesselNetworkCalculator<3 >;
template class MechanicalStimulusCalculator<2 >;
template class MechanicalStimulusCalculator<3 >;
template class MetabolicStimulusCalculator<2 >;
template class MetabolicStimulusCalculator<3 >;
template class RadiusCalculator<2 >;
template class RadiusCalculator<3 >;
template class ShrinkingStimulusCalculator<2 >;
template class ShrinkingStimulusCalculator<3 >;
template class ViscosityCalculator<2 >;
template class ViscosityCalculator<3 >;
template class WallShearStressCalculator<2 >;
template class WallShearStressCalculator<3 >;
template class AbstractStructuralAdaptationSolver<2 >;
template class AbstractStructuralAdaptationSolver<3 >;
template class StructuralAdaptationSolver<2 >;
template class StructuralAdaptationSolver<3 >;
template class AbstractHaematocritSolver<2 >;
template class AbstractHaematocritSolver<3 >;
template class AlarconHaematocritSolver<2 >;
template class AlarconHaematocritSolver<3 >;
template class ConstantHaematocritSolver<2 >;
template class ConstantHaematocritSolver<3 >;
template class MicrovesselSolver<2 >;
template class MicrovesselSolver<3 >;
template class MicrovesselSimulationModifier<2 >;
template class MicrovesselSimulationModifier<3 >;
template class Owen2011TrackingModifier<2 >;
template class Owen2011TrackingModifier<3 >;
template class AbstractCellBasedSimulationModifier<2,2 >;
template class AbstractCellBasedSimulationModifier<3,3 >;
template class VesselNetworkReader<2 >;
template class VesselNetworkReader<3 >;
template class NodeFlowProperties<2 >;
template class NodeFlowProperties<3 >;
template class SegmentFlowProperties<2 >;
template class SegmentFlowProperties<3 >;
template class VesselFlowProperties<2 >;
template class VesselFlowProperties<3 >;
template class VesselNode<2 >;
template class VesselNode<3 >;
template class VesselSegment<2 >;
template class VesselSegment<3 >;
template class Vessel<2 >;
template class Vessel<3 >;
template class VesselNetwork<2 >;
template class VesselNetwork<3 >;
template class VesselNetworkGenerator<2 >;
template class VesselNetworkGenerator<3 >;
template class AbstractVesselNetworkComponent<2 >;
template class AbstractVesselNetworkComponent<3 >;
template class VesselNetworkCellPopulationInteractor<2 >;
template class VesselNetworkCellPopulationInteractor<3 >;
template class VesselNetworkWriter<2 >;
template class VesselNetworkWriter<3 >;
template class DistanceMap<2 >;
template class DistanceMap<3 >;
template class VesselNetworkGeometryCalculator<2 >;
template class VesselNetworkGeometryCalculator<3 >;
template class VesselNetworkGraphCalculator<2 >;
template class VesselNetworkGraphCalculator<3 >;
template class AbstractVesselNetworkComponentProperties<2 >;
template class AbstractVesselNetworkComponentProperties<3 >;
template class AbstractVesselNetworkComponentFlowProperties<2 >;
template class AbstractVesselNetworkComponentFlowProperties<3 >;
template class AbstractVesselNetworkComponentChemicalProperties<2 >;
template class AbstractVesselNetworkComponentChemicalProperties<3 >;
template class MicrovesselVtkScene<2 >;
template class MicrovesselVtkScene<3 >;
template class CellPopulationActorGenerator<2 >;
template class CellPopulationActorGenerator<3 >;
template class DiscreteContinuumMeshActorGenerator<2 >;
template class DiscreteContinuumMeshActorGenerator<3 >;
template class PartActorGenerator<2 >;
template class PartActorGenerator<3 >;
template class RegularGridActorGenerator<2 >;
template class RegularGridActorGenerator<3 >;
template class VesselNetworkActorGenerator<2 >;
template class VesselNetworkActorGenerator<3 >;
template class AbstractActorGenerator<2 >;
template class AbstractActorGenerator<3 >;
template class AbstractMicrovesselModifier<2 >;
template class AbstractMicrovesselModifier<3 >;
template class VtkSceneMicrovesselModifier<2 >;
template class VtkSceneMicrovesselModifier<3 >;
template class ImageToMesh<2 >;
template class ImageToMesh<3 >;
template class NetworkToImage<2 >;
template class NetworkToImage<3 >;

// Typedef for nicer naming in Py++ 
namespace pyplusplus{ 
namespace aliases{ 
typedef std::map<std::string,std::string > MapString_String;
typedef std::map<std::string, double > MapStringDouble;
typedef std::set<unsigned > SetUnsigned;
typedef std::pair<unsigned int, unsigned int > PairUnsignedUnsigned;
typedef std::pair<std::vector<double>, double > PairVectorDoubleDouble;
typedef std::vector<double > VectorDouble;
typedef std::vector<unsigned > VectorUnsigned;
typedef std::vector<int > VectorInt;
typedef std::vector<bool > VectorBool;
typedef std::vector<std::string > VectorString;
typedef std::vector<c_vector<double,3> > VectorCVectorDouble3;
typedef std::vector<c_vector<double,2> > VectorCVectorDouble2;
typedef std::vector<c_vector<unsigned,5> > VectorCVectorUnsigned5;
typedef std::vector<std::set<unsigned int> > VectorSetUnsigned;
typedef std::vector<std::vector<unsigned int> > VectorVectorUnsigned;
typedef std::vector<std::pair<unsigned int, unsigned int> > VectorPairUnsignedUnsigned;
typedef std::vector<std::pair<std::vector<double>, double> > VectorPairVectorDoubleDouble;
typedef c_vector<double,2 > CVectorDouble_2;
typedef c_vector<double,3 > CVectorDouble_3;
typedef c_vector<unsigned,5 > CVectorUnsigned_5;
typedef Polygon<2 > Polygon2;
typedef Polygon<2 >* Polygon2Ptr;
typedef boost::shared_ptr<Polygon<2 > > SharedPtrPolygon2;
typedef std::vector<Polygon2Ptr> VectorPolygon2Ptr;
typedef std::vector<SharedPtrPolygon2> VectorSharedPtrPolygon2;
typedef Polygon<3 > Polygon3;
typedef Polygon<3 >* Polygon3Ptr;
typedef boost::shared_ptr<Polygon<3 > > SharedPtrPolygon3;
typedef std::vector<Polygon3Ptr> VectorPolygon3Ptr;
typedef std::vector<SharedPtrPolygon3> VectorSharedPtrPolygon3;
typedef Facet<2 > Facet2;
typedef Facet<2 >* Facet2Ptr;
typedef boost::shared_ptr<Facet<2 > > SharedPtrFacet2;
typedef std::vector<Facet2Ptr> VectorFacet2Ptr;
typedef std::vector<SharedPtrFacet2> VectorSharedPtrFacet2;
typedef Facet<3 > Facet3;
typedef Facet<3 >* Facet3Ptr;
typedef boost::shared_ptr<Facet<3 > > SharedPtrFacet3;
typedef std::vector<Facet3Ptr> VectorFacet3Ptr;
typedef std::vector<SharedPtrFacet3> VectorSharedPtrFacet3;
typedef Part<2 > Part2;
typedef Part<3 > Part3;
typedef NetworkToSurface<2 > NetworkToSurface2;
typedef NetworkToSurface<3 > NetworkToSurface3;
typedef VesselSurfaceGenerator<2 > VesselSurfaceGenerator2;
typedef VesselSurfaceGenerator<3 > VesselSurfaceGenerator3;
typedef AbstractMesh<2,2 > AbstractMesh2_2;
typedef AbstractMesh<3,3 > AbstractMesh3_3;
typedef AbstractTetrahedralMesh<2,2 > AbstractTetrahedralMesh2_2;
typedef AbstractTetrahedralMesh<3,3 > AbstractTetrahedralMesh3_3;
typedef TetrahedralMesh<2,2 > TetrahedralMesh2_2;
typedef TetrahedralMesh<3,3 > TetrahedralMesh3_3;
typedef AbstractDiscreteContinuumGrid<2,2 > AbstractDiscreteContinuumGrid2_2;
typedef AbstractDiscreteContinuumGrid<3,3 > AbstractDiscreteContinuumGrid3_3;
typedef DimensionalChastePoint<2 > DimensionalChastePoint2;
typedef DimensionalChastePoint<2 >* DimensionalChastePoint2Ptr;
typedef boost::shared_ptr<DimensionalChastePoint<2 > > SharedPtrDimensionalChastePoint2;
typedef std::vector<DimensionalChastePoint2Ptr> VectorDimensionalChastePoint2Ptr;
typedef std::vector<SharedPtrDimensionalChastePoint2> VectorSharedPtrDimensionalChastePoint2;
typedef DimensionalChastePoint<3 > DimensionalChastePoint3;
typedef DimensionalChastePoint<3 >* DimensionalChastePoint3Ptr;
typedef boost::shared_ptr<DimensionalChastePoint<3 > > SharedPtrDimensionalChastePoint3;
typedef std::vector<DimensionalChastePoint3Ptr> VectorDimensionalChastePoint3Ptr;
typedef std::vector<SharedPtrDimensionalChastePoint3> VectorSharedPtrDimensionalChastePoint3;
typedef DiscreteContinuumMesh<2,2 > DiscreteContinuumMesh2_2;
typedef DiscreteContinuumMesh<3,3 > DiscreteContinuumMesh3_3;
typedef DiscreteContinuumMeshGenerator<2,2 > DiscreteContinuumMeshGenerator2_2;
typedef DiscreteContinuumMeshGenerator<3,3 > DiscreteContinuumMeshGenerator3_3;
typedef GridCalculator<2 > GridCalculator2;
typedef GridCalculator<3 > GridCalculator3;
typedef MultiFormatMeshWriter<2 > MultiFormatMeshWriter2;
typedef MultiFormatMeshWriter<3 > MultiFormatMeshWriter3;
typedef RegularGrid<2 > RegularGrid2;
typedef RegularGrid<3 > RegularGrid3;
typedef std::vector<std::vector<boost::shared_ptr<VesselNode<2> > > > VectorVectorSharedPtrVesselNode2;
typedef std::vector<std::vector<boost::shared_ptr<VesselNode<3> > > > VectorVectorSharedPtrVesselNode3;
typedef std::vector<std::vector<boost::shared_ptr<VesselSegment<2> > > > VectorVectorSharedPtrVesselSegment2;
typedef std::vector<std::vector<boost::shared_ptr<VesselSegment<3> > > > VectorVectorSharedPtrVesselSegment3;
typedef std::vector<std::vector<boost::shared_ptr<Cell> > > VectorVectorSharedPtrCell;
typedef std::vector<DimensionalChastePoint2 > VectorDimensionalChastePoint2;
typedef std::vector<DimensionalChastePoint3 > VectorDimensionalChastePoint3;
typedef std::pair<DimensionalChastePoint2, DimensionalChastePoint2 > PairDimensionalChastePoint2DimensionalChastePoint2;
typedef std::pair<DimensionalChastePoint3, DimensionalChastePoint3 > PairDimensionalChastePoint3DimensionalChastePoint3;
typedef AbstractCellKiller<2 > AbstractCellKiller2;
typedef AbstractCellKiller<3 > AbstractCellKiller3;
typedef Owen11CellPopulationGenerator<2 > Owen11CellPopulationGenerator2;
typedef Owen11CellPopulationGenerator<3 > Owen11CellPopulationGenerator3;
typedef CaBasedCellPopulation<2 > CaBasedCellPopulation2;
typedef CaBasedCellPopulation<3 > CaBasedCellPopulation3;
typedef LQRadiotherapyCellKiller<2 > LQRadiotherapyCellKiller2;
typedef LQRadiotherapyCellKiller<3 > LQRadiotherapyCellKiller3;
typedef Owen11CaBasedDivisionRule<2 > Owen11CaBasedDivisionRule2;
typedef Owen11CaBasedDivisionRule<3 > Owen11CaBasedDivisionRule3;
typedef Owen11CaUpdateRule<2 > Owen11CaUpdateRule2;
typedef Owen11CaUpdateRule<3 > Owen11CaUpdateRule3;
typedef TipAttractionLatticeBasedMigrationRule<2 > TipAttractionLatticeBasedMigrationRule2;
typedef TipAttractionLatticeBasedMigrationRule<3 > TipAttractionLatticeBasedMigrationRule3;
typedef AngiogenesisSolver<2 > AngiogenesisSolver2;
typedef AngiogenesisSolver<3 > AngiogenesisSolver3;
typedef Owen2011MigrationRule<2 > Owen2011MigrationRule2;
typedef Owen2011MigrationRule<3 > Owen2011MigrationRule3;
typedef Owen2011SproutingRule<2 > Owen2011SproutingRule2;
typedef Owen2011SproutingRule<3 > Owen2011SproutingRule3;
typedef OffLatticeMigrationRule<2 > OffLatticeMigrationRule2;
typedef OffLatticeMigrationRule<3 > OffLatticeMigrationRule3;
typedef OffLatticeSproutingRule<2 > OffLatticeSproutingRule2;
typedef OffLatticeSproutingRule<3 > OffLatticeSproutingRule3;
typedef AbstractMigrationRule<2 > AbstractMigrationRule2;
typedef AbstractMigrationRule<3 > AbstractMigrationRule3;
typedef AbstractSproutingRule<2 > AbstractSproutingRule2;
typedef AbstractSproutingRule<3 > AbstractSproutingRule3;
typedef LatticeBasedMigrationRule<2 > LatticeBasedMigrationRule2;
typedef LatticeBasedMigrationRule<3 > LatticeBasedMigrationRule3;
typedef CellPopulationMigrationRule<2 > CellPopulationMigrationRule2;
typedef CellPopulationMigrationRule<3 > CellPopulationMigrationRule3;
typedef RegressionSolver<2 > RegressionSolver2;
typedef RegressionSolver<3 > RegressionSolver3;
typedef WallShearStressBasedRegressionSolver<2 > WallShearStressBasedRegressionSolver2;
typedef WallShearStressBasedRegressionSolver<3 > WallShearStressBasedRegressionSolver3;
typedef LatticeBasedSproutingRule<2 > LatticeBasedSproutingRule2;
typedef LatticeBasedSproutingRule<3 > LatticeBasedSproutingRule3;
typedef DiscreteContinuumBoundaryCondition<2 > DiscreteContinuumBoundaryCondition2;
typedef DiscreteContinuumBoundaryCondition<3 > DiscreteContinuumBoundaryCondition3;
typedef DiscreteSource<2 > DiscreteSource2;
typedef DiscreteSource<2 >* DiscreteSource2Ptr;
typedef boost::shared_ptr<DiscreteSource<2 > > SharedPtrDiscreteSource2;
typedef std::vector<DiscreteSource2Ptr> VectorDiscreteSource2Ptr;
typedef std::vector<SharedPtrDiscreteSource2> VectorSharedPtrDiscreteSource2;
typedef DiscreteSource<3 > DiscreteSource3;
typedef DiscreteSource<3 >* DiscreteSource3Ptr;
typedef boost::shared_ptr<DiscreteSource<3 > > SharedPtrDiscreteSource3;
typedef std::vector<DiscreteSource3Ptr> VectorDiscreteSource3Ptr;
typedef std::vector<SharedPtrDiscreteSource3> VectorSharedPtrDiscreteSource3;
typedef CellStateDependentDiscreteSource<2 > CellStateDependentDiscreteSource2;
typedef CellStateDependentDiscreteSource<3 > CellStateDependentDiscreteSource3;
typedef CellBasedDiscreteSource<2 > CellBasedDiscreteSource2;
typedef CellBasedDiscreteSource<3 > CellBasedDiscreteSource3;
typedef SolutionDependentDiscreteSource<2 > SolutionDependentDiscreteSource2;
typedef SolutionDependentDiscreteSource<3 > SolutionDependentDiscreteSource3;
typedef VesselBasedDiscreteSource<2 > VesselBasedDiscreteSource2;
typedef VesselBasedDiscreteSource<3 > VesselBasedDiscreteSource3;
typedef AbstractDiscreteContinuumPde<2,2 > AbstractDiscreteContinuumPde2_2;
typedef AbstractDiscreteContinuumPde<3,3 > AbstractDiscreteContinuumPde3_3;
typedef AbstractDiscreteContinuumLinearEllipticPde<2,2 > AbstractDiscreteContinuumLinearEllipticPde2_2;
typedef AbstractDiscreteContinuumLinearEllipticPde<3,3 > AbstractDiscreteContinuumLinearEllipticPde3_3;
typedef DiscreteContinuumLinearEllipticPde<2,2 > DiscreteContinuumLinearEllipticPde2_2;
typedef DiscreteContinuumLinearEllipticPde<3,3 > DiscreteContinuumLinearEllipticPde3_3;
typedef AbstractDiscreteContinuumNonLinearEllipticPde<2,2 > AbstractDiscreteContinuumNonLinearEllipticPde2_2;
typedef AbstractDiscreteContinuumNonLinearEllipticPde<3,3 > AbstractDiscreteContinuumNonLinearEllipticPde3_3;
typedef MichaelisMentenSteadyStateDiffusionReactionPde<2,2 > MichaelisMentenSteadyStateDiffusionReactionPde2_2;
typedef MichaelisMentenSteadyStateDiffusionReactionPde<3,3 > MichaelisMentenSteadyStateDiffusionReactionPde3_3;
typedef AbstractDiscreteContinuumParabolicPde<2,2 > AbstractDiscreteContinuumParabolicPde2_2;
typedef AbstractDiscreteContinuumParabolicPde<3,3 > AbstractDiscreteContinuumParabolicPde3_3;
typedef ParabolicDiffusionReactionPde<2,2 > ParabolicDiffusionReactionPde2_2;
typedef ParabolicDiffusionReactionPde<3,3 > ParabolicDiffusionReactionPde3_3;
typedef CoupledVegfPelletDiffusionReactionPde<2,2 > CoupledVegfPelletDiffusionReactionPde2_2;
typedef CoupledVegfPelletDiffusionReactionPde<3,3 > CoupledVegfPelletDiffusionReactionPde3_3;
typedef AbstractDiscreteContinuumSolver<2 > AbstractDiscreteContinuumSolver2;
typedef AbstractDiscreteContinuumSolver<2 >* AbstractDiscreteContinuumSolver2Ptr;
typedef boost::shared_ptr<AbstractDiscreteContinuumSolver<2 > > SharedPtrAbstractDiscreteContinuumSolver2;
typedef std::vector<AbstractDiscreteContinuumSolver2Ptr> VectorAbstractDiscreteContinuumSolver2Ptr;
typedef std::vector<SharedPtrAbstractDiscreteContinuumSolver2> VectorSharedPtrAbstractDiscreteContinuumSolver2;
typedef AbstractDiscreteContinuumSolver<3 > AbstractDiscreteContinuumSolver3;
typedef AbstractDiscreteContinuumSolver<3 >* AbstractDiscreteContinuumSolver3Ptr;
typedef boost::shared_ptr<AbstractDiscreteContinuumSolver<3 > > SharedPtrAbstractDiscreteContinuumSolver3;
typedef std::vector<AbstractDiscreteContinuumSolver3Ptr> VectorAbstractDiscreteContinuumSolver3Ptr;
typedef std::vector<SharedPtrAbstractDiscreteContinuumSolver3> VectorSharedPtrAbstractDiscreteContinuumSolver3;
typedef AbstractRegularGridDiscreteContinuumSolver<2 > AbstractRegularGridDiscreteContinuumSolver2;
typedef AbstractRegularGridDiscreteContinuumSolver<3 > AbstractRegularGridDiscreteContinuumSolver3;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<2 > AbstractUnstructuredGridDiscreteContinuumSolver2;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<3 > AbstractUnstructuredGridDiscreteContinuumSolver3;
typedef AbstractMixedGridDiscreteContinuumSolver<2 > AbstractMixedGridDiscreteContinuumSolver2;
typedef AbstractMixedGridDiscreteContinuumSolver<3 > AbstractMixedGridDiscreteContinuumSolver3;
typedef AbstractFiniteDifferenceSolverBase<2 > AbstractFiniteDifferenceSolverBase2;
typedef AbstractFiniteDifferenceSolverBase<3 > AbstractFiniteDifferenceSolverBase3;
typedef CoupledLumpedSystemFiniteDifferenceSolver<2 > CoupledLumpedSystemFiniteDifferenceSolver2;
typedef CoupledLumpedSystemFiniteDifferenceSolver<3 > CoupledLumpedSystemFiniteDifferenceSolver3;
typedef SimpleLinearEllipticFiniteDifferenceSolver<2 > SimpleLinearEllipticFiniteDifferenceSolver2;
typedef SimpleLinearEllipticFiniteDifferenceSolver<3 > SimpleLinearEllipticFiniteDifferenceSolver3;
typedef SimpleNonLinearEllipticFiniteDifferenceSolver<2 > SimpleNonLinearEllipticFiniteDifferenceSolver2;
typedef SimpleNonLinearEllipticFiniteDifferenceSolver<3 > SimpleNonLinearEllipticFiniteDifferenceSolver3;
typedef AbstractFiniteElementSolverBase<2 > AbstractFiniteElementSolverBase2;
typedef AbstractFiniteElementSolverBase<3 > AbstractFiniteElementSolverBase3;
typedef CoupledLumpedSystemFiniteElementSolver<2 > CoupledLumpedSystemFiniteElementSolver2;
typedef CoupledLumpedSystemFiniteElementSolver<3 > CoupledLumpedSystemFiniteElementSolver3;
typedef SimpleLinearEllipticFiniteElementSolver<2 > SimpleLinearEllipticFiniteElementSolver2;
typedef SimpleLinearEllipticFiniteElementSolver<3 > SimpleLinearEllipticFiniteElementSolver3;
typedef SimpleNonLinearEllipticFiniteElementSolver<2 > SimpleNonLinearEllipticFiniteElementSolver2;
typedef SimpleNonLinearEllipticFiniteElementSolver<3 > SimpleNonLinearEllipticFiniteElementSolver3;
typedef SimpleParabolicFiniteElementSolver<2 > SimpleParabolicFiniteElementSolver2;
typedef SimpleParabolicFiniteElementSolver<3 > SimpleParabolicFiniteElementSolver3;
typedef SimpleParabolicFiniteDifferenceSolver<2 > SimpleParabolicFiniteDifferenceSolver2;
typedef SimpleParabolicFiniteDifferenceSolver<3 > SimpleParabolicFiniteDifferenceSolver3;
typedef FunctionMap<2 > FunctionMap2;
typedef FunctionMap<3 > FunctionMap3;
typedef DensityMap<2 > DensityMap2;
typedef DensityMap<3 > DensityMap3;
typedef FlowSolver<2 > FlowSolver2;
typedef FlowSolver<3 > FlowSolver3;
typedef VesselImpedanceCalculator<2 > VesselImpedanceCalculator2;
typedef VesselImpedanceCalculator<3 > VesselImpedanceCalculator3;
typedef BetteridgeHaematocritSolver<2 > BetteridgeHaematocritSolver2;
typedef BetteridgeHaematocritSolver<3 > BetteridgeHaematocritSolver3;
typedef AbstractVesselNetworkCalculator<2 > AbstractVesselNetworkCalculator2;
typedef AbstractVesselNetworkCalculator<3 > AbstractVesselNetworkCalculator3;
typedef MechanicalStimulusCalculator<2 > MechanicalStimulusCalculator2;
typedef MechanicalStimulusCalculator<3 > MechanicalStimulusCalculator3;
typedef MetabolicStimulusCalculator<2 > MetabolicStimulusCalculator2;
typedef MetabolicStimulusCalculator<3 > MetabolicStimulusCalculator3;
typedef RadiusCalculator<2 > RadiusCalculator2;
typedef RadiusCalculator<3 > RadiusCalculator3;
typedef ShrinkingStimulusCalculator<2 > ShrinkingStimulusCalculator2;
typedef ShrinkingStimulusCalculator<3 > ShrinkingStimulusCalculator3;
typedef ViscosityCalculator<2 > ViscosityCalculator2;
typedef ViscosityCalculator<3 > ViscosityCalculator3;
typedef WallShearStressCalculator<2 > WallShearStressCalculator2;
typedef WallShearStressCalculator<3 > WallShearStressCalculator3;
typedef AbstractStructuralAdaptationSolver<2 > AbstractStructuralAdaptationSolver2;
typedef AbstractStructuralAdaptationSolver<3 > AbstractStructuralAdaptationSolver3;
typedef StructuralAdaptationSolver<2 > StructuralAdaptationSolver2;
typedef StructuralAdaptationSolver<3 > StructuralAdaptationSolver3;
typedef AbstractHaematocritSolver<2 > AbstractHaematocritSolver2;
typedef AbstractHaematocritSolver<3 > AbstractHaematocritSolver3;
typedef AlarconHaematocritSolver<2 > AlarconHaematocritSolver2;
typedef AlarconHaematocritSolver<3 > AlarconHaematocritSolver3;
typedef ConstantHaematocritSolver<2 > ConstantHaematocritSolver2;
typedef ConstantHaematocritSolver<3 > ConstantHaematocritSolver3;
typedef MicrovesselSolver<2 > MicrovesselSolver2;
typedef MicrovesselSolver<3 > MicrovesselSolver3;
typedef MicrovesselSimulationModifier<2 > MicrovesselSimulationModifier2;
typedef MicrovesselSimulationModifier<3 > MicrovesselSimulationModifier3;
typedef Owen2011TrackingModifier<2 > Owen2011TrackingModifier2;
typedef Owen2011TrackingModifier<3 > Owen2011TrackingModifier3;
typedef AbstractCellBasedSimulationModifier<2,2 > AbstractCellBasedSimulationModifier2_2;
typedef AbstractCellBasedSimulationModifier<3,3 > AbstractCellBasedSimulationModifier3_3;
typedef VesselNetworkReader<2 > VesselNetworkReader2;
typedef VesselNetworkReader<3 > VesselNetworkReader3;
typedef NodeFlowProperties<2 > NodeFlowProperties2;
typedef NodeFlowProperties<3 > NodeFlowProperties3;
typedef SegmentFlowProperties<2 > SegmentFlowProperties2;
typedef SegmentFlowProperties<3 > SegmentFlowProperties3;
typedef VesselFlowProperties<2 > VesselFlowProperties2;
typedef VesselFlowProperties<3 > VesselFlowProperties3;
typedef VesselNode<2 > VesselNode2;
typedef VesselNode<2 >* VesselNode2Ptr;
typedef boost::shared_ptr<VesselNode<2 > > SharedPtrVesselNode2;
typedef std::vector<VesselNode2Ptr> VectorVesselNode2Ptr;
typedef std::vector<SharedPtrVesselNode2> VectorSharedPtrVesselNode2;
typedef VesselNode<3 > VesselNode3;
typedef VesselNode<3 >* VesselNode3Ptr;
typedef boost::shared_ptr<VesselNode<3 > > SharedPtrVesselNode3;
typedef std::vector<VesselNode3Ptr> VectorVesselNode3Ptr;
typedef std::vector<SharedPtrVesselNode3> VectorSharedPtrVesselNode3;
typedef VesselSegment<2 > VesselSegment2;
typedef VesselSegment<2 >* VesselSegment2Ptr;
typedef boost::shared_ptr<VesselSegment<2 > > SharedPtrVesselSegment2;
typedef std::vector<VesselSegment2Ptr> VectorVesselSegment2Ptr;
typedef std::vector<SharedPtrVesselSegment2> VectorSharedPtrVesselSegment2;
typedef VesselSegment<3 > VesselSegment3;
typedef VesselSegment<3 >* VesselSegment3Ptr;
typedef boost::shared_ptr<VesselSegment<3 > > SharedPtrVesselSegment3;
typedef std::vector<VesselSegment3Ptr> VectorVesselSegment3Ptr;
typedef std::vector<SharedPtrVesselSegment3> VectorSharedPtrVesselSegment3;
typedef Vessel<2 > Vessel2;
typedef Vessel<2 >* Vessel2Ptr;
typedef boost::shared_ptr<Vessel<2 > > SharedPtrVessel2;
typedef std::vector<Vessel2Ptr> VectorVessel2Ptr;
typedef std::vector<SharedPtrVessel2> VectorSharedPtrVessel2;
typedef Vessel<3 > Vessel3;
typedef Vessel<3 >* Vessel3Ptr;
typedef boost::shared_ptr<Vessel<3 > > SharedPtrVessel3;
typedef std::vector<Vessel3Ptr> VectorVessel3Ptr;
typedef std::vector<SharedPtrVessel3> VectorSharedPtrVessel3;
typedef VesselNetwork<2 > VesselNetwork2;
typedef VesselNetwork<3 > VesselNetwork3;
typedef VesselNetworkGenerator<2 > VesselNetworkGenerator2;
typedef VesselNetworkGenerator<3 > VesselNetworkGenerator3;
typedef AbstractVesselNetworkComponent<2 > AbstractVesselNetworkComponent2;
typedef AbstractVesselNetworkComponent<3 > AbstractVesselNetworkComponent3;
typedef VesselNetworkCellPopulationInteractor<2 > VesselNetworkCellPopulationInteractor2;
typedef VesselNetworkCellPopulationInteractor<3 > VesselNetworkCellPopulationInteractor3;
typedef VesselNetworkWriter<2 > VesselNetworkWriter2;
typedef VesselNetworkWriter<3 > VesselNetworkWriter3;
typedef DistanceMap<2 > DistanceMap2;
typedef DistanceMap<3 > DistanceMap3;
typedef VesselNetworkGeometryCalculator<2 > VesselNetworkGeometryCalculator2;
typedef VesselNetworkGeometryCalculator<3 > VesselNetworkGeometryCalculator3;
typedef VesselNetworkGraphCalculator<2 > VesselNetworkGraphCalculator2;
typedef VesselNetworkGraphCalculator<3 > VesselNetworkGraphCalculator3;
typedef AbstractVesselNetworkComponentProperties<2 > AbstractVesselNetworkComponentProperties2;
typedef AbstractVesselNetworkComponentProperties<3 > AbstractVesselNetworkComponentProperties3;
typedef AbstractVesselNetworkComponentFlowProperties<2 > AbstractVesselNetworkComponentFlowProperties2;
typedef AbstractVesselNetworkComponentFlowProperties<3 > AbstractVesselNetworkComponentFlowProperties3;
typedef AbstractVesselNetworkComponentChemicalProperties<2 > AbstractVesselNetworkComponentChemicalProperties2;
typedef AbstractVesselNetworkComponentChemicalProperties<3 > AbstractVesselNetworkComponentChemicalProperties3;
typedef MicrovesselVtkScene<2 > MicrovesselVtkScene2;
typedef MicrovesselVtkScene<3 > MicrovesselVtkScene3;
typedef CellPopulationActorGenerator<2 > CellPopulationActorGenerator2;
typedef CellPopulationActorGenerator<3 > CellPopulationActorGenerator3;
typedef DiscreteContinuumMeshActorGenerator<2 > DiscreteContinuumMeshActorGenerator2;
typedef DiscreteContinuumMeshActorGenerator<3 > DiscreteContinuumMeshActorGenerator3;
typedef PartActorGenerator<2 > PartActorGenerator2;
typedef PartActorGenerator<3 > PartActorGenerator3;
typedef RegularGridActorGenerator<2 > RegularGridActorGenerator2;
typedef RegularGridActorGenerator<3 > RegularGridActorGenerator3;
typedef VesselNetworkActorGenerator<2 > VesselNetworkActorGenerator2;
typedef VesselNetworkActorGenerator<3 > VesselNetworkActorGenerator3;
typedef AbstractActorGenerator<2 > AbstractActorGenerator2;
typedef AbstractActorGenerator<3 > AbstractActorGenerator3;
typedef AbstractMicrovesselModifier<2 > AbstractMicrovesselModifier2;
typedef AbstractMicrovesselModifier<3 > AbstractMicrovesselModifier3;
typedef VtkSceneMicrovesselModifier<2 > VtkSceneMicrovesselModifier2;
typedef VtkSceneMicrovesselModifier<3 > VtkSceneMicrovesselModifier3;
typedef ImageToMesh<2 > ImageToMesh2;
typedef ImageToMesh<3 > ImageToMesh3;
typedef NetworkToImage<2 > NetworkToImage2;
typedef NetworkToImage<3 > NetworkToImage3;
typedef units::quantity<unit:: number_density,double> NumberDensityQuantity;
typedef std::vector<units::quantity<unit:: number_density,double> > VectorNumberDensityQuantity;
typedef ParameterInstance<unit:: number_density> ParameterInstanceNumberDensityQuantity;
typedef units::quantity<unit:: rate,double> RateQuantity;
typedef std::vector<units::quantity<unit:: rate,double> > VectorRateQuantity;
typedef ParameterInstance<unit:: rate> ParameterInstanceRateQuantity;
typedef units::quantity<unit:: per_area,double> PerAreaQuantity;
typedef std::vector<units::quantity<unit:: per_area,double> > VectorPerAreaQuantity;
typedef ParameterInstance<unit:: per_area> ParameterInstancePerAreaQuantity;
typedef units::quantity<unit:: force,double> ForceQuantity;
typedef std::vector<units::quantity<unit:: force,double> > VectorForceQuantity;
typedef ParameterInstance<unit:: force> ParameterInstanceForceQuantity;
typedef units::quantity<unit:: concentration,double> ConcentrationQuantity;
typedef std::vector<units::quantity<unit:: concentration,double> > VectorConcentrationQuantity;
typedef ParameterInstance<unit:: concentration> ParameterInstanceConcentrationQuantity;
typedef units::quantity<unit:: volumetric_solubility,double> VolumetricSolubilityQuantity;
typedef std::vector<units::quantity<unit:: volumetric_solubility,double> > VectorVolumetricSolubilityQuantity;
typedef ParameterInstance<unit:: volumetric_solubility> ParameterInstanceVolumetricSolubilityQuantity;
typedef units::quantity<unit:: flow_impedance,double> FlowImpedanceQuantity;
typedef std::vector<units::quantity<unit:: flow_impedance,double> > VectorFlowImpedanceQuantity;
typedef ParameterInstance<unit:: flow_impedance> ParameterInstanceFlowImpedanceQuantity;
typedef units::quantity<unit:: concentration_flow_rate,double> ConcentrationFlowRateQuantity;
typedef std::vector<units::quantity<unit:: concentration_flow_rate,double> > VectorConcentrationFlowRateQuantity;
typedef ParameterInstance<unit:: concentration_flow_rate> ParameterInstanceConcentrationFlowRateQuantity;
typedef units::quantity<unit:: area,double> AreaQuantity;
typedef std::vector<units::quantity<unit:: area,double> > VectorAreaQuantity;
typedef ParameterInstance<unit:: area> ParameterInstanceAreaQuantity;
typedef units::quantity<unit:: concentration_gradient,double> ConcentrationGradientQuantity;
typedef std::vector<units::quantity<unit:: concentration_gradient,double> > VectorConcentrationGradientQuantity;
typedef ParameterInstance<unit:: concentration_gradient> ParameterInstanceConcentrationGradientQuantity;
typedef units::quantity<unit:: membrane_permeability,double> MembranePermeabilityQuantity;
typedef std::vector<units::quantity<unit:: membrane_permeability,double> > VectorMembranePermeabilityQuantity;
typedef ParameterInstance<unit:: membrane_permeability> ParameterInstanceMembranePermeabilityQuantity;
typedef units::quantity<unit:: molar_flux,double> MolarFluxQuantity;
typedef std::vector<units::quantity<unit:: molar_flux,double> > VectorMolarFluxQuantity;
typedef ParameterInstance<unit:: molar_flux> ParameterInstanceMolarFluxQuantity;
typedef units::quantity<unit:: concentration_flux,double> ConcentrationFluxQuantity;
typedef std::vector<units::quantity<unit:: concentration_flux,double> > VectorConcentrationFluxQuantity;
typedef ParameterInstance<unit:: concentration_flux> ParameterInstanceConcentrationFluxQuantity;
typedef units::quantity<unit:: diffusivity,double> DiffusivityQuantity;
typedef std::vector<units::quantity<unit:: diffusivity,double> > VectorDiffusivityQuantity;
typedef ParameterInstance<unit:: diffusivity> ParameterInstanceDiffusivityQuantity;
typedef units::quantity<unit:: dimensionless,double> DimensionlessQuantity;
typedef std::vector<units::quantity<unit:: dimensionless,double> > VectorDimensionlessQuantity;
typedef ParameterInstance<unit:: dimensionless> ParameterInstanceDimensionlessQuantity;
typedef units::quantity<unit:: solubility,double> SolubilityQuantity;
typedef std::vector<units::quantity<unit:: solubility,double> > VectorSolubilityQuantity;
typedef ParameterInstance<unit:: solubility> ParameterInstanceSolubilityQuantity;
typedef units::quantity<unit:: volume,double> VolumeQuantity;
typedef std::vector<units::quantity<unit:: volume,double> > VectorVolumeQuantity;
typedef ParameterInstance<unit:: volume> ParameterInstanceVolumeQuantity;
typedef units::quantity<unit:: pressure,double> PressureQuantity;
typedef std::vector<units::quantity<unit:: pressure,double> > VectorPressureQuantity;
typedef ParameterInstance<unit:: pressure> ParameterInstancePressureQuantity;
typedef units::quantity<unit:: diffusivity_per_concentration,double> DiffusivityPerConcentrationQuantity;
typedef std::vector<units::quantity<unit:: diffusivity_per_concentration,double> > VectorDiffusivityPerConcentrationQuantity;
typedef ParameterInstance<unit:: diffusivity_per_concentration> ParameterInstanceDiffusivityPerConcentrationQuantity;
typedef units::quantity<unit:: flow_rate,double> FlowRateQuantity;
typedef std::vector<units::quantity<unit:: flow_rate,double> > VectorFlowRateQuantity;
typedef ParameterInstance<unit:: flow_rate> ParameterInstanceFlowRateQuantity;
typedef units::quantity<unit:: dynamic_viscosity,double> DynamicViscosityQuantity;
typedef std::vector<units::quantity<unit:: dynamic_viscosity,double> > VectorDynamicViscosityQuantity;
typedef ParameterInstance<unit:: dynamic_viscosity> ParameterInstanceDynamicViscosityQuantity;
typedef units::quantity<unit:: molar_flow_rate,double> MolarFlowRateQuantity;
typedef std::vector<units::quantity<unit:: molar_flow_rate,double> > VectorMolarFlowRateQuantity;
typedef ParameterInstance<unit:: molar_flow_rate> ParameterInstanceMolarFlowRateQuantity;
typedef units::quantity<unit:: molar_mass,double> MolarMassQuantity;
typedef std::vector<units::quantity<unit:: molar_mass,double> > VectorMolarMassQuantity;
typedef ParameterInstance<unit:: molar_mass> ParameterInstanceMolarMassQuantity;
typedef units::quantity<unit:: length,double> LengthQuantity;
typedef std::vector<units::quantity<unit:: length,double> > VectorLengthQuantity;
typedef ParameterInstance<unit:: length> ParameterInstanceLengthQuantity;
typedef units::quantity<unit:: mass,double> MassQuantity;
typedef std::vector<units::quantity<unit:: mass,double> > VectorMassQuantity;
typedef ParameterInstance<unit:: mass> ParameterInstanceMassQuantity;
typedef units::quantity<unit:: per_length,double> PerLengthQuantity;
typedef std::vector<units::quantity<unit:: per_length,double> > VectorPerLengthQuantity;
typedef ParameterInstance<unit:: per_length> ParameterInstancePerLengthQuantity;
typedef units::quantity<unit:: time,double> TimeQuantity;
typedef std::vector<units::quantity<unit:: time,double> > VectorTimeQuantity;
typedef ParameterInstance<unit:: time> ParameterInstanceTimeQuantity;
typedef units::quantity<unit:: velocity,double> VelocityQuantity;
typedef std::vector<units::quantity<unit:: velocity,double> > VectorVelocityQuantity;
typedef ParameterInstance<unit:: velocity> ParameterInstanceVelocityQuantity;
typedef units::quantity<unit:: rate_per_concentration,double> RatePerConcentrationQuantity;
typedef std::vector<units::quantity<unit:: rate_per_concentration,double> > VectorRatePerConcentrationQuantity;
typedef ParameterInstance<unit:: rate_per_concentration> ParameterInstanceRatePerConcentrationQuantity;
    }
}

// Need to specifically instantiate PETSc Vec and Mat 
typedef boost::filesystem::path boost_filesystem_path;
typedef std::map<unsigned, units::quantity<unit::concentration_flow_rate> > MapUnsigned_ConcentrationFlowRate;
typedef std::map<unsigned, units::quantity<unit::concentration> > MapUnsigned_Concentration;
typedef std::map<std::string, std::vector<units::quantity<unit::concentration> > > MapString_VectorConcentration;

 inline int Instantiation()
{
return sizeof(Mat) + sizeof(Vec)
+ sizeof(pyplusplus::aliases::NumberDensityQuantity)
+ sizeof(ParameterInstance<unit:: number_density>)
+ sizeof(pyplusplus::aliases::RateQuantity)
+ sizeof(ParameterInstance<unit:: rate>)
+ sizeof(pyplusplus::aliases::PerAreaQuantity)
+ sizeof(ParameterInstance<unit:: per_area>)
+ sizeof(pyplusplus::aliases::ForceQuantity)
+ sizeof(ParameterInstance<unit:: force>)
+ sizeof(pyplusplus::aliases::ConcentrationQuantity)
+ sizeof(ParameterInstance<unit:: concentration>)
+ sizeof(pyplusplus::aliases::VolumetricSolubilityQuantity)
+ sizeof(ParameterInstance<unit:: volumetric_solubility>)
+ sizeof(pyplusplus::aliases::FlowImpedanceQuantity)
+ sizeof(ParameterInstance<unit:: flow_impedance>)
+ sizeof(pyplusplus::aliases::ConcentrationFlowRateQuantity)
+ sizeof(ParameterInstance<unit:: concentration_flow_rate>)
+ sizeof(pyplusplus::aliases::AreaQuantity)
+ sizeof(ParameterInstance<unit:: area>)
+ sizeof(pyplusplus::aliases::ConcentrationGradientQuantity)
+ sizeof(ParameterInstance<unit:: concentration_gradient>)
+ sizeof(pyplusplus::aliases::MembranePermeabilityQuantity)
+ sizeof(ParameterInstance<unit:: membrane_permeability>)
+ sizeof(pyplusplus::aliases::MolarFluxQuantity)
+ sizeof(ParameterInstance<unit:: molar_flux>)
+ sizeof(pyplusplus::aliases::ConcentrationFluxQuantity)
+ sizeof(ParameterInstance<unit:: concentration_flux>)
+ sizeof(pyplusplus::aliases::DiffusivityQuantity)
+ sizeof(ParameterInstance<unit:: diffusivity>)
+ sizeof(pyplusplus::aliases::DimensionlessQuantity)
+ sizeof(ParameterInstance<unit:: dimensionless>)
+ sizeof(pyplusplus::aliases::SolubilityQuantity)
+ sizeof(ParameterInstance<unit:: solubility>)
+ sizeof(pyplusplus::aliases::VolumeQuantity)
+ sizeof(ParameterInstance<unit:: volume>)
+ sizeof(pyplusplus::aliases::PressureQuantity)
+ sizeof(ParameterInstance<unit:: pressure>)
+ sizeof(pyplusplus::aliases::DiffusivityPerConcentrationQuantity)
+ sizeof(ParameterInstance<unit:: diffusivity_per_concentration>)
+ sizeof(pyplusplus::aliases::FlowRateQuantity)
+ sizeof(ParameterInstance<unit:: flow_rate>)
+ sizeof(pyplusplus::aliases::DynamicViscosityQuantity)
+ sizeof(ParameterInstance<unit:: dynamic_viscosity>)
+ sizeof(pyplusplus::aliases::MolarFlowRateQuantity)
+ sizeof(ParameterInstance<unit:: molar_flow_rate>)
+ sizeof(pyplusplus::aliases::MolarMassQuantity)
+ sizeof(ParameterInstance<unit:: molar_mass>)
+ sizeof(pyplusplus::aliases::LengthQuantity)
+ sizeof(ParameterInstance<unit:: length>)
+ sizeof(pyplusplus::aliases::MassQuantity)
+ sizeof(ParameterInstance<unit:: mass>)
+ sizeof(pyplusplus::aliases::PerLengthQuantity)
+ sizeof(ParameterInstance<unit:: per_length>)
+ sizeof(pyplusplus::aliases::TimeQuantity)
+ sizeof(ParameterInstance<unit:: time>)
+ sizeof(pyplusplus::aliases::VelocityQuantity)
+ sizeof(ParameterInstance<unit:: velocity>)
+ sizeof(pyplusplus::aliases::RatePerConcentrationQuantity)
+ sizeof(ParameterInstance<unit:: rate_per_concentration>)
;
}

 inline Mat GetPetscMatForWrapper()
{
Mat A;
PetscTools::SetupMat(A, 3, 3, 3);
return A;
}
#endif // MICROVESSELCHASTEHEADERS_HPP_
