"""
This file contains a list of classes that are to be wrapped. 

Each class includes metadata such as template arguments, excluded methods,
methods with special pointer management requirements, and any special
declaration code needed for wrapping. A minimal, and typical, case is just to
specify the class name.
"""

from wrapper_utilities.class_data import CppClass

################################## GEOMETRY ##########################################

geometry_classes = [CppClass('Polygon', include_vec_ptr_self=True),
                    CppClass('Facet', include_vec_ptr_self=True),
                    CppClass('Part'),
                    CppClass('MappableGridGenerator'), CppClass('NetworkToSurface'),
                    CppClass('VesselSurfaceGenerator'), CppClass('BoundaryExtractor'),
                    CppClass('SurfaceCleaner'), 
                    CppClass('GeometryFormat', component = "geometry", needs_include_file = False),
                    CppClass('GeometryWriter'),      
                    CppClass('std::vector', skip_wrapping=True, needs_include_file=False, needs_instantiation = False,
                          template_args=[["std::pair<DimensionalChastePoint<2>, int>"],
                                         ["std::pair<DimensionalChastePoint<3>, int>"],
                                         ["std::pair<DimensionalChastePoint<3>, unsigned>"],
                                         ["std::pair<DimensionalChastePoint<3>, unsigned>"],
                                         ["std::pair<std::pair<int, int>, int>"],
                                         ["std::map<std::string, double>"]],)            
                    #CppClass('std::vector', skip_wrapping=True, needs_include_file=False, needs_instantiation = False,
                          #template_args=[["units::quantity<unit::length>"]
                                         #]),
                                         ]

################################## MESH ##########################################

mesh_classes = [
                CppClass('AbstractMesh', component="mesh"),
                CppClass('AbstractTetrahedralMesh', component="mesh"),
                CppClass('TetrahedralMesh', component="mesh", excluded_methods = ["FreeTriangulateIo", "InitialiseTriangulateIo"]),
                CppClass('AbstractDiscreteContinuumGrid'),
                CppClass('DimensionalChastePoint', include_vec_ptr_self=True),
                CppClass('DiscreteContinuumMesh'),
                CppClass('DiscreteContinuumMeshGenerator'),
                CppClass('GridCalculator'), 
                CppClass('MeshReader'),
                CppClass('MultiFormatMeshWriter'), CppClass('RegularGrid'),
                CppClass('RegularGridWriter'), CppClass('MeshFormat', needs_include_file = False),
                CppClass('std::vector', skip_wrapping=True, needs_include_file=False, needs_instantiation = False,
                          template_args=[["std::vector<boost::shared_ptr<VesselNode<2> > >"],
                                         ["std::vector<boost::shared_ptr<VesselNode<3> > >"],
                                         ["std::vector<Node<3>* >"],
                                         ["std::vector<Node<2>* >"],
                                         ["Node<3>*"],
                                         ["Node<2>*"],
                                         ["std::vector<boost::shared_ptr<VesselSegment<2> > >"],
                                         ["std::vector<boost::shared_ptr<VesselSegment<3> > >"],
                                         ["std::vector<boost::shared_ptr<Cell> >"],
                                         ["DimensionalChastePoint2"], 
                                         ["DimensionalChastePoint3"],]),
                CppClass('std::pair', skip_wrapping=True, needs_include_file=False, needs_instantiation = False,
                          template_args=[["DimensionalChastePoint2, DimensionalChastePoint2"], 
                                         ["DimensionalChastePoint3, DimensionalChastePoint3"],])]

################################## CELL ##########################################

cell_classes = [CppClass('AbstractCellKiller'), CppClass('AbstractCellMutationState'),
                CppClass('CancerCellMutationState'), CppClass('QuiescentCancerCellMutationState'),
                CppClass('StalkCellMutationState'), CppClass('TipCellMutationState'),
                CppClass('VesselCellMutationState'), CppClass('MacrophageMutationState'),
                CppClass('Owen2011OxygenBasedCellCycleModel'), CppClass('Owen11CellPopulationGenerator'),
                CppClass('CaBasedCellPopulation'), 
                CppClass('LQRadiotherapyCellKiller'),
                CppClass('Owen11CaBasedDivisionRule'), CppClass('Owen11CaUpdateRule')]

################################## ANGIOGENESIS ##########################################

angiogenesis_classes = [CppClass('TipAttractionLatticeBasedMigrationRule'), CppClass('AngiogenesisSolver'),
                        CppClass('Owen2011MigrationRule'), CppClass('Owen2011SproutingRule'),
                        CppClass('OffLatticeMigrationRule'), CppClass('OffLatticeSproutingRule'),
                        CppClass('AbstractMigrationRule'), CppClass('AbstractSproutingRule'),
                        CppClass('LatticeBasedMigrationRule'), CppClass('CellPopulationMigrationRule'),
                        CppClass('RegressionSolver'),CppClass('WallShearStressBasedRegressionSolver'),
                        CppClass('LatticeBasedSproutingRule'),]

################################## FLOW ##########################################

flow_classes = [CppClass('FlowSolver'), CppClass('WallShearStressCalculator'),
                CppClass('VesselImpedanceCalculator'), CppClass('BetteridgeHaematocritSolver'),
                CppClass('AbstractVesselNetworkCalculator'), CppClass('MechanicalStimulusCalculator'),
                CppClass('MetabolicStimulusCalculator'), CppClass('RadiusCalculator'),
                CppClass('ShrinkingStimulusCalculator'), CppClass('ViscosityCalculator'),
                CppClass('WallShearStressCalculator'),CppClass('AbstractStructuralAdaptationSolver'),
                CppClass('StructuralAdaptationSolver'),CppClass('AbstractHaematocritSolver'),
                CppClass('AlarconHaematocritSolver'),CppClass('ConstantHaematocritSolver')]

################################## PDE ##########################################

pde_classes = [CppClass('DiscreteContinuumBoundaryCondition'),
               CppClass('BoundaryConditionType', needs_include_file=False, component = "pde"),
               CppClass('BoundaryConditionSource', needs_include_file=False, component = "pde"),
               CppClass('DiscreteSource', include_vec_ptr_self=True), 
               CppClass('CellStateDependentDiscreteSource'),
               CppClass('CellBasedDiscreteSource'), 
               CppClass('SolutionDependentDiscreteSource'),
               CppClass('VesselBasedDiscreteSource'), CppClass('AbstractDiscreteContinuumPde'),
               CppClass('AbstractDiscreteContinuumLinearEllipticPde'), CppClass('DiscreteContinuumLinearEllipticPde'),
               CppClass('AbstractDiscreteContinuumNonLinearEllipticPde'),CppClass('MichaelisMentenSteadyStateDiffusionReactionPde'),
               CppClass('AbstractDiscreteContinuumParabolicPde'),CppClass('ParabolicDiffusionReactionPde'),
               CppClass('CoupledVegfPelletDiffusionReactionPde'),
               CppClass('AbstractDiscreteContinuumSolver', include_vec_ptr_self=True),
               CppClass('AbstractRegularGridDiscreteContinuumSolver'),
               CppClass('AbstractUnstructuredGridDiscreteContinuumSolver'),
               CppClass('AbstractMixedGridDiscreteContinuumSolver'), 
               CppClass('AbstractFiniteDifferenceSolverBase'),
               CppClass('CoupledLumpedSystemFiniteDifferenceSolver'),
               CppClass('SimpleLinearEllipticFiniteDifferenceSolver'),
               CppClass('SimpleNonLinearEllipticFiniteDifferenceSolver'),
               CppClass('AbstractFiniteElementSolverBase'),
               CppClass('CoupledLumpedSystemFiniteElementSolver'),
               CppClass('SimpleLinearEllipticFiniteElementSolver'),
               CppClass('SimpleNonLinearEllipticFiniteElementSolver'),
               CppClass('SimpleParabolicFiniteElementSolver'),
               CppClass('SimpleParabolicFiniteDifferenceSolver'),
               #CppClass('AbstractGreensFunctionSolverBase'),
               #CppClass('SimpleLinearEllipticGreensFunctionSolver'),
               CppClass('FunctionMap'),
               CppClass('DensityMap')]

################################## SIMUALTION ##########################################

simulation_classes = [CppClass('MicrovesselSolver'),
                      CppClass('MicrovesselSimulationModifier'),
                      CppClass('AbstractMicrovesselModifier'),
                      CppClass('VtkSceneMicrovesselModifier'),
                      CppClass('Owen2011TrackingModifier'),
                      CppClass('AbstractCellBasedSimulationModifier'),
                      CppClass('CornealMicropocketSimulation'),
                      CppClass('DomainType',needs_include_file=False, component = "simulation")]

################################## VESSEL ##########################################

vessel_classes = [CppClass('VesselNetworkReader'),CppClass('NodeFlowProperties'),
                  CppClass('SegmentFlowProperties'),CppClass('VesselFlowProperties'),
                  CppClass('VesselNode', include_vec_ptr_self=True),
                  CppClass('VesselSegment', include_vec_ptr_self=True),
                  CppClass('Vessel', include_vec_ptr_self=True),
                  CppClass('VesselNetwork'),
                  CppClass('VesselNetworkVtkConverter'),
                  CppClass('VesselNetworkPropertyManager'),
                  CppClass('VesselNetworkPartitioner'),                  
                  CppClass('VesselNetworkGenerator'),CppClass('AbstractVesselNetworkComponent'),
                  CppClass('VesselNetworkCellPopulationInteractor'),CppClass('VesselNetworkWriter'),
                  CppClass('DistanceMap'),
                  CppClass('VesselNetworkGeometryCalculator'),CppClass('VesselNetworkGraphCalculator'),
                  CppClass('AbstractVesselNetworkComponentProperties'),CppClass('AbstractVesselNetworkComponentFlowProperties'),
                  CppClass('AbstractVesselNetworkComponentChemicalProperties')]

################################## VISUALIZATION ##########################################

visualization_classes = [CppClass('MicrovesselVtkScene'),
                  CppClass('CellPopulationActorGenerator'),CppClass('DiscreteContinuumMeshActorGenerator'),
                  CppClass('PartActorGenerator'),CppClass('RegularGridActorGenerator'),
                  CppClass('VesselNetworkActorGenerator'),CppClass('AbstractActorGenerator'),
                  CppClass('AbstractMicrovesselModifier'),CppClass('VtkSceneMicrovesselModifier'),]

################################## IMAGE ##########################################
image_classes = [CppClass('ImageReader'), 
                 CppClass('ImageToMesh'),
                 CppClass('ImageToSurface'),
                 CppClass('NetworkToImage')]

################################## UTILITY ##########################################

units = {"membrane_permeability": "metre_per_second", 
             "volumetric_solubility": "per_pascal",
             "solubility": "mole_per_metre_cubed_per_second",
             "diffusivity_per_concentration": "metre_pow5_per_second_per_mole",
             "diffusivity": "metre_squared_per_second",
             "flow_impedance": "pascal_second_per_metre_cubed",
             "flow_rate": "metre_cubed_per_second",
             "dynamic_viscosity": "poiseuille",
             "pressure": "pascal",
             "force": "newton",
             "velocity": "metre_per_second",
             "number_density": "per_metre_cubed",
             "molar_mass": "mole_per_kg",
             "rate_per_concentration": "metre_cubed_per_mole_per_second",
             "concentration_gradient": "mole_per_metre_pow4",
             "concentration_flux": "mole_per_metre_pow5_per_second",
             "concentration_flow_rate": "mole_per_metre_cubed_per_second",
             "concentration": "mole_per_metre_cubed",
             "molar_flux": "mole_per_metre_squared_per_second",
             "molar_flow_rate": "mole_per_second",
             "mass": "kg",
             "per_area": "per_metre_squared",
             "per_length": "per_metre",
             "volume": "metre_cubed",
             "area": "metre_squared",
             "length": "metre",
             "rate":"per_second",
             "time":"second", 
             "dimensionless" : "dimensionless",       
             }

utility_classes = [CppClass('ParameterInstance', skip_wrapping=True, needs_instantiation = False), # needs custom code
                   CppClass('BaseUnits'),
                   CppClass('UnitCollection', skip_wrapping=True, needs_instantiation = False),
                   CppClass('ParameterCollection'),
                   CppClass('BaseParameterInstance'),
                   CppClass('Owen11Parameters'),
                   CppClass('Connor17Parameters'),
                   CppClass('Secomb04Parameters'),
                   CppClass('GenericParameters'),]

extra_classes = [
                 CppClass("std::string", "extra", skip_wrapping=True, needs_include_file=False, needs_instantiation = False),
                 CppClass("std::map","extra",  [["std::string", "std::string"],
                                                ["int", "std::string"],
                                                ["std::string, double"]], skip_wrapping=True, needs_include_file=False, needs_instantiation = False),
                 CppClass("std::set","extra",  [["unsigned"]], skip_wrapping=True, needs_include_file=False, needs_instantiation = False),
                 CppClass("std::pair","extra", [["unsigned int, unsigned int"],
                                                ["std::vector<double>, double"]], skip_wrapping=True, needs_include_file=False, needs_instantiation = False),
                 CppClass("std::vector","extra", [["double"], 
                                          ["unsigned"], 
                                          ["int"], 
                                          ["bool"],
                                          ["std::string"],
                                          ["c_vector<double,3>"],
                                          ["c_vector<double,2>"],
                                          ["c_vector<unsigned,5>"],
                                          ["std::set<unsigned int>"],
                                          ["std::vector<unsigned int>"],
                                          ["std::pair<unsigned int, unsigned int>"],
                                          ["std::pair<std::vector<double>, double>"]], 
                          skip_wrapping=True, needs_include_file=False, needs_instantiation = False),
                 CppClass("c_vector","extra", [["double", 2], ["double", 3], ["unsigned", 5]], 
                          skip_wrapping=True, needs_include_file=False, needs_instantiation = False)]

# parameter_instance_class = utility_classes[0]
# parameter_instance_class.template_args=[]
# for eachUnit in units.keys():
#     print eachUnit
#     parameter_instance_class.template_args.append("unit::"+eachUnit)

# This final list will be pulled in by the autowrapper code by name...i.e. don't change the name.
classes = extra_classes + geometry_classes + mesh_classes + cell_classes + angiogenesis_classes + pde_classes 
classes += flow_classes + simulation_classes + vessel_classes + visualization_classes + utility_classes
classes += image_classes