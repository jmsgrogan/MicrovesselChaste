---
layout: page-full-width 
title: Test Blood Flow Tutorial
---
This tutorial is automatically generated from the file test/tutorials//TestBloodFlowTutorial.hpp.
Note that the code is given in full at the bottom of the page.



# Modelling Blood Flow Tutorial
This tutorial demonstrates functionality for modelling blood flow, structural adaptation and vessel
regression in a vessel network.

This tutorial covers:
 * Managing parameter values
 * Running a minimal Poiseuille flow simulation and looking at results
 * Adding haematocrit
 * Adding structural adaptation in response to flow
 * Adding vessel regression in low flow regions
 
# The Test
Start by introducing the necessary header files, explained in previous tutorials.

```cpp
#include <vector>
#include <cxxtest/TestSuite.h>
#include "Owen11Parameters.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
```

A container for flow information (boundary conditions, pressure values) for nodes.

```cpp
#include "NodeFlowProperties.hpp"
```

A collection of useful literature parameter values and a way to dump values to
file after use.

```cpp
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
```

The flow and haematocrit solvers, along with neccessary calculators.

```cpp
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
```

The vessel regression solver and a generic solver to collect all the
flow solvers.

```cpp
#include "WallShearStressBasedRegressionSolver.hpp"
#include "MicrovesselSolver.hpp"
```

Keep this last.

```cpp
#include "PetscSetupAndFinalize.hpp"

class TestBloodFlowLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:
```

## Test 1 - Simulating 1d Flow in a Bifurcating Network

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/bifurcation_network_flow.png)

In the first test we will simulate blood flow in a simple bifurcating vessel network. Subsequent tests will add detail in the form of
more complex networks, structural adaptation and vessel regression.

```cpp
    void TestSimpleFlowProblem() throw (Exception)
    {
```

We will work in microns

```cpp
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
```

First make the network using a generator. Start with a simple unit.

```cpp
        QLength vessel_length(100.0*unit::microns);
        DimensionalChastePoint<2> start_point(0.0, 0.0);
        VesselNetworkGenerator<2> network_generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateBifurcationUnit(vessel_length, start_point);
```

Next, pattern it to make a larger network

```cpp
        std::vector<unsigned> num_units_per_direction;
        num_units_per_direction.push_back(2);
        num_units_per_direction.push_back(0);
        network_generator.PatternUnitByTranslation(p_network, num_units_per_direction);
```

Specify which nodes will be the inlets and outlets of the network for the flow problem. This information, as well
as all other info related to the flow problem, is contained in a `NodeFlowProperties` instance. Also, set the inlet and
outlet pressures in Pa or mmHg. We can mix and match these thanks to the dimensional analysis functionality. Because
the network is simple, we can figure out which node is which just from their index. For more complicated networks
we need to use spatial locators and `NearestNode` type methods.

```cpp
        QPressure inlet_pressure(50.0 * unit::mmHg);
        p_network->GetNode(0)->GetFlowProperties()->SetIsInputNode(true);
        p_network->GetNode(0)->GetFlowProperties()->SetPressure(inlet_pressure);
```

It would be useful if we had a record of which parameter values we have used in the simulation and where they are sourced in the
literature. Instead of manually entering parameter values like above we can use a `ParameterCollection` singleton which allows for
some extra metadata storage. We will take some parameter values from a paper by Owen et al. (2011).

```cpp
        p_network->GetNode(p_network->GetNumberOfNodes()-1)->GetFlowProperties()->SetIsOutputNode(true);
        p_network->GetNode(p_network->GetNumberOfNodes()-1)->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
```

Now set the segment radii and viscosity values.

```cpp
        QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
```

We use a calculator to work out the impedance of each vessel based on assumptions of Poiseuille flow and cylindrical vessels. This
updates the value of the impedance in the vessel.

```cpp
        VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
```

Check that the impedance is as expected in one of the vessels

```cpp
        QFlowImpedance expected_impedance = 8.0 * viscosity* vessel_length/(M_PI*Qpow4(vessel_radius));
        TS_ASSERT_DELTA(p_network->GetVessel(0)->GetSegment(0)->GetFlowProperties()->GetImpedance().value(), expected_impedance.value(), 1.e-6);
```

Now we can solve for the flow rates in each vessel based on the inlet and outlet pressures and impedances. The solver
updates the value of pressures and flow rates in each vessel and node in the network.

```cpp
        FlowSolver<2> flow_solver = FlowSolver<2>();
        flow_solver.SetVesselNetwork(p_network);
        flow_solver.Solve();
```

Check the pressure, it is expected to drop linearly so should be the average of the input and output half way along the network.

```cpp
        QPressure expected_pressure = (inlet_pressure + Owen11Parameters::mpOutletPressure->GetValue())/2.0;
        TS_ASSERT_DELTA(p_network->GetNode(7)->GetFlowProperties()->GetPressure().value(), expected_pressure.value(), 1.e-6);
```

Next we write out the network, including updated flow data, to file.

```cpp
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestSimpleFlowProblem"));
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "bifurcating_network_results.vtp");
```

Now we can visualize the results in Paraview. To view the network import the file
`TestBloodFlowLiteratePaper\bifurcating_network.vtp` into Paraview. For a nicer rendering you can do `Filters->Alphabetical->Tube`.
Finally, dump our parameter collection to an xml file and, importantly, clear it for the next test.

```cpp
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath() + "parameter_collection.xml");
        ParameterCollection::Instance()->Destroy();
        BaseUnits::Instance()->Destroy();
    }
```

## Test 2 - Simulating Haematocrit Transport in 3

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/haematocrit.png)

In this test we will simulate haematocrit transport in a 3d vessel network.

```cpp
    void TestFlowProblemWithHaematocrit() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemWithHaematocrit", false));
```

This time we solve a flow problem and then use the solution to calculate the haematocrit distribution,
assuming it has no effect on the flow. Set up the network as before

```cpp
        QLength cell_width(25.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(cell_width);
        QLength target_width = 100.0 * cell_width;
        QLength target_height = 30.0 * cell_width;
        QLength vessel_length = 4.0 * cell_width;
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);
```

We will use a locator to mark the bottom left and top right nodes as respective inlets and outlets

```cpp
        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, cell_width);
        DimensionalChastePoint<3> outlet_locator(target_width/cell_width, target_height/cell_width, 0.0, cell_width);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());

```
Now set the segment radii and viscosity values.

```cpp
        QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
```

Next some simple extra functionality is demonstrated by mapping the network onto a hemisphere

```cpp
        QLength sphere_radius = 400.0 * cell_width;
        QLength sphere_thickess = 1.0 * cell_width;
        double sphere_azimuth = M_PI;
        double sphere_polar = M_PI/2.0;
        network_generator.MapToSphere(p_network, sphere_radius, sphere_thickess, sphere_azimuth, sphere_polar);
```

Get the impedance

```cpp
        VesselImpedanceCalculator<3> impedance_calculator = VesselImpedanceCalculator<3>();
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
```

Solve the flow problem

```cpp
        FlowSolver<3> flow_solver = FlowSolver<3>();
        flow_solver.SetVesselNetwork(p_network);
        flow_solver.Solve();
```

Solve the haematocrit problem

```cpp
        AlarconHaematocritSolver<3> haematocrit_solver = AlarconHaematocritSolver<3>();
        haematocrit_solver.SetVesselNetwork(p_network);
        haematocrit_solver.SetHaematocrit(0.45);
        haematocrit_solver.Calculate();
```

Next we write out the network, including updated flow data, to file.

```cpp
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "network_haematocrit.vtp");
```

Now we can visualize the results in Paraview. To view the network import the file
`TestBloodFlowLiteratePaper\bifurcating_network.vtp` into Paraview. For a nicer rendering you can do `Filters->Alphabetical->Tube`.

```cpp
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath() + "parameter_collection.xml");
        ParameterCollection::Instance()->Destroy();
    }
```

## Test 3 - Simulating Flow With Structural Adaptation

![Off Lattice Angiogenesis Image](https://github.com/jmsgrogan/MicrovesselChaste/raw/master/test/tutorials/images/structural_adaptation.png)

In this test the vessel network will adapt over time as a result of flow conditions.

```cpp
    void TestFlowProblemStucturalAdaptation() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemStucturalAdaptation", false));
```

We will work in microns

```cpp
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
```

Set up a hexagonal vessel network

```cpp
        QLength target_width(8000*unit::microns);
        QLength target_height(2000*unit::microns);
        QLength vessel_length(300.0*unit::microns);
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);
```

We will use a locator to mark the bottom left and top right nodes as respective inlets and outlets as before.

```cpp
        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, reference_length);
        DimensionalChastePoint<3> outlet_locator(target_width/reference_length, target_height/reference_length, 0.0, reference_length);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
```

Set the radius and viscosity and write the initial network to file.

```cpp
        QLength vessel_radius(40.0*unit::microns);
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"initial_network.vtp");
```

Set up the timer and time scale. The structural adaptation alrogithm works by calcualting growth or shrinkage stimuli
from several flow derived sources. We can specify how the network adapts as a function of flow by adding a collection
of calculators for wall shear stress, haematocrit etc.

```cpp
        BaseUnits::Instance()->SetReferenceTimeScale(60.0*unit::seconds);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();
```

Set up the structural adaptation solver, which manages iterations over a flow solve and executes each calculator in the order it has been added.

```cpp
        StructuralAdaptationSolver<3> structural_adaptation_solver;
        structural_adaptation_solver.SetVesselNetwork(p_network);
        structural_adaptation_solver.SetWriteOutput(true);
        structural_adaptation_solver.SetOutputFileName(p_handler->GetOutputDirectoryFullPath()+"adaptation_data.dat");
        structural_adaptation_solver.SetTolerance(0.0001);
        structural_adaptation_solver.SetMaxIterations(10);
        structural_adaptation_solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_haematocrit_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_wss_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        structural_adaptation_solver.SetTimeIncrement(0.01 * unit::seconds);
```

Do the solve and write the network to file.

```cpp
        structural_adaptation_solver.Solve();
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"network_initial_sa.vtp");
    }
```

# Test 4 - Simulating Flow With Regression

In this test the vessel network will adapt over time as a result of flow conditions and also vessels will be removed
to regression in low wall shear stress regions.

```cpp
    void TestFlowProblemStucturalAdaptationWithRegression() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemStucturalAdaptationWithRegression", false));
```

Set up the problem as before.

```cpp
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        QLength target_width(8000*unit::microns);
        QLength target_height(2000*unit::microns);
        QLength vessel_length(300.0*unit::microns);
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);

        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, reference_length);
        DimensionalChastePoint<3> outlet_locator(target_width/reference_length, target_height/reference_length, 0.0, reference_length);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
        QLength vessel_radius(40.0*unit::microns);
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"initial_network.vtp");

        BaseUnits::Instance()->SetReferenceTimeScale(60.0*unit::seconds);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();

        boost::shared_ptr<StructuralAdaptationSolver<3> > p_structural_adaptation_solver = StructuralAdaptationSolver<3>::Create();
        p_structural_adaptation_solver->SetVesselNetwork(p_network);
        p_structural_adaptation_solver->SetWriteOutput(true);
        p_structural_adaptation_solver->SetOutputFileName(p_handler->GetOutputDirectoryFullPath()+"adaptation_data.dat");
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(10);
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_wss_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        p_structural_adaptation_solver->SetTimeIncrement(0.01 * unit::seconds);
```

Set up a regression solver

```cpp
        boost::shared_ptr<WallShearStressBasedRegressionSolver<3> > p_regression_solver = WallShearStressBasedRegressionSolver<3>::Create();
        p_regression_solver->SetMaximumTimeWithLowWallShearStress(2.0*3600.0*unit::seconds);
        p_regression_solver->SetLowWallShearStressThreshold(1.e-06*unit::pascals);
        p_regression_solver->SetVesselNetwork(p_network);
```

Set up a `MicrovesselSolver` to manage all solves.

```cpp
        MicrovesselSolver<3> microvessel_solver;
        microvessel_solver.SetRegressionSolver(p_regression_solver);
        microvessel_solver.SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        microvessel_solver.SetVesselNetwork(p_network);
        microvessel_solver.SetOutputFileHandler(p_handler);
        microvessel_solver.SetOutputFrequency(1);
```

Run the solver

```cpp
        microvessel_solver.Run();
    }
};
```


# Code 
The full code is given below


## File name `TestBloodFlowTutorial.hpp` 

```cpp
#include <vector>
#include <cxxtest/TestSuite.h>
#include "Owen11Parameters.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "OutputFileHandler.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "UnitCollection.hpp"
#include "BaseUnits.hpp"
#include "NodeFlowProperties.hpp"
#include "Owen11Parameters.hpp"
#include "GenericParameters.hpp"
#include "ParameterCollection.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "FlowSolver.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "ConstantHaematocritSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestBloodFlowLiteratePaper : public AbstractCellBasedWithTimingsTestSuite
{
public:
    void TestSimpleFlowProblem() throw (Exception)
    {
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        QLength vessel_length(100.0*unit::microns);
        DimensionalChastePoint<2> start_point(0.0, 0.0);
        VesselNetworkGenerator<2> network_generator;
        boost::shared_ptr<VesselNetwork<2> > p_network = network_generator.GenerateBifurcationUnit(vessel_length, start_point);
        std::vector<unsigned> num_units_per_direction;
        num_units_per_direction.push_back(2);
        num_units_per_direction.push_back(0);
        network_generator.PatternUnitByTranslation(p_network, num_units_per_direction);
        QPressure inlet_pressure(50.0 * unit::mmHg);
        p_network->GetNode(0)->GetFlowProperties()->SetIsInputNode(true);
        p_network->GetNode(0)->GetFlowProperties()->SetPressure(inlet_pressure);
        p_network->GetNode(p_network->GetNumberOfNodes()-1)->GetFlowProperties()->SetIsOutputNode(true);
        p_network->GetNode(p_network->GetNumberOfNodes()-1)->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
        QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        VesselImpedanceCalculator<2> impedance_calculator = VesselImpedanceCalculator<2>();
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
        QFlowImpedance expected_impedance = 8.0 * viscosity* vessel_length/(M_PI*Qpow4(vessel_radius));
        TS_ASSERT_DELTA(p_network->GetVessel(0)->GetSegment(0)->GetFlowProperties()->GetImpedance().value(), expected_impedance.value(), 1.e-6);
        FlowSolver<2> flow_solver = FlowSolver<2>();
        flow_solver.SetVesselNetwork(p_network);
        flow_solver.Solve();
        QPressure expected_pressure = (inlet_pressure + Owen11Parameters::mpOutletPressure->GetValue())/2.0;
        TS_ASSERT_DELTA(p_network->GetNode(7)->GetFlowProperties()->GetPressure().value(), expected_pressure.value(), 1.e-6);
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestSimpleFlowProblem"));
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "bifurcating_network_results.vtp");
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath() + "parameter_collection.xml");
        ParameterCollection::Instance()->Destroy();
        BaseUnits::Instance()->Destroy();
    }
    void TestFlowProblemWithHaematocrit() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemWithHaematocrit", false));
        QLength cell_width(25.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(cell_width);
        QLength target_width = 100.0 * cell_width;
        QLength target_height = 30.0 * cell_width;
        QLength vessel_length = 4.0 * cell_width;
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);
        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, cell_width);
        DimensionalChastePoint<3> outlet_locator(target_width/cell_width, target_height/cell_width, 0.0, cell_width);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());

        QLength vessel_radius(GenericParameters::mpCapillaryRadius->GetValue());
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        QLength sphere_radius = 400.0 * cell_width;
        QLength sphere_thickess = 1.0 * cell_width;
        double sphere_azimuth = M_PI;
        double sphere_polar = M_PI/2.0;
        network_generator.MapToSphere(p_network, sphere_radius, sphere_thickess, sphere_azimuth, sphere_polar);
        VesselImpedanceCalculator<3> impedance_calculator = VesselImpedanceCalculator<3>();
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
        FlowSolver<3> flow_solver = FlowSolver<3>();
        flow_solver.SetVesselNetwork(p_network);
        flow_solver.Solve();
        AlarconHaematocritSolver<3> haematocrit_solver = AlarconHaematocritSolver<3>();
        haematocrit_solver.SetVesselNetwork(p_network);
        haematocrit_solver.SetHaematocrit(0.45);
        haematocrit_solver.Calculate();
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        p_network->Write(p_handler->GetOutputDirectoryFullPath() + "network_haematocrit.vtp");
        ParameterCollection::Instance()->DumpToFile(p_handler->GetOutputDirectoryFullPath() + "parameter_collection.xml");
        ParameterCollection::Instance()->Destroy();
    }
    void TestFlowProblemStucturalAdaptation() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemStucturalAdaptation", false));
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        QLength target_width(8000*unit::microns);
        QLength target_height(2000*unit::microns);
        QLength vessel_length(300.0*unit::microns);
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);
        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, reference_length);
        DimensionalChastePoint<3> outlet_locator(target_width/reference_length, target_height/reference_length, 0.0, reference_length);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
        QLength vessel_radius(40.0*unit::microns);
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"initial_network.vtp");
        BaseUnits::Instance()->SetReferenceTimeScale(60.0*unit::seconds);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();
        StructuralAdaptationSolver<3> structural_adaptation_solver;
        structural_adaptation_solver.SetVesselNetwork(p_network);
        structural_adaptation_solver.SetWriteOutput(true);
        structural_adaptation_solver.SetOutputFileName(p_handler->GetOutputDirectoryFullPath()+"adaptation_data.dat");
        structural_adaptation_solver.SetTolerance(0.0001);
        structural_adaptation_solver.SetMaxIterations(10);
        structural_adaptation_solver.AddPreFlowSolveCalculator(p_impedance_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_haematocrit_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_wss_calculator);
        structural_adaptation_solver.AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        structural_adaptation_solver.SetTimeIncrement(0.01 * unit::seconds);
        structural_adaptation_solver.Solve();
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"network_initial_sa.vtp");
    }
    void TestFlowProblemStucturalAdaptationWithRegression() throw (Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestBloodFlowLiteratePaper/TestFlowProblemStucturalAdaptationWithRegression", false));
        QLength reference_length(1.0 * unit::microns);
        BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
        QLength target_width(8000*unit::microns);
        QLength target_height(2000*unit::microns);
        QLength vessel_length(300.0*unit::microns);
        VesselNetworkGenerator<3> network_generator;
        boost::shared_ptr<VesselNetwork<3> > p_network = network_generator.GenerateHexagonalNetwork(target_width,
                                                                                                    target_height,
                                                                                                    vessel_length);

        DimensionalChastePoint<3> inlet_locator(0.0, 0.0, 0.0, reference_length);
        DimensionalChastePoint<3> outlet_locator(target_width/reference_length, target_height/reference_length, 0.0, reference_length);
        boost::shared_ptr<VesselNode<3> > p_inlet_node = p_network->GetNearestNode(inlet_locator);
        boost::shared_ptr<VesselNode<3> > p_outlet_node = p_network->GetNearestNode(outlet_locator);
        p_inlet_node->GetFlowProperties()->SetIsInputNode(true);
        p_inlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpInletPressure->GetValue());
        p_outlet_node->GetFlowProperties()->SetIsOutputNode(true);
        p_outlet_node->GetFlowProperties()->SetPressure(Owen11Parameters::mpOutletPressure->GetValue());
        QLength vessel_radius(40.0*unit::microns);
        p_network->SetSegmentRadii(vessel_radius);
        QDynamicViscosity viscosity = Owen11Parameters::mpPlasmaViscosity->GetValue();
        p_network->SetSegmentViscosity(viscosity);
        p_network->Write(p_handler->GetOutputDirectoryFullPath()+"initial_network.vtp");

        BaseUnits::Instance()->SetReferenceTimeScale(60.0*unit::seconds);
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 1);
        boost::shared_ptr<VesselImpedanceCalculator<3> > p_impedance_calculator = VesselImpedanceCalculator<3>::Create();
        boost::shared_ptr<ConstantHaematocritSolver<3> > p_haematocrit_calculator = ConstantHaematocritSolver<3>::Create();
        p_haematocrit_calculator->SetHaematocrit(0.45);
        boost::shared_ptr<WallShearStressCalculator<3> > p_wss_calculator = WallShearStressCalculator<3>::Create();
        boost::shared_ptr<MechanicalStimulusCalculator<3> > p_mech_stimulus_calculator = MechanicalStimulusCalculator<3>::Create();

        boost::shared_ptr<StructuralAdaptationSolver<3> > p_structural_adaptation_solver = StructuralAdaptationSolver<3>::Create();
        p_structural_adaptation_solver->SetVesselNetwork(p_network);
        p_structural_adaptation_solver->SetWriteOutput(true);
        p_structural_adaptation_solver->SetOutputFileName(p_handler->GetOutputDirectoryFullPath()+"adaptation_data.dat");
        p_structural_adaptation_solver->SetTolerance(0.0001);
        p_structural_adaptation_solver->SetMaxIterations(10);
        p_structural_adaptation_solver->AddPreFlowSolveCalculator(p_impedance_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_haematocrit_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_wss_calculator);
        p_structural_adaptation_solver->AddPostFlowSolveCalculator(p_mech_stimulus_calculator);
        p_structural_adaptation_solver->SetTimeIncrement(0.01 * unit::seconds);
        boost::shared_ptr<WallShearStressBasedRegressionSolver<3> > p_regression_solver = WallShearStressBasedRegressionSolver<3>::Create();
        p_regression_solver->SetMaximumTimeWithLowWallShearStress(2.0*3600.0*unit::seconds);
        p_regression_solver->SetLowWallShearStressThreshold(1.e-06*unit::pascals);
        p_regression_solver->SetVesselNetwork(p_network);
        MicrovesselSolver<3> microvessel_solver;
        microvessel_solver.SetRegressionSolver(p_regression_solver);
        microvessel_solver.SetStructuralAdaptationSolver(p_structural_adaptation_solver);
        microvessel_solver.SetVesselNetwork(p_network);
        microvessel_solver.SetOutputFileHandler(p_handler);
        microvessel_solver.SetOutputFrequency(1);
        microvessel_solver.Run();
    }
};
```

