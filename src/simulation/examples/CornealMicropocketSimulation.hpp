/*

Copyright (c) 2005-2017, University of Oxford.
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

#ifndef CORNEALMICROPOCKETSIMULATION_HPP_
#define CORNEALMICROPOCKETSIMULATION_HPP_

#include <vector>
#include <string>

#include "StructuralAdaptationSolver.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"
#include "AbstractCellPopulation.hpp"
#include "AngiogenesisSolver.hpp"
#include "RegressionSolver.hpp"
#include "AbstractMicrovesselModifier.hpp"

/**
 * Helper struct for defining the type of boundary condition.
 */
struct DomainType
{
    /**
     * The different types of boundary condition
     */
    enum Value
    {
        PLANAR_2D, PLANAR_3D, PLANAR_2D_FINITE, PLANAR_3D_FINITE, CIRCLE_2D, CIRCLE_3D, HEMISPHERE
    };
};

/**
 * A template simulation for the corneal micropocket assay
 */
template<unsigned DIM>
class CornealMicropocketSimulation
{

    /**
     * The domain type
     */
    DomainType::Value mDomainType;

    /**
     * The cornea radius
     */
    QLength mCorneaRadius;

    /**
     * The cornea thickness
     */
    QLength mCorneaThickness;

    /**
     * The pellet height
     */
    QLength mPelletHeight;

    /**
     * The pellet thickness
     */
    QLength mPelletThickness;

    /**
     * The pellet radius
     */
    QLength mPelletRadius;

    /**
     * The limbal offset
     */
    QLength mLimbalOffset;

    /**
     * The grid spacing
     */
    QLength mGridSpacing;

    /**
     * The element area (2D)
     */
    units::quantity<unit::volume> mElementArea2d;

    /**
     * The element area (3D)
     */
    units::quantity<unit::volume> mElementArea3d;

    /**
     * The node spacing
     */
    QLength mNodeSpacing;

    /**
     * The density grid spacing
     */
    QLength mDensityGridSpacing;

    /**
     * The sample x
     */
    QLength mSampleSpacingX;

    /**
     * The sample Y
     */
    QLength mSampleSpacingY;

    /**
     * The sample Z
     */
    QLength mSampleSpacingZ;

    /**
     * Whether to use a pellet
     */
    bool mUsePellet;

    /**
     * Whether to use a finite pellet with
     */
    bool mFinitePelletWidth;

    /**
     * The sprouting probability
     */
    units::quantity<unit::rate> mSproutingProbability;

    /**
     * The attraction strength
     */
    double mAttractionStrength;

    /**
     * The chemotactic strength
     */
    double mChemotacticStrength;

    /**
     * The persistence angle
     */
    double mPersistenceAngle;

    /**
     * The tip exclusion radius
     */
    QLength mTipExclusionRadius;

    /**
     * Whether to do anastomosis
     */
    bool mDoAnastamosis;

    /**
     * The pellet concentration
     */
    units::quantity<unit::concentration> mPelletConcentration;

    /**
     * The vegf diffusivity
     */
    units::quantity<unit::diffusivity> mVegfDiffusivity;

    /**
     * The vegf decay rate
     */
    units::quantity<unit::rate> mVegfDecayRate;

    /**
     * The binding constant
     */
    double mVegfBindingConstant;

    /**
     * The vegf blood concentration
     */
    units::quantity<unit::concentration> mVegfBloodConcentration;

    /**
     * The vegf permeability
     */
    units::quantity<unit::membrane_permeability> mVegfPermeability;

    /**
     * The uptake rate per cell
     */
    units::quantity<unit::molar_flow_rate> mUptakeRatePerCell;

    /**
     * The pde time increment
     */
    double mPdeTimeIncrement;

    /**
     * Include vessel sink
     */
    bool mIncludeVesselSink;

    /**
     * Use a fixed gradient
     */
    bool mUseFixedGradient;

    /**
     * Use PDE only
     */
    bool mUsePdeOnly;

    /**
     * The total time
     */
    units::quantity<unit::time> mTotalTime;

    /**
     * The time step size
     */
    units::quantity<unit::time> mTimeStepSize;

    /**
     * The run number
     */
    unsigned mRunNumber;

    /**
     * The random seed
     */
    unsigned mRandomSeed;

    std::shared_ptr<Part<DIM> > mpDomain;

    std::vector<DimensionalChastePoint<DIM> > mHoles;

    std::string mWorkDirectory;

    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > mpGrid;

    std::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > mpSolver;

    std::vector<vtkSmartPointer<vtkPoints> > mSampleLines;

    unsigned mNumSampleY;
    unsigned mNumSampleZ;

    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > mpSamplingGrid;

    bool mOnlyPerfusedSprout;

    unsigned mSampleFrequency;

    std::vector<double> mStoredSample;

    units::quantity<unit::velocity> mSproutVelocity;

public:

    /**
     * Constructor.
     */
    CornealMicropocketSimulation();

    /**
     * Destructor.
     */
    virtual ~CornealMicropocketSimulation();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static std::shared_ptr<CornealMicropocketSimulation> Create();

    void SetTipVelocity(units::quantity<unit::velocity> velocity);

    void SetOnlyPerfusedSprout(bool onlyPerfused);

    void SetSampleFrequency(unsigned freq);

    void DoSampling(std::ofstream& rStream,
            std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver,
            double time,
            double multfact=1.0,
            bool sampleOnce=false);

    std::shared_ptr<Part<DIM> > SetUpDomain();

    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > SetUpGrid(bool mSampling=false);

    std::shared_ptr<VesselNetwork<DIM> > SetUpVesselNetwork();

    void SetUpSolver();

    void SetWorkDir(std::string workDir);

    void SetUpSamplePoints();

    /**
     * Set the domain type
     * @param domainType the domain type
     */
    void SetDomainType(DomainType::Value domainType);

    /**
     * Set the cornea radius
     * @param corneaRadius the cornea radius
     */
    void SetCorneaRadius(QLength corneaRadius);

    /**
     * Set the cornea thickness
     * @param corneaRadius the cornea thickness
     */
    void SetCorneaThickness(QLength corneaThickness);

    /**
     * Set the pellet height
     * @param pelletHeight the pellet height
     */
    void SetPelletHeight(QLength pelletHeight);

    /**
     * Set the pellet thickness
     * @param pelletThickness the pellet thickness
     */
    void SetPelletThickness(QLength pelletThickness);

    /**
     * Set the pellet radius
     * @param pelletRadius the pellet radius
     */
    void SetPelletRadius(QLength pelletRadius);

    /**
     * Set the limbal offset
     * @param limbalOffset the limbal offset
     */
    void SetLimbalOffset(QLength limbalOffset);

    void SetAttractionStrength(double attractionStrength);

    void SetChemotacticStrength(double chemotacticStrength);

    void SetDensityGridSpacing(QLength densityGridSpacing);

    void SetDoAnastamosis(bool doAnastamosis);

    void SetElementArea2d(units::quantity<unit::volume> elementArea2d);

    void SetElementArea3d(units::quantity<unit::volume> elementArea3d);

    void SetFinitePelletWidth(bool finitePelletWidth);

    void SetGridSpacing(QLength gridSpacing);

    void SetIncludeVesselSink(bool includeVesselSink);

    void SetNodeSpacing(QLength nodeSpacing);

    void SetPdeTimeIncrement(double pdeTimeIncrement);

    void SetPelletConcentration(units::quantity<unit::concentration> pelletConcentration);

    void SetPersistenceAngle(double persistenceAngle);

    void SetRandomSeed(unsigned randomSeed);

    void SetRunNumber(unsigned runNumber);

    void SetSampleSpacingX(QLength sampleSpacingX);

    void SetSampleSpacingY(QLength sampleSpacingY);

    void SetSampleSpacingZ(QLength sampleSpacingZ);

    void SetSproutingProbability(units::quantity<unit::rate> sproutingProbability);

    void SetTimeStepSize(units::quantity<unit::time> timeStepSize);

    void SetTipExclusionRadius(QLength tipExclusionRadius);

    void SetTotalTime(units::quantity<unit::time> totalTime);

    void SetUptakeRatePerCell(units::quantity<unit::molar_flow_rate> uptakeRatePerCell);

    void SetUseFixedGradient(bool useFixedGradient);

    void SetUsePdeOnly(bool usePdeOnly);

    void SetUsePellet(bool usePellet);

    void SetVegfBindingConstant(double vegfBindingConstant);

    void SetVegfBloodConcentration(units::quantity<unit::concentration> vegfBloodConcentration);

    void SetVegfDecayRate(const units::quantity<unit::rate> vegfDecayRate);

    void SetVegfDiffusivity(units::quantity<unit::diffusivity> vegfDiffusivity);

    void SetVegfPermeability(units::quantity<unit::membrane_permeability> vegfPermeability);

    void Run();


};

#endif /* CornealMicropocketSimulation_HPP_ */
