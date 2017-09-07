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

#include <memory>
#include <vector>
#include <string>
#include "VesselNetwork.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "AbstractDiscreteContinuumGrid.hpp"

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
    QVolume mElementArea2d;

    /**
     * The element area (3D)
     */
    QVolume mElementArea3d;

    /**
     * The node spacing
     */
    QLength mNodeSpacing;

    /**
     * The sample spacing X
     */
    QLength mSampleSpacingX;

    /**
     * The sample spacing Y
     */
    QLength mSampleSpacingY;

    /**
     * Whether to use a pellet
     */
    bool mUsePellet;

    /**
     * Whether to use a finite pellet width
     */
    bool mFinitePelletWidth;

    /**
     * The sprouting probability
     */
    QRate mSproutingProbability;

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
    bool mUseTipExclusion;

    /**
     * Whether to do anastomosis
     */
    bool mDoAnastamosis;

    /**
     * The pellet concentration
     */
    QConcentration mPelletConcentration;

    /**
     * The vegf diffusivity
     */
    QDiffusivity mVegfDiffusivity;

    /**
     * The vegf decay rate
     */
    QRate mVegfDecayRate;

    /**
     * The binding constant
     */
    double mVegfBindingConstant;

    /**
     * The vegf blood concentration
     */
    QConcentration mVegfBloodConcentration;

    /**
     * The vegf permeability
     */
    QMembranePermeability mVegfPermeability;

    /**
     * The uptake rate per cell
     */
    QMolarFlowRate mUptakeRatePerCell;

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
    QTime mTotalTime;

    /**
     * The time step size
     */
    QTime mTimeStepSize;

    /**
     * The anastamosis radius
     */
    QLength mAnastamosisRadius;

    /**
     * The cell length
     */
    QLength mCellLength;

    /**
     * The run number
     */
    unsigned mRunNumber;

    /**
     * The random seed
     */
    unsigned mRandomSeed;

    PartPtr<DIM> mpDomain;

    std::vector<Vertex<DIM> > mHoles;

    std::string mWorkDirectory;

    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > mpGrid;

    std::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > mpSolver;

    vtkSmartPointer<vtkUnstructuredGrid> mpSampleGrid;

    unsigned mNumSampleY;

    std::vector<unsigned> mSamplingIndices;

    bool mOnlyPerfusedSprout;

    unsigned mSampleFrequency;

    std::vector<double> mStoredSample;

    QVelocity mSproutVelocity;

    bool mUseFiniteDifferenceGrid;

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

    void SetAnastamosisRadius(QLength radius);

    QLength GetAnastamosisRadius();

    void SetCellLength(QLength length);

    QLength GetCellLength();

    void SetTipVelocity(QVelocity velocity);

    void SetOnlyPerfusedSprout(bool onlyPerfused);

    void SetSampleFrequency(unsigned freq);

    void DoSampling(std::ofstream& rStream,
            vtkSmartPointer<vtkUnstructuredGrid> pSampleGrid,
            std::string sampleType,
            double time,
            double multfact=1.0,
            bool sampleOnce=false);

    PartPtr<DIM> SetUpDomain();

    std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > SetUpGrid();

    std::shared_ptr<VesselNetwork<DIM> > SetUpVesselNetwork();

    void SetUpSolver();

    void SetWorkDir(std::string workDir);

    void SetUpSampleGrid();

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

    void SetDoAnastamosis(bool doAnastamosis);

    void SetElementArea2d(QVolume elementArea2d);

    void SetElementArea3d(QVolume elementArea3d);

    void SetFinitePelletWidth(bool finitePelletWidth);

    void SetGridSpacing(QLength gridSpacing);

    void SetIncludeVesselSink(bool includeVesselSink);

    void SetNodeSpacing(QLength nodeSpacing);

    void SetPdeTimeIncrement(double pdeTimeIncrement);

    void SetPelletConcentration(QConcentration pelletConcentration);

    void SetPersistenceAngle(double persistenceAngle);

    void SetRandomSeed(unsigned randomSeed);

    void SetRunNumber(unsigned runNumber);

    void SetSampleSpacingX(QLength sampleSpacingX);

    void SetSampleSpacingY(QLength sampleSpacingY);

    void SetSproutingProbability(QRate sproutingProbability);

    void SetTimeStepSize(QTime timeStepSize);

    void SetUseTipExclusion(bool usetipexclusion);

    void SetTotalTime(QTime totalTime);

    void SetUptakeRatePerCell(QMolarFlowRate uptakeRatePerCell);

    void SetUseFixedGradient(bool useFixedGradient);

    void SetUsePdeOnly(bool usePdeOnly);

    void SetUsePellet(bool usePellet);

    void SetVegfBindingConstant(double vegfBindingConstant);

    void SetVegfBloodConcentration(QConcentration vegfBloodConcentration);

    void SetVegfDecayRate(const QRate vegfDecayRate);

    void SetVegfDiffusivity(QDiffusivity vegfDiffusivity);

    void SetVegfPermeability(QMembranePermeability vegfPermeability);

    void Run();


};

#endif /* CornealMicropocketSimulation_HPP_ */
