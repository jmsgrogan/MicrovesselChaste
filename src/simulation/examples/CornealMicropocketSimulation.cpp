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

#include <string>
#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkHexahedron.h>
#include <vtkAppendFilter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include "UblasIncludes.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "VesselNetworkWriter.hpp"
#include "SolutionDependentDiscreteSource.hpp"
#include "AbstractDiscreteContinuumPde.hpp"
#include "MappableGridGenerator.hpp"
#include "VesselNetworkGenerator.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"
#include "FunctionMap.hpp"
#include "RandomNumberGenerator.hpp"
#include "SimulationTime.hpp"
#include "BaseUnits.hpp"
#include "CornealMicropocketSimulation.hpp"
#include "VesselBasedDiscreteSource.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "AngiogenesisSolver.hpp"
#include "CoupledLumpedSystemFiniteElementSolver.hpp"
#include "Timer.hpp"

template<unsigned DIM>
CornealMicropocketSimulation<DIM>::CornealMicropocketSimulation() :
    mDomainType(DomainType::PLANAR_2D),
    mCorneaRadius(1.3_mm),
    mCorneaThickness(100_um),
    mPelletHeight(1_mm),
    mPelletThickness(40_um),
    mPelletRadius(200_um),
    mLimbalOffset(100_um),
    mGridSpacing(40_um),
    mElementArea2d(1e3*unit::microns_cubed),
    mElementArea3d(1e4*unit::microns_cubed),
    mNodeSpacing(40_um),
    mSampleSpacingX(180_um),
    mSampleSpacingY(60_um),
    mUsePellet(true),
    mFinitePelletWidth(false),
    mSproutingProbability(0.5*unit::per_hour),
    mAttractionStrength(0.0),
    mChemotacticStrength(0.5),
    mPersistenceAngle(5.0),
    mUseTipExclusion(true),
    mDoAnastamosis(true),
    mPelletConcentration(3.0e5_nM),
    mVegfDiffusivity(24984.0*unit::micron_squared_per_hour),
    mVegfDecayRate(-0.8*unit::per_hour),
    mVegfBindingConstant(100.0),
    mVegfBloodConcentration(0_M),
    mVegfPermeability(300.0*unit::microns_per_hour),
    mUptakeRatePerCell(4.e-9*unit::nanomole_per_hour),
    mPdeTimeIncrement(0.01),
    mIncludeVesselSink(true),
    mUseFixedGradient(false),
    mUsePdeOnly(false),
    mTotalTime(24_h),
    mTimeStepSize(0.5_h),
    mAnastamosisRadius(5_um),
    mCellLength(20_um),
    mRunNumber(0),
    mRandomSeed(0),
    mpDomain(),
    mHoles(),
    mWorkDirectory(),
    mpGrid(),
    mpNetwork(),
    mpSolver(),
    mpSampleGrid(),
    mNumSampleY(1),
    mSamplingIndices(),
    mOnlyPerfusedSprout(false),
    mSampleFrequency(5),
    mStoredSample(),
    mSproutVelocity(20.0*unit::microns_per_hour),
    mUseFiniteDifferenceGrid(false)
{

}

template<unsigned DIM>
CornealMicropocketSimulation<DIM>::~CornealMicropocketSimulation()
{

}

template<unsigned DIM>
std::shared_ptr<CornealMicropocketSimulation<DIM> > CornealMicropocketSimulation<DIM>::Create()
{
    return std::make_shared<CornealMicropocketSimulation<DIM> >();

}

template<unsigned DIM>
QLength CornealMicropocketSimulation<DIM>::GetAnastamosisRadius()
{
    return mAnastamosisRadius;
}

template<unsigned DIM>
QLength CornealMicropocketSimulation<DIM>::GetCellLength()
{
    return mCellLength;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetAnastamosisRadius(QLength radius)
{
    mAnastamosisRadius = radius;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetCellLength(QLength length)
{
    mCellLength = length;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTipVelocity(QVelocity velocity)
{
    mSproutVelocity = velocity;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetOnlyPerfusedSprout(bool onlyPerfused)
{
    mOnlyPerfusedSprout = onlyPerfused;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPelletHeight(QLength pelletHeight)
{
    mPelletHeight = pelletHeight;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetCorneaRadius(QLength corneaRadius)
{
    mCorneaRadius = corneaRadius;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPelletThickness(QLength pelletThickness)
{
    mPelletThickness = pelletThickness;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPelletRadius(QLength pelletRadius)
{
    mPelletRadius = pelletRadius;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetLimbalOffset(QLength limbalOffset)
{
    mLimbalOffset = limbalOffset;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSampleFrequency(unsigned freq)
{
    mSampleFrequency = freq;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetAttractionStrength(double attractionStrength)
{
    mAttractionStrength = attractionStrength;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetChemotacticStrength(double chemotacticStrength)
{
    mChemotacticStrength = chemotacticStrength;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetCorneaThickness(QLength corneaThickness)
{
    mCorneaThickness = corneaThickness;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetDoAnastamosis(bool doAnastamosis)
{
    mDoAnastamosis = doAnastamosis;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetElementArea2d(QVolume elementArea2d)
{
    mElementArea2d = elementArea2d;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetElementArea3d(QVolume elementArea3d)
{
    mElementArea3d = elementArea3d;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetFinitePelletWidth(bool finitePelletWidth)
{
    mFinitePelletWidth = finitePelletWidth;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetGridSpacing(QLength gridSpacing)
{
    mGridSpacing = gridSpacing;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetIncludeVesselSink(bool includeVesselSink)
{
    mIncludeVesselSink = includeVesselSink;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetNodeSpacing(QLength nodeSpacing)
{
    mNodeSpacing = nodeSpacing;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPdeTimeIncrement(double pdeTimeIncrement)
{
    mPdeTimeIncrement = pdeTimeIncrement;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPelletConcentration(QConcentration pelletConcentration)
{
    mPelletConcentration = pelletConcentration;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPersistenceAngle(double persistenceAngle)
{
    mPersistenceAngle = persistenceAngle;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetRandomSeed(unsigned randomSeed)
{
    mRandomSeed = randomSeed;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetRunNumber(unsigned runNumber)
{
    mRunNumber = runNumber;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSampleSpacingX(QLength sampleSpacingX)
{
    mSampleSpacingX = sampleSpacingX;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSampleSpacingY(QLength sampleSpacingY)
{
    mSampleSpacingY = sampleSpacingY;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSproutingProbability(QRate sproutingProbability)
{
    mSproutingProbability = sproutingProbability;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTimeStepSize(QTime timeStepSize)
{
    mTimeStepSize = timeStepSize;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUseTipExclusion(bool exclude)
{
    mUseTipExclusion = exclude;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTotalTime(QTime totalTime)
{
    mTotalTime = totalTime;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUptakeRatePerCell(QMolarFlowRate uptakeRatePerCell)
{
    mUptakeRatePerCell = uptakeRatePerCell;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUseFixedGradient(bool useFixedGradient)
{
    mUseFixedGradient = useFixedGradient;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUsePdeOnly(bool usePdeOnly)
{
    mUsePdeOnly = usePdeOnly;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUsePellet(bool usePellet)
{
    mUsePellet = usePellet;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfBindingConstant(double vegfBindingConstant)
{
    mVegfBindingConstant = vegfBindingConstant;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfBloodConcentration(QConcentration vegfBloodConcentration)
{
    mVegfBloodConcentration = vegfBloodConcentration;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetDomainType(DomainType::Value domainType)
{
    mDomainType = domainType;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfDecayRate(QRate vegfDecayRate)
{
    mVegfDecayRate = vegfDecayRate;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetWorkDir(std::string workDir)
{
    mWorkDirectory = workDir;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfDiffusivity(QDiffusivity vegfDiffusivity)
{
    mVegfDiffusivity = vegfDiffusivity;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfPermeability(QMembranePermeability vegfPermeability)
{
    mVegfPermeability = vegfPermeability;
}

template<unsigned DIM>
PartPtr<DIM> CornealMicropocketSimulation<DIM>::SetUpDomain()
{
    mpDomain = Part<DIM>::Create();
    mHoles.clear();

    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    if(mDomainType == DomainType::PLANAR_2D)
    {
        QLength domain_width = 2.0 * M_PI * mCorneaRadius;
        QLength domain_height = mPelletHeight + mLimbalOffset;
        if(!mUsePellet)
        {
            mpDomain->AddRectangle(domain_width, domain_height);
            mpDomain->AddAttributeToEdgeIfFound(Vertex<DIM>(M_PI*mCorneaRadius, mPelletHeight, 0_m),
                    "Pellet Interface", 1.0);
        }
        else
        {
            std::vector<VertexPtr<DIM> > points;
            points.push_back(Vertex<DIM>::Create(0_m));
            points.push_back(Vertex<DIM>::Create(domain_width));
            points.push_back(Vertex<DIM>::Create(domain_width, domain_height));
            points.push_back(Vertex<DIM>::Create((domain_width+2.0 * M_PI *mPelletRadius)/2.0, domain_height));
            points.push_back(Vertex<DIM>::Create((domain_width-2.0 * M_PI *mPelletRadius)/2.0, domain_height));
            points.push_back(Vertex<DIM>::Create(0_m, domain_height));

            auto p_polygon = Polygon<DIM>::Create(points);
            mpDomain->AddPolygon(p_polygon);
            mpDomain->AddAttributeToEdgeIfFound(Vertex<DIM>(domain_width/2.0, domain_height),
                    "Pellet Interface", 1.0);
        }
    }
    else if(mDomainType == DomainType::PLANAR_3D)
    {
        QLength domain_width = 2.0 * M_PI * mCorneaRadius;
        QLength domain_height = mPelletHeight + mLimbalOffset;
        if(!mUsePellet)
        {
            mpDomain->AddCuboid(domain_width, domain_height, mCorneaThickness);
            std::vector<FacetPtr<DIM> > facets = mpDomain->GetFacets();
            Vertex<DIM> probe_loc(domain_width/2.0, domain_height, mCorneaThickness/2.0);
            for(auto& facet:mpDomain->GetFacets())
            {
                QLength distance = facet->GetCentroid().GetDistance(probe_loc);
                if (double(distance/reference_length) < 1e-3)
                {
                    facet->GetPolygons()[0]->AddAttribute("Pellet Interface", 1.0);
                }
            }
        }
        else
        {
            mpDomain->AddCuboid(domain_width, domain_height, mCorneaThickness);
            QLength gap = (mCorneaThickness - mPelletThickness)/2.0;
            std::vector<VertexPtr<DIM> > points;
            QLength left_side = (domain_width-2.0 * M_PI *mPelletRadius)/2.0;
            QLength right_side = (domain_width+2.0 * M_PI *mPelletRadius)/2.0;

            points.push_back(Vertex<DIM>::Create(left_side, domain_height, gap));
            points.push_back(Vertex<DIM>::Create(right_side,domain_height, gap));
            points.push_back(Vertex<DIM>::Create(right_side, domain_height, mCorneaThickness-gap));
            points.push_back(Vertex<DIM>::Create(left_side, domain_height, mCorneaThickness-gap));

            auto p_polygon = Polygon<DIM>::Create(points);
            p_polygon->AddAttribute("Pellet Interface", 1.0);
            Vertex<DIM> probe_loc(domain_width/2.0, domain_height, mCorneaThickness/2.0);
            for(auto& facet:mpDomain->GetFacets())
            {
                QLength distance = facet->GetCentroid().GetDistance(probe_loc);
                if (double(distance/reference_length) < 1e-3)
                {
                    mpDomain->AddPolygon(p_polygon, false, facet);
                }
            }
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D)
    {
        QLength delta = mPelletHeight + mLimbalOffset -mCorneaRadius + mPelletRadius;
        mpDomain->AddCircle(mCorneaRadius, Vertex<DIM>(), 24);
        if (mUsePellet)
        {
            PolygonPtr<DIM> p_polygon = mpDomain->AddCircle(mPelletRadius, Vertex<DIM>(0_m, -1.0*delta), 24);
            p_polygon->AddAttributeToAllEdges("Pellet Interface", 1.0);
            mpDomain->AddHoleMarker(Vertex<DIM>(0_m, -1.0*delta));
            mHoles.push_back(Vertex<DIM>(0_m, -1.0*delta));
        }
    }
    else if(mDomainType == DomainType::CIRCLE_3D)
    {
        QLength delta = mPelletHeight + mLimbalOffset -mCorneaRadius+mPelletRadius;
        PolygonPtr<DIM> p_circle = mpDomain->AddCircle(mCorneaRadius, Vertex<DIM>(), 24);
        mpDomain->Extrude(p_circle, mCorneaThickness);

        if (mUsePellet)
        {
            QLength gap = (mCorneaThickness - mPelletThickness)/2.0;

            auto p_pellet = Part<DIM>::Create();
            p_circle = p_pellet->AddCircle(mPelletRadius, Vertex<DIM>(0_m, -1.0*delta), 24);
            p_pellet->Extrude(p_circle, mPelletThickness);
            p_pellet->Translate(Vertex<DIM>(0_m, 0_m, gap));
            std::vector<PolygonPtr<DIM> >polygons = p_pellet->GetPolygons();

            QLength half_height = mCorneaThickness/2.0;
            mpDomain->AddHoleMarker(Vertex<DIM>(0_m, -1.0*delta, half_height));
            mHoles.push_back(Vertex<DIM>(0_m, -1.0*delta, half_height));
            mpDomain->AppendPart(p_pellet);
            for(auto& polygon:polygons)
            {
                polygon->AddAttribute("Pellet Interface", 1.0);
            }
        }
    }
    else if(mDomainType == DomainType::HEMISPHERE)
    {
        auto p_generator = MappableGridGenerator<DIM>::Create();

        unsigned num_divisions_x = 20;
        unsigned num_divisions_y = 20;
        double azimuth_angle = 1.0 * M_PI;
        double polar_angle = 0.999 * M_PI;
        mpDomain = p_generator->GenerateHemisphere(mCorneaRadius, mCorneaThickness,
                num_divisions_x, num_divisions_y, azimuth_angle, polar_angle);

        if(mUsePellet)
        {
            auto p_pellet_domain = Part<DIM>::Create();
            QLength gap = (mCorneaThickness - mPelletThickness)/8.0;
            QLength base = mCorneaRadius + gap - mCorneaThickness;

            p_pellet_domain->AddCylinder(mPelletRadius, mPelletThickness, Vertex<DIM>(0_m, 0_m, base));
            std::vector<PolygonPtr<DIM> > polygons = p_pellet_domain->GetPolygons();

            double height_fraction = double((mPelletHeight + mLimbalOffset)/mCorneaRadius);
            if(height_fraction>1.0)
            {
            	height_fraction=1.0;
            }
            double rotation_angle = M_PI/2.0 - std::asin(height_fraction);
            Vertex<DIM> pellet_centre(0_m, 0_m, base + mPelletThickness/2.0);
            c_vector<double, 3> axis;
            axis[0] = 0.0;
            axis[1] = 1.0;
            axis[2] = 0.0;
            p_pellet_domain->RotateAboutAxis(axis, rotation_angle);
            pellet_centre.RotateAboutAxis(axis, rotation_angle);
            mpDomain->AppendPart(p_pellet_domain);
            mpDomain->AddHoleMarker(pellet_centre);
            for(auto& polygon:polygons)
            {
                polygon->AddAttribute("Pellet Interface", 1.0);
            }
        }
    }
    return mpDomain;
}

template<unsigned DIM>
std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > CornealMicropocketSimulation<DIM>::SetUpGrid()
{
    if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
    {
        if(!mFinitePelletWidth)
        {
            mpGrid = RegularGrid<DIM>::Create();
            auto p_regular_grid = std::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGrid);
            p_regular_grid->GenerateFromPart(mpDomain, mGridSpacing);
        }
        else
        {
            DiscreteContinuumMeshGenerator<DIM, DIM> generator;
            generator.SetDomain(mpDomain);
            if(mDomainType == DomainType::PLANAR_2D)
            {
                generator.SetMaxElementArea(mElementArea2d);
            }
            else
            {
                generator.SetMaxElementArea(mElementArea3d);
            }
            generator.Update();
            mpGrid = generator.GetMesh();
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D or
            mDomainType == DomainType::CIRCLE_3D or
            mDomainType == DomainType::HEMISPHERE)
    {
        DiscreteContinuumMeshGenerator<DIM, DIM> generator;
        generator.SetDomain(mpDomain);
        if(mDomainType == DomainType::CIRCLE_2D)
        {
            generator.SetMaxElementArea(mElementArea2d);
        }
        else
        {
            generator.SetMaxElementArea(mElementArea3d);
        }
        if (mHoles.size() > 0)
        {
            generator.SetHoles(mHoles);
        }
        generator.Update();
        mpGrid = generator.GetMesh();
    }
    return mpGrid;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > CornealMicropocketSimulation<DIM>::SetUpVesselNetwork()
{
    mpNetwork = VesselNetwork<DIM>::Create();

    if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
    {
        VesselNetworkGenerator<DIM> generator;
        QLength domain_length = 2.0*M_PI*mCorneaRadius;
        unsigned divisions = unsigned(float(domain_length/mNodeSpacing)) - 2;
        unsigned alignment_axis = 0;
        QLength midpoint = 0_m;
        if(mDomainType == DomainType::PLANAR_3D)
        {
            midpoint = mCorneaThickness/2.0;
        }
        mpNetwork = generator.GenerateSingleVessel(domain_length,
                                                 Vertex<DIM>(0_m, mLimbalOffset, midpoint).rGetLocation(),
                                                 divisions, alignment_axis);
    }
    else if(mDomainType == DomainType::CIRCLE_2D or mDomainType == DomainType::CIRCLE_3D)
    {
        QLength sampling_radius = mCorneaRadius-mLimbalOffset;
        unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
        double sweep_angle = 2.0*M_PI/double(num_nodes);
        QLength midpoint = 0_m;
        if(mDomainType == DomainType::CIRCLE_3D)
        {
            midpoint = mCorneaThickness/2.0;
        }
        std::vector<std::shared_ptr<VesselNode<DIM> > > nodes;
        for(unsigned idx=0;idx<num_nodes;idx++)
        {
            double this_angle = double(idx)*sweep_angle+M_PI;
            QLength x_coord = sampling_radius*std::sin(this_angle);
            QLength y_coord = sampling_radius*std::cos(this_angle);
            nodes.push_back(VesselNode<DIM>::Create(Vertex<DIM>(x_coord, y_coord, midpoint)));
        }
        for(unsigned idx=1;idx<num_nodes;idx++)
        {
            mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[idx-1], nodes[idx]));
        }
        mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[nodes.size()-1], nodes[0]));
    }

    else if(mDomainType == DomainType::HEMISPHERE)
    {
        QLength cornea_mid_radius = (mCorneaRadius-mCorneaThickness/2.0);
        QLength sampling_radius = Qsqrt(cornea_mid_radius*cornea_mid_radius-mLimbalOffset*mLimbalOffset);
        unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
        double sweep_angle = 2.0*M_PI/double(num_nodes);

        std::vector<std::shared_ptr<VesselNode<DIM> > > nodes;
        for(unsigned idx=0;idx<num_nodes;idx++)
        {
            double this_angle = double(idx)*sweep_angle+M_PI;
            QLength x_coord = sampling_radius*std::sin(this_angle);
            QLength y_coord = sampling_radius*std::cos(this_angle);
            nodes.push_back(VesselNode<DIM>::Create(Vertex<DIM>(x_coord, y_coord, mLimbalOffset)));
        }
        for(unsigned idx=1;idx<num_nodes;idx++)
        {
            mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[idx-1], nodes[idx]));
        }
    }
    for(auto& node:mpNetwork->GetNodes())
    {
        node->GetFlowProperties()->SetPressure(1_Pa);
        node->SetRadius(5_um);
    }
    for(auto& segment:mpNetwork->GetVesselSegments())
    {
        segment->SetRadius(5_um);
        segment->GetCellularProperties()->SetAverageCellLengthLongitudinal(mCellLength);
    }
    return mpNetwork;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUpSolver()
{

    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    QConcentration reference_concentration = BaseUnits::Instance()->GetReferenceConcentrationScale();
    auto file_handler = std::make_shared<OutputFileHandler>(mWorkDirectory, false);

    if(!mUseFixedGradient)
    {
        auto p_pde = CoupledVegfPelletDiffusionReactionPde<DIM, DIM>::Create();
        p_pde->SetIsotropicDiffusionConstant(mVegfDiffusivity);
        p_pde->SetContinuumLinearInUTerm(mVegfDecayRate);
        p_pde->SetCurrentVegfInPellet(mPelletConcentration);
        p_pde->SetPelletBindingConstant(mVegfBindingConstant);
        p_pde->SetPelletDepth(mPelletThickness);

        QArea surface_area = 2.0*M_PI*mPelletRadius*mPelletThickness;
        surface_area = surface_area + 2.0*M_PI*mPelletRadius*mPelletRadius;
        QVolume volume = M_PI*mPelletRadius*mPelletRadius*mPelletThickness;
        p_pde->SetPelletSurfaceArea(surface_area);
        p_pde->SetPelletVolume(volume);
        p_pde->SetCorneaPelletPermeability(0.002*p_pde->GetCorneaPelletPermeability());

        if(mIncludeVesselSink and !mUsePdeOnly)
        {
            auto p_sink = VesselBasedDiscreteSource<DIM>::Create();
            p_sink->SetReferenceConcentration(mVegfBloodConcentration);
            p_sink->SetVesselPermeability(mVegfPermeability);
            p_sink->SetReferenceHaematocrit(0.45);
            p_sink->SetUptakeRatePerCell(-1.0*mUptakeRatePerCell);
            p_pde->AddDiscreteSource(p_sink);
        }

        if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
        {
            if(!mFinitePelletWidth)
            {
                mpSolver = CoupledLumpedSystemFiniteDifferenceSolver<DIM>::Create();
                mpSolver->SetGrid(mpGrid);
                mpSolver->SetPde(p_pde);
                mpSolver->SetLabel("vegf");
            }
            else
            {
                auto p_boundary_condition = DiscreteContinuumBoundaryCondition<DIM>::Create();
                p_boundary_condition->SetValue(mPelletConcentration);
                if(mDomainType == DomainType::PLANAR_2D)
                {
                    p_boundary_condition->SetType(BoundaryConditionType::EDGE);
                }
                else
                {
                    p_boundary_condition->SetType(BoundaryConditionType::POLYGON);
                }
                p_boundary_condition->SetIsRobin(true);
                p_boundary_condition->SetLabel("Pellet Interface");
                p_boundary_condition->SetDomain(mpDomain);

                mpSolver = CoupledLumpedSystemFiniteElementSolver<DIM>::Create();
                mpSolver->SetGrid(mpGrid);
                mpSolver->SetPde(p_pde);
                mpSolver->SetLabel("vegf");
                mpSolver->AddBoundaryCondition(p_boundary_condition);
            }
        }
        else if(mDomainType == DomainType::CIRCLE_2D)
        {
            auto p_boundary_condition = DiscreteContinuumBoundaryCondition<DIM>::Create();
             p_boundary_condition->SetValue(mPelletConcentration);
             p_boundary_condition->SetType(BoundaryConditionType::EDGE);
             p_boundary_condition->SetIsRobin(true);
             p_boundary_condition->SetLabel("Pellet Interface");
             p_boundary_condition->SetDomain(mpDomain);
             mpSolver = CoupledLumpedSystemFiniteElementSolver<DIM>::Create();
             mpSolver->SetGrid(mpGrid);
             mpSolver->SetPde(p_pde);
             mpSolver->SetLabel("vegf");
             mpSolver->AddBoundaryCondition(p_boundary_condition);
        }
        else if(mDomainType == DomainType::CIRCLE_3D or mDomainType == DomainType::HEMISPHERE)
        {
            auto p_boundary_condition = DiscreteContinuumBoundaryCondition<DIM>::Create();
             p_boundary_condition->SetValue(mPelletConcentration);
             p_boundary_condition->SetType(BoundaryConditionType::POLYGON);
             p_boundary_condition->SetIsRobin(true);
             p_boundary_condition->SetLabel("Pellet Interface");
             p_boundary_condition->SetDomain(mpDomain);
             mpSolver = CoupledLumpedSystemFiniteElementSolver<DIM>::Create();
             mpSolver->SetGrid(mpGrid);
             mpSolver->SetPde(p_pde);
             mpSolver->SetLabel("vegf");
             mpSolver->AddBoundaryCondition(p_boundary_condition);
        }
        mpSolver->SetFileHandler(file_handler);
        mpSolver->SetWriteSolution(true);

        auto p_fe_solver = std::dynamic_pointer_cast<CoupledLumpedSystemFiniteElementSolver<DIM> >(this->mpSolver);
        if(p_fe_solver)
        {
            p_fe_solver->SetTargetTimeIncrement(mPdeTimeIncrement);
            p_fe_solver->SetUseCoupling(true);
        }

        auto p_fd_solver = std::dynamic_pointer_cast<CoupledLumpedSystemFiniteDifferenceSolver<DIM> >(this->mpSolver);
        if(p_fd_solver)
        {
            p_fd_solver->SetTargetTimeIncrement(mPdeTimeIncrement);
            p_fd_solver->SetUseCoupling(true);
        }

        if(p_pde->GetDiscreteSources().size()>0)
        {
            p_pde->GetDiscreteSources()[0]->SetDensityMap(mpSolver->GetDensityMap());
        }
        if(mpNetwork)
        {
            mpSolver->GetDensityMap()->SetVesselNetwork(mpNetwork);
        }
    }
    else
    {
        if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
        {
            mpSolver = FunctionMap<DIM>::Create();
            mpSolver->SetGrid(mpGrid);
            std::vector<double> vegf_field;
            for(unsigned idx=0;idx<mpGrid->GetNumberOfPoints();idx++)
            {
                double y_loc = mpGrid->GetPoint(idx).Convert(reference_length)[1];
                double dimless_height = (mPelletHeight+mLimbalOffset)/reference_length;
                double frac = y_loc/dimless_height;
                vegf_field.push_back(frac*(mPelletConcentration/reference_concentration));
            }
            mpSolver->SetLabel("vegf");
            mpSolver->UpdateSolution(vegf_field);
        }
        else if(mDomainType == DomainType::CIRCLE_2D or mDomainType == DomainType::CIRCLE_3D)
        {
            mpSolver = FunctionMap<DIM>::Create();
            mpSolver->SetGrid(mpGrid);
            std::vector<double> vegf_field;
            for(unsigned idx=0;idx<mpGrid->GetNumberOfPoints();idx++)
            {
                double x_loc = mpGrid->GetPoint(idx).Convert(reference_length)[0];
                double y_loc = mpGrid->GetPoint(idx).Convert(reference_length)[1];
                double radius = std::sqrt(x_loc*x_loc + y_loc*y_loc);
                double dimless_cornea_radius = mCorneaRadius/reference_length;
                double frac = (1.0 - double(radius/dimless_cornea_radius))/(1.0-double(mPelletRadius/mCorneaRadius));
                vegf_field.push_back(frac*double(mPelletConcentration/reference_concentration));
            }
            mpSolver->SetLabel("vegf");
            mpSolver->UpdateSolution(vegf_field);
        }
        else if(mDomainType == DomainType::HEMISPHERE)
        {
            mpSolver = FunctionMap<DIM>::Create();
            mpSolver->SetGrid(mpGrid);
            std::vector<double> vegf_field;
            double dimless_offset = mLimbalOffset.Convert(reference_length);
            double dimless_height = mPelletHeight.Convert(reference_length);
            double mid_radius = mCorneaRadius.Convert(reference_length) - mCorneaThickness.Convert(reference_length)/2.0;
            double offset_angle = std::asin(mLimbalOffset.Convert(reference_length)/mid_radius);
            for(unsigned idx=0;idx<mpGrid->GetNumberOfPoints();idx++)
            {
                double x_loc = mpGrid->GetPoint(idx).Convert(reference_length)[0];
                double y_loc = mpGrid->GetPoint(idx).Convert(reference_length)[1];
                double z_loc = mpGrid->GetPoint(idx).Convert(reference_length)[2];
                double radius = std::sqrt(x_loc*x_loc + y_loc*y_loc);
                double angle = std::atan(z_loc/radius);
                double dist = mid_radius*(angle-offset_angle);
                double factor = dimless_offset/(dimless_height+dimless_offset);
                factor += dist/(dimless_height+dimless_offset);
                if(factor<0)
                {
                    factor=0.0;
                }
                vegf_field.push_back(factor*double(mPelletConcentration/reference_concentration));
            }
            mpSolver->SetLabel("vegf");
            mpSolver->UpdateSolution(vegf_field);
        }
        mpSolver->SetFileHandler(file_handler);
        mpSolver->SetFileName("Vegf_Field");
        mpSolver->Write();
    }
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUpSampleGrid()
{
    vtkSmartPointer<vtkAppendFilter> p_append = vtkSmartPointer<vtkAppendFilter>::New();
    mSamplingIndices.clear();

    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
    {
        mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;
        unsigned n = int(double((2.0*M_PI*mCorneaRadius)/mSampleSpacingX));

        double dimless_sample_spacing_y = mSampleSpacingY/reference_length;
        double dimless_sample_spacing_x = mSampleSpacingX/reference_length;
        double dimless_depth = mCorneaThickness/reference_length;
        double dimless_z_origin = 0.0;
        if(mDomainType == DomainType::PLANAR_2D)
        {
            dimless_z_origin = -dimless_depth/2.0;
        }
        unsigned sample_counter=0;
        for(unsigned idx=0;idx<mNumSampleY;idx++)
        {
            vtkSmartPointer<vtkUnstructuredGrid> p_sample_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
            vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
            for(unsigned jdx=0;jdx<n; jdx++)
            {
                double bottom = dimless_sample_spacing_y*double(idx);
                double top = dimless_sample_spacing_y*double(idx + 1);
                double left = dimless_sample_spacing_x*double(jdx);
                double right = dimless_sample_spacing_x*double(jdx+1);
                p_points->InsertNextPoint(left, bottom, dimless_z_origin);
                p_points->InsertNextPoint(right, bottom, dimless_z_origin);
                p_points->InsertNextPoint(right, top, dimless_z_origin);
                p_points->InsertNextPoint(left, top, dimless_z_origin);
                p_points->InsertNextPoint(left, bottom, dimless_z_origin + dimless_depth);
                p_points->InsertNextPoint(right, bottom, dimless_z_origin + dimless_depth);
                p_points->InsertNextPoint(right, top, dimless_z_origin + dimless_depth);
                p_points->InsertNextPoint(left, top, dimless_z_origin + dimless_depth);
            }
            p_sample_grid->SetPoints(p_points);
            for(unsigned jdx=0;jdx<n; jdx++)
            {
                vtkSmartPointer<vtkHexahedron> p_hex = vtkSmartPointer<vtkHexahedron>::New();
                for(unsigned hex_id=0; hex_id<8; hex_id++)
                {
                    p_hex->GetPointIds()->SetId(hex_id, hex_id + 8*jdx);
                }
                p_sample_grid->InsertNextCell(p_hex->GetCellType(), p_hex->GetPointIds());
            }
            sample_counter += n;
            mSamplingIndices.push_back(sample_counter);
            p_append->AddInputData(p_sample_grid);
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D or mDomainType == DomainType::CIRCLE_3D)
    {
        mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;
        double dimless_depth = mCorneaThickness/reference_length;
        double dimless_z_origin = 0.0;
        if(mDomainType == DomainType::CIRCLE_2D)
        {
            dimless_z_origin = -dimless_depth/2.0;
        }

        unsigned sample_counter=0;
        for(unsigned idx=0;idx<mNumSampleY;idx++)
        {
            vtkSmartPointer<vtkUnstructuredGrid> p_sample_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
            vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
            QLength outer_radius = mCorneaRadius - double(idx)*mSampleSpacingY;
            QLength inner_radius = mCorneaRadius - double(idx+1)*mSampleSpacingY;
            unsigned n = int(double((2.0*M_PI*outer_radius)/mSampleSpacingX)) + 1;
            double sweep_angle = 2.0*M_PI/double(n);
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (outer_radius/reference_length)*std::sin(this_angle);
                double y_coord = (outer_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, dimless_z_origin);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (inner_radius/reference_length)*std::sin(this_angle);
                double y_coord = (inner_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, dimless_z_origin);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (outer_radius/reference_length)*std::sin(this_angle);
                double y_coord = (outer_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, dimless_z_origin + dimless_depth);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (inner_radius/reference_length)*std::sin(this_angle);
                double y_coord = (inner_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, dimless_z_origin + dimless_depth);
            }
            p_sample_grid->SetPoints(p_points);

            for(unsigned jdx=0;jdx<n; jdx++)
            {
                unsigned next_index = jdx + 1;
                if(jdx==n-1)
                {
                    next_index = 0;
                }
                vtkSmartPointer<vtkHexahedron> p_hex = vtkSmartPointer<vtkHexahedron>::New();
                p_hex->GetPointIds()->SetId(0, jdx);
                p_hex->GetPointIds()->SetId(1, next_index);
                p_hex->GetPointIds()->SetId(2, n + next_index);
                p_hex->GetPointIds()->SetId(3, n + jdx);
                p_hex->GetPointIds()->SetId(4, 2*n + jdx);
                p_hex->GetPointIds()->SetId(5, 2*n + next_index);
                p_hex->GetPointIds()->SetId(6, 3*n + next_index);
                p_hex->GetPointIds()->SetId(7, 3*n + jdx);
                p_sample_grid->InsertNextCell(p_hex->GetCellType(), p_hex->GetPointIds());
            }
            sample_counter += n;
            mSamplingIndices.push_back(sample_counter);
            p_append->AddInputData(p_sample_grid);
        }
    }
    else if(mDomainType == DomainType::HEMISPHERE)
    {
        double pellet_angle = std::asin(double(mPelletHeight/mCorneaRadius));
        QLength y_extent = mCorneaRadius*(pellet_angle);
        mNumSampleY = int(float(y_extent/mSampleSpacingY)) + 1;
        unsigned sample_counter=0;

        for(unsigned idx=0;idx<mNumSampleY;idx++)
        {
            vtkSmartPointer<vtkUnstructuredGrid> p_sample_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
            vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
            double bottom_angle = double(idx)*pellet_angle/double(mNumSampleY);
            double top_angle = double(idx+1)*pellet_angle/double(mNumSampleY);
            QLength outer_radius = mCorneaRadius*std::cos(bottom_angle);
            QLength outer_height = mCorneaRadius*std::sin(bottom_angle);
            QLength inner_radius = (mCorneaRadius-mCorneaThickness)*std::cos(bottom_angle);
            QLength inner_height = (mCorneaRadius-mCorneaThickness)*std::sin(bottom_angle);
            QLength top_outer_radius = mCorneaRadius*std::cos(top_angle);
            QLength top_outer_height = mCorneaRadius*std::sin(top_angle);
            QLength top_inner_radius = (mCorneaRadius-mCorneaThickness)*std::cos(top_angle);
            QLength top_inner_height = (mCorneaRadius-mCorneaThickness)*std::sin(top_angle);

            unsigned n = int(double((2.0*M_PI*outer_radius)/mSampleSpacingX)) + 1;
            double sweep_angle = 2.0*M_PI/double(n);
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (outer_radius/reference_length)*std::sin(this_angle);
                double y_coord = (outer_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, outer_height/reference_length);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (inner_radius/reference_length)*std::sin(this_angle);
                double y_coord = (inner_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, inner_height/reference_length);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (top_outer_radius/reference_length)*std::sin(this_angle);
                double y_coord = (top_outer_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, top_outer_height/reference_length);
            }
            for(unsigned jdx=0;jdx<n;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle;
                double x_coord = (top_inner_radius/reference_length)*std::sin(this_angle);
                double y_coord = (top_inner_radius/reference_length)*std::cos(this_angle);
                p_points->InsertNextPoint(x_coord, y_coord, top_inner_height/reference_length);
            }
            p_sample_grid->SetPoints(p_points);
            for(unsigned jdx=0;jdx<n; jdx++)
            {
                unsigned next_index = jdx + 1;
                if(jdx==n-1)
                {
                    next_index = 0;
                }
                vtkSmartPointer<vtkHexahedron> p_hex = vtkSmartPointer<vtkHexahedron>::New();
                p_hex->GetPointIds()->SetId(0, jdx);
                p_hex->GetPointIds()->SetId(1, next_index);
                p_hex->GetPointIds()->SetId(2, n + next_index);
                p_hex->GetPointIds()->SetId(3, n + jdx);
                p_hex->GetPointIds()->SetId(4, 2*n + jdx);
                p_hex->GetPointIds()->SetId(5, 2*n + next_index);
                p_hex->GetPointIds()->SetId(6, 3*n + next_index);
                p_hex->GetPointIds()->SetId(7, 3*n + jdx);
                p_sample_grid->InsertNextCell(p_hex->GetCellType(), p_hex->GetPointIds());
            }
            sample_counter += n;
            mSamplingIndices.push_back(sample_counter);
            p_append->AddInputData(p_sample_grid);
        }
    }
    p_append->Update();
    mpSampleGrid = p_append->GetOutput();
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::DoSampling(std::ofstream& rStream,
        vtkSmartPointer<vtkUnstructuredGrid> pSampleGrid, std::string sampleType,
        double time, double multfact, bool sampleOnce)
{
    rStream << time << ",";

    for(unsigned idx=0;idx<mNumSampleY; idx++)
    {
        if(sampleOnce and time>0.0)
        {
            rStream << mStoredSample[idx] << ",";
        }
        else
        {
            if(sampleType=="Line")
            {
                unsigned low_index = 0;
                if(idx>0)
                {
                    low_index = mSamplingIndices[idx-1];
                }
                unsigned high_index = mSamplingIndices[idx];
                double average = 0.0;
                for(unsigned jdx=low_index; jdx<high_index; jdx++)
                {
                    average += pSampleGrid->GetCellData()->GetArray("Line Density")->GetTuple1(jdx);
                }
                average/=double(high_index-low_index);
                rStream << average*multfact << ",";
                if(sampleOnce and time==0.0)
                {
                    mStoredSample.push_back(average*multfact);
                }
            }
            else
            {
                unsigned low_index = 0;
                if(idx>0)
                {
                    low_index = mSamplingIndices[idx-1];
                }
                unsigned high_index = mSamplingIndices[idx];
                double average = 0.0;
                for(unsigned jdx=low_index; jdx<high_index; jdx++)
                {
                    average += pSampleGrid->GetCellData()->GetArray("Tip Density")->GetTuple1(jdx);
                }
                average/=double(high_index-low_index);
                rStream << average*multfact << ",";
                if(sampleOnce and time==0.0)
                {
                    mStoredSample.push_back(average*multfact);
                }
            }
        }
    }
    rStream << "\n";
    rStream.flush();
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::Run()
{

    SimulationTime::Instance()->SetStartTime(0.0);
    RandomNumberGenerator::Instance()->Reseed(mRandomSeed);
    auto p_file_handler = std::make_shared<OutputFileHandler>(mWorkDirectory, true);

    // self.parameter_collection.save(file_handler.GetOutputDirectoryFullPath() + "/adopted_parameter_collection.p")
    Timer::PrintAndReset("Starting Simulation");

    // Initialize length scales
    QLength reference_length = 1_um;
    QTime reference_time = 1_h;
    QConcentration reference_concentration = 1_nM;
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
    BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);

    // Set up domain, grid, vessel network and pde solver
    SetUpDomain();
    mpDomain->Write(p_file_handler->GetOutputDirectoryFullPath() + "simulation_domain.vtp");

    SetUpGrid();

    if(!mUsePdeOnly)
    {
        SetUpVesselNetwork();
    }
    SetUpSolver();

    auto p_angiogenesis_solver = AngiogenesisSolver<DIM>::Create();
    if(!mUsePdeOnly)
    {
        auto p_migration_rule = OffLatticeMigrationRule<DIM>::Create();
        p_migration_rule->SetDiscreteContinuumSolver(mpSolver);
        p_migration_rule->SetNetwork(mpNetwork);
        p_migration_rule->SetAttractionStrength(mAttractionStrength);
        p_migration_rule->SetChemotacticStrength(mChemotacticStrength);
        p_migration_rule->SetPersistenceAngleSdv((mPersistenceAngle/180.0)*M_PI);
        p_migration_rule->SetSproutingVelocity(mSproutVelocity);

        auto p_sprouting_rule = OffLatticeSproutingRule<DIM>::Create();
        p_sprouting_rule->SetDiscreteContinuumSolver(mpSolver);
        p_sprouting_rule->SetVesselNetwork(mpNetwork);
        p_sprouting_rule->SetSproutingProbability(mSproutingProbability);
        p_sprouting_rule->SetOnlySproutIfPerfused(mOnlyPerfusedSprout);
        p_sprouting_rule->SetUseLateralInhibition(true);

        p_angiogenesis_solver->SetVesselNetwork(mpNetwork);
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_angiogenesis_solver->SetOutputFileHandler(p_file_handler);
        p_angiogenesis_solver->SetBoundingDomain(mpDomain);
        p_angiogenesis_solver->SetDoAnastomosis(mDoAnastamosis);
        p_angiogenesis_solver->SetAnastamosisRadius(mAnastamosisRadius);
    }

    unsigned num_steps = unsigned(mTotalTime/mTimeStepSize);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(mTotalTime/reference_time, num_steps);
    VesselNetworkWriter<DIM> network_writer;

    SetUpSampleGrid();
    vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
    std::string file_name = p_file_handler->GetOutputDirectoryFullPath() + "samplying_grid_initial.vtu";
    p_writer->SetFileName(file_name.c_str());
    p_writer->SetInputData(mpSampleGrid);
    p_writer->Write();

    SetUpGrid();

    std::vector<std::shared_ptr<std::ofstream > > output_density_files;
    std::vector<std::string> output_density_quantities;
    output_density_quantities.push_back("Line");
    output_density_quantities.push_back("Tip");

    if(!mUsePdeOnly)
    {
        for(unsigned idx=0;idx<output_density_quantities.size();idx++)
        {
            auto p_output_file = std::make_shared<std::ofstream >();
            std::string file_path = p_file_handler->GetOutputDirectoryFullPath()+"Sampled_" +
                    output_density_quantities[idx] + "_density.txt";
            p_output_file->open(file_path.c_str());
            (*p_output_file) << "Time, ";
            for(unsigned jdx=0;jdx<mNumSampleY; jdx++)
            {
                double y_coord = 0.0;
                double z_coord = 0.0;
                if(mDomainType == DomainType::CIRCLE_3D or mDomainType == DomainType::CIRCLE_2D)
                {
                    y_coord = double(jdx)*mSampleSpacingY/reference_length + 0.5*mSampleSpacingY/reference_length;
                }
                else if(mDomainType == DomainType::HEMISPHERE)
                {
                    z_coord = double(jdx)*mSampleSpacingY/reference_length + 0.5*mSampleSpacingY/reference_length;
                    y_coord = (mCorneaRadius/reference_length)*std::asin(z_coord*reference_length/mCorneaRadius);
                }
                else
                {
                    y_coord = double(jdx)*mSampleSpacingY/reference_length + 0.5*mSampleSpacingY/reference_length;
                }
                (*p_output_file) << y_coord << ",";
            }
            (*p_output_file) << "\n";
            output_density_files.push_back(p_output_file);
        }
    }

//    std::ofstream pde_output_file;
//    std::string pde_file_path = p_file_handler->GetOutputDirectoryFullPath()+"Sampled_PDE.txt";
//    pde_output_file.open(pde_file_path.c_str());
//    pde_output_file << "Time, ";
//    for(unsigned jdx=0;jdx<mNumSampleY; jdx++)
//    {
//        double y_coord = 0.0;
//        double z_coord = 0.0;
//        if(mDomainType == DomainType::CIRCLE_3D or mDomainType == DomainType::CIRCLE_2D)
//        {
//            y_coord = mCorneaRadius/reference_length - double(jdx)*mSampleSpacingY/reference_length;
//        }
//        else if(mDomainType == DomainType::HEMISPHERE)
//        {
//            z_coord = double(jdx)*mSampleSpacingY/reference_length;
//            y_coord = (mCorneaRadius/reference_length)*std::asin(z_coord*reference_length/mCorneaRadius);
//        }
//        else
//        {
//            y_coord = double(jdx)*mSampleSpacingY/reference_length;
//        }
//        pde_output_file << y_coord << ",";
//    }
//    pde_output_file << "\n";

    double old_time = SimulationTime::Instance()->GetTime();
    unsigned counter = 0;
    while (!SimulationTime::Instance()->IsFinished())
    {
        bool sample_this_step =(counter%mSampleFrequency==0);
        double elapsed_time = SimulationTime::Instance()->GetTimeStepsElapsed();
        double time = SimulationTime::Instance()->GetTime();

        if(!mUseFixedGradient)
        {
            auto p_fe_solver =
                        std::dynamic_pointer_cast<CoupledLumpedSystemFiniteElementSolver<DIM> >(this->mpSolver);
            if(p_fe_solver)
            {
                p_fe_solver->SetStartTime(old_time);
                if(time==old_time)
                {
                    p_fe_solver->SetEndTime(time+mPdeTimeIncrement);
                }
                else
                {
                    p_fe_solver->SetEndTime(time);
                }
            }
            auto p_fd_solver =
                        std::dynamic_pointer_cast<CoupledLumpedSystemFiniteDifferenceSolver<DIM> >(this->mpSolver);
            if(p_fd_solver)
            {
                p_fd_solver->SetStartTime(old_time);
                if(time==old_time)
                {
                    p_fd_solver->SetEndTime(time+mPdeTimeIncrement);
                }
                else
                {
                    p_fd_solver->SetEndTime(time);
                }
            }
            mpSolver->SetFileName("Vegf_Solution" + std::to_string(int(elapsed_time)));
            mpSolver->Solve();
        }

//        if(sample_this_step)
//        {
//            DoSampling(pde_output_file, mpSolver, time, 1e3, mUseFixedGradient);
//        }

        if(!mUsePdeOnly)
        {
            if(sample_this_step)
            {
                DensityMap<DIM> density_map;
                DensityMap<DIM>::GetVesselLineDensity(mpSampleGrid, mpNetwork, reference_length);
                DensityMap<DIM>::GetVesselTipDensity(mpSampleGrid, mpNetwork, reference_length);
                DoSampling(*output_density_files[0], mpSampleGrid, "Line", time);
                DoSampling(*output_density_files[1], mpSampleGrid, "Tip", time);
                vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_sample_writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
                std::string file_name = p_file_handler->GetOutputDirectoryFullPath() + "/sampled_density" +
                        std::to_string(int(elapsed_time)) + ".vtu";
                p_sample_writer->SetFileName(file_name.c_str());
                p_sample_writer->SetInputData(mpSampleGrid);
                p_sample_writer->Write();
            }
            network_writer.SetFileName(
                    p_file_handler->GetOutputDirectoryFullPath() + "/vessel_network_" +
                    std::to_string(int(elapsed_time)) + ".vtp");
            network_writer.SetVesselNetwork(mpNetwork);
            network_writer.Write();
            p_angiogenesis_solver->Increment();
        }
        SimulationTime::Instance()->IncrementTimeOneStep();
        old_time = time;
        counter++;
    }
    if(!mUsePdeOnly)
    {
        for(unsigned idx=0;idx<output_density_quantities.size();idx++)
        {
            output_density_files[idx]->close();
        }
    }

    //pde_output_file.close();
    SimulationTime::Instance()->Destroy();
}

// Explicit instantiation
template class CornealMicropocketSimulation<2>;
template class CornealMicropocketSimulation<3>;
