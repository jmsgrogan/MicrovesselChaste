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

#include <boost/lexical_cast.hpp>
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

#include "Debug.hpp"

template<unsigned DIM>
CornealMicropocketSimulation<DIM>::CornealMicropocketSimulation() :
    mDomainType(DomainType::PLANAR_2D),
    mCorneaRadius(1.3E-3*unit::metres),
    mCorneaThickness(100.0e-6*unit::metres),
    mPelletHeight(1.0e-3*unit::metres),
    mPelletThickness(40.e-6*unit::metres),
    mPelletRadius(200.e-6*unit::metres),
    mLimbalOffset(100.0e-6*unit::metres),
    mGridSpacing(40.0e-6*unit::metres),
    mElementArea2d(1e3*(1.e-18*unit::metres_cubed)),
    mElementArea3d(1e4*(1.e-18*unit::metres_cubed)),
    mNodeSpacing(40.0e-6*unit::metres),
    mDensityGridSpacing(40.0e-6*unit::metres),
    mSampleSpacingX(20.0e-6*unit::metres),
    mSampleSpacingY(20.0e-6*unit::metres),
    mSampleSpacingZ(20.0e-6*unit::metres),
    mUsePellet(true),
    mFinitePelletWidth(false),
    mSproutingProbability(0.5 /(3600.0*unit::seconds)),
    mAttractionStrength(0.0),
    mChemotacticStrength(0.5),
    mPersistenceAngle(5.0),
    mTipExclusionRadius(40.0e-6*unit::metres),
    mDoAnastamosis(true),
    mPelletConcentration(0.3*unit::mole_per_metre_cubed),
    mVegfDiffusivity(6.94e-11*unit::metre_squared_per_second),
    mVegfDecayRate((-0.8/3600.0) * unit::per_second),
    mVegfBindingConstant(100.0),
    mVegfBloodConcentration(0.0*unit::mole_per_metre_cubed),
    mVegfPermeability((3.e-4/3600.0)*unit::metre_per_second),
    mUptakeRatePerCell((4.e-18/3600.0)*unit::mole_per_second),
    mPdeTimeIncrement(0.001),
    mIncludeVesselSink(true),
    mUseFixedGradient(false),
    mUsePdeOnly(false),
    mTotalTime(24.0*3600.0*unit::seconds),
    mTimeStepSize(0.5*3600.0*unit::seconds),
    mRunNumber(0),
    mRandomSeed(0),
    mpDomain(),
    mHoles(),
    mWorkDirectory(),
    mpGrid(),
    mpNetwork(),
    mpSolver(),
    mSampleLines(),
    mNumSampleY(1),
    mNumSampleZ(1),
    mpSamplingGrid()
{

}

template<unsigned DIM>
CornealMicropocketSimulation<DIM>::~CornealMicropocketSimulation()
{

}

template<unsigned DIM>
boost::shared_ptr<CornealMicropocketSimulation<DIM> > CornealMicropocketSimulation<DIM>::Create()
{
    MAKE_PTR(CornealMicropocketSimulation<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<Part<DIM> > CornealMicropocketSimulation<DIM>::SetUpDomain()
{
    mpDomain = Part<DIM>::Create();
    mHoles.clear();

    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    if(mDomainType == DomainType::PLANAR_2D)
    {
        units::quantity<unit::length> domain_width = 2.0 * M_PI * mCorneaRadius;
        units::quantity<unit::length> domain_height = mPelletHeight + mLimbalOffset;
        if(!mUsePellet)
        {
            mpDomain->AddRectangle(domain_width,
                                domain_height,
                                DimensionalChastePoint<DIM>(0.0, 0.0, 0.0));

            mpDomain->AddAttributeToEdgeIfFound(DimensionalChastePoint<DIM>(M_PI*mCorneaRadius/reference_length,
                    mPelletHeight/reference_length, 0, reference_length), "Pellet Interface", 1.0);
        }
        else
        {
            double norm_domain_height = domain_height/reference_length;
            double norm_domain_width = domain_width/reference_length;
            std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > points;
            points.push_back(DimensionalChastePoint<DIM>::Create(0.0, 0.0, 0.0, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(norm_domain_width, 0.0, 0.0, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(norm_domain_width, norm_domain_height, 0.0, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create((norm_domain_width+norm_domain_height)/2.0,
                    norm_domain_height, 0.0, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create((norm_domain_width-norm_domain_height)/2.0,
                    norm_domain_height, 0.0, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(0.0, norm_domain_height, 0.0, reference_length));

            boost::shared_ptr<Polygon<DIM > > p_polygon = Polygon<DIM>::Create(points);
            mpDomain->AddPolygon(p_polygon);
            mpDomain->AddAttributeToEdgeIfFound(DimensionalChastePoint<DIM>(domain_width/(2.0*reference_length),
                        domain_height/reference_length, 0, reference_length), "Pellet Interface", 1.0);
        }
    }
    else if(mDomainType == DomainType::PLANAR_3D)
    {
        units::quantity<unit::length> domain_width = 2.0 * M_PI * mCorneaRadius;
        units::quantity<unit::length> domain_height = mPelletHeight + mLimbalOffset;
        double norm_domain_height = domain_height/reference_length;
        double norm_domain_width = domain_width/reference_length;
        if(!mUsePellet)
        {
            mpDomain->AddCuboid(domain_width, domain_height, mCorneaThickness,
                                DimensionalChastePoint<DIM>(0.0, 0.0, 0.0));
            std::vector<boost::shared_ptr<Facet<DIM> > > facets = mpDomain->GetFacets();
            for(unsigned idx=0;idx<facets.size();idx++)
            {
                DimensionalChastePoint<DIM> probe_loc(norm_domain_width/2.0,
                        norm_domain_height, mCorneaThickness/(2.0*reference_length));
                units::quantity<unit::length> distance = facets[idx]->GetCentroid().GetDistance(probe_loc);
                if (double(distance/reference_length) < 1e-3)
                {
                    facets[idx]->GetPolygons()[0]->AddAttribute("Pellet Interface", 1.0);
                }
            }
        }
        else
        {
            mpDomain->AddCuboid(domain_width, domain_height, mCorneaThickness,
                                DimensionalChastePoint<DIM>(0.0, 0.0, 0.0));
            double gap = (mCorneaThickness - mPelletThickness)/(2.0*reference_length);
            std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > points;
            double left_side = (domain_width-mPelletRadius)/(2.0*reference_length);
            double right_side = (domain_width+mPelletRadius)/(2.0*reference_length);

            points.push_back(DimensionalChastePoint<DIM>::Create(left_side, norm_domain_height, gap, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(right_side,norm_domain_height, gap, reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(right_side, norm_domain_height,
                    mCorneaThickness/reference_length-gap,reference_length));
            points.push_back(DimensionalChastePoint<DIM>::Create(left_side, norm_domain_height,
                    mCorneaThickness/reference_length-gap,reference_length));

            boost::shared_ptr<Polygon<DIM > > p_polygon = Polygon<DIM>::Create(points);
            p_polygon->AddAttribute("Pellet Interface", 1.0);
            std::vector<boost::shared_ptr<Facet<DIM> > > facets = mpDomain->GetFacets();
            for(unsigned idx=0;idx<facets.size();idx++)
            {
                DimensionalChastePoint<DIM> probe_loc(norm_domain_width/2.0,
                        norm_domain_height, mCorneaThickness/(2.0*reference_length));
                units::quantity<unit::length> distance = facets[idx]->GetCentroid().GetDistance(probe_loc);
                if (double(distance/reference_length) < 1e-3)
                {
                    mpDomain->AddPolygon(p_polygon, false, facets[idx]);
                }
            }
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D)
    {
        units::quantity<unit::length> delta = mPelletHeight + mLimbalOffset -mCorneaRadius + mPelletRadius;
        mpDomain->AddCircle(mCorneaRadius, DimensionalChastePoint<DIM>(0.0, 0.0, 0.0), 24);
        if (mUsePellet)
        {
            boost::shared_ptr<Polygon<DIM > > p_polygon = mpDomain->AddCircle(mPelletRadius,
                    DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, 0.0, reference_length), 24);
            p_polygon->AddAttributeToAllEdges("Pellet Interface", 1.0);
            mpDomain->AddHoleMarker(DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, 0.0, reference_length));
            mHoles.push_back(DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, 0.0, reference_length));
        }
    }
    else if(mDomainType == DomainType::CIRCLE_3D)
    {
        units::quantity<unit::length> delta = mPelletHeight + mLimbalOffset -mCorneaRadius+mPelletRadius;
        boost::shared_ptr<Polygon<DIM > > p_circle = mpDomain->AddCircle(mCorneaRadius,
                DimensionalChastePoint<DIM>(0.0, 0.0, 0.0), 24);
        mpDomain->Extrude(p_circle, mCorneaThickness);

        if (mUsePellet)
        {
            double gap = (mCorneaThickness - mPelletThickness)/(2.0*reference_length);

             boost::shared_ptr<Part<DIM> > p_pellet = Part<DIM>::Create();
             p_circle = p_pellet->AddCircle(mPelletRadius,
                     DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, 0.0, reference_length), 24);
             p_pellet->Extrude(p_circle, mPelletThickness);
             p_pellet->Translate(DimensionalChastePoint<DIM>(0.0, 0.0, gap, reference_length));
             std::vector<boost::shared_ptr<Polygon<DIM > > >polygons = p_pellet->GetPolygons();

             double half_height = mCorneaThickness/(2.0*reference_length);
             mpDomain->AddHoleMarker(DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, half_height, reference_length));
             mHoles.push_back(DimensionalChastePoint<DIM>(0.0, -1.0*delta/reference_length, half_height, reference_length));
             mpDomain->AppendPart(p_pellet);
             for(unsigned idx=0;idx<polygons.size();idx++)
             {
                 polygons[idx]->AddAttribute("Pellet Interface", 1.0);
             }
        }
    }
    else if(mDomainType == DomainType::HEMISPHERE)
    {
        boost::shared_ptr<MappableGridGenerator<DIM> > p_generator = MappableGridGenerator<DIM>::Create();

        unsigned num_divisions_x = 20;
        unsigned num_divisions_y = 20;
        double azimuth_angle = 1.0 * M_PI;
        double polar_angle = 0.999 * M_PI;

        mpDomain = p_generator->GenerateHemisphere(mCorneaRadius,
                mCorneaThickness, num_divisions_x, num_divisions_y, azimuth_angle, polar_angle);

        if(mUsePellet)
        {
            boost::shared_ptr<Part<DIM> > p_pellet_domain = Part<DIM>::Create();
            double gap = (mCorneaThickness - mPelletThickness)/(2.0*reference_length)/4.0;
            double base = mCorneaRadius/reference_length + gap - mCorneaThickness/reference_length;

            p_pellet_domain->AddCylinder(mPelletRadius,
                    mPelletThickness, DimensionalChastePoint<DIM>(0.0, 0.0, base, reference_length));
            std::vector<boost::shared_ptr<Polygon<DIM > > > polygons = p_pellet_domain->GetPolygons();

            double height_fraction = double((mPelletHeight+mPelletRadius+ mLimbalOffset)/mCorneaRadius);
            double rotation_angle = std::acos(std::ceil(height_fraction - 0.5));

            DimensionalChastePoint<DIM> pellet_centre(
                    0.0, 0.0, base + mPelletThickness/(2.0*reference_length), reference_length);

            c_vector<double, 3> axis;
            axis[0] = 0.0;
            axis[1] = 1.0;
            axis[2] = 0.0;
            p_pellet_domain->RotateAboutAxis(axis, rotation_angle);
            pellet_centre.RotateAboutAxis(axis, rotation_angle);

            mpDomain->AppendPart(p_pellet_domain);
            mpDomain->AddHoleMarker(pellet_centre);

            for(unsigned idx=0;idx<polygons.size();idx++)
            {
                polygons[idx]->AddAttribute("Pellet Interface", 1.0);
            }
        }
    }
    return mpDomain;
}

template<unsigned DIM>
boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > CornealMicropocketSimulation<DIM>::SetUpGrid()
{
    if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
    {
        if(!mFinitePelletWidth)
        {
            mpGrid = RegularGrid<DIM>::Create();
            boost::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                        boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGrid);
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
    else if(mDomainType == DomainType::CIRCLE_2D or mDomainType == DomainType::CIRCLE_3D or
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
boost::shared_ptr<VesselNetwork<DIM> > CornealMicropocketSimulation<DIM>::SetUpVesselNetwork()
{
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    mpNetwork = VesselNetwork<DIM>::Create();

    if(mDomainType == DomainType::PLANAR_2D or mDomainType == DomainType::PLANAR_3D)
    {
        VesselNetworkGenerator<DIM> generator;
        units::quantity<unit::length> domain_length = 2.0*M_PI*mCorneaRadius;
        unsigned divisions = unsigned(float(domain_length/mNodeSpacing)) - 2;
        unsigned alignment_axis = 0;
        double midpoint = 0.0;
        if(mDomainType == DomainType::PLANAR_3D)
        {
            midpoint = mCorneaThickness/(2.0*reference_length);
        }
        mpNetwork = generator.GenerateSingleVessel(domain_length,
                                                 DimensionalChastePoint<DIM>(0.0,
                                                                         double(mLimbalOffset/reference_length),
                                                                         midpoint, reference_length),
                                                 divisions, alignment_axis);
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = mpNetwork->GetNodes();
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            nodes[idx]->GetFlowProperties()->SetPressure(1.0*unit::pascals);
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D or mDomainType == DomainType::CIRCLE_3D)
    {
        units::quantity<unit::length> sampling_radius = mCorneaRadius-mLimbalOffset;
        unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
        double sweep_angle = 2.0*M_PI/double(num_nodes);
        double midpoint = 0.0;
        if(mDomainType == DomainType::CIRCLE_3D)
        {
            midpoint = mCorneaThickness/(2.0*reference_length);
        }
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
        for(unsigned idx=0;idx<num_nodes;idx++)
        {
            double this_angle = double(idx)*sweep_angle+M_PI;
            double x_coord = (sampling_radius/reference_length)*std::sin(this_angle);
            double y_coord = (sampling_radius/reference_length)*std::cos(this_angle);
            nodes.push_back(VesselNode<DIM>::Create(
                DimensionalChastePoint<DIM>(x_coord, y_coord, midpoint, reference_length)));
        }
        for(unsigned idx=1;idx<num_nodes;idx++)
        {
            mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[idx-1], nodes[idx]));
        }
        mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[nodes.size()-1], nodes[0]));
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            nodes[idx]->GetFlowProperties()->SetPressure(1.0*unit::pascals);
        }
    }

    else if(mDomainType == DomainType::HEMISPHERE)
    {
        double dimless_cornea_mid_radius = double((mCorneaRadius-mCorneaThickness/2.0)/reference_length);
        double dimless_offset = double(mLimbalOffset/reference_length);
        units::quantity<unit::length> sampling_radius = std::sqrt(dimless_cornea_mid_radius*dimless_cornea_mid_radius-
                dimless_offset*dimless_offset)*reference_length;
        unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
        double sweep_angle = 2.0*M_PI/double(num_nodes);

        std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
        for(unsigned idx=0;idx<num_nodes;idx++)
        {
            double this_angle = double(idx)*sweep_angle+M_PI;
            double x_coord = (sampling_radius/reference_length)*std::sin(this_angle);
            double y_coord = (sampling_radius/reference_length)*std::cos(this_angle);
            double z_coord = dimless_offset;
            nodes.push_back(VesselNode<DIM>::Create(
                DimensionalChastePoint<DIM>(x_coord, y_coord, z_coord, reference_length)));
        }
        for(unsigned idx=1;idx<num_nodes;idx++)
        {
            mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[idx-1], nodes[idx]));
        }
        mpNetwork->AddVessel(Vessel<DIM>::Create(nodes[nodes.size()-1], nodes[0]));
        for(unsigned idx=0;idx<nodes.size();idx++)
        {
            nodes[idx]->GetFlowProperties()->SetPressure(1.0*unit::pascals);
        }
    }
    return mpNetwork;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUpSolver()
{
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    units::quantity<unit::concentration> reference_concentration =
            BaseUnits::Instance()->GetReferenceConcentrationScale();
    boost::shared_ptr<OutputFileHandler> file_handler =
            boost::shared_ptr<OutputFileHandler>(new OutputFileHandler(mWorkDirectory, false));

    if(!mUseFixedGradient)
    {
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<DIM, DIM> > p_pde =
                CoupledVegfPelletDiffusionReactionPde<DIM, DIM>::Create();
        p_pde->SetIsotropicDiffusionConstant(mVegfDiffusivity);
        p_pde->SetContinuumLinearInUTerm(mVegfDecayRate);
        p_pde->SetCurrentVegfInPellet(mPelletConcentration);
        p_pde->SetPelletBindingConstant(mVegfBindingConstant);
        p_pde->SetPelletDepth(mPelletThickness);

        units::quantity<unit::area> surface_area = 2.0*M_PI*mPelletRadius*mPelletThickness;
        surface_area += 2.0*M_PI*mPelletRadius*mPelletRadius;
        p_pde->SetPelletSurfaceArea(surface_area);
        p_pde->SetCorneaPelletPermeability(0.002*p_pde->GetCorneaPelletPermeability());

        if(mIncludeVesselSink and !mUsePdeOnly)
        {
            boost::shared_ptr<VesselBasedDiscreteSource<DIM> > p_sink =
                    VesselBasedDiscreteSource<DIM>::Create();
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
                boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > p_boundary_condition =
                        DiscreteContinuumBoundaryCondition<DIM>::Create();
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
            boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > p_boundary_condition =
                     DiscreteContinuumBoundaryCondition<DIM>::Create();
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
            boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > p_boundary_condition =
                     DiscreteContinuumBoundaryCondition<DIM>::Create();
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

        boost::shared_ptr<CoupledLumpedSystemFiniteElementSolver<DIM> > p_fe_solver =
                    boost::dynamic_pointer_cast<CoupledLumpedSystemFiniteElementSolver<DIM> >(this->mpSolver);

        p_fe_solver->SetTargetTimeIncrement(mPdeTimeIncrement);
        p_fe_solver->SetUseCoupling(true);

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
                double y_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[1];
                double dimless_pellet_height = (mPelletHeight/reference_length);
                double normalized_distance = y_loc/dimless_pellet_height;
                vegf_field.push_back(normalized_distance*(mPelletConcentration/reference_concentration));
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
                double x_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[0];
                double y_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[1];
                double radius = std::sqrt(x_loc*x_loc + y_loc*y_loc);
                double dimless_pellet_height = (mCorneaRadius/reference_length);
                double normalized_distance = double(radius/dimless_pellet_height);
                vegf_field.push_back((1.0-normalized_distance)*double(mPelletConcentration/reference_concentration));
            }
            mpSolver->SetLabel("vegf");
            mpSolver->UpdateSolution(vegf_field);
        }
        else if(mDomainType == DomainType::HEMISPHERE)
        {
            mpSolver = FunctionMap<DIM>::Create();
            mpSolver->SetGrid(mpGrid);

            std::vector<double> vegf_field;
            for(unsigned idx=0;idx<mpGrid->GetNumberOfPoints();idx++)
            {
                double x_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[0];
                double y_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[1];
                double z_loc = mpGrid->GetPoint(idx).GetLocation(reference_length)[2];
                double radius = std::sqrt(x_loc*x_loc + y_loc*y_loc);
                double angle = std::atan(z_loc/radius);
                double frac = angle/(M_PI/2.0);
                vegf_field.push_back(frac*double(mPelletConcentration/reference_concentration));
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
void CornealMicropocketSimulation<DIM>::SetUpSamplePoints()
{
    mSampleLines.clear();
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    if(mDomainType == DomainType::PLANAR_2D)
    {
        units::quantity<unit::length> domain_width = 2.0*M_PI*mCorneaRadius;
        unsigned num_sample_points_x = int(double(domain_width/mSampleSpacingX)) + 1;
        unsigned mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;

        double dimless_sample_spacing_x = mSampleSpacingX/reference_length;
        double dimless_sample_spacing_y = mSampleSpacingY/reference_length;
        double dimless_limbal_offset = mLimbalOffset/reference_length;
        for(unsigned idx=0;idx<mNumSampleY;idx++)
        {
            vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
            for(unsigned jdx=0;jdx<num_sample_points_x;jdx++)
            {
                p_sample_points->InsertNextPoint(double(jdx)*dimless_sample_spacing_x,
                        double(idx)*dimless_sample_spacing_y + dimless_limbal_offset, 0.0);
            }
            mSampleLines.push_back(p_sample_points);
        }
    }
    else if(mDomainType == DomainType::PLANAR_3D)
    {
        units::quantity<unit::length> domain_width = 2.0*M_PI*mCorneaRadius;
        unsigned num_sample_points_x = int(double(domain_width/mSampleSpacingX)) + 1;
        mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;
        mNumSampleZ = int(double(mCorneaThickness/mSampleSpacingZ)) + 1;

        double dimless_sample_spacing_x = mSampleSpacingX/reference_length;
        double dimless_sample_spacing_y = mSampleSpacingY/reference_length;
        double dimless_sample_spacing_z = mSampleSpacingZ/reference_length;
        double dimless_limbal_offset = mLimbalOffset/reference_length;
        for(unsigned kdx=0;kdx<mNumSampleZ;kdx++)
        {
            for(unsigned idx=0;idx<mNumSampleY;idx++)
            {
                vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
                for(unsigned jdx=0;jdx<num_sample_points_x;jdx++)
                {
                    p_sample_points->InsertNextPoint(double(jdx)*dimless_sample_spacing_x,
                            double(idx)*dimless_sample_spacing_y + dimless_limbal_offset,
                            double(kdx)*dimless_sample_spacing_z);
                }
                mSampleLines.push_back(p_sample_points);
            }
        }
    }
    else if(mDomainType == DomainType::CIRCLE_2D)
    {
        units::quantity<unit::length> domain_width = 2.0*M_PI*mCorneaRadius;
        unsigned num_sample_points_x = int(double(domain_width/mSampleSpacingX)) + 1;
        mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;
        for(unsigned idx=0;idx<mNumSampleY;idx++)
        {
            vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
            units::quantity<unit::length> sampling_radius = mCorneaRadius-mLimbalOffset-
                    double(idx)*mSampleSpacingY;
            unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
            double sweep_angle = 2.0*M_PI/double(num_nodes);
            for(unsigned jdx=0;jdx<num_sample_points_x;jdx++)
            {
                double this_angle = double(jdx)*sweep_angle+M_PI;
                double x_coord = (sampling_radius/reference_length)*std::sin(this_angle);
                double y_coord = (sampling_radius/reference_length)*std::cos(this_angle);
                p_sample_points->InsertNextPoint(x_coord,y_coord, 0.0);
            }
            mSampleLines.push_back(p_sample_points);
        }
    }
    else if(mDomainType == DomainType::CIRCLE_3D)
    {
        units::quantity<unit::length> domain_width = 2.0*M_PI*mCorneaRadius;
        unsigned num_sample_points_x = int(double(domain_width/mSampleSpacingX)) + 1;
        mNumSampleY = int(double((mPelletHeight)/mSampleSpacingY)) + 1;
        mNumSampleZ = int(double(mCorneaThickness/mSampleSpacingZ)) + 1;
        double dimless_sample_spacing_z = mSampleSpacingZ/reference_length;
        for(unsigned kdx=0;kdx<mNumSampleZ;kdx++)
        {
            for(unsigned idx=0;idx<mNumSampleY;idx++)
            {
                vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
                units::quantity<unit::length> sampling_radius = mCorneaRadius-mLimbalOffset-
                        double(idx)*mSampleSpacingY;
                unsigned num_nodes = int(double((2.0*M_PI*sampling_radius)/mNodeSpacing)) + 1;
                double sweep_angle = 2.0*M_PI/double(num_nodes);
                for(unsigned jdx=0;jdx<num_sample_points_x;jdx++)
                {
                    double this_angle = double(jdx)*sweep_angle+M_PI;
                    double x_coord = (sampling_radius/reference_length)*std::sin(this_angle);
                    double y_coord = (sampling_radius/reference_length)*std::cos(this_angle);
                    p_sample_points->InsertNextPoint(x_coord,y_coord, double(kdx)*dimless_sample_spacing_z);
                }
                mSampleLines.push_back(p_sample_points);
            }
        }
    }
    else if(mDomainType == DomainType::HEMISPHERE)
    {
        double pellet_angle = std::asin(double(mPelletHeight/mCorneaRadius));
        double offset_angle = std::asin(double(mLimbalOffset/mCorneaRadius));

        units::quantity<unit::length> y_extent = mCorneaRadius*(pellet_angle-offset_angle);
        mNumSampleY = int(float(y_extent/mSampleSpacingY)) + 1;
        mNumSampleZ = int(float(mCorneaThickness/mSampleSpacingZ)) + 1;

        for(unsigned kdx=0;kdx<mNumSampleZ;kdx++)
        {
            for(unsigned idx=0;idx<mNumSampleY;idx++)
            {
                vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
                double current_angle = offset_angle +
                        double(idx)*(pellet_angle-offset_angle)/double(mNumSampleY);
                units::quantity<unit::length> sample_offset_from_outside = double(kdx)*mSampleSpacingZ;
                units::quantity<unit::length> current_radius =
                        (mCorneaRadius-sample_offset_from_outside)*std::cos(current_angle);
                units::quantity<unit::length> current_height =
                        (mCorneaRadius-sample_offset_from_outside)*std::sin(current_angle);
                unsigned num_nodes = int(double((2.0*M_PI*current_radius)/mNodeSpacing)) + 1;
                double sweep_angle = 2.0*M_PI/num_nodes;
                for(unsigned jdx=0;jdx<num_nodes;jdx++)
                {
                    double this_angle = double(jdx)*sweep_angle+M_PI;
                    double x_coord = (current_radius/reference_length)*std::sin(this_angle);
                    double y_coord = (current_radius/reference_length)*std::cos(this_angle);
                    p_sample_points->InsertNextPoint(x_coord,y_coord, current_height/reference_length);
                }
                mSampleLines.push_back(p_sample_points);
            }
        }
    }
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
void CornealMicropocketSimulation<DIM>::SetCorneaThickness(units::quantity<unit::length> corneaThickness)
{
    mCorneaThickness = corneaThickness;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetDensityGridSpacing(units::quantity<unit::length> densityGridSpacing)
{
    mDensityGridSpacing = densityGridSpacing;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetDoAnastamosis(bool doAnastamosis)
{
    mDoAnastamosis = doAnastamosis;
}


template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetElementArea2d(units::quantity<unit::volume> elementArea2d)
{
    mElementArea2d = elementArea2d;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetElementArea3d(units::quantity<unit::volume> elementArea3d)
{
    mElementArea3d = elementArea3d;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetFinitePelletWidth(bool finitePelletWidth)
{
    mFinitePelletWidth = finitePelletWidth;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetGridSpacing(units::quantity<unit::length> gridSpacing)
{
    mGridSpacing = gridSpacing;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetIncludeVesselSink(bool includeVesselSink)
{
    mIncludeVesselSink = includeVesselSink;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetNodeSpacing(units::quantity<unit::length> nodeSpacing)
{
    mNodeSpacing = nodeSpacing;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPdeTimeIncrement(double pdeTimeIncrement)
{
    mPdeTimeIncrement = pdeTimeIncrement;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetPelletConcentration(units::quantity<unit::concentration> pelletConcentration)
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
void CornealMicropocketSimulation<DIM>::SetSampleSpacingX(units::quantity<unit::length> sampleSpacingX)
{
    mSampleSpacingX = sampleSpacingX;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSampleSpacingY(units::quantity<unit::length> sampleSpacingY)
{
    mSampleSpacingY = sampleSpacingY;
}


template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSampleSpacingZ(units::quantity<unit::length> sampleSpacingZ)
{
    mSampleSpacingZ = sampleSpacingZ;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetSproutingProbability(units::quantity<unit::rate> sproutingProbability)
{
    mSproutingProbability = sproutingProbability;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTimeStepSize(units::quantity<unit::time> timeStepSize)
{
    mTimeStepSize = timeStepSize;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTipExclusionRadius(units::quantity<unit::length> tipExclusionRadius)
{
    mTipExclusionRadius = tipExclusionRadius;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetTotalTime(units::quantity<unit::time> totalTime)
{
    mTotalTime = totalTime;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetUptakeRatePerCell(units::quantity<unit::molar_flow_rate> uptakeRatePerCell)
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
void CornealMicropocketSimulation<DIM>::SetVegfBloodConcentration(units::quantity<unit::concentration> vegfBloodConcentration)
{
    mVegfBloodConcentration = vegfBloodConcentration;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetDomainType(DomainType::Value domainType)
{
    mDomainType = domainType;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfDecayRate(units::quantity<unit::rate> vegfDecayRate)
{
    mVegfDecayRate = vegfDecayRate;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetWorkDir(std::string workDir)
{
    mWorkDirectory = workDir;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfDiffusivity(units::quantity<unit::diffusivity> vegfDiffusivity)
{
    mVegfDiffusivity = vegfDiffusivity;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::SetVegfPermeability(units::quantity<unit::membrane_permeability> vegfPermeability)
{
    mVegfPermeability = vegfPermeability;
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::DoSampling(std::ofstream& rStream,
        boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver, double time, double multfact)
{
    rStream << time << ",";
    for(unsigned idx=0;idx<mNumSampleY; idx++)
    {
        double mean = 0.0;
        for(unsigned jdx=0;jdx<mNumSampleZ; jdx++)
        {
            unsigned sample_index = jdx*mNumSampleY + idx;
            std::vector<double> solution = pSolver->GetSolution(mSampleLines[sample_index]);
            double average = std::accumulate(solution.begin(), solution.end(), 0.0)/solution.size();
            mean += average;
        }
        mean /= double(mNumSampleZ);
        rStream << mean*multfact << ",";
    }

    rStream << "\n";
    rStream.flush();
}

template<unsigned DIM>
void CornealMicropocketSimulation<DIM>::Run()
{
    SimulationTime::Instance()->SetStartTime(0.0);
    RandomNumberGenerator::Instance()->Reseed(mRandomSeed);
    boost::shared_ptr<OutputFileHandler> p_file_handler =
            boost::shared_ptr<OutputFileHandler>(new OutputFileHandler(mWorkDirectory, true));

    // self.parameter_collection.save(file_handler.GetOutputDirectoryFullPath() + "/adopted_parameter_collection.p")
    std::cout << "Running Simulation in: " << p_file_handler->GetOutputDirectoryFullPath() << std::endl;
    std::cout << "With Fixed Gradient: " << mUseFixedGradient << std::endl;
    std::cout << "With PDE Only: " << mUsePdeOnly << std::endl;

    // Initialize length scales
    units::quantity<unit::length> reference_length = 1.e-6*unit::metres;
    units::quantity<unit::time> reference_time = 3600.0*unit::seconds;
    units::quantity<unit::concentration> reference_concentration = 1.e-6*unit::mole_per_metre_cubed;
    BaseUnits::Instance()->SetReferenceLengthScale(reference_length);
    BaseUnits::Instance()->SetReferenceTimeScale(reference_time);
    BaseUnits::Instance()->SetReferenceConcentrationScale(reference_concentration);

    // Set up domain, grid, vessel network and pde solver
    SetUpDomain();
    SetUpGrid();
    if(!mUsePdeOnly)
    {
        SetUpVesselNetwork();
    }
    SetUpSolver();
    boost::shared_ptr<AngiogenesisSolver<DIM> > p_angiogenesis_solver =
            AngiogenesisSolver<DIM>::Create();

    if(!mUsePdeOnly)
    {
        boost::shared_ptr<OffLatticeMigrationRule<DIM> > p_migration_rule =
                OffLatticeMigrationRule<DIM>::Create();
        p_migration_rule->SetDiscreteContinuumSolver(mpSolver);
        p_migration_rule->SetNetwork(mpNetwork);
        p_migration_rule->SetAttractionStrength(mAttractionStrength);
        p_migration_rule->SetChemotacticStrength(mChemotacticStrength);
        p_migration_rule->SetPersistenceAngleSdv((mPersistenceAngle/180.0)*M_PI);

        boost::shared_ptr<OffLatticeSproutingRule<DIM> > p_sprouting_rule =
                OffLatticeSproutingRule<DIM>::Create();
        p_sprouting_rule->SetDiscreteContinuumSolver(mpSolver);
        p_sprouting_rule->SetVesselNetwork(mpNetwork);
        p_sprouting_rule->SetSproutingProbability(mSproutingProbability);
        //p_sprouting_rule->SetOnlySproutIfPerfused(true);
        p_sprouting_rule->SetTipExclusionRadius(mTipExclusionRadius);

        p_angiogenesis_solver->SetVesselNetwork(mpNetwork);
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);
        p_angiogenesis_solver->SetSproutingRule(p_sprouting_rule);
        p_angiogenesis_solver->SetOutputFileHandler(p_file_handler);
        p_angiogenesis_solver->SetBoundingDomain(mpDomain);
        p_angiogenesis_solver->SetDoAnastomosis(mDoAnastamosis);
    }

    unsigned num_steps = unsigned(mTotalTime/mTimeStepSize);
    SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(mTotalTime/reference_time, num_steps);
    VesselNetworkWriter<DIM> network_writer;

    SetUpSamplePoints();
    if(mDomainType == DomainType::PLANAR_3D or mDomainType == DomainType::PLANAR_2D)
    {
        mpSamplingGrid = RegularGrid<DIM>::Create();
        boost::shared_ptr<RegularGrid<DIM> > p_regular_grid =
                    boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpSamplingGrid);
        p_regular_grid->GenerateFromPart(mpDomain, mDensityGridSpacing);
    }
    else
    {
        mpSamplingGrid = mpGrid;
    }

    std::vector<boost::shared_ptr<std::ofstream > > output_density_files;
    std::vector<std::string> output_density_quantities;
    output_density_quantities.push_back("Line");
    output_density_quantities.push_back("Branch");
    output_density_quantities.push_back("Tip");

    if(!mUsePdeOnly)
    {
        for(unsigned idx=0;idx<output_density_quantities.size();idx++)
        {
            boost::shared_ptr<std::ofstream > p_output_file =
                    boost::shared_ptr<std::ofstream >(new std::ofstream);
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
                    y_coord = mCorneaRadius/reference_length + mSampleLines[jdx]->GetPoint(0)[1];
                }
                else if(mDomainType == DomainType::HEMISPHERE)
                {
                    z_coord = mSampleLines[jdx]->GetPoint(0)[2];
                    y_coord = (mCorneaRadius/reference_length)*std::atan(z_coord*reference_length/mCorneaRadius);
                }
                else
                {
                    y_coord = mSampleLines[jdx]->GetPoint(0)[1];
                }
                (*p_output_file) << y_coord << ",";
            }
            (*p_output_file) << "\n";
            output_density_files.push_back(p_output_file);
        }
    }

    std::ofstream pde_output_file;
    std::string pde_file_path = p_file_handler->GetOutputDirectoryFullPath()+"Sampled_PDE.txt";
    pde_output_file.open(pde_file_path.c_str());
    pde_output_file << "Time, ";
    for(unsigned jdx=0;jdx<mNumSampleY; jdx++)
    {
        double y_coord = 0.0;
        double z_coord = 0.0;
        if(mDomainType == DomainType::CIRCLE_3D or mDomainType == DomainType::CIRCLE_2D)
        {
            y_coord = mCorneaRadius/reference_length + mSampleLines[jdx]->GetPoint(0)[1];
        }
        else if(mDomainType == DomainType::HEMISPHERE)
        {
            z_coord = mSampleLines[jdx]->GetPoint(0)[2];
            y_coord = (mCorneaRadius/reference_length)*std::atan(z_coord*reference_length/mCorneaRadius);
        }
        else
        {
            y_coord = mSampleLines[jdx]->GetPoint(0)[1];
        }
        pde_output_file << y_coord << ",";
    }
    pde_output_file << "\n";

    double old_time = SimulationTime::Instance()->GetTime();
    while (!SimulationTime::Instance()->IsFinished())
    {
        std::cout << "Simulation Time: " << old_time << " of " << mTotalTime << std::endl;
        double elapsed_time = SimulationTime::Instance()->GetTimeStepsElapsed();
        double time = SimulationTime::Instance()->GetTime();

        if(!mUseFixedGradient)
        {
            boost::shared_ptr<CoupledLumpedSystemFiniteElementSolver<DIM> > p_fe_solver =
                        boost::dynamic_pointer_cast<CoupledLumpedSystemFiniteElementSolver<DIM> >(this->mpSolver);

            p_fe_solver->SetStartTime(old_time);
            if(time==old_time)
            {
                p_fe_solver->SetEndTime(time+mPdeTimeIncrement);
            }
            else
            {
                p_fe_solver->SetEndTime(time);
            }
            mpSolver->SetFileName("Vegf_Solution" + boost::lexical_cast<std::string>(elapsed_time));
            mpSolver->Solve();
        }

        DoSampling(pde_output_file, mpSolver, time, 1e3);

        if(!mUsePdeOnly)
        {
            DensityMap<DIM> density_map;
            density_map.SetGrid(mpSamplingGrid);
            density_map.SetVesselNetwork(mpNetwork);
            boost::shared_ptr<FunctionMap<DIM> > p_density_map_result = FunctionMap<DIM>::Create();
            p_density_map_result->SetGrid(mpSamplingGrid);
            p_density_map_result->SetVesselNetwork(mpNetwork);
            p_density_map_result->SetFileHandler(p_file_handler);

            for(unsigned idx=0;idx<output_density_quantities.size();idx++)
            {
                p_density_map_result->SetFileName("/" + output_density_quantities[idx] + "_Density" +
                        boost::lexical_cast<std::string>(elapsed_time));
                if(output_density_quantities[idx]=="Line")
                {
                    if(mDomainType == DomainType::PLANAR_3D or mDomainType == DomainType::PLANAR_2D)
                    {
                        p_density_map_result->UpdateSolution(density_map.rGetVesselLineDensity());
                    }
                    else
                    {
                        p_density_map_result->UpdateElementSolution(density_map.rGetVesselLineDensity());
                    }
                }
                if(output_density_quantities[idx]=="Tip")
                {
                    if(mDomainType == DomainType::PLANAR_3D or mDomainType == DomainType::PLANAR_2D)
                    {
                        p_density_map_result->UpdateSolution(density_map.rGetVesselTipDensity());
                    }
                    else
                    {
                        p_density_map_result->UpdateElementSolution(density_map.rGetVesselTipDensity());
                    }
                }
                if(output_density_quantities[idx]=="Branch")
                {
                    if(mDomainType == DomainType::PLANAR_3D or mDomainType == DomainType::PLANAR_2D)
                    {
                        p_density_map_result->UpdateSolution(density_map.rGetVesselBranchDensity());
                    }
                    else
                    {
                        p_density_map_result->UpdateElementSolution(density_map.rGetVesselBranchDensity());
                    }
                }

                p_density_map_result->Write();
                DoSampling(*output_density_files[idx], p_density_map_result, time);
            }
            network_writer.SetFileName(
                    p_file_handler->GetOutputDirectoryFullPath() + "/vessel_network_" +
                    boost::lexical_cast<std::string>(elapsed_time) + ".vtp");
            network_writer.SetVesselNetwork(mpNetwork);
            network_writer.Write();
            p_angiogenesis_solver->Increment();
        }
        SimulationTime::Instance()->IncrementTimeOneStep();
        old_time = time;
    }
    if(!mUsePdeOnly)
    {
        for(unsigned idx=0;idx<output_density_quantities.size();idx++)
        {
            output_density_files[idx]->close();
        }
    }

    pde_output_file.close();
    SimulationTime::Instance()->Destroy();
}

// Explicit instantiation
template class CornealMicropocketSimulation<2>;
template class CornealMicropocketSimulation<3>;
