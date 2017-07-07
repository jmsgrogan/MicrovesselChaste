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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkImageData.h>
#include <vtkImageEuclideanDistance.h>
#include <vtkGradientFilter.h>
#include <vtkProbeFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include "GeometryTools.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "RandomNumberGenerator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
OffLatticeMigrationRule<DIM>::OffLatticeMigrationRule()
    : AbstractMigrationRule<DIM>(),
      mGlobalX(unit_vector<double>(3,0)),
      mGlobalY(unit_vector<double>(3,1)),
      mGlobalZ(unit_vector<double>(3,2)),
      mMeanAngles(std::vector<QAngle >(3, 0.0*unit::radians)),
      mSdvAngles(std::vector<QAngle >(3, M_PI/6.0*unit::radians)), //formerly pi/18
      mVelocity(20.0 *(1.e-6/3600.0) * unit::metre_per_second),
      mChemotacticStrength(0.6),
      mAttractionStrength(0.0), // was 1.0
      mProbeLength(5.0 * 1.e-6 * unit::metres),
      mCriticalMutualAttractionLength(100.0 * 1.e-6 *unit::metres),
      mSurfaceRepulsion(true),
      mNumGradientEvaluationDivisions(8),
      mpDomainDistanceMap()
{

}

template <unsigned DIM>
std::shared_ptr<OffLatticeMigrationRule<DIM> > OffLatticeMigrationRule<DIM>::Create()
{
    return std::make_shared<OffLatticeMigrationRule<DIM> >();

}

template<unsigned DIM>
OffLatticeMigrationRule<DIM>::~OffLatticeMigrationRule()
{

}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::CalculateDomainDistanceMap()
{
    // No mesh provided, so need to set up our own
    if(!this->mpBoundingDomain)
    {
        EXCEPTION("Can't calculate a domain distance map without a bounding domain.");
    }

    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    std::vector<QLength > bbox = this->mpBoundingDomain->GetBoundingBox();
    vtkSmartPointer<vtkImageData> p_image = vtkSmartPointer<vtkImageData>::New();

    double spacing = (bbox[1] - bbox[0])/(20.0*reference_length);
    unsigned num_x = unsigned((bbox[1] - bbox[0])/(reference_length*spacing)) + 1;
    unsigned num_y = unsigned((bbox[3] - bbox[2])/(reference_length*spacing)) + 1;
    unsigned num_z = unsigned((bbox[5] - bbox[4])/(reference_length*spacing)) + 1;
    p_image->SetOrigin(bbox[0]/reference_length, bbox[2]/reference_length, bbox[4]/reference_length);
    p_image->SetSpacing(spacing, spacing, spacing);
    p_image->SetDimensions(num_x, num_y, num_z);

    vtkSmartPointer<vtkImageEuclideanDistance> p_distance = vtkSmartPointer<vtkImageEuclideanDistance>::New();
	#if VTK_MAJOR_VERSION <= 5
    p_distance->SetInput( 0, this->mpBoundingDomain->GetVtk());
    p_distance->SetInput( 1, p_image);
	#else
    p_distance->SetInputData( 0, this->mpBoundingDomain->GetVtk());
    p_distance->SetInputData( 1, p_image);
	#endif
    p_distance->Update();
    mpDomainDistanceMap = p_distance->GetOutput();

    vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_gradient->SetInput(mpDomainDistanceMap);
    #else
        p_gradient->SetInputData(mpDomainDistanceMap);
    #endif
    p_gradient->SetResultArrayName("Distance Gradients");
    p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
    p_gradient->Update();
    mpDomainDistanceMap->GetPointData()->AddArray(p_gradient->GetOutput()->GetPointData()->GetArray("Distance Gradients"));

}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::CalculateDomainDistanceMap(std::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    if(!this->mpBoundingDomain)
    {
        EXCEPTION("Can't calculate a domain distance map without a bounding domain.");
    }
    mpDomainDistanceMap = pGrid->CalculateDistanceMap(this->mpBoundingDomain);

    vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_gradient->SetInput(mpDomainDistanceMap);
    #else
        p_gradient->SetInputData(mpDomainDistanceMap);
    #endif
    p_gradient->SetResultArrayName("Distance Gradients");
    p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
    p_gradient->Update();
    mpDomainDistanceMap->GetPointData()->AddArray(p_gradient->GetOutput()->GetPointData()->GetArray("Distance Gradients"));
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetSproutingVelocity(QVelocity velocity)
{
    mVelocity = velocity;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetChemotacticStrength(double strength)
{
    mChemotacticStrength = strength;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetAttractionStrength(double strength)
{
    mAttractionStrength = strength;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetPersistenceAngleSdv(double angle)
{
    mSdvAngles = std::vector<QAngle >(3, angle*unit::radians);
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetNumGradientEvaluationDivisions(unsigned numDivisions)
{
    mNumGradientEvaluationDivisions = numDivisions;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > OffLatticeMigrationRule<DIM>::GetDirections(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if (this->mIsSprouting)
    {
        return GetDirectionsForSprouts(rNodes);
    }
    else
    {
        if(!mpDomainDistanceMap and mSurfaceRepulsion)
        {
            if(this->mpSolver)
            {
                CalculateDomainDistanceMap(this->mpSolver->GetDensityMap()->GetGridCalculator()->GetGrid());
            }
            else
            {
                CalculateDomainDistanceMap();
            }
        }

        QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
        std::vector<DimensionalChastePoint<DIM> > movement_vectors;

        // We want to probe the PDE solution all at once first if needed, as this is an expensive operation if done node-by-node.
        // Every node has 4 probes in 2D and 6 in 3D.
        std::vector<c_vector<double, 3> > solution_gradients(rNodes.size(), zero_vector<double>(3));
        vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();

        c_vector<double, DIM> current_loc;
        c_vector<double, DIM> current_dir;
        DimensionalChastePoint<DIM> currentDirection;
        for(unsigned idx=0; idx<rNodes.size(); idx++)
        {
            current_loc = rNodes[idx]->rGetLocation().GetLocation(reference_length);
            if(DIM==3)
            {
                p_probe_locations->InsertNextPoint(&current_loc[0]);
            }
            else
            {
                p_probe_locations->InsertNextPoint(current_loc[0], current_loc[1], 0.0);
            }
        }
        if(this->mpSolver)
        {
            if(p_probe_locations->GetNumberOfPoints()>0)
            {
                solution_gradients = this->mpSolver->GetSolutionGradients(p_probe_locations);
            }
        }

        // Also probe the distance map and gradients
        std::vector<c_vector<double, 3> > distance_map_gradients(rNodes.size(), zero_vector<double>(3));
        std::vector<double> distance_map_values(rNodes.size(), 0.0);

        if(mSurfaceRepulsion)
        {
            // Sample the field at these locations
            vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
            p_polydata->SetPoints(p_probe_locations);

            vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_probe_filter->SetInput(p_polydata);
                p_probe_filter->SetSource(mpDomainDistanceMap);
            #else
                p_probe_filter->SetInputData(p_polydata);
                p_probe_filter->SetSourceData(mpDomainDistanceMap);
            #endif
            p_probe_filter->SetValidPointMaskArrayName("Valid Distance");
            p_probe_filter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance Gradients");
            p_probe_filter->Update();
            vtkDataArray* p_results = p_probe_filter->GetOutput()->GetPointData()->GetArray("Distance Gradients");
            unsigned num_points = p_results->GetNumberOfTuples();
            for(unsigned idx=0; idx<num_points; idx++)
            {
                c_vector<double, 3> result;
                p_results->GetTuple(idx, &result[0]);
                distance_map_gradients[idx] = result;
            }

            p_probe_filter->SetValidPointMaskArrayName("Valid Distance");
            p_probe_filter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
            p_probe_filter->Update();

            p_results = p_probe_filter->GetOutput()->GetPointData()->GetArray("Distance");
            num_points = p_results->GetNumberOfTuples();
            for(unsigned idx=0; idx<num_points; idx++)
            {
                if(p_probe_filter->GetOutput()->GetPointData()->GetArray("Valid Distance")->GetTuple1(idx))
                {
                    distance_map_values[idx] = p_results->GetTuple1(idx);
                }
                else
                {
                    // Have left domain
                    distance_map_values[idx] = -1.0;
                }
            }
        }

        for(unsigned idx=0; idx<rNodes.size(); idx++)
        {
            QTime time_increment = SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
            currentDirection = rNodes[idx]->rGetLocation()-rNodes[idx]->GetSegments()[0]->GetOppositeNode(rNodes[idx])->rGetLocation();
            current_dir = currentDirection.GetUnitVector();

            c_vector<double, DIM> new_direction = zero_vector<double>(DIM);
            c_vector<double, DIM> persistence_direction = zero_vector<double>(DIM);
            c_vector<double, DIM> repulsion_direction = zero_vector<double>(DIM);
            c_vector<double, DIM> chemotaxis_direction = zero_vector<double>(DIM);
            c_vector<double, DIM> attraction_direction = zero_vector<double>(DIM);

            // Persistent random walk
            QRate rate_of_angular_change = 1.0/(0.5*3600.0*unit::seconds);
            double angle_fraction = rate_of_angular_change*time_increment;

            QAngle angle_x = mMeanAngles[0];
            if(mSdvAngles[0]>0.0*unit::radians)
            {
                angle_x =RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[0]/unit::radians,
                        mSdvAngles[0]/unit::radians)*unit::radians*angle_fraction;

            }
            QAngle angle_y = mMeanAngles[1];
            if(mSdvAngles[1]>0.0*unit::radians)
            {
                angle_y = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[1]/unit::radians,
                        mSdvAngles[1]/unit::radians)*unit::radians*angle_fraction;

            }
            QAngle angle_z = mMeanAngles[2];
            if(mSdvAngles[2]>0.0*unit::radians)
            {
                angle_z = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[2]/unit::radians,
                        mSdvAngles[2]/unit::radians)*unit::radians*angle_fraction;
            }

            if(DIM==3)
            {
                c_vector<double, DIM> new_direction_z = RotateAboutAxis<DIM>(currentDirection.GetUnitVector(), mGlobalZ, angle_z);
                c_vector<double, DIM> new_direction_y = RotateAboutAxis<DIM>(new_direction_z, mGlobalY, angle_y);
                persistence_direction = RotateAboutAxis<DIM>(new_direction_y, mGlobalX, angle_x);
                persistence_direction /= norm_2(persistence_direction);
            }
            else
            {
                persistence_direction = RotateAboutAxis<DIM>(currentDirection.GetUnitVector(), mGlobalZ, angle_z);
                persistence_direction /= norm_2(persistence_direction);
            }

            // Chemotaxis
            if(this->mpSolver and mChemotacticStrength>0.0)
            {
                // Get the gradients
                c_vector<double, 3> gradient = solution_gradients[idx];
                if (norm_2(gradient)>0.0)
                {
                    gradient/=norm_2(gradient);
                }

                if(DIM==3)
                {
                    chemotaxis_direction = gradient;
                }
                else
                {
                    c_vector<double, DIM> gradient_2d;
                    gradient_2d[0] = gradient[0];
                    gradient_2d[1] = gradient[1];
                    chemotaxis_direction = gradient_2d;
                }
            }

            if(mAttractionStrength>0.0)
            {
                // Mutual Attraction
                std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = this->mpVesselNetwork->GetNodes();
                QLength min_distance = 1.e12*unit::metres;
                c_vector<double, DIM> min_direction = zero_vector<double>(DIM);
                for(unsigned jdx=0; jdx<nodes.size(); jdx++)
                {
                    if(IsPointInCone<DIM>(nodes[jdx]->rGetLocation(), rNodes[idx]->rGetLocation(), rNodes[idx]->rGetLocation() +
                                          DimensionalChastePoint<DIM>(currentDirection.GetLocation(reference_length) * double(mCriticalMutualAttractionLength/reference_length), reference_length), M_PI/3.0))
                    {
                        QLength distance = rNodes[idx]->rGetLocation().GetDistance(nodes[jdx]->rGetLocation());
                        if(distance < min_distance)
                        {
                            min_distance = distance;
                            DimensionalChastePoint<DIM> dim_min_direction = nodes[jdx]->rGetLocation() - rNodes[idx]->rGetLocation();
                            min_direction = dim_min_direction.GetUnitVector();
                        }
                    }
                }

                double strength = 0.0;
                if(min_distance < mCriticalMutualAttractionLength)
                {
                    strength = (1.0 - (min_distance * min_distance) / (mCriticalMutualAttractionLength * mCriticalMutualAttractionLength));
                }
                attraction_direction =  strength * min_direction;
                attraction_direction /= norm_2(attraction_direction);
            }

            // Surface repulsion
            if(mSurfaceRepulsion)
            {
                double critical_repulsion_distance = (25.0e-6*unit::metres)/reference_length;
                double max_repulsion_strength = 5.0;
                if(this->mpBoundingDomain or this->mpSolver)
                {
                    double current_distance = distance_map_values[idx];
                    current_loc = rNodes[idx]->rGetLocation().GetLocation(reference_length);
                    if(current_distance<critical_repulsion_distance)
                    {
                        if(current_distance>=0.0)
                        {
                            c_vector<double, 3> distance_gradient = distance_map_gradients[idx];
                            distance_gradient/=norm_2(distance_gradient);

                            double value = 1.0-(2.0*current_distance/(critical_repulsion_distance+current_distance));
                            double repulsion_strength = max_repulsion_strength*value;
                            if(DIM==3)
                            {
                                double dot_prod = inner_prod(new_direction, distance_gradient);
                                c_vector<double, DIM> arb_normal = GetArbitaryUnitNormal<DIM>(distance_gradient);
                                if(dot_prod<-0.95 and dot_prod>-1.05)
                                {
                                    double angle = 2.0*M_PI*RandomNumberGenerator::Instance()->ranf();
                                    c_vector<double, DIM> random_normal = RotateAboutAxis<DIM>(arb_normal, distance_gradient, angle*unit::radians);
                                    repulsion_direction = 2.0 * repulsion_strength * random_normal;
                                }
                                else
                                {
                                    c_vector<double, DIM> directed_normal = current_dir -
                                            inner_prod(current_dir, distance_gradient)*distance_gradient;
                                    directed_normal /=norm_2(directed_normal);
                                    repulsion_direction = repulsion_strength * (directed_normal + 0.1*distance_gradient);
                                }
                            }
                            else
                            {
                                c_vector<double, DIM> dist_grad_2d;
                                dist_grad_2d[0] = distance_gradient[0];
                                dist_grad_2d[1] = distance_gradient[1];
                                double dot_prod = inner_prod(new_direction, dist_grad_2d);
                                c_vector<double, DIM> arb_normal = GetArbitaryUnitNormal<DIM>(dist_grad_2d);
                                if(dot_prod<-0.95 and dot_prod>-1.05)
                                {
                                    double mult = 1.0;
                                    if(RandomNumberGenerator::Instance()->ranf()<0.5)
                                    {
                                        mult=-1.0;
                                    }
                                    repulsion_direction= 2.0*repulsion_strength * mult*arb_normal;
                                }
                                else
                                {
                                    if(inner_prod(current_dir, arb_normal)<0.0)
                                    {
                                        arb_normal*=-1.0;
                                    }
                                    repulsion_direction = repulsion_strength * (arb_normal + 0.1*dist_grad_2d);

                                }
                            }
                        }
                        else
                        {
                            // Have left domain...reflect back in, almost back on self
                            repulsion_direction = max_repulsion_strength * -(current_dir+0.1*GetArbitaryUnitNormal<DIM>(current_dir));
                        }
                        repulsion_direction /= norm_2(repulsion_direction);
                    }
                }
            }

            new_direction = (persistence_direction + mChemotacticStrength*chemotaxis_direction + mAttractionStrength*attraction_direction)/
                    (1.0 + mChemotacticStrength +  mAttractionStrength);
            new_direction /= norm_2(new_direction); // just making sure

            // This goes last to over-ride everything else
            new_direction += repulsion_direction;
            new_direction /= norm_2(new_direction);

            // Get the movement increment
            QLength increment_length = time_increment* mVelocity;
            movement_vectors.push_back(OffsetAlongVector<DIM>(new_direction, increment_length, reference_length));
        }
        return movement_vectors;
    }
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > OffLatticeMigrationRule<DIM>::GetDirectionsForSprouts(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    std::vector<DimensionalChastePoint<DIM> > movement_vectors;
    QLength reference_length = BaseUnits::Instance()->GetReferenceLengthScale();

    // Collect the probe locations for each node
    std::vector<QConcentration > probed_solutions(rNodes.size());
    std::vector<c_vector<double, 3> > solution_gradients(rNodes.size(), zero_vector<double>(3));
    vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();

    c_vector<double, DIM> current_loc;
    c_vector<double, DIM> current_dir;
    DimensionalChastePoint<DIM> currentDirection;
    for(unsigned idx=0; idx<rNodes.size(); idx++)
    {
        current_loc = rNodes[idx]->rGetLocation().GetLocation(reference_length);
        if(DIM==3)
        {
            p_probe_locations->InsertNextPoint(&current_loc[0]);
        }
        else
        {
            p_probe_locations->InsertNextPoint(current_loc[0], current_loc[1], 0.0);
        }
    }
    if(this->mpSolver)
    {
        if(p_probe_locations->GetNumberOfPoints()>0)
        {
            probed_solutions = this->mpSolver->GetConcentrations(p_probe_locations);
            solution_gradients = this->mpSolver->GetSolutionGradients(p_probe_locations);
        }
    }
    std::vector<bool> candidate_locations_inside_domain(rNodes.size(), true);
    if(this->mpSolver)
    {
        if (this->mpBoundingDomain)
        {
            candidate_locations_inside_domain = this->mpBoundingDomain->IsPointInPart(p_probe_locations);
        }
    }

    // Decide on the sprout directions
    for(unsigned idx=0; idx<rNodes.size(); idx++)
    {
        current_loc = rNodes[idx]->rGetLocation().GetLocation(reference_length);
        c_vector<double, DIM> new_direction = zero_vector<double>(DIM);

        // Get the segment tangent
        c_vector<double, DIM> dir1 = rNodes[idx]->GetSegment(0)->GetUnitTangent();
        c_vector<double, DIM> dir2 = rNodes[idx]->GetSegment(1)->GetUnitTangent();

        c_vector<double, DIM> av_tangent = (dir1+dir2);
        if(norm_2(av_tangent)==0.0)
        {
            av_tangent = (dir1-dir2);
        }
        av_tangent/=norm_2(av_tangent);
        if(this->mpSolver)
        {
            c_vector<double, 3> gradient = solution_gradients[idx];
            gradient/=norm_2(gradient);
            if(DIM==3)
            {
                c_vector<double, 3> av_tangent_3d;
                av_tangent_3d[0] = av_tangent[0];
                av_tangent_3d[1] = av_tangent[1];
                av_tangent_3d[2] = av_tangent[2];
                c_vector<double, 3> rot_axis = VectorProduct(gradient, av_tangent_3d);
                if(norm_2(rot_axis)==0.0)
                {
                    new_direction = GetArbitaryUnitNormal<DIM>(av_tangent);
                    double rand = RandomNumberGenerator::Instance()->ranf()*2.0*M_PI;
                    new_direction = RotateAboutAxis<DIM>(av_tangent, new_direction, rand*unit::radians);
                }
                else
                {
                    double angle = std::acos(inner_prod(av_tangent, gradient));
                    double angle_threshold = M_PI/20.0;
                    double rot_angle = M_PI/2.0;
                    if(angle<angle_threshold)
                    {
                        if(RandomNumberGenerator::Instance()->ranf()>0.5)
                        {
                            rot_angle*=-1.0;
                        }
                    }
                    else if(angle<0.0)
                    {
                        rot_angle = -M_PI/2.0 - angle;
                    }
                    else if(angle >0.0)
                    {
                        rot_angle = M_PI/2.0 - angle;
                    }
                    new_direction = RotateAboutAxis<DIM>(gradient, rot_axis, rot_angle*unit::radians);
                }
            }
            else
            {
                c_vector<double, 2> gradient_2d;
                gradient_2d[0] = solution_gradients[idx][0];
                gradient_2d[1] = solution_gradients[idx][1];
                gradient_2d/=norm_2(gradient_2d);
                double angle = std::acos(inner_prod(av_tangent, gradient_2d));
                double rot_angle = M_PI/2.0;
                double angle_threshold = M_PI/20.0;
                if(angle<angle_threshold)
                {
                    if(RandomNumberGenerator::Instance()->ranf()>0.5)
                    {
                        rot_angle*=-1.0;
                    }
                }
                else if(angle<0.0)
                {
                    rot_angle = -M_PI/2.0 - angle;
                }
                else if(angle >0.0)
                {
                    rot_angle = M_PI/2.0 - angle;
                }
                c_vector<double, 3> rot_axis;
                rot_axis[0] = 0.0;
                rot_axis[1] = 0.0;
                rot_axis[2] = 1.0;
                new_direction = RotateAboutAxis<DIM>(gradient_2d, rot_axis, rot_angle*unit::radians);
            }
        }
        else
        {
            if(DIM==2)
            {
                new_direction = GetArbitaryUnitNormal<DIM>(av_tangent);
            }
            else
            {
                new_direction = GetArbitaryUnitNormal<DIM>(av_tangent);
                double rand = RandomNumberGenerator::Instance()->ranf()*2.0*M_PI;
                new_direction = RotateAboutAxis<DIM>(av_tangent, new_direction, rand*unit::radians);
            }
        }
        new_direction/=norm_2(new_direction);
        QTime time_increment = SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
        QLength increment_length = time_increment* mVelocity;
        movement_vectors.push_back(OffsetAlongVector<DIM>(new_direction, increment_length, reference_length));
    }

    return movement_vectors;
}

// Explicit instantiation
template class OffLatticeMigrationRule<2> ;
template class OffLatticeMigrationRule<3> ;
