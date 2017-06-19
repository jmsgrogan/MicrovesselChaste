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
#include "GeometryTools.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "RandomNumberGenerator.hpp"
#include "BaseUnits.hpp"
#include "Debug.hpp"

template<unsigned DIM>
OffLatticeMigrationRule<DIM>::OffLatticeMigrationRule()
    : AbstractMigrationRule<DIM>(),
      mGlobalX(unit_vector<double>(3,0)),
      mGlobalY(unit_vector<double>(3,1)),
      mGlobalZ(unit_vector<double>(3,2)),
      mMeanAngles(std::vector<units::quantity<unit::plane_angle> >(3, 0.0*unit::radians)),
      mSdvAngles(std::vector<units::quantity<unit::plane_angle> >(3, M_PI/6.0*unit::radians)), //formerly pi/18
      mVelocity(20.0 *(1.e-6/3600.0) * unit::metre_per_second),
      mChemotacticStrength(0.6),
      mAttractionStrength(0.0), // was 1.0
      mProbeLength(5.0 * 1.e-6 * unit::metres),
      mCriticalMutualAttractionLength(100.0 * 1.e-6 *unit::metres),
      mSurfaceRepulsion(false),
      mNumGradientEvaluationDivisions(8),
      mpDomainDistanceMap()
{

}

template <unsigned DIM>
boost::shared_ptr<OffLatticeMigrationRule<DIM> > OffLatticeMigrationRule<DIM>::Create()
{
    MAKE_PTR(OffLatticeMigrationRule<DIM>, pSelf);
    return pSelf;
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

    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
    std::vector<units::quantity<unit::length> > bbox = this->mpBoundingDomain->GetBoundingBox();
    vtkSmartPointer<vtkImageData> p_image = vtkSmartPointer<vtkImageData>::New();

    double spacing = (bbox[1] - bbox[0])/(20.0*reference_length);
    unsigned num_x = unsigned((bbox[1] - bbox[0])/(reference_length*spacing)) + 1;
    unsigned num_y = unsigned((bbox[3] - bbox[2])/(reference_length*spacing)) + 1;
    unsigned num_z = unsigned((bbox[5] - bbox[4])/(reference_length*spacing)) + 1;
    p_image->SetOrigin(bbox[0]/reference_length, bbox[2]/reference_length, bbox[4]/reference_length);
    p_image->SetSpacing(spacing, spacing, spacing);
    p_image->SetDimensions(num_x, num_y, num_z);

    vtkSmartPointer<vtkImageEuclideanDistance> p_distance = vtkSmartPointer<vtkImageEuclideanDistance>::New();
    p_distance->SetInputData( 0, this->mpBoundingDomain->GetVtk());
    p_distance->SetInputData( 1, p_image);
    p_distance->Update();
    mpDomainDistanceMap = p_distance->GetOutput();

    vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_gradient->SetInput(mpDomainDistanceMap);
    #else
        p_gradient->SetInputData(mpDomainDistanceMap);
    #endif
    p_gradient->SetResultArrayName("Gradients");
    //p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, this->mLabel.c_str());
    p_gradient->Update();
    mpDomainDistanceMap->GetPointData()->AddArray(p_gradient->GetOutput()->GetPointData()->GetArray("Gradients"));

}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::CalculateDomainDistanceMap(boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    if(!this->mpBoundingDomain)
    {
        EXCEPTION("Can't calculate a domain distance map without a bounding domain.");
    }
    MARK;
    mpDomainDistanceMap = pGrid->CalculateDistanceMap(this->mpBoundingDomain);

    MARK;
    vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_gradient->SetInput(mpDomainDistanceMap);
    #else
        p_gradient->SetInputData(mpDomainDistanceMap);
    #endif
    p_gradient->SetResultArrayName("Gradients");
    p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
    MARK;
    p_gradient->Update();
    MARK;
    mpDomainDistanceMap->GetPointData()->AddArray(p_gradient->GetOutput()->GetPointData()->GetArray("Gradients"));
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetSproutingVelocity(units::quantity<unit::velocity> velocity)
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
    mSdvAngles = std::vector<units::quantity<unit::plane_angle> >(3, angle*unit::radians);
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetNumGradientEvaluationDivisions(unsigned numDivisions)
{
    mNumGradientEvaluationDivisions = numDivisions;
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > OffLatticeMigrationRule<DIM>::GetDirections(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if (this->mIsSprouting)
    {
        return GetDirectionsForSprouts(rNodes);
    }
    else
    {
        if(!mpDomainDistanceMap)
        {
            if(this->mpSolver)
            {
                CalculateDomainDistanceMap(this->mpSolver->GetDensityMap()->GetGridCalculator()->GetGrid());
            }
            else
            {
                CalculateDomainDistanceMap();
            }

            vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
            #if VTK_MAJOR_VERSION <= 5
                p_gradient->SetInput(mpDomainDistanceMap);
            #else
                p_gradient->SetInputData(mpDomainDistanceMap);
            #endif
            p_gradient->SetResultArrayName("Gradients");
            p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
            p_gradient->Update();
        }

        units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
        std::vector<DimensionalChastePoint<DIM> > movement_vectors;

        // We want to probe the PDE solution all at once first if needed, as this is an expensive operation if done node-by-node.
        // Every node has 4 probes in 2D and 6 in 3D.
        std::vector<c_vector<double, 3> > solution_gradients(rNodes.size(), zero_vector<double>(3));
        vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();

        c_vector<double, DIM> current_loc;
        c_vector<double, DIM> current_dir;
        DimensionalChastePoint<DIM> currentDirection;
        if(this->mpSolver)
        {
            for(unsigned idx=0; idx<rNodes.size(); idx++)
            {
                p_probe_locations->InsertNextPoint(&current_loc[0]);
            }
            if(p_probe_locations->GetNumberOfPoints()>0)
            {
                solution_gradients = this->mpSolver->GetSolutionGradients(p_probe_locations);
            }
        }

        // Also probe the distance map and gradients
        std::vector<c_vector<double, 3> > distance_map_gradients(rNodes.size(), zero_vector<double>(3));
        std::vector<double> distance_map_values(rNodes.size(), 0.0);

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
        p_probe_filter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Gradients");
        p_probe_filter->Update();
        vtkDataArray* p_results = p_probe_filter->GetOutput()->GetPointData()->GetArray("Gradients");
        unsigned num_points = p_results->GetNumberOfTuples();
        for(unsigned idx=0; idx<num_points; idx++)
        {
            c_vector<double, 3> result;
            p_results->GetTuple(idx, &result[0]);
            distance_map_gradients[idx] = result;
        }
        p_probe_filter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Distance");
        p_probe_filter->Update();
        p_results = p_probe_filter->GetOutput()->GetPointData()->GetArray("Distance");
        num_points = p_results->GetNumberOfTuples();
        for(unsigned idx=0; idx<num_points; idx++)
        {
            distance_map_values[idx] = p_results->GetTuple1(idx);
        }

        for(unsigned idx=0; idx<rNodes.size(); idx++)
        {
            current_loc = rNodes[idx]->rGetLocation().GetLocation(reference_length);
            currentDirection = rNodes[idx]->rGetLocation()-rNodes[idx]->GetSegments()[0]->GetOppositeNode(rNodes[idx])->rGetLocation();
            current_dir = currentDirection.GetUnitVector();

            // Persistent random walk
            units::quantity<unit::plane_angle> angle_x = mMeanAngles[0];
            if(mSdvAngles[0]>0.0*unit::radians)
            {
                angle_x =RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[0]/unit::radians,
                        mSdvAngles[0]/unit::radians)*unit::radians;

            }
            units::quantity<unit::plane_angle> angle_y = mMeanAngles[1];
            if(mSdvAngles[1]>0.0*unit::radians)
            {
                angle_y = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[1]/unit::radians,
                        mSdvAngles[1]/unit::radians)*unit::radians;

            }
            units::quantity<unit::plane_angle> angle_z = mMeanAngles[2];
            if(mSdvAngles[2]>0.0*unit::radians)
            {
                angle_z = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[2]/unit::radians,
                        mSdvAngles[2]/unit::radians)*unit::radians;
            }
            c_vector<double, DIM> new_direction;
            if(DIM==3)
            {
                c_vector<double, DIM> new_direction_z = RotateAboutAxis<DIM>(currentDirection.GetUnitVector(), mGlobalZ, angle_z);
                c_vector<double, DIM> new_direction_y = RotateAboutAxis<DIM>(new_direction_z, mGlobalY, angle_y);
                new_direction = RotateAboutAxis<DIM>(new_direction_y, mGlobalX, angle_x);
                new_direction /= norm_2(new_direction);
            }
            else
            {
                new_direction = RotateAboutAxis<DIM>(currentDirection.GetUnitVector(), mGlobalZ, angle_z);
                new_direction /= norm_2(new_direction);
            }

            // Chemotaxis
            if(this->mpSolver)
            {
                // Get the gradients
                c_vector<double, 3> gradient = solution_gradients[idx];
                gradient/=norm_2(gradient);
                if(DIM==3)
                {
                    new_direction = new_direction + mChemotacticStrength*gradient;
                }
                else
                {
                    c_vector<double, DIM> gradient_2d;
                    gradient_2d[0] = gradient[0];
                    gradient_2d[1] = gradient[1];
                    new_direction = new_direction + mChemotacticStrength*gradient_2d;
                }
            }
            new_direction /= norm_2(new_direction);

            // Mutual Attraction
            std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = this->mpVesselNetwork->GetNodes();
            units::quantity<unit::length> min_distance = 1.e12*unit::metres;
            c_vector<double, DIM> min_direction = zero_vector<double>(DIM);
            for(unsigned jdx=0; jdx<nodes.size(); jdx++)
            {
                if(IsPointInCone<DIM>(nodes[jdx]->rGetLocation(), rNodes[idx]->rGetLocation(), rNodes[idx]->rGetLocation() +
                                      DimensionalChastePoint<DIM>(currentDirection.GetLocation(reference_length) * double(mCriticalMutualAttractionLength/reference_length), reference_length), M_PI/3.0))
                {
                    units::quantity<unit::length> distance = rNodes[idx]->rGetLocation().GetDistance(nodes[jdx]->rGetLocation());
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
                strength = mAttractionStrength *  (1.0 - (min_distance * min_distance) / (mCriticalMutualAttractionLength * mCriticalMutualAttractionLength));
            }
            new_direction += strength * min_direction;
            new_direction /= norm_2(new_direction);

            // Surface repulsion
            double critical_repulsion_distance = (20.0e-6*unit::metres)/reference_length;
            double max_repulsion_strength = 5.0;
            if(this->mpBoundingDomain)
            {
                double current_distance = distance_map_values[idx];
                if(current_distance<critical_repulsion_distance)
                {
                    c_vector<double, 3> distance_gradient = distance_map_gradients[idx];
                    distance_gradient/=norm_2(distance_gradient);
                    double value = 1.0-(2.0*current_distance/(critical_repulsion_distance+current_distance));
                    double repulsion_strength = max_repulsion_strength*value;
                    if(DIM==3)
                    {
                        new_direction += repulsion_strength * distance_gradient;
                    }
                    else
                    {
                        c_vector<double, DIM> dist_grad_2d;
                        dist_grad_2d[0] = distance_gradient[0];
                        dist_grad_2d[1] = distance_gradient[1];
                        new_direction += repulsion_strength * dist_grad_2d;
                    }
                    new_direction /= norm_2(new_direction);
                }
            }

            // Get the movement increment
            units::quantity<unit::time> time_increment = SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
            units::quantity<unit::length> increment_length = time_increment* mVelocity;
            movement_vectors.push_back(OffsetAlongVector<DIM>(new_direction, increment_length, reference_length));
        }
        return movement_vectors;
    }
}

template<unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > OffLatticeMigrationRule<DIM>::GetDirectionsForSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    std::vector<DimensionalChastePoint<DIM> > movement_vectors;
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();

    // Collect the probe locations for each node
    std::vector<units::quantity<unit::concentration> > probed_solutions;
    unsigned probes_per_node = 3;
    if(DIM==3)
    {
        probes_per_node = 5;
    }
    vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();
    std::vector<bool> candidate_locations_inside_domain(probes_per_node*rNodes.size(), true);
    // Get a normal to the segments, will depend on whether they are parallel
    for(unsigned idx = 0; idx < rNodes.size(); idx++)
    {
        if(rNodes[idx]->GetNumberOfSegments() !=2)
        {
            EXCEPTION("Attempting to form sprout at tip or existing branch. This is not supported.");
        }

        c_vector<double, DIM> sprout_direction;
        c_vector<double, DIM> cross_product;
        double sum = 0.0;

        if(DIM==3)
        {
            cross_product = VectorProduct(rNodes[idx]->GetSegments()[0]->GetUnitTangent(),
                                                                rNodes[idx]->GetSegments()[1]->GetUnitTangent());
            for(unsigned jdx=0; jdx<DIM; jdx++)
            {
                sum += std::abs(cross_product[jdx]);
            }
        }
        else
        {
            c_vector<double, DIM> tangent1 = rNodes[idx]->GetSegments()[0]->GetUnitTangent();
            c_vector<double, DIM> tangent2 = rNodes[idx]->GetSegments()[1]->GetUnitTangent();
            for(unsigned jdx=0; jdx<DIM; jdx++)
            {
                sum += std::abs(tangent1[jdx]-tangent2[jdx]);
            }
        }

        if (sum<=1.e-6)
        {
            // more or less parallel segments, chose any normal to the first tangent
            c_vector<double, DIM> normal;
            c_vector<double, DIM> tangent = rNodes[idx]->GetSegments()[0]->GetUnitTangent();
            if(DIM==2 or tangent[2]==0.0)
            {
                if(tangent[1] == 0.0)
                {
                    normal[0] = 0.0;
                    normal[1] = 1.0;
                }
                else
                {
                    normal[0] = 1.0;
                    normal[1] = -tangent[0] /tangent[1];
                }
            }
            else
            {
                if(std::abs(tangent[0]) + std::abs(tangent[1]) == 0.0)
                {
                    normal[0] = 1.0;
                    normal[1] = 1.0;
                }
                else
                {
                    normal[0] = 1.0;
                    normal[1] = 1.0;
                    normal[2] = -(tangent[0] + tangent[1])/tangent[2];
                }
            }
            if(RandomNumberGenerator::Instance()->ranf()>=0.5)
            {
                sprout_direction = normal/norm_2(normal);
            }
            else
            {
                sprout_direction = -normal/norm_2(normal);
            }
        }
        else
        {
            if(DIM==3)
            {
                // otherwise the direction is out of the plane of the segment tangents
                if(RandomNumberGenerator::Instance()->ranf()>=0.5)
                {
                    sprout_direction = cross_product/norm_2(cross_product);
                }
                else
                {
                    sprout_direction = -cross_product/norm_2(cross_product);
                }
            }
            else
            {
                c_vector<double, DIM> tangent1 = rNodes[idx]->GetSegments()[0]->GetUnitTangent();
                c_vector<double, DIM> tangent2 = rNodes[idx]->GetSegments()[1]->GetUnitTangent();
                c_vector<double, DIM> av_tangent = (tangent1 + tangent2)/2.0;
                av_tangent/=norm_2(av_tangent);

                c_vector<double, DIM> normal;
                if(av_tangent[1] == 0.0)
                {
                    normal[0] = 0.0;
                    normal[1] = 1.0;
                }
                else
                {
                    normal[0] = 1.0;
                    normal[1] = -av_tangent[0] /av_tangent[1];
                }

                if(RandomNumberGenerator::Instance()->ranf()>=0.5)
                {
                    sprout_direction = normal/norm_2(normal);
                }
                else
                {
                    sprout_direction = -normal/norm_2(normal);
                }
            }
        }
        units::quantity<unit::plane_angle> angle = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[0]/unit::radians,
                                                                                                          mSdvAngles[0]/unit::radians)*unit::radians;
        vtkSmartPointer<vtkPoints> p_local_probes = GetProbeLocationsInternalPoint<DIM>(DimensionalChastePoint<DIM>(sprout_direction, reference_length),
                                                                                                rNodes[idx]->rGetLocation(),
                                                                                                DimensionalChastePoint<DIM>(rNodes[idx]->GetSegments()[0]->GetUnitTangent(), reference_length),
                                                                                                mProbeLength,
                                                                                                angle);
        for(unsigned jdx=0;jdx<probes_per_node; jdx++)
        {
            p_probe_locations->InsertNextPoint(p_local_probes->GetPoint(jdx));
        }
    }
    if(this->mpSolver)
    {
        if(p_probe_locations->GetNumberOfPoints()>0)
        {
            probed_solutions = this->mpSolver->GetConcentrations(p_probe_locations);
        }

        if (this->mpBoundingDomain)
        {
            candidate_locations_inside_domain = this->mpBoundingDomain->IsPointInPart(p_probe_locations);
        }
    }

    // Decide on the sprout directions
    for(unsigned idx=0; idx<rNodes.size(); idx++)
    {
        DimensionalChastePoint<DIM> new_direction(0.0, 0.0, 0.0, reference_length);
        // Solution dependent contribution
        if(this->mpSolver)
        {
            // Get the gradients
            std::vector<units::quantity<unit::concentration_gradient> > gradients(probes_per_node-1, 0.0*unit::mole_per_metre_pow_4);
            if(probed_solutions.size()>=idx*probes_per_node+probes_per_node)
            {
                for(unsigned jdx=1; jdx<probes_per_node; jdx++)
                {
                    gradients[jdx-1] = ((probed_solutions[idx*probes_per_node+jdx] - probed_solutions[idx*probes_per_node]) / mProbeLength);
                    if(!candidate_locations_inside_domain[idx*probes_per_node+jdx])
                    {
                        gradients[jdx-1] = 0.0*unit::mole_per_metre_pow_4;
                    }
                }
            }
            else
            {
                EXCEPTION("Incorrect number of solution gradient probes per node.");
            }

            // Get the index of the max viable gradient
            units::quantity<unit::concentration_gradient> max_grad = 0.0 * unit::mole_per_metre_pow_4;
            int my_index = -1;
            for(unsigned jdx = 0; jdx<gradients.size(); jdx++)
            {
                if(gradients[jdx]>=max_grad)
                {
                    max_grad = gradients[jdx];
                    my_index = int(jdx);
                }
            }

            double loc[3];
            int max_index=1;
            if(DIM==3)
            {
                max_index = 3;
            }
            if(my_index>=0 and my_index<=max_index)
            {
                p_probe_locations->GetPoint(idx*probes_per_node + 1 + my_index, loc);
                new_direction = DimensionalChastePoint<DIM>(loc[0], loc[1], loc[2], reference_length) -rNodes[idx]->rGetLocation();
            }
//            else
//            {
//                EXCEPTION("Out of bounds in sprout direction calculation");
//            }
        }
        units::quantity<unit::time> time_increment = SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
        units::quantity<unit::length> increment_length = time_increment* mVelocity;

        movement_vectors.push_back(OffsetAlongVector<DIM>(new_direction.GetUnitVector(), increment_length, reference_length));
    }

    return movement_vectors;
}

// Explicit instantiation
template class OffLatticeMigrationRule<2> ;
template class OffLatticeMigrationRule<3> ;
