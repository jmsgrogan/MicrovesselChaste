/*

Copyright (c) 2005-2016, University of Oxford.
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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include "Vessel.hpp"
#include "VesselSegment.hpp"
#include "ChastePoint.hpp"
#include "LinearSystem.hpp"
#include "ReplicatableVector.hpp"
#include "UblasMatrixInclude.hpp"
#include "UnitCollection.hpp"
#include "RegularGridWriter.hpp"
#include "GeometryWriter.hpp"
#include "GreensFunctionSolver.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
GreensFunctionSolver<DIM>::GreensFunctionSolver()
    : AbstractRegularGridDiscreteContinuumSolver<DIM>(),
      mpDomain(),
      mSinkCoordinates(),
      mSinkPointMap(),
      mSubSegmentCoordinates(),
      mSubSegmentLengths(),
      mSinkRates(),
      mSourceRates(),
      mSegmentConcentration(),
      mSegmentPointMap(),
      mGtt(),
      mGvv(),
      mGvt(),
      mGtv(),
      mSubsegmentCutoff(1.0*unit::microns)
{

}

template<unsigned DIM>
GreensFunctionSolver<DIM>::~GreensFunctionSolver()
{

}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::SetSubSegmentCutoff(units::quantity<unit::length> value)
{
    mSubsegmentCutoff = value;
}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::Solve()
{
    // Set up the sub-segment and tissue point co-ordinates
    GenerateSubSegments();
    GenerateTissuePoints();

    // Generate the greens function matrices
    UpdateGreensFunctionMatrices(1, 1, 1, 1);

    // Get the sink rates
    unsigned number_of_sinks = mSinkCoordinates.size();
    units::quantity<unit::concentration_flow_rate> sink_rate = this->mpPde->ComputeConstantInUSourceTerm();
    units::quantity<unit::volume> sink_volume = units::pow<3>(this->mpRegularGrid->GetSpacing());
    mSinkRates = std::vector<units::quantity<unit::molar_flow_rate> >(number_of_sinks, sink_rate * sink_volume);
    units::quantity<unit::molar_flow_rate> total_sink_rate = std::accumulate(mSinkRates.begin(), mSinkRates.end(), 0.0*unit::mole_per_second);

    // Get the sink substance demand on each vessel subsegment
    unsigned number_of_subsegments = mSubSegmentCoordinates.size();
    units::quantity<unit::diffusivity> diffusivity = this->mpPde->ComputeIsotropicDiffusionTerm();
    std::vector<units::quantity<unit::concentration> > sink_demand_per_subsegment(number_of_subsegments, 0.0*this->mReferenceConcentration);
    for (unsigned idx = 0; idx < number_of_subsegments; idx++)
    {
        for (unsigned jdx = 0; jdx < number_of_sinks; jdx++)
        {
            sink_demand_per_subsegment[idx] += ((*mGvt)[idx][jdx] / diffusivity) * mSinkRates[jdx];
        }
    }

    mSegmentConcentration = std::vector<units::quantity<unit::concentration> >(number_of_subsegments, 1.0*this->mReferenceConcentration);
    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(number_of_sinks, 0.0*this->mReferenceConcentration);

    // Solve for the subsegment source rates required to meet the sink substance demand
    double tolerance = 1.e-10;
    units::quantity<unit::concentration> g0 = 0.0 * this->mReferenceConcentration;
    mSourceRates = std::vector<units::quantity<unit::molar_flow_rate> >(number_of_subsegments, 0.0*unit::mole_per_second);

    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::concentration> reference_concentration = this->mReferenceConcentration;
    units::quantity<unit::amount> reference_amount(1.0*unit::moles);
    LinearSystem linear_system(number_of_subsegments + 1, number_of_subsegments + 1);
    linear_system.SetKspType("bcgs");

    for (unsigned iteration = 0; iteration < 10; iteration++)
    {
        linear_system.AssembleIntermediateLinearSystem();
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            linear_system.SetRhsVectorElement(i, (mSegmentConcentration[i] - sink_demand_per_subsegment[i])/reference_concentration);
        }

        linear_system.SetRhsVectorElement(number_of_subsegments, -total_sink_rate*(reference_time/reference_amount));

        // Set up Linear system matrix
        for (unsigned iter = 0; iter < number_of_subsegments; iter++)
        {
            for (unsigned jter = 0; jter < number_of_subsegments; jter++)
            {
                linear_system.SetMatrixElement(iter, jter, ((*mGvv)[iter][jter] / diffusivity)*(reference_amount/(reference_time*reference_concentration)));
            }
            linear_system.SetMatrixElement(number_of_subsegments, iter, 1.0);
            linear_system.SetMatrixElement(iter, number_of_subsegments, 1.0);
        }
        linear_system.SetMatrixElement(number_of_subsegments, number_of_subsegments, 0.0);

        // Solve the linear system
        linear_system.AssembleFinalLinearSystem();
        ReplicatableVector soln_repl(linear_system.Solve());

        // Populate the solution vector
        std::vector<units::quantity<unit::molar_flow_rate> > solution_vector(number_of_subsegments + 1);
        for (unsigned row = 0; row < number_of_subsegments + 1; row++)
        {
            (solution_vector)[row] = soln_repl[row]*(reference_amount/reference_time);
        }

        // Check convergence
        bool all_in_tolerance = true;
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            units::quantity<unit::molar_flow_rate> diff = units::abs(mSourceRates[i] - solution_vector[i]);
            if (diff > tolerance*unit::mole_per_second)
            {
                all_in_tolerance = false;
                break;
            }
        }
        // Retrieve the solution
        for (unsigned i = 0; i < number_of_subsegments; i++)
        {
            mSourceRates[i] = solution_vector[i];
        }
        g0 = solution_vector[number_of_subsegments]*reference_concentration*(reference_time/reference_amount);

        if (all_in_tolerance)
        {
            break;
        }
        else
        {
            if (iteration == 9)
            {
                std::cout << "Did not converge\n";
            }
        }
    }

    // Get the tissue concentration and write the solution
    for (unsigned i = 0; i < number_of_sinks; i++)
    {
        this->mConcentrations[i] = 0.0 * this->mReferenceConcentration;
        for (unsigned j = 0; j < number_of_sinks; j++)
        {
            this->mConcentrations[i] += (*mGtt)[i][j] * mSinkRates[j] / diffusivity;
        }

        for (unsigned j = 0; j < number_of_subsegments; j++)
        {
            this->mConcentrations[i] += (*mGtv)[i][j] * mSourceRates[j] / diffusivity;
        }
        this->mConcentrations[i] += g0;
    }

    std::map<std::string, std::vector<units::quantity<unit::concentration> > > segmentPointData;
    segmentPointData[this->mLabel] = mSegmentConcentration;

    this->UpdateSolution(this->mConcentrations);
    if(this->mWriteSolution)
    {
        this->WriteSolution(segmentPointData);
    }
}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::GenerateSubSegments()
{
    // Set up the sub-segment points and map to original segments
    units::quantity<unit::length> max_subsegment_length = mSubsegmentCutoff;

    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels = this->mpNetwork->GetVessels();
    typename std::vector<boost::shared_ptr<Vessel<DIM> > >::iterator vessel_iter;
    typename std::vector<boost::shared_ptr<VesselSegment<DIM> > >::iterator segment_iter;

    // Iterate over all segments and store midpoints and lengths of subsegment regions for
    // the greens functions calculation. Create a map of subsegment index to the parent segment
    // for later use.
    for (vessel_iter = vessels.begin(); vessel_iter != vessels.end(); vessel_iter++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments = (*vessel_iter)->GetSegments();
        for (segment_iter = segments.begin(); segment_iter != segments.end(); segment_iter++)
        {
            units::quantity<unit::length> segment_length = (*segment_iter)->GetLength();

            // If the segment is shorter than the max length just use its mid-point
            if (segment_length < 1.01 * max_subsegment_length)
            {
                mSubSegmentCoordinates.push_back((*segment_iter)->GetMidPoint());
                mSubSegmentLengths.push_back(segment_length);
                mSegmentPointMap[mSubSegmentCoordinates.size() - 1] = (*segment_iter);
            }
            // Otherwise generate subsegment points along its length
            else
            {
                DimensionalChastePoint<DIM> start_point = (*segment_iter)->GetNode(0)->rGetLocation();
                DimensionalChastePoint<DIM> end_point = (*segment_iter)->GetNode(1)->rGetLocation();

                double subsegment_length_factor = segment_length / max_subsegment_length;
                unsigned num_subsegments = std::floor(subsegment_length_factor) + 1;
                units::quantity<unit::length> subsegment_length = segment_length / double(num_subsegments);
                units::quantity<unit::length> node_length_scale = (*segment_iter)->GetNode(0)->GetReferenceLengthScale();

                c_vector<double, DIM> increment = (end_point.rGetLocation() - start_point.rGetLocation()) / (segment_length/node_length_scale);

                for (unsigned i = 0; i < num_subsegments; i++)
                {
                    c_vector<double, DIM> location = start_point.rGetLocation() + (double(i) + 0.5) * increment * (subsegment_length/node_length_scale);
                    mSubSegmentCoordinates.push_back(DimensionalChastePoint<DIM>(location));
                    mSubSegmentLengths.push_back(subsegment_length);
                    mSegmentPointMap[mSubSegmentCoordinates.size() - 1] = (*segment_iter);
                }
            }
        }
    }
}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::GenerateTissuePoints()
{
    unsigned num_points = this->mpRegularGrid->GetNumberOfPoints();
    mSinkCoordinates = std::vector<DimensionalChastePoint<DIM> >(num_points);
    mSinkPointMap = std::vector<unsigned>(num_points);
    for(unsigned idx=0; idx<num_points; idx++)
    {
        mSinkCoordinates[idx] = this->mpRegularGrid->GetLocationOf1dIndex(idx);
        mSinkPointMap[idx] = idx;
    }
}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::UpdateGreensFunctionMatrices(bool updateGtt, bool updateGvv, bool updateGtv,
                                                                 bool updateGvt)
{
    // Get the Greens Function coefficient matrices
    if (updateGtt)
    {
        mGtt = GetTissueTissueInteractionMatrix();
    }
    if (updateGvv)
    {
        mGvv = GetVesselVesselInteractionMatrix();
    }
    if (updateGtv)
    {
        mGtv = GetTissueVesselInteractionMatrix();
    }
    if (updateGvt)
    {
        mGvt = GetVesselTissueInteractionMatrix();
    }
}

template<unsigned DIM>
boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > GreensFunctionSolver<DIM>::GetVesselVesselInteractionMatrix()
{
    typedef boost::multi_array<units::quantity<unit::per_length>, 2>::index index;
    unsigned num_sub_segments = mSubSegmentCoordinates.size();
    double coefficient = 1.0 / (4.0 * M_PI);

    boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > p_interaction_matrix(new boost::multi_array<units::quantity<unit::per_length>, 2>(boost::extents[num_sub_segments][num_sub_segments]));
    for (index iter = 0; iter < num_sub_segments; iter++)
    {
        for (index iter2 = 0; iter2 < num_sub_segments; iter2++)
        {
            if (iter <= iter2)
            {
                units::quantity<unit::length> distance = mSubSegmentCoordinates[iter2].GetDistance(mSubSegmentCoordinates[iter]);
                units::quantity<unit::per_length> term;
                if (distance < mSegmentPointMap[iter]->GetRadius())
                {
                    units::quantity<unit::length> radius = mSegmentPointMap[iter]->GetRadius();
                    units::quantity<unit::length> max_segment_length = std::max(mSubSegmentLengths[iter], mSubSegmentLengths[iter2]);
                    double green_correction = 0.6 * std::exp(-0.45 * max_segment_length /radius);
                    if (iter != iter2)
                    {
                        distance = radius;
                    }
                    term = (1.298 / (1.0 + 0.297 * pow(max_segment_length /radius, 0.838))- green_correction * pow(distance / radius, 2))*coefficient/radius;
                }
                else
                {
                    term = coefficient / distance;
                }
                (*p_interaction_matrix)[iter][iter2] = term;
                (*p_interaction_matrix)[iter2][iter] = term;
            }
        }
    }
    return p_interaction_matrix;
}

template<unsigned DIM>
boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > GreensFunctionSolver<DIM>::GetTissueTissueInteractionMatrix()
{
    typedef boost::multi_array<units::quantity<unit::per_length>, 2>::index index;
    unsigned num_points = mSinkCoordinates.size();
    double coefficient = 1.0 / (4.0 * M_PI);
    units::quantity<unit::volume> tissue_point_volume = units::pow<3>(this->mpRegularGrid->GetSpacing());
    units::quantity<unit::length> equivalent_tissue_point_radius = units::root<3>(tissue_point_volume * 0.75 / M_PI);
    boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2 > > p_interaction_matrix(new boost::multi_array<units::quantity<unit::per_length>, 2>(boost::extents[num_points][num_points]));
    for (index iter = 0; iter < num_points; iter++)
    {
        for (index iter2 = 0; iter2 < num_points; iter2++)
        {
            if (iter < iter2)
            {
                units::quantity<unit::length> distance = mSinkCoordinates[iter2].GetDistance(mSinkCoordinates[iter]);
                (*p_interaction_matrix)[iter][iter2] = coefficient / distance;
                (*p_interaction_matrix)[iter2][iter] = coefficient / distance;
            }
            else if (iter == iter2)
            {
                (*p_interaction_matrix)[iter][iter2] = 1.2 * coefficient / equivalent_tissue_point_radius;
            }
        }
    }
    return p_interaction_matrix;
}

template<unsigned DIM>
boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > GreensFunctionSolver<DIM>::GetTissueVesselInteractionMatrix()
{
    typedef boost::multi_array<units::quantity<unit::per_length>, 2>::index index;
    unsigned num_sinks = mSinkCoordinates.size();
    unsigned num_subsegments = mSubSegmentCoordinates.size();

    units::quantity<unit::volume> tissue_point_volume = units::pow<3>(this->mpRegularGrid->GetSpacing());
    units::quantity<unit::length> equivalent_tissue_point_radius = units::root<3>(tissue_point_volume * 0.75 / M_PI);
    double coefficient = 1.0 / (4.0 * M_PI);

    boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > p_interaction_matrix(new boost::multi_array<units::quantity<unit::per_length>, 2>(boost::extents[num_sinks][num_subsegments]));
    for (index iter = 0; iter < num_sinks; iter++)
    {
        for (index iter2 = 0; iter2 < num_subsegments; iter2++)
        {
            units::quantity<unit::length> distance = mSinkCoordinates[iter2].GetDistance(mSinkCoordinates[iter]);
            units::quantity<unit::per_length> term;
            if (distance <= equivalent_tissue_point_radius)
            {
                term = coefficient * (1.5 - 0.5 * (pow(distance / equivalent_tissue_point_radius, 2))) / equivalent_tissue_point_radius;
            }
            else
            {
                term = coefficient / distance;
            }
            (*p_interaction_matrix)[iter][iter2] = term;
        }
    }
    return p_interaction_matrix;
}

template<unsigned DIM>
boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > GreensFunctionSolver<DIM>::GetVesselTissueInteractionMatrix()
{
    typedef boost::multi_array<units::quantity<unit::per_length>, 2>::index index;
    unsigned num_subsegments = mSubSegmentCoordinates.size();
    unsigned num_sinks = mSinkCoordinates.size();
    double coefficient = 1.0 / (4.0 * M_PI);

    units::quantity<unit::volume> tissue_point_volume = units::pow<3>(this->mpRegularGrid->GetSpacing());
    units::quantity<unit::length> equivalent_tissue_point_radius = units::root<3>(tissue_point_volume * 0.75 / M_PI);

    boost::shared_ptr<boost::multi_array<units::quantity<unit::per_length>, 2> > p_interaction_matrix(new boost::multi_array<units::quantity<unit::per_length>, 2>(boost::extents[num_subsegments][num_sinks]));
    for (index iter = 0; iter < num_subsegments; iter++)
    {
        for (index iter2 = 0; iter2 < num_sinks; iter2++)
        {
            units::quantity<unit::length> distance = mSinkCoordinates[iter2].GetDistance(mSinkCoordinates[iter]);
            units::quantity<unit::per_length> term;
            if (distance <= equivalent_tissue_point_radius)
            {
                term = coefficient * (1.5 - 0.5 * (pow(distance / equivalent_tissue_point_radius, 2)))/ equivalent_tissue_point_radius;
            }
            else
            {
                term = coefficient / distance;
            }
            (*p_interaction_matrix)[iter][iter2] = term;
        }
    }
    return p_interaction_matrix;
}

template<unsigned DIM>
void GreensFunctionSolver<DIM>::WriteSolution(std::map<std::string, std::vector<units::quantity<unit::concentration> > >& segmentPointData)
{
    // Write the tissue point data
    RegularGridWriter writer;
    writer.SetFilename(this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/solution.vti");
    writer.SetImage(this->mpVtkSolution);
    writer.Write();

    // Add the segment points
    vtkSmartPointer<vtkPolyData> pPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> pPoints = vtkSmartPointer<vtkPoints>::New();
    for (unsigned i = 0; i < mSubSegmentCoordinates.size(); i++)
    {
        DimensionalChastePoint<DIM> location = mSubSegmentCoordinates[i];
        pPoints->InsertNextPoint(location[0], location[1], location[2]);
    }
    pPolyData->SetPoints(pPoints);

    // Add the segment point data
    std::map<std::string, std::vector<units::quantity<unit::concentration> > >::iterator segment_iter;
    for (segment_iter = segmentPointData.begin(); segment_iter != segmentPointData.end(); ++segment_iter)
    {
        vtkSmartPointer<vtkDoubleArray> pInfo = vtkSmartPointer<vtkDoubleArray>::New();
        pInfo->SetNumberOfComponents(1);
        pInfo->SetNumberOfTuples(mSubSegmentCoordinates.size());
        pInfo->SetName(segment_iter->first.c_str());

        for (unsigned i = 0; i < mSubSegmentCoordinates.size(); i++)
        {
            pInfo->SetValue(i, segmentPointData[segment_iter->first][i]/this->mReferenceConcentration);
        }
        pPolyData->GetPointData()->AddArray(pInfo);
    }

    GeometryWriter geometry_writer;
    geometry_writer.SetFileName(this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/segments.vtp");
    geometry_writer.SetInput(pPolyData);
    geometry_writer.Write();
}

// Explicit instantiation
template class GreensFunctionSolver<2>;
template class GreensFunctionSolver<3>;
