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

#include <sstream>
#include <boost/lexical_cast.hpp>
#include "RandomNumberGenerator.hpp"
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Exception.hpp"
#include "VesselNetworkGenerator.hpp"

#include "Timer.hpp"

template<unsigned DIM>
VesselNetworkGenerator<DIM>::VesselNetworkGenerator() :
    mReferenceLength(1.e-6 * unit::metres)
{
}

template<unsigned DIM>
VesselNetworkGenerator<DIM>::~VesselNetworkGenerator()
{
}

template <unsigned DIM>
void VesselNetworkGenerator<DIM>::SetReferenceLengthScale(units::quantity<unit::length> rReferenceLength)
{
    mReferenceLength = rReferenceLength;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateOvalNetwork(units::quantity<unit::length> scale_factor,
                                                                                      unsigned num_increments,
                                                                                      double a_param,
                                                                                      double b_param)
{
    // It is 'melon' shaped with one input and output vessel
    double increment_size = (2.0 * M_PI) / double(num_increments);

    std::vector<boost::shared_ptr<VesselNode<DIM> > > v1_nodes;
    v1_nodes.push_back(VesselNode<DIM>::Create(0.0, 0.0, 0.0, mReferenceLength));
    v1_nodes.push_back(VesselNode<DIM>::Create(increment_size * scale_factor/mReferenceLength, 0.0, 0.0, mReferenceLength));
    boost::shared_ptr<Vessel<DIM> > p_vessel_1 = Vessel<DIM>::Create(v1_nodes);
    p_vessel_1->SetId(1);

    double x_offset = increment_size + std::sqrt(b_param*b_param + a_param*a_param);
    c_vector<double, 2*DIM> bounds = zero_vector<double>(2*DIM);
    bounds[0] = 0.0;
    bounds[4] = 0.0;
    bounds[5] = 2.0;

    std::vector<boost::shared_ptr<VesselNode<DIM> > > v2_nodes;
    double y_max = 0.0;
    for(unsigned idx=0; idx<= num_increments/2; idx++)
    {
        double t = double(idx) * increment_size;
        double c_value = a_param*a_param*std::cos(2.0 * t) +
                std::sqrt(pow(b_param,4) -  a_param*a_param*pow(std::sin(2.0 * t),2));
        double x = std::cos(t) * std::sqrt(c_value) + x_offset;
        double y = std::sin(t) * std::sqrt(c_value);
        if(y>y_max)
        {
            y_max=y;
        }
        v2_nodes.push_back(VesselNode<DIM>::Create(x * scale_factor/mReferenceLength, y * scale_factor/mReferenceLength, 0.0, mReferenceLength));
    }

    std::vector<boost::shared_ptr<VesselNode<DIM> > > v3_nodes;
    for(unsigned idx=num_increments/2; idx<= num_increments; idx++)
    {
        double t = double(idx) * increment_size;
        double c_value = a_param*a_param*std::cos(2.0 * t) +
                std::sqrt(pow(b_param,4) -  a_param*a_param*pow(std::sin(2.0 * t),2));
        double x = std::cos(t) * std::sqrt(c_value) + x_offset;
        double y = std::sin(t) * std::sqrt(c_value);
        if(y>y_max)
        {
            y_max=y;
        }
        v3_nodes.push_back(VesselNode<DIM>::Create(x * scale_factor/mReferenceLength, y * scale_factor/mReferenceLength, 0.0, mReferenceLength));
    }

    boost::shared_ptr<Vessel<DIM> > p_vessel_2 = Vessel<DIM>::Create(v2_nodes);
    boost::shared_ptr<Vessel<DIM> > p_vessel_3 = Vessel<DIM>::Create(v3_nodes);
    boost::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();
    p_vessel_2->SetId(2);
    p_vessel_3->SetId(3);

    std::vector<boost::shared_ptr<VesselNode<DIM> > > v4_nodes;
    v4_nodes.push_back(VesselNode<DIM>::Create((std::sqrt(a_param*a_param + b_param*b_param) + x_offset) * scale_factor/mReferenceLength, 0.0, 0.0, mReferenceLength));
    v4_nodes.push_back(VesselNode<DIM>::Create((std::sqrt(a_param*a_param + b_param*b_param) + increment_size + x_offset) * scale_factor/mReferenceLength, 0.0, 0.0, mReferenceLength));
    bounds[1] = (std::sqrt(a_param*a_param + b_param*b_param) + increment_size + x_offset) * scale_factor/mReferenceLength;
    bounds[2] = -y_max * scale_factor/mReferenceLength;
    bounds[3] = y_max * scale_factor/mReferenceLength;

    boost::shared_ptr<Vessel<DIM> > p_vessel_4 = Vessel<DIM>::Create(v4_nodes);
    p_vessel_4->SetId(4);

    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->MergeCoincidentNodes(1.e-6);

    p_network->SetSegmentProperties(p_vessel_1->GetSegments()[0]);
    p_vessel_1->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
    p_vessel_4->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
    return p_network;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateParrallelNetwork(boost::shared_ptr<Part<DIM> > domain,
                                                                                             units::quantity<unit::per_area> targetDensity,
                                                                    VesselDistribution::Value distributionType,
                                                                    units::quantity<unit::length> exclusionDistance,
                                                                    bool useBbox,
                                                                    std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > seeds)
{
    // Get the bounding box of the domain and the volume of the bbox
    std::vector<units::quantity<unit::length> > bbox = domain->GetBoundingBox();
    units::quantity<unit::length> delta_x = bbox[1] - bbox[0];
    units::quantity<unit::length> delta_y = bbox[3] - bbox[2];
    units::quantity<unit::length> delta_z = 0.0*unit::metres;
    if(DIM==3)
    {
        delta_z = bbox[5] - bbox[4];
    }
    if(delta_x == 0.0*unit::metres || delta_y == 0.0*unit::metres)
    {
        EXCEPTION("The domain must be at least two-dimensional.");
    }

    unsigned num_x = unsigned(units::sqrt(targetDensity) * delta_x);
    unsigned num_y = unsigned(units::sqrt(targetDensity) * delta_y);
    if(num_x == 0 || num_y == 0)
    {
        EXCEPTION("The domain is not large enough to contain any vessels at the requested density.");
    }
    units::quantity<unit::length> spacing_x = delta_x / double(num_x);
    units::quantity<unit::length> spacing_y = delta_y / double(num_y);

    // Generate the start and end nodes
    std::vector<boost::shared_ptr<VesselNode<DIM> > > start_nodes;
    std::vector<boost::shared_ptr<VesselNode<DIM> > > end_nodes;
    if(distributionType == VesselDistribution::REGULAR)
    {
        for(unsigned idx=0; idx<num_y; idx++)
        {
           for(unsigned jdx=0; jdx<num_x; jdx++)
           {
               units::quantity<unit::length> location_x = bbox[0] + spacing_x / 2.0 + double(jdx) * spacing_x;
               units::quantity<unit::length> location_y = bbox[2] + spacing_y / 2.0 + double(idx) * spacing_y;
               if(DIM==2)
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, 0.0, mReferenceLength));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, 0.0, mReferenceLength));
               }
               else
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, bbox[4]/mReferenceLength, mReferenceLength));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, (bbox[4] + delta_z)/mReferenceLength, mReferenceLength));
               }
           }
        }
    }
    else if(distributionType == VesselDistribution::UNIFORM)
    {
        unsigned attempts = 0;
        for(unsigned idx=0; idx<1.e9; idx++)
        {
           // Generate with uniform random positions
           units::quantity<unit::length> location_x = bbox[0] + RandomNumberGenerator::Instance()->ranf() * delta_x;
           units::quantity<unit::length> location_y = bbox[2] + RandomNumberGenerator::Instance()->ranf() * delta_y;

           // Get the distance to existing vessels and the boundaries
           bool free_space = true;
           bool outside_x = location_x - bbox[0] < exclusionDistance || bbox[1] - location_x < exclusionDistance;
           bool outside_y = location_y - bbox[2] < exclusionDistance || bbox[3] - location_y < exclusionDistance;

           if(outside_x || outside_y)
           {
               free_space = false;
               attempts++;
           }
           else
           {
               for(unsigned kdx=0; kdx<start_nodes.size();kdx++)
               {
                   double sq_distance = pow((start_nodes[kdx]->rGetLocation().GetLocation(mReferenceLength)[1]- location_y/mReferenceLength),2) +
                           pow((start_nodes[kdx]->rGetLocation().GetLocation(mReferenceLength)[0]- location_x/mReferenceLength),2);
                   if(sq_distance*mReferenceLength*mReferenceLength < (exclusionDistance * exclusionDistance))
                   {
                       free_space = false;
                       attempts++;
                       break;
                   }
               }
           }

           if(free_space)
           {
               if(DIM==2)
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength ,0.0, mReferenceLength));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, 0.0, mReferenceLength));
               }
               else
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, bbox[4]/mReferenceLength, mReferenceLength));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, (bbox[4] + delta_z)/mReferenceLength, mReferenceLength));
               }
               attempts = 0;
           }
           if(start_nodes.size() == num_x * num_y)
           {
               break;
           }
           if(attempts == 1000)
           {
               EXCEPTION("Too many attempts to locate a vessel");
           }
        }
    }
    else if(distributionType == VesselDistribution::TWO_LAYER)
    {
        unsigned attempts = 0;

        // Uniformly distribute kernels
        unsigned num_kernels = 8;
        std::vector<std::vector<units::quantity<unit::length> > > kernel_locations;
        for(unsigned kdx=0; kdx<num_kernels; kdx++)
        {
            std::vector<units::quantity<unit::length> > location;
            location.push_back(bbox[0] + RandomNumberGenerator::Instance()->ranf() * delta_x);
            location.push_back(bbox[2] + RandomNumberGenerator::Instance()->ranf() * delta_y);
            kernel_locations.push_back(location);
        }

        // Pick locations randomly from the kernels
        for(unsigned jdx=0;jdx<1.e9;jdx++)
        {
            units::quantity<unit::length> deviation = 100.0* 1.e-6*unit::metres;
            units::quantity<unit::length> location_x = RandomNumberGenerator::Instance()->NormalRandomDeviate(0.0, deviation/mReferenceLength)*mReferenceLength;
            units::quantity<unit::length> location_y = RandomNumberGenerator::Instance()->NormalRandomDeviate(0.0, deviation/mReferenceLength)*mReferenceLength;
            unsigned kernel_index = RandomNumberGenerator::Instance()->randMod(num_kernels);
            location_x += kernel_locations[kernel_index][0];
            location_y += kernel_locations[kernel_index][1];

            // Get the distance to existing vessels and the boundaries
            bool free_space = true;
            bool outside_x = location_x - bbox[0] < exclusionDistance || bbox[1] - location_x < exclusionDistance;
            bool outside_y = location_y - bbox[2] < exclusionDistance || bbox[3] - location_y < exclusionDistance;

            if(outside_x || outside_y)
            {
                free_space = false;
                attempts++;
            }
            else
            {
                for(unsigned kdx=0; kdx<start_nodes.size();kdx++)
                {
                    double sq_distance = pow((start_nodes[kdx]->rGetLocation().GetLocation(mReferenceLength)[1]- location_y/mReferenceLength),2) +
                            pow((start_nodes[kdx]->rGetLocation().GetLocation(mReferenceLength)[0]- location_x/mReferenceLength),2);
                    if(sq_distance*mReferenceLength*mReferenceLength < (exclusionDistance * exclusionDistance))
                    {
                        free_space = false;
                        attempts++;
                        break;
                    }
                }
            }

            if(free_space)
            {
                if(DIM==2)
                {
                    start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, 0.0, mReferenceLength));
                    end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, 0.0, mReferenceLength));
                }
                else
                {
                    start_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, bbox[4]/mReferenceLength, mReferenceLength));
                    end_nodes.push_back(VesselNode<DIM>::Create(location_x/mReferenceLength, location_y/mReferenceLength, (bbox[4] + delta_z)/mReferenceLength, mReferenceLength));
                }
                attempts = 0;
            }
            if(start_nodes.size() == num_x * num_y)
            {
                break;
            }
            if(attempts == 1000)
            {
                EXCEPTION("Too many attempts to locate a vessel");
            }
        }
    }

    // Set up the vessels
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels;
    for(unsigned idx=0; idx<start_nodes.size(); idx++)
    {
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(start_nodes[idx], end_nodes[idx])));
    }

    // Generate and write the network
    boost::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();
    p_network->AddVessels(vessels);

    return p_network;
}

template<unsigned DIM>
void VesselNetworkGenerator<DIM>::PatternUnitByTranslation(boost::shared_ptr<VesselNetwork<DIM> > input_unit,
                                                         std::vector<unsigned> numberOfUnits)
{
    // Get unit dimensions
    std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > extents = input_unit->GetExtents();

    // For each number of units in each dimension copy the original vessels and move the copy to the desired location
    double num_x = 0;
    double num_y = 0;
    double num_z = 0;
    if(numberOfUnits.size() >= 1)
    {
        num_x = numberOfUnits[0];
        if(numberOfUnits.size() >= 2)
        {
            num_y = numberOfUnits[1];
            if(numberOfUnits.size() >= 3 && DIM ==3)
            {
                num_z = numberOfUnits[2];
            }
        }
    }

    // Keep track of the current vessels
    std::vector<boost::shared_ptr<Vessel<DIM> > > original_vessels = input_unit->GetVessels();
    for(unsigned idx =0; idx < num_x; idx++)
    {
        DimensionalChastePoint<DIM>translation_vector(double(idx+1) * (extents.second.GetLocation(mReferenceLength)[0] - extents.first.GetLocation(mReferenceLength)[0]), 0.0, 0.0, mReferenceLength);
        std::vector<boost::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(original_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();

    std::vector<boost::shared_ptr<Vessel<DIM> > > x_transform_vessels = input_unit->GetVessels();
    for(unsigned idx =0; idx < num_y; idx++)
    {
        DimensionalChastePoint<DIM>translation_vector(0.0, double(idx+1) * (extents.second.GetLocation(mReferenceLength)[1] - extents.first.GetLocation(mReferenceLength)[1]), 0.0, mReferenceLength);
        std::vector<boost::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(x_transform_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();
    std::vector<boost::shared_ptr<Vessel<DIM> > > y_transform_vessels = input_unit->GetVessels();

    for(unsigned idx =0; idx < num_z; idx++)
    {
        DimensionalChastePoint<DIM>translation_vector(0.0, 0.0, double(idx+1) * (extents.second.GetLocation(mReferenceLength)[2] - extents.first.GetLocation(mReferenceLength)[2]), mReferenceLength);
        std::vector<boost::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(y_transform_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalUnit(units::quantity<unit::length> vesselLength)
{
    // Generate the nodes
    double dimensionless_vessel_length = vesselLength/mReferenceLength;
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0.0, 0.0, 0.0, mReferenceLength)); //0
    nodes.push_back(VesselNode<DIM>::Create(dimensionless_vessel_length, dimensionless_vessel_length, 0.0, mReferenceLength)); //1
    nodes.push_back(VesselNode<DIM>::Create(0.0, 2.0 * dimensionless_vessel_length, 0.0, mReferenceLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(2.0 * dimensionless_vessel_length, dimensionless_vessel_length, 0.0, mReferenceLength)); //3
    nodes.push_back(VesselNode<DIM>::Create(3.0 * dimensionless_vessel_length, 0.0, 0.0, mReferenceLength)); //4
    nodes.push_back(VesselNode<DIM>::Create(4.0 * dimensionless_vessel_length, 0.0, 0.0, mReferenceLength)); //5
    nodes.push_back(VesselNode<DIM>::Create(3.0 * dimensionless_vessel_length, 2.0 * dimensionless_vessel_length, 0.0, mReferenceLength)); //6
    if (DIM == 3)
    {
        nodes.push_back(VesselNode<DIM>::Create(0.0, 0.0, 1.5*dimensionless_vessel_length, mReferenceLength)); //7
        nodes.push_back(VesselNode<DIM>::Create(dimensionless_vessel_length, dimensionless_vessel_length, 1.5*dimensionless_vessel_length, mReferenceLength)); //8
        nodes.push_back(VesselNode<DIM>::Create(2.0 * dimensionless_vessel_length, dimensionless_vessel_length, 1.5*dimensionless_vessel_length, mReferenceLength)); //9
        nodes.push_back(VesselNode<DIM>::Create(3.0 * dimensionless_vessel_length, 0.0, 1.5*dimensionless_vessel_length, mReferenceLength)); //9
    }

    // Generate the segments and vessels
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[1])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[2])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[3])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[4])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[5])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[6])));

    if (DIM == 3)
    {
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[7])));
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[8])));
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[9])));
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[10])));
    }

    // Generate the network
    boost::shared_ptr<VesselNetwork<DIM> > pNetwork(new VesselNetwork<DIM>());
    pNetwork->AddVessels(vessels);

    return pNetwork;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateBifurcationUnit(units::quantity<unit::length> vesselLength,
                                                                                          DimensionalChastePoint<DIM> startPosition)
{
    // Generate the nodes
    double dimensionless_vessel_length = vesselLength/mReferenceLength;
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0.0, dimensionless_vessel_length, 0.0, mReferenceLength)); //0
    nodes.push_back(VesselNode<DIM>::Create(dimensionless_vessel_length, dimensionless_vessel_length, 0.0, mReferenceLength)); //1
    nodes.push_back(VesselNode<DIM>::Create(2.0 * dimensionless_vessel_length, 2.0 * dimensionless_vessel_length, 0.0, mReferenceLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(2.0 * dimensionless_vessel_length, 0.0, 0.0, mReferenceLength)); //3
    nodes.push_back(VesselNode<DIM>::Create(3.0 * dimensionless_vessel_length, dimensionless_vessel_length, 0.0, mReferenceLength)); //4
    nodes.push_back(VesselNode<DIM>::Create(4.0 * dimensionless_vessel_length, dimensionless_vessel_length, 0.0, mReferenceLength)); //5

    // Generate the segments and vessels
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[1])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[2])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[3])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[2], nodes[4])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[4])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[5])));

    // Generate the network
    boost::shared_ptr<VesselNetwork<DIM> > p_network(new VesselNetwork<DIM>());
    p_network->AddVessels(vessels);
    p_network->Translate(startPosition);
    return p_network;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateSingleVessel(units::quantity<unit::length> vesselLength,
                                                                                       DimensionalChastePoint<DIM> startPosition,
                                                                                       unsigned divisions, unsigned axis)
{
    double dimensionless_vessel_length = vesselLength/mReferenceLength;
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes;
    nodes.push_back(VesselNode<DIM>::Create(startPosition)); //0
    double interval = dimensionless_vessel_length/double(divisions + 1);

    for(unsigned idx=0;idx<divisions+1;idx++)
    {
        c_vector<double, DIM> dimless_position = startPosition.GetLocation(mReferenceLength);
        if(DIM==2)
        {
            if(axis==0)
            {
                nodes.push_back(VesselNode<DIM>::Create(dimless_position[0] + interval*double(idx+1), dimless_position[1], 0.0, mReferenceLength)); //1
            }
            else
            {
                nodes.push_back(VesselNode<DIM>::Create(dimless_position[0], dimless_position[1] + interval*double(idx+1), 0.0, mReferenceLength)); //1
            }

        }
        else
        {
            if(axis==0)
            {
                nodes.push_back(VesselNode<DIM>::Create(dimless_position[0] + interval*double(idx+1), dimless_position[1], dimless_position[2], mReferenceLength)); //1
            }
            else if(axis==1)
            {
                nodes.push_back(VesselNode<DIM>::Create(dimless_position[0], dimless_position[1] + interval*double(idx+1), dimless_position[2], mReferenceLength)); //2
            }
            else
            {
                nodes.push_back(VesselNode<DIM>::Create(dimless_position[0], dimless_position[1], dimless_position[2] + interval*double(idx+1), mReferenceLength)); //3
            }
        }
    }

    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(nodes));

    // Generate the network
    boost::shared_ptr<VesselNetwork<DIM> > p_network(new VesselNetwork<DIM>());
    p_network->AddVessels(vessels);
    return p_network;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateFromPart(boost::shared_ptr<Part<DIM> > part)
{
    boost::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();

    // Get the polygons
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons = part->GetPolygons();
    std::vector<boost::shared_ptr<Vessel<DIM> > > vessels;
    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments;
        std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > vertices = polygons[idx]->GetVertices();
        for (unsigned jdx = 1; jdx < vertices.size(); jdx++)
        {
            segments.push_back(VesselSegment<DIM>::Create(VesselNode<DIM>::Create(*vertices[jdx-1]), VesselNode<DIM>::Create(*vertices[jdx])));
        }
        vessels.push_back(Vessel<DIM>::Create(segments));
    }
    p_network->AddVessels(vessels);
    p_network->MergeCoincidentNodes();
    return p_network;
}

template<unsigned DIM>
void VesselNetworkGenerator<DIM>::MapToSphere(boost::shared_ptr<VesselNetwork<DIM> > pInputUnit,
                                              units::quantity<unit::length> radius, units::quantity<unit::length> thickess,
                                              double azimuthExtent, double polarExtent)
{
    std::pair<DimensionalChastePoint<DIM>, DimensionalChastePoint<DIM> > extents = pInputUnit->GetExtents();
    std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = pInputUnit->GetNodes();
    for(unsigned idx =0; idx<nodes.size(); idx++)
    {
        c_vector<double, DIM> node_loc = nodes[idx]->rGetLocation().GetLocation(mReferenceLength);
        c_vector<double, DIM> offset = (extents.second - extents.first).GetLocation(mReferenceLength);

        double x_frac = node_loc[0] / offset[0];
        double azimuth_angle = x_frac * azimuthExtent;

        double y_frac = (node_loc[1] + 3.0) / offset[1];
        double polar_angle = y_frac * polarExtent;

        double dimless_radius = radius/mReferenceLength;
        if(offset[2]>0.0)
        {
            double z_frac = node_loc[2]  / offset[2];
            dimless_radius = dimless_radius - (thickess/mReferenceLength) * z_frac;
        }
        DimensionalChastePoint<DIM>new_position(dimless_radius * std::cos(azimuth_angle) * std::sin(polar_angle),
                                              dimless_radius * std::cos(polar_angle),
                                              dimless_radius * std::sin(azimuth_angle) * std::sin(polar_angle),
                                              mReferenceLength);
        nodes[idx]->SetLocation(new_position);
    }
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalNetwork(units::quantity<unit::length> width,
                                                                                           units::quantity<unit::length> height,
                                                                                           units::quantity<unit::length> vessel_length)
{
    // Vessels are laid out on a regular grid in a hexagonal pattern.
    // The repeating unit looks like this:
    //    \_/
    //    / \_
    // There is an extra set of lines along the top to 'close' the pattern.
    double grid_length = vessel_length/mReferenceLength;
    double unit_height = 2.0 * grid_length;
    double unit_width = 4.0 * grid_length;

    // The pattern may not reach the extents of the target area.
    unsigned units_in_x_direction = floor((width/mReferenceLength)/ unit_width);
    unsigned units_in_y_direction = floor((height/mReferenceLength) / unit_height);

    // If the number of units is less than 1, just make a single unit in that direction
    if (units_in_x_direction < 1)
    {
        units_in_x_direction = 1;
    }

    if (units_in_y_direction < 1)
    {
        units_in_x_direction = 1;
    }

    // Create vessels by looping over the units, x direction is inside loop.
    boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    for(unsigned jdx = 0; jdx<units_in_y_direction; jdx++)
    {
        for(unsigned idx=0; idx<units_in_x_direction; idx++)
        {
            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 0.0, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + grid_length,
                                                                                                   double(jdx)*unit_height + grid_length, 0, mReferenceLength)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 2.0*grid_length, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + grid_length,
                                                                                                   double(jdx)*unit_height + grid_length, 0, mReferenceLength)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + grid_length,
                                                                                  double(jdx)*unit_height + grid_length, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*grid_length,
                                                                                                   double(jdx)*unit_height + grid_length, 0, mReferenceLength)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*grid_length,
                                                                                  double(jdx)*unit_height + grid_length, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*grid_length,
                                                                                                   double(jdx)*unit_height + 2.0*grid_length, 0, mReferenceLength)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*grid_length,
                                                                                  double(jdx)*unit_height + grid_length, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*grid_length,
                                                                                                   double(jdx)*unit_height + 0.0, 0, mReferenceLength)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*grid_length,
                                                                                  double(jdx)*unit_height + 0.0, 0, mReferenceLength),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 4.0*grid_length,
                                                                                                   double(jdx)*unit_height + 0.0, 0, mReferenceLength)));
        }
    }

    // Add an extra line of vessels along the top
    for(unsigned idx=0; idx<units_in_x_direction; idx++)
    {
        pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*grid_length,
                                                                              double(units_in_y_direction)*unit_height, 0, mReferenceLength),
                                                                       VesselNode<DIM>::Create(double(idx)*unit_width + 4.0 *grid_length,
                                                                                               double(units_in_y_direction)*unit_height, 0, mReferenceLength)));
    }

    Timer::PrintAndReset("Internal Network set up");
    pVesselNetwork->MergeCoincidentNodes();
    Timer::PrintAndReset("Nodes merged");
    return pVesselNetwork;
}

//Explicit instantiation
template class VesselNetworkGenerator<2> ;
template class VesselNetworkGenerator<3> ;
