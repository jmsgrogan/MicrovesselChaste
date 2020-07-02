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
#include "VesselNetworkGeometryCalculator.hpp"
#include "VesselNetworkPropertyManager.hpp"

template<unsigned DIM>
VesselNetworkGenerator<DIM>::VesselNetworkGenerator() :
    mReferenceLength(1_um)
{
}

template<unsigned DIM>
VesselNetworkGenerator<DIM>::~VesselNetworkGenerator()
{
}

template <unsigned DIM>
void VesselNetworkGenerator<DIM>::SetReferenceLengthScale(QLength rReferenceLength)
{
    mReferenceLength = rReferenceLength;
}

template<unsigned DIM>
VesselNetworkPtr<DIM> VesselNetworkGenerator<DIM>::GenerateOvalNetwork(QLength scale_factor,
                                                                       unsigned num_increments,
                                                                       double a_param,
                                                                       double b_param)
{
    // It is 'melon' shaped with one input and output vessel
    double increment_size = (2.0 * M_PI) / double(num_increments);

    std::vector<VesselNodePtr<DIM> > v1_nodes;
    v1_nodes.push_back(VesselNode<DIM>::Create(0_m));
    v1_nodes.push_back(VesselNode<DIM>::Create(increment_size * scale_factor, 0_m, 0_m));
    std::shared_ptr<Vessel<DIM> > p_vessel_1 = Vessel<DIM>::Create(v1_nodes);
    p_vessel_1->SetId(1);

    double x_offset = increment_size + std::sqrt(b_param*b_param + a_param*a_param);
    c_vector<double, 2*DIM> bounds = zero_vector<double>(2*DIM);
    bounds[0] = 0.0;
    bounds[4] = 0.0;
    bounds[5] = 2.0;

    std::vector<VesselNodePtr<DIM> > v2_nodes;
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
        v2_nodes.push_back(VesselNode<DIM>::Create(x * scale_factor, y * scale_factor, 0_m));
    }

    std::vector<VesselNodePtr<DIM> > v3_nodes;
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
        v3_nodes.push_back(VesselNode<DIM>::Create(x * scale_factor, y * scale_factor));
    }

    std::shared_ptr<Vessel<DIM> > p_vessel_2 = Vessel<DIM>::Create(v2_nodes);
    std::shared_ptr<Vessel<DIM> > p_vessel_3 = Vessel<DIM>::Create(v3_nodes);
    std::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();
    p_vessel_2->SetId(2);
    p_vessel_3->SetId(3);

    std::vector<VesselNodePtr<DIM> > v4_nodes;
    v4_nodes.push_back(VesselNode<DIM>::Create((std::sqrt(a_param*a_param + b_param*b_param) + x_offset) * scale_factor));
    v4_nodes.push_back(VesselNode<DIM>::Create((std::sqrt(a_param*a_param + b_param*b_param) + increment_size + x_offset) * scale_factor));
    bounds[1] = (std::sqrt(a_param*a_param + b_param*b_param) + increment_size + x_offset) * scale_factor/mReferenceLength;
    bounds[2] = -y_max * scale_factor/mReferenceLength;
    bounds[3] = y_max * scale_factor/mReferenceLength;

    std::shared_ptr<Vessel<DIM> > p_vessel_4 = Vessel<DIM>::Create(v4_nodes);
    p_vessel_4->SetId(4);

    p_network->AddVessel(p_vessel_1);
    p_network->AddVessel(p_vessel_2);
    p_network->AddVessel(p_vessel_3);
    p_network->AddVessel(p_vessel_4);
    p_network->MergeCoincidentNodes(1.e-6);

    VesselNetworkPropertyManager<DIM>::SetSegmentProperties(p_network, p_vessel_1->GetSegments()[0]);
    p_vessel_1->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
    p_vessel_4->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
    return p_network;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateParallelNetwork(PartPtr<DIM> domain,
        QPerArea targetDensity, VesselDistribution::Value distributionType, QLength exclusionDistance,
        bool useBbox, std::vector<std::shared_ptr<Vertex<DIM> > > seeds)
{
    // Get the bounding box of the domain and the volume of the bbox
    std::array<QLength, 6> bbox = domain->GetBoundingBox();
    QLength delta_x = bbox[1] - bbox[0];
    QLength delta_y = bbox[3] - bbox[2];
    QLength delta_z = 0_m;
    if(DIM==3)
    {
        delta_z = bbox[5] - bbox[4];
    }
    if(delta_x == 0_m || delta_y == 0_m)
    {
        EXCEPTION("The domain must be at least two-dimensional.");
    }

    unsigned num_x = unsigned(delta_x*Qsqrt(targetDensity));
    unsigned num_y = unsigned(delta_y*Qsqrt(targetDensity));
    if(num_x == 0 || num_y == 0)
    {
        EXCEPTION("The domain is not large enough to contain any vessels at the requested density.");
    }
    QLength spacing_x = delta_x / double(num_x);
    QLength spacing_y = delta_y / double(num_y);

    // Generate the start and end nodes
    std::vector<VesselNodePtr<DIM> > start_nodes;
    std::vector<VesselNodePtr<DIM> > end_nodes;
    if(distributionType == VesselDistribution::REGULAR)
    {
        for(unsigned idx=0; idx<num_y; idx++)
        {
           for(unsigned jdx=0; jdx<num_x; jdx++)
           {
               QLength location_x = bbox[0] + spacing_x / 2.0 + double(jdx) * spacing_x;
               QLength location_y = bbox[2] + spacing_y / 2.0 + double(idx) * spacing_y;
               if(DIM==2)
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
               }
               else
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, bbox[4]));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, (bbox[4] + delta_z)));
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
           QLength location_x = bbox[0] + RandomNumberGenerator::Instance()->ranf() * delta_x;
           QLength location_y = bbox[2] + RandomNumberGenerator::Instance()->ranf() * delta_y;

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
                   double sq_distance = pow((start_nodes[kdx]->rGetLocation().Convert(mReferenceLength)[1]- location_y/mReferenceLength),2) +
                           pow((start_nodes[kdx]->rGetLocation().Convert(mReferenceLength)[0]- location_x/mReferenceLength),2);
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
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
               }
               else
               {
                   start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, bbox[4]));
                   end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, (bbox[4] + delta_z)));
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
        std::vector<std::vector<QLength > > kernel_locations;
        for(unsigned kdx=0; kdx<num_kernels; kdx++)
        {
            std::vector<QLength > location;
            location.push_back(bbox[0] + RandomNumberGenerator::Instance()->ranf() * delta_x);
            location.push_back(bbox[2] + RandomNumberGenerator::Instance()->ranf() * delta_y);
            kernel_locations.push_back(location);
        }

        // Pick locations randomly from the kernels
        for(unsigned jdx=0;jdx<1.e9;jdx++)
        {
            QLength deviation = 100.0* 1_um;
            QLength location_x = RandomNumberGenerator::Instance()->NormalRandomDeviate(0.0, deviation/mReferenceLength)*mReferenceLength;
            QLength location_y = RandomNumberGenerator::Instance()->NormalRandomDeviate(0.0, deviation/mReferenceLength)*mReferenceLength;
            unsigned kernel_index = RandomNumberGenerator::Instance()->randMod(num_kernels);
            location_x = location_x + kernel_locations[kernel_index][0];
            location_y = location_y + kernel_locations[kernel_index][1];

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
                    double sq_distance = pow((start_nodes[kdx]->rGetLocation().Convert(mReferenceLength)[1]- location_y/mReferenceLength),2) +
                            pow((start_nodes[kdx]->rGetLocation().Convert(mReferenceLength)[0]- location_x/mReferenceLength),2);
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
                    start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
                    end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y));
                }
                else
                {
                    start_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, bbox[4]));
                    end_nodes.push_back(VesselNode<DIM>::Create(location_x, location_y, (bbox[4] + delta_z)));
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
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    for(unsigned idx=0; idx<start_nodes.size(); idx++)
    {
        vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(start_nodes[idx], end_nodes[idx])));
    }

    // Generate and write the network
    std::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();
    p_network->AddVessels(vessels);

    return p_network;
}

template<unsigned DIM>
void VesselNetworkGenerator<DIM>::PatternUnitByTranslation(std::shared_ptr<VesselNetwork<DIM> > input_unit,
        std::array<unsigned, DIM> numberOfUnits)
{
    // Get unit dimensions
    std::pair<Vertex<DIM>, Vertex<DIM> > extents = VesselNetworkGeometryCalculator<DIM>::GetExtents(input_unit);

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
    std::vector<std::shared_ptr<Vessel<DIM> > > original_vessels = input_unit->GetVessels();

    for(unsigned idx =0; idx < num_x; idx++)
    {
        Vertex<DIM>translation_vector(double(idx+1) * (extents.second[0] - extents.first[0]));
        std::vector<std::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(original_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();

    std::vector<std::shared_ptr<Vessel<DIM> > > x_transform_vessels = input_unit->GetVessels();
    for(unsigned idx =0; idx < num_y; idx++)
    {
        Vertex<DIM>translation_vector(0_m, double(idx+1) * (extents.second[1] - extents.first[1]));
        std::vector<std::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(x_transform_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();
    std::vector<std::shared_ptr<Vessel<DIM> > > y_transform_vessels = input_unit->GetVessels();

    for(unsigned idx =0; idx < num_z; idx++)
    {
        Vertex<DIM>translation_vector(0_m, 0_m, double(idx+1) * (extents.second[2] - extents.first[2]));
        std::vector<std::shared_ptr<Vessel<DIM> > > copied_vessels = input_unit->CopyVessels(y_transform_vessels);
        input_unit->Translate(translation_vector, copied_vessels);
    }
    input_unit->MergeCoincidentNodes();
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalUnit(QLength vesselLength)
{
    // Generate the nodes
    std::vector<VesselNodePtr<DIM> > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0_m)); //0
    nodes.push_back(VesselNode<DIM>::Create(vesselLength, vesselLength)); //1
    nodes.push_back(VesselNode<DIM>::Create(0_m, 2.0 * vesselLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(2.0 * vesselLength, vesselLength)); //3
    nodes.push_back(VesselNode<DIM>::Create(3.0 * vesselLength)); //4
    nodes.push_back(VesselNode<DIM>::Create(4.0 * vesselLength)); //5
    nodes.push_back(VesselNode<DIM>::Create(3.0 * vesselLength, 2.0 * vesselLength)); //6
    if (DIM == 3)
    {
        nodes.push_back(VesselNode<DIM>::Create(0_m, 0_m, 1.5*vesselLength)); //7
        nodes.push_back(VesselNode<DIM>::Create(vesselLength, vesselLength, 1.5*vesselLength)); //8
        nodes.push_back(VesselNode<DIM>::Create(2.0 * vesselLength, vesselLength, 1.5*vesselLength)); //9
        nodes.push_back(VesselNode<DIM>::Create(3.0 * vesselLength, 0.0, 1.5*vesselLength)); //9
    }

    // Generate the segments and vessels
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
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
    std::shared_ptr<VesselNetwork<DIM> > pNetwork(new VesselNetwork<DIM>());
    pNetwork->AddVessels(vessels);

    return pNetwork;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateTwoHexagonalUnits(QLength vesselLength, QLength inputRadius, double alpha)
{
    // Generate the nodes
    std::vector<VesselNodePtr<DIM> > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0_m, 0_m)); //0
    nodes.push_back(VesselNode<DIM>::Create(vesselLength/sqrt(2.0), vesselLength/sqrt(2.0))); //1
    nodes.push_back(VesselNode<DIM>::Create(0_m, sqrt(2.0) * vesselLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(vesselLength/sqrt(2.0), (sqrt(2.0) + 1.0/sqrt(2.0))*vesselLength)); //3
    nodes.push_back(VesselNode<DIM>::Create((1.0 + 1.0/sqrt(2.0)) * vesselLength, (sqrt(2.0) + 1.0/sqrt(2.0))*vesselLength)); //4
    nodes.push_back(VesselNode<DIM>::Create((1.0 + sqrt(2.0)) * vesselLength, vesselLength*sqrt(2.0))); //5
    nodes.push_back(VesselNode<DIM>::Create((1.0 + 1.0/sqrt(2.0)) * vesselLength,vesselLength/sqrt(2.0))); //6
    nodes.push_back(VesselNode<DIM>::Create((1.0 + sqrt(2.0)) * vesselLength, 2.0*sqrt(2.0)*vesselLength)); //7
    nodes.push_back(VesselNode<DIM>::Create((2.0 + sqrt(2.0)) * vesselLength, 2.0*sqrt(2.0)*vesselLength)); //8
    nodes.push_back(VesselNode<DIM>::Create((2.0 + sqrt(2.0) + 1.0/sqrt(2.0)) * vesselLength, (sqrt(2.0) + 1.0/sqrt(2.0))*vesselLength)); //9
    nodes.push_back(VesselNode<DIM>::Create((2.0 + sqrt(2.0)) * vesselLength,vesselLength*sqrt(2.0))); //10
    nodes.push_back(VesselNode<DIM>::Create((2.0 + sqrt(2.0) + 1.0/sqrt(2.0)) * vesselLength,(2*sqrt(2.0) + 1.0/sqrt(2.0)) * vesselLength)); //11

    // Generate the segments and vessels
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[1]))); //0
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[2]))); //1
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[2], nodes[3]))); //2
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[4]))); //3
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[5]))); //4
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[5], nodes[6]))); //5
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[6], nodes[1]))); //6
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[7]))); //7
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[7], nodes[8]))); //8
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[8], nodes[9]))); //9
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[9], nodes[10]))); //10
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[10], nodes[5]))); //11
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[8], nodes[11]))); //12

    QLength first_branch_radius = inputRadius*pow(alpha, 1.0/3.0);
    QLength second_branch_radius = inputRadius*pow(1.0 - alpha, 1.0/3.0);
    QLength second_gen_first_branch_radius = first_branch_radius*pow(alpha, 1.0/3.0);
    QLength second_gen_second_branch_radius = first_branch_radius*pow(1.0 - alpha, 1.0/3.0);
    QLength radius_result = pow(pow(second_branch_radius, 3.0) + pow(second_gen_second_branch_radius, 3.0), 1.0/3.0);
    QLength radius_second_result =  pow(pow(second_gen_first_branch_radius, 3.0) + pow(radius_result, 3.0), 1.0/3.0);

    vessels[0]->SetRadius(inputRadius);
    vessels[1]->SetRadius(first_branch_radius);
    vessels[2]->SetRadius(first_branch_radius);
    vessels[3]->SetRadius(first_branch_radius);
    vessels[5]->SetRadius(second_branch_radius);
    vessels[6]->SetRadius(second_branch_radius);
    vessels[4]->SetRadius(second_gen_second_branch_radius);
    vessels[7]->SetRadius(second_gen_first_branch_radius);
    vessels[8]->SetRadius(second_gen_first_branch_radius);
    vessels[9]->SetRadius(radius_result);
    vessels[10]->SetRadius(radius_result);
    vessels[11]->SetRadius(radius_result);
    vessels[12]->SetRadius(radius_second_result);

    // Generate the network
    std::shared_ptr<VesselNetwork<DIM> > pNetwork(new VesselNetwork<DIM>());
    pNetwork->AddVessels(vessels);

    return pNetwork;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateBifurcationUnit(QLength vesselLength,
                                                                                          Vertex<DIM> startPosition)
{
    // Generate the nodes
    std::vector<VesselNodePtr<DIM> > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0_m, vesselLength)); //0
    nodes.push_back(VesselNode<DIM>::Create(vesselLength, vesselLength)); //1
    nodes.push_back(VesselNode<DIM>::Create(2.0 * vesselLength, 2.0 * vesselLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(2.0 * vesselLength)); //3
    nodes.push_back(VesselNode<DIM>::Create(3.0 * vesselLength, vesselLength)); //4
    nodes.push_back(VesselNode<DIM>::Create(4.0 * vesselLength, vesselLength)); //5

    // Generate the segments and vessels
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[1])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[2])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[3])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[2], nodes[4])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[4])));
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[4], nodes[5])));

    // Generate the network
    std::shared_ptr<VesselNetwork<DIM> > p_network(new VesselNetwork<DIM>());
    p_network->AddVessels(vessels);
    p_network->Translate(startPosition);
    p_network->UpdateAll();
    return p_network;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateTrueForkedBifurcationUnit(QLength vesselLength, QLength inputRadius, QDimensionless alpha)
{
    // Generate the nodes
    std::vector<VesselNodePtr<DIM> > nodes;
    nodes.push_back(VesselNode<DIM>::Create(0_m, vesselLength)); //0
    nodes.push_back(VesselNode<DIM>::Create(vesselLength, vesselLength)); //1
    nodes.push_back(VesselNode<DIM>::Create(vesselLength*(1.0 + 1.0/sqrt(2.0)), (1.0-1.0/sqrt(2.0)) * vesselLength)); //2
    nodes.push_back(VesselNode<DIM>::Create(vesselLength*(1.0 + 1.0/sqrt(2.0)), (1.0+1.0/sqrt(2.0)) * vesselLength)); //3
    nodes.push_back(VesselNode<DIM>::Create(vesselLength*(2.0 + 1.0/sqrt(2.0)), (1.0-1.0/sqrt(2.0)) * vesselLength)); //4
    nodes.push_back(VesselNode<DIM>::Create(vesselLength*(2.0 + 1.0/sqrt(2.0)), (1.0+1.0/sqrt(2.0)) * vesselLength)); //5

    // Generate the segments and vessels
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[0], nodes[1]))); //0
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[2]))); //1
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[1], nodes[3]))); //2
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[2], nodes[4]))); //3
    vessels.push_back(Vessel<DIM>::Create(VesselSegment<DIM>::Create(nodes[3], nodes[5]))); //4

    QLength left_vessel_radius = inputRadius*pow(alpha, 1.0/3.0);
    QLength right_vessel_radius = inputRadius*pow(1 - alpha, 1.0/3.0);

    vessels[0]->SetRadius(inputRadius);
    vessels[1]->SetRadius(right_vessel_radius);
    vessels[2]->SetRadius(left_vessel_radius);
    vessels[3]->SetRadius(right_vessel_radius);
    vessels[4]->SetRadius(left_vessel_radius);

    // Generate the network
    std::shared_ptr<VesselNetwork<DIM> > p_network(new VesselNetwork<DIM>());
    p_network->AddVessels(vessels);
    return p_network;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateBranchingNetwork(unsigned order,
                                                                                           QLength main_length,
											   QLength input_radius,
											   double twicelambda)
{
  // There will be no heterogeneity in radii between any two daughters
  double alpha = 1.0;

  double dimless_length = 1.0;

  for(unsigned i_aux3=1; i_aux3<order+1; i_aux3++)
    {
dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux3-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux3-1)));
    }
  //dimensional horizontal length of the domain
  //QLength domain_length = dimless_length*2.0*twicelambda*input_radius;

  // Vessels are laid out on a regular grid in forking pattern
  // There are extra two vessels - input and output
  // Create vessels by looping over the units, x direction is inside loop.

  std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
  std::shared_ptr<Vessel<DIM> >   pAuxVessel;

  for(unsigned i=0; i<order; i++)
  {

  double aux_dimless_length = 1.0;

  for(unsigned i_aux2=0; i_aux2<i; i_aux2++)
    {
aux_dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux2)/3.0)-pow(0.9,2.0)*pow(2.0, -2.0*double(i_aux2)));
    }

  // auxiliary variable calculating x-coordinates for nodes of vessels to be added
  QLength aux_dimensional_length = aux_dimless_length*twicelambda*input_radius;

    // the following for loop calculates order of the vessels being added
      // the order is stored in "count" variable after the for loop is finished, and will be used to assign vessel radii
    for(unsigned j=0; j<pow(2,i); j++)
    {
    unsigned aux = j;
    unsigned count=0;
    while(aux>0)
    {
    count+=aux%2;
    aux/=2;
    }

    // add vessels to the first half of the domain

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
               VesselNode<DIM>::Create((aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2.0)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
                                                                     pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));

    // OwnerRank here will serve to store information on vessel order
    pAuxVessel->SetOwnerRank(i+1);
    // radii follow Murray's law
          pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
    // distance to previous bifurcation is hardcoded
    pAuxVessel->SetDistToPrevBif(twicelambda*pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));
          // preference for haematocrit is hardcoded
    if(j % 2 == 0)
    {
    pAuxVessel->SetPreference(1);
    }
    else
    {
    pAuxVessel->SetPreference(0);
    }

    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                           VesselNode<DIM>::Create((aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
                                                                                pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));

          pAuxVessel->SetOwnerRank(i+1);

          pAuxVessel->SetRadius(pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

    pAuxVessel->SetDistToPrevBif(twicelambda*pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));

    if(j % 2 == 0)
    {
    pAuxVessel->SetPreference(0);
    }
    else
    {
    pAuxVessel->SetPreference(1);
    }
          pVesselNetwork->AddVessel(pAuxVessel);


    // add vessels to the second half of the domain

          }

  }

  // add input vessel

  pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                2.0*main_length),
                                                                         VesselNode<DIM>::Create(twicelambda*input_radius,
                                                                                2.0*main_length));

  pAuxVessel->SetOwnerRank(0);
  pAuxVessel->SetRadius(input_radius);
  pVesselNetwork->AddVessel(pAuxVessel);

 // add output vessel



  pVesselNetwork->MergeCoincidentNodes();
  pVesselNetwork->UpdateAll();

  return pVesselNetwork;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateSingleVessel(QLength vesselLength,
                                                                                       Vertex<DIM> startPosition,
                                                                                       unsigned divisions, unsigned axis)
{
    std::vector<VesselNodePtr<DIM> > nodes;
    nodes.push_back(VesselNode<DIM>::Create(startPosition)); //0
    QLength interval = vesselLength/double(divisions + 1);

    for(unsigned idx=0;idx<divisions+1;idx++)
    {
        if(DIM==2)
        {
            if(axis==0)
            {
                nodes.push_back(VesselNode<DIM>::Create(startPosition[0] + interval*double(idx+1), startPosition[1])); //1
            }
            else
            {
                nodes.push_back(VesselNode<DIM>::Create(startPosition[0], startPosition[1] + interval*double(idx+1))); //1
            }

        }
        else
        {
            if(axis==0)
            {
                nodes.push_back(VesselNode<DIM>::Create(startPosition[0] + interval*double(idx+1), startPosition[1], startPosition[2])); //1
            }
            else if(axis==1)
            {
                nodes.push_back(VesselNode<DIM>::Create(startPosition[0], startPosition[1] + interval*double(idx+1), startPosition[2])); //2
            }
            else
            {
                nodes.push_back(VesselNode<DIM>::Create(startPosition[0], startPosition[1], startPosition[2] + interval*double(idx+1))); //3
            }
        }
    }

    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    vessels.push_back(Vessel<DIM>::Create(nodes));

    // Generate the network
    std::shared_ptr<VesselNetwork<DIM> > p_network(new VesselNetwork<DIM>());
    p_network->AddVessels(vessels);
    return p_network;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateFromPart(PartPtr<DIM> part)
{
    std::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();

    // Get the polygons
    std::vector<PolygonPtr<DIM> > polygons = part->GetPolygons();
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels;
    for (unsigned idx = 0; idx < polygons.size(); idx++)
    {
        std::vector<std::shared_ptr<VesselSegment<DIM> > > segments;
        std::vector<std::shared_ptr<Vertex<DIM> > > vertices = polygons[idx]->rGetVertices();
        for (unsigned jdx = 1; jdx < vertices.size(); jdx++)
        {
            segments.push_back(VesselSegment<DIM>::Create(VesselNode<DIM>::Create(*vertices[jdx-1]),
                    VesselNode<DIM>::Create(*vertices[jdx])));
        }
        vessels.push_back(Vessel<DIM>::Create(segments));
    }
    p_network->AddVessels(vessels);
    p_network->MergeCoincidentNodes();
    return p_network;
}

template<unsigned DIM>
void VesselNetworkGenerator<DIM>::MapToSphere(std::shared_ptr<VesselNetwork<DIM> > pInputUnit,
                                              QLength radius, QLength thickess,
                                              double azimuthExtent, double polarExtent)
{
    std::pair<Vertex<DIM>, Vertex<DIM> > extents = VesselNetworkGeometryCalculator<DIM>::GetExtents(pInputUnit);
    std::vector<VesselNodePtr<DIM> > nodes = pInputUnit->GetNodes();
    for (auto& node : nodes)
    {
        c_vector<double, DIM> node_loc = node->rGetLocation().Convert(mReferenceLength);
        c_vector<double, DIM> offset = (extents.second - extents.first).Convert(mReferenceLength);

        double x_frac = node_loc[0] / offset[0];
        double azimuth_angle = x_frac * azimuthExtent;

        double y_frac = (node_loc[1] + 3.0) / offset[1];
        double polar_angle = y_frac * polarExtent;
        if(offset[2]>0.0)
        {
            double z_frac = node_loc[2]  / offset[2];
            radius = radius - thickess * z_frac;
        }
        Vertex<DIM>new_position(radius * std::cos(azimuth_angle) * std::sin(polar_angle),
                radius * std::cos(polar_angle),
                radius * std::sin(azimuth_angle) * std::sin(polar_angle));
        node->SetLocation(new_position);
    }
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalNetwork(QLength width,
                                                                                           QLength height,
                                                                                           QLength vessel_length,
                                                                                           bool fillDomain)
{
    // Vessels are laid out on a regular grid in a hexagonal pattern.
    // The repeating unit looks like this:
    //    \_/
    //    / \_
    // There is an extra set of lines along the top to 'close' the pattern.
    QLength unit_height = 2.0 * vessel_length;
    QLength unit_width = 4.0 * vessel_length;

    // The pattern may not reach the extents of the target area.
    unsigned units_in_x_direction = floor(width/ unit_width);
    unsigned units_in_y_direction = floor(height/ unit_height);
    bool extended_in_x = false;
    bool extended_in_y = false;

    if(fillDomain and width/unit_width > float(units_in_x_direction))
    {
        units_in_x_direction+=1;
        extended_in_x = true;
    }
    if(fillDomain and height/unit_height > float(units_in_y_direction))
    {
        units_in_y_direction+=1;
        extended_in_y=true;
    }

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
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    for(unsigned jdx = 0; jdx<units_in_y_direction; jdx++)
    {
        for(unsigned idx=0; idx<units_in_x_direction; idx++)
        {
            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length,
                                                                                                   double(jdx)*unit_height + vessel_length)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 2.0*vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length,
                                                                                                   double(jdx)*unit_height + vessel_length)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*vessel_length,
                                                                                                   double(jdx)*unit_height + vessel_length)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*vessel_length,
                                                                                                   double(jdx)*unit_height + 2.0*vessel_length)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 2.0*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*vessel_length,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + 4.0*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0)));
        }
    }

    // Add an extra line of vessels along the top
    for(unsigned idx=0; idx<units_in_x_direction; idx++)
    {
        pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 3.0*vessel_length,
                                                                              double(units_in_y_direction)*unit_height),
                                                                       VesselNode<DIM>::Create(double(idx)*unit_width + 4.0 *vessel_length,
                                                                                               double(units_in_y_direction)*unit_height)));
    }

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
        double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
        if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
        else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
    }

//    // Move single nodes outside the bounds to the bounds
//    // Remove vessels with both nodes outside of the bounds
//    vessels = pVesselNetwork->GetVessels();
//    for(unsigned idx=0; idx<vessels.size(); idx++)
//    {
//        c_vector<double, DIM> loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength);
//        c_vector<double, DIM> loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength);
//
//        if(loc_0[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], loc_1[2], mReferenceLength);
//            }
//        }
//
//        if(loc_0[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, loc_1[2], mReferenceLength);
//            }
//        }
//    }

    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}



template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalNetworkEquilateral(QLength width,
                                                                                           QLength height,
                                                                                           QLength vessel_length,
                                                                                           bool fillDomain)
{
    // Vessels are laid out on a regular grid in a hexagonal pattern.
    // The repeating unit looks like this:
    //    \_/
    //    / \_
    // There is an extra set of lines along the top to 'close' the pattern.
    QLength unit_height = sqrt(2.0) * vessel_length;
    QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    unsigned units_in_x_direction = floor(width/ unit_width);
    unsigned units_in_y_direction = floor(height/ unit_height);
    bool extended_in_x = false;
    bool extended_in_y = false;

    if(fillDomain and width/unit_width > float(units_in_x_direction))
    {
        units_in_x_direction+=1;
        extended_in_x = true;
    }
    if(fillDomain and height/unit_height > float(units_in_y_direction))
    {
        units_in_y_direction+=1;
        extended_in_y=true;
    }

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
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    for(unsigned jdx = 0; jdx<units_in_y_direction; jdx++)
    {
        for(unsigned idx=0; idx<units_in_x_direction; idx++)
        {
            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0))));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + sqrt(2.0)*vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0))));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0))));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + sqrt(2.0)*vessel_length)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0)));

            pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0)));
        }
    }

    // Add an extra line of vessels along the top
    for(unsigned idx=0; idx<units_in_x_direction; idx++)
    {
        pVesselNetwork->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                              double(units_in_y_direction)*unit_height),
                                                                       VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
                                                                                               double(units_in_y_direction)*unit_height)));
    }

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
        double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
        if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
        else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
    }

//    // Move single nodes outside the bounds to the bounds
//    // Remove vessels with both nodes outside of the bounds
//    vessels = pVesselNetwork->GetVessels();
//    for(unsigned idx=0; idx<vessels.size(); idx++)
//    {
//        c_vector<double, DIM> loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength);
//        c_vector<double, DIM> loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength);
//
//        if(loc_0[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], loc_1[2], mReferenceLength);
//            }
//        }
//
//        if(loc_0[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, loc_1[2], mReferenceLength);
//            }
//        }
//    }

    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}

template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateHexagonalNetworkRadius(QLength width,
                                                                                           QLength height,
                                                                                           QLength vessel_length,
											   QLength main_radius,
                                                                                           bool fillDomain)
{
    // Vessels are laid out on a regular grid in a hexagonal pattern.
    // The repeating unit looks like this:
    //    \_/
    //    / \_
    // There is an extra set of lines along the top to 'close' the pattern.
    QLength unit_height = sqrt(2.0) * vessel_length;
    QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    unsigned units_in_x_direction = floor(width/ unit_width);
    unsigned units_in_y_direction = floor(height/ unit_height);
    std::cout << units_in_x_direction << "\n";
    std::cout << units_in_y_direction << "\n";
    bool extended_in_x = false;
    bool extended_in_y = false;

    if(fillDomain and width/unit_width > float(units_in_x_direction))
    {
        units_in_x_direction+=1;
        extended_in_x = true;
    }
    if(fillDomain and height/unit_height > float(units_in_y_direction))
    {
        units_in_y_direction+=1;
        extended_in_y=true;
    }

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

    std::cout << units_in_x_direction << "\n";
    std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;
    for(unsigned jdx = 0; jdx<units_in_y_direction; jdx++)
    {
        for(unsigned idx=0; idx<units_in_x_direction; idx++)
        {
	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0)));


		    if(2.0*double(idx)+double(jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
                        pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
                    }
		    if(2.0*double(idx)+double(jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			  if(jdx<units_in_y_direction-1)
			  {
                          pAuxVessel->SetOwnerRank(2*idx+jdx);
                          pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			  }
			  else
			  {
			  pAuxVessel->SetOwnerRank(2*idx+jdx);
                          pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			  }
                    }
		     if(2.0*double(idx)+double(jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx>0)
			{
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+2.0)/3.0));
		    	}
			else
			{
			pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+1.0)/3.0));
			}

		    }



	    pVesselNetwork->AddVessel(pAuxVessel);



	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + 0.0,
                                                                                  double(jdx)*unit_height + sqrt(2.0)*vessel_length),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0)));



		    if(2.0*double(idx)+double(jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx<units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+1.0)/3.0));
			}
			else
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)-1.0)/3.0));
			}
                    }
		    if(double(2*idx+jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx<units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			}
			else
			{
			pAuxVessel->SetOwnerRank(2*idx+jdx-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)-1.0)/3.0));
			}
                    }
		     if(double(2*idx+jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {

			if(jdx>0)
			{
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+2.0)/3.0));
		    	}
			else
			{
			pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+2.0)/3.0));
			}

		    }


            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + vessel_length/sqrt(2.0),
                                                                                                   double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)));



		    if(double(2*idx+jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			//if((jdx==units_in_y_direction-1) && (idx==0))
			if(jdx==units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			}
			else
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+1.0)/3.0));
			}
                    }
		    if(double(2*idx+jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
                        pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
                    }
		     if(double(2*idx+jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx>0)
			{
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+1.0)/3.0));
		    	}
			else
			{
			pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx));
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx)))/3.0));
			}
		    }

            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + sqrt(2.0)*vessel_length));



		    if(double(2*idx+jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx==units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+1.0)/3.0));
			}
			else
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+2.0)/3.0));
			}
                    }
		    if(double(2*idx+jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
                        //pAuxVessel->SetOwnerRank(2*idx+jdx+1);
			pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
                    }
		     if(double(2*idx+jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx));
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx)))/3.0));
		    }
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+1.0/sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + vessel_length/sqrt(2.0)),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0));


		    if(double(2*idx+jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx==units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+2.0)/3.0));
			}
			else
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+2.0)/3.0));
			}
                    }
		    if(double(2*idx+jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
                        //pAuxVessel->SetOwnerRank(2*idx+jdx+1);

		        if(jdx>0)
			{
			pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			}
			else
			{
			pAuxVessel->SetOwnerRank(2*idx+jdx-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)-1.0)/3.0));
			}
		    }
		     if(double(2*idx+jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx>0)
			{
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+1.0)/3.0));
		    	}
			else
			{
			pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))-1.0)/3.0));
			}
		    }

            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                                  double(jdx)*unit_height + 0.0),
                                                                           VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
                                                                                                   double(jdx)*unit_height + 0.0));


 		    if(double(2*idx+jdx) < double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx==units_in_y_direction-1)
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+2.0)/3.0));
			}
			else
			{
                        pAuxVessel->SetOwnerRank(2*idx+jdx+2);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)+2.0)/3.0));
			}
                    }
		    if(double(2*idx+jdx) == double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
                        //pAuxVessel->SetOwnerRank(2*idx+jdx+1);
			if(jdx>0)
			{
			pAuxVessel->SetOwnerRank(2*idx+jdx);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx))/3.0));
			}
			else
			{
			pAuxVessel->SetOwnerRank(2*idx+jdx-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*double(idx)+double(jdx)-1.0)/3.0));
			}
                    }
		     if(double(2*idx+jdx) > double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
		    {
			if(jdx>0)
			{
		  	pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)+1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))+1.0)/3.0));
		    	}
			else
			{
			pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)+(units_in_y_direction-1)-(2*idx+jdx)-1);
                        pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)+(double(units_in_y_direction)-1.0)-(2.0*double(idx)+double(jdx))-1.0)/3.0));
			}
		    }

            pVesselNetwork->AddVessel(pAuxVessel);



        }
    }

    // Add an extra line of vessels along the top
    for(unsigned idx=0; idx<units_in_x_direction; idx++)
    {

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
                                                                              double(units_in_y_direction)*unit_height),
                                                                       VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
                                                                                               double(units_in_y_direction)*unit_height));


            if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    {
	    pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	    }
	    else
	    {
            pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	    }
            pVesselNetwork->AddVessel(pAuxVessel);
    }

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    for(unsigned idx=0; idx<vessels.size(); idx++)
    {
        double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
        double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
        double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
        if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
        else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
        {
            pVesselNetwork->RemoveVessel(vessels[idx], true);
            pVesselNetwork->UpdateAll();
        }
    }

//    // Move single nodes outside the bounds to the bounds
//    // Remove vessels with both nodes outside of the bounds
//    vessels = pVesselNetwork->GetVessels();
//    for(unsigned idx=0; idx<vessels.size(); idx++)
//    {
//        c_vector<double, DIM> loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength);
//        c_vector<double, DIM> loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength);
//
//        if(loc_0[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(width/mReferenceLength, loc_0[1], loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[0]>width/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(width/mReferenceLength, loc_1[1], loc_1[2], mReferenceLength);
//            }
//        }
//
//        if(loc_0[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetStartNode()->SetLocation(loc_0[0], height/mReferenceLength, loc_0[2], mReferenceLength);
//            }
//        }
//        if(loc_1[1]>height/mReferenceLength)
//        {
//            if(DIM==2)
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, 0.0, mReferenceLength);
//            }
//            else
//            {
//                vessels[idx]->GetEndNode()->SetLocation(loc_1[0], height/mReferenceLength, loc_1[2], mReferenceLength);
//            }
//        }
//    }

    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}



template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateDichotomousNetwork(unsigned order,
                                                                                           QLength main_length,
											   QLength main_radius,
                                                                                           bool fillDomain)
{

    QLength domain_length = (1.0+double(order))*2.0*main_length;
    // Vessels are laid out on a regular grid in dichotomous pattern
    // The repeating unit looks like this:
    //     ___
    //    |
    //    |___
    //
    // There is an extra set of lines along the top to 'close' the pattern.
    //QLength unit_height = sqrt(2.0) * vessel_length;
    //QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    //unsigned units_in_x_direction = floor(width/ unit_width);
    //unsigned units_in_y_direction = floor(height/ unit_height);
    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    //bool extended_in_x = false;
    //bool extended_in_y = false;

    //if(fillDomain and width/unit_width > float(units_in_x_direction))
    //{
    //    units_in_x_direction+=1;
    //    extended_in_x = true;
    //}
    //if(fillDomain and height/unit_height > float(units_in_y_direction))
    //{
    //    units_in_y_direction+=1;
    //    extended_in_y=true;
    //}

    // If the number of units is less than 1, just make a single unit in that direction
    //if (units_in_x_direction < 1)
    //{
    //   units_in_x_direction = 1;
    //}

    //if (units_in_y_direction < 1)
    //{
     //   units_in_x_direction = 1;
    //}

    // Create vessels by looping over the units, x direction is inside loop.

    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);








	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(main_radius/pow(2.0,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);
            }

    }

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    // Add an extra line of vessels along the top
    //for(unsigned idx=0; idx<units_in_x_direction; idx++)
    //{

	//    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
        //                                                                      double(units_in_y_direction)*unit_height),
         //                                                              VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
         //                                                                                      double(units_in_y_direction)*unit_height));
	 //

            //if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    //{
	    //pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	   // }
	    //else
	    //{
            //pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	   // }
            //pVesselNetwork->AddVessel(pAuxVessel);
    //}

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    //std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    //for(unsigned idx=0; idx<vessels.size(); idx++)
    //{
    //    double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
    //    {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
     //   }
     //   else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
     //   {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
      //  }
    //}

//    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateDichotomousNetworkUneven(unsigned order,
                                                                                           QLength main_length,
											   QLength main_radius,
											   double alpha,
                                                                                           bool fillDomain)
{

    QLength domain_length = (1.0+double(order))*2.0*main_length;
    // Vessels are laid out on a regular grid in dichotomous pattern
    // The repeating unit looks like this:
    //     ___
    //    |
    //    |___
    //
    // There is an extra set of lines along the top to 'close' the pattern.
    //QLength unit_height = sqrt(2.0) * vessel_length;
    //QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    //unsigned units_in_x_direction = floor(width/ unit_width);
    //unsigned units_in_y_direction = floor(height/ unit_height);
    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    //bool extended_in_x = false;
    //bool extended_in_y = false;

    //if(fillDomain and width/unit_width > float(units_in_x_direction))
    //{
    //    units_in_x_direction+=1;
    //    extended_in_x = true;
    //}
    //if(fillDomain and height/unit_height > float(units_in_y_direction))
    //{
    //    units_in_y_direction+=1;
    //    extended_in_y=true;
    //}

    // If the number of units is less than 1, just make a single unit in that direction
    //if (units_in_x_direction < 1)
    //{
    //   units_in_x_direction = 1;
    //}

    //if (units_in_y_direction < 1)
    //{
     //   units_in_x_direction = 1;
    //}

    // Create vessels by looping over the units, x direction is inside loop.

    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    unsigned aux = j;
	    unsigned count=0;
	    while(aux>0)
	    {
	    count+=aux%2;
	    aux/=2;
	    std::cout << "Count is  " << count << "  and aux is  " << aux << "\n";
	    }

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))),
                                                                           VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);








	    pAuxVessel = Vessel<DIM>::Create(
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))),
VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))),
VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))),
VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

	    pAuxVessel = Vessel<DIM>::Create(
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))),
VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);
            }

    }

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    // Add an extra line of vessels along the top
    //for(unsigned idx=0; idx<units_in_x_direction; idx++)
    //{

	//    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
        //                                                                      double(units_in_y_direction)*unit_height),
         //                                                              VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
         //                                                                                      double(units_in_y_direction)*unit_height));
	 //

            //if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    //{
	    //pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	   // }
	    //else
	    //{
            //pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	   // }
            //pVesselNetwork->AddVessel(pAuxVessel);
    //}

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    //std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    //for(unsigned idx=0; idx<vessels.size(); idx++)
    //{
    //    double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
    //    {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
     //   }
     //   else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
     //   {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
      //  }
    //}

//    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateDichotomousNetworkUnevenNoCorners(unsigned order,
                                                                                           QLength main_length,
											   QLength main_radius,
											   double alpha,
                                                                                           bool fillDomain)
{

    QLength domain_length = (1.0+double(order))*2.0*main_length;
    // Vessels are laid out on a regular grid in dichotomous pattern
    // The repeating unit looks like this:
    //     ___
    //    |
    //    |___
    //
    // There is an extra set of lines along the top to 'close' the pattern.
    //QLength unit_height = sqrt(2.0) * vessel_length;
    //QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    //unsigned units_in_x_direction = floor(width/ unit_width);
    //unsigned units_in_y_direction = floor(height/ unit_height);
    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    //bool extended_in_x = false;
    //bool extended_in_y = false;

    //if(fillDomain and width/unit_width > float(units_in_x_direction))
    //{
    //    units_in_x_direction+=1;
    //    extended_in_x = true;
    //}
    //if(fillDomain and height/unit_height > float(units_in_y_direction))
    //{
    //    units_in_y_direction+=1;
    //    extended_in_y=true;
    //}

    // If the number of units is less than 1, just make a single unit in that direction
    //if (units_in_x_direction < 1)
    //{
    //   units_in_x_direction = 1;
    //}

    //if (units_in_y_direction < 1)
    //{
     //   units_in_x_direction = 1;
    //}

    // Create vessels by looping over the units, x direction is inside loop.

    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    unsigned aux = j;
	    unsigned count=0;
	    while(aux>0)
	    {
	    count+=aux%2;
	    aux/=2;
	    std::cout << "Count is  " << count << "  and aux is  " << aux << "\n";
	    }

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
  					     VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create((double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                             VesselNode<DIM>::Create((double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);







	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-(double(i)+1.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(double(i)+2.0)*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

            }

    }

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    // Add an extra line of vessels along the top
    //for(unsigned idx=0; idx<units_in_x_direction; idx++)
    //{

	//    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
        //                                                                      double(units_in_y_direction)*unit_height),
         //                                                              VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
         //                                                                                      double(units_in_y_direction)*unit_height));
	 //

            //if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    //{
	    //pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	   // }
	    //else
	    //{
            //pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	   // }
            //pVesselNetwork->AddVessel(pAuxVessel);
    //}

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    //std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    //for(unsigned idx=0; idx<vessels.size(); idx++)
    //{
    //    double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
    //    {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
     //   }
     //   else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
     //   {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
      //  }
    //}

//    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}


template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateDichotomousNetworkUnevenNoCornersVaryDistance(unsigned order,
                                                                                           QLength main_length,
											   QLength main_radius,
											   double alpha,
											   double theta,
                                                                                           bool fillDomain)
{

    //QLength domain_length = (1.0+double(order))*2.0*main_length;
    QLength domain_length = ((1.0-pow(theta,double(order)+1.0))/(1.0-theta))*2.0*main_length;
    // Vessels are laid out on a regular grid in dichotomous pattern
    // The repeating unit looks like this:
    //     ___
    //    |
    //    |___
    //
    // There is an extra set of lines along the top to 'close' the pattern.
    //QLength unit_height = sqrt(2.0) * vessel_length;
    //QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    //unsigned units_in_x_direction = floor(width/ unit_width);
    //unsigned units_in_y_direction = floor(height/ unit_height);
    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    //bool extended_in_x = false;
    //bool extended_in_y = false;

    //if(fillDomain and width/unit_width > float(units_in_x_direction))
    //{
    //    units_in_x_direction+=1;
    //    extended_in_x = true;
    //}
    //if(fillDomain and height/unit_height > float(units_in_y_direction))
    //{
    //    units_in_y_direction+=1;
    //    extended_in_y=true;
    //}

    // If the number of units is less than 1, just make a single unit in that direction
    //if (units_in_x_direction < 1)
    //{
    //   units_in_x_direction = 1;
    //}

    //if (units_in_y_direction < 1)
    //{
     //   units_in_x_direction = 1;
    //}

    // Create vessels by looping over the units, x direction is inside loop.

    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    unsigned aux = j;
	    unsigned count=0;
	    while(aux>0)
	    {
	    count+=aux%2;
	    aux/=2;
	    std::cout << "Count is  " << count << "  and aux is  " << aux << "\n";
	    }

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(((1.0-pow(theta,double(i)+1.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
  					     VesselNode<DIM>::Create(((1.0-pow(theta,double(i)+2.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

	    pAuxVessel->SetDistToPrevBif(main_length*sqrt(pow(theta,2.0*double(i))+pow(0.5,2.0*(double(i)-1))));

	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(1);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(0);
	    }

	    pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(((1.0-pow(theta,double(i)+1.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                             VesselNode<DIM>::Create(((1.0-pow(theta,double(i)+2.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

	    pAuxVessel->SetDistToPrevBif(main_length*sqrt(pow(theta,2.0*double(i))+pow(0.5,2.0*(double(i)-1))));

	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(0);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(1);
	    }
            pVesselNetwork->AddVessel(pAuxVessel);







	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-((1.0-pow(theta,double(i)+1.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-((1.0-pow(theta,double(i)+2.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-((1.0-pow(theta,double(i)+1.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-((1.0-pow(theta,double(i)+2.0))/(1.0-theta))*main_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

            }

    }

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-1.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    // Add an extra line of vessels along the top
    //for(unsigned idx=0; idx<units_in_x_direction; idx++)
    //{

	//    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
        //                                                                      double(units_in_y_direction)*unit_height),
         //                                                              VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
         //                                                                                      double(units_in_y_direction)*unit_height));
	 //

            //if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    //{
	    //pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	   // }
	    //else
	    //{
            //pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	   // }
            //pVesselNetwork->AddVessel(pAuxVessel);
    //}

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    //std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    //for(unsigned idx=0; idx<vessels.size(); idx++)
    //{
    //    double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
    //    {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
     //   }
     //   else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
     //   {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
      //  }
    //}

//    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}



template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateDichotomousNetworkUnevenNoCornersVaryDistanceLengthsFollowRadii(unsigned order,
                                                                                           QLength main_length,
											   QLength main_radius,
											   double alpha,
											   double theta,
											   double lambda,
                                                                                           bool fillDomain)
{

    double dimless_length = 1.0;

    for(unsigned i_aux3=1; i_aux3<order+1; i_aux3++)
    	{
	dimless_length += sqrt(pow(2.0,-2.0*double(i_aux3)/3.0)-pow(2.0, -2.0*double(i_aux3)));
    	}

    QLength domain_length = dimless_length*2.0*lambda*main_radius;

    std::cout << "Domain length is:" << domain_length << "\n";
    // Vessels are laid out on a regular grid in dichotomous pattern
    // The repeating unit looks like this:
    //     ___
    //    |
    //    |___
    //
    // There is an extra set of lines along the top to 'close' the pattern.
    //QLength unit_height = sqrt(2.0) * vessel_length;
    //QLength unit_width = (2.0+sqrt(2.0)) * vessel_length;

    // The pattern may not reach the extents of the target area.
    //unsigned units_in_x_direction = floor(width/ unit_width);
    //unsigned units_in_y_direction = floor(height/ unit_height);
    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    //bool extended_in_x = false;
    //bool extended_in_y = false;

    //if(fillDomain and width/unit_width > float(units_in_x_direction))
    //{
    //    units_in_x_direction+=1;
    //    extended_in_x = true;
    //}
    //if(fillDomain and height/unit_height > float(units_in_y_direction))
    //{
    //    units_in_y_direction+=1;
    //    extended_in_y=true;
    //}

    // If the number of units is less than 1, just make a single unit in that direction
    //if (units_in_x_direction < 1)
    //{
    //   units_in_x_direction = 1;
    //}

    //if (units_in_y_direction < 1)
    //{
     //   units_in_x_direction = 1;
    //}

    // Create vessels by looping over the units, x direction is inside loop.

    //std::cout << units_in_x_direction << "\n";
    //std::cout << units_in_y_direction << "\n";
    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {

    double aux_dimless_length = 1.0;

    for(unsigned i_aux2=0; i_aux2<i; i_aux2++)
    	{
	aux_dimless_length += sqrt(pow(2.0,-2.0*(double(i_aux2)+1.0)/3.0)-pow(2.0, -2.0*(double(i_aux2)+1.0)));
    	}

    QLength aux_dimensional_length = aux_dimless_length*lambda*main_radius;

    std::cout << "Auxiliary dimensional length is is:" << aux_dimensional_length << "\n";
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    unsigned aux = j;
	    unsigned count=0;
	    while(aux>0)
	    {
	    count+=aux%2;
	    aux/=2;
	    std::cout << "Count is  " << count << "  and aux is  " << aux << "\n";
	    }

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
  					     VesselNode<DIM>::Create((aux_dimless_length+sqrt(pow(2.0,-2.0*(double(i)+1.0)/3.0)-pow(2.0, -2.0*(double(i)+1.0))))*lambda*main_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

	    pAuxVessel->SetDistToPrevBif(lambda*pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));

	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(1);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(0);
	    }

	    pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                             VesselNode<DIM>::Create((aux_dimless_length+sqrt(pow(2.0,-2.0*(double(i)+1.0)/3.0)-pow(2.0, -2.0*(double(i)+1.0))))*lambda*main_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

	    pAuxVessel->SetDistToPrevBif(lambda*pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));

	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(0);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(1);
	    }
            pVesselNetwork->AddVessel(pAuxVessel);







	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(aux_dimless_length+sqrt(pow(2.0,-2.0*(double(i)+1.0)/3.0)-pow(2.0, -2.0*(double(i)+1.0))))*lambda*main_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(aux_dimless_length+sqrt(pow(2.0,-2.0*(double(i)+1.0)/3.0)-pow(2.0, -2.0*(double(i)+1.0))))*lambda*main_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));
	    pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*main_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

            }

    }

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(2.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-2.0*main_length,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(main_radius);
    pVesselNetwork->AddVessel(pAuxVessel);


    // Add an extra line of vessels along the top
    //for(unsigned idx=0; idx<units_in_x_direction; idx++)
    //{

	//    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(double(idx)*unit_width + (1.0+sqrt(2.0))*vessel_length,
        //                                                                      double(units_in_y_direction)*unit_height),
         //                                                              VesselNode<DIM>::Create(double(idx)*unit_width + (2.0+sqrt(2.0))*vessel_length,
         //                                                                                      double(units_in_y_direction)*unit_height));
	 //

            //if(double(2*idx+units_in_y_direction) >= double((units_in_y_direction -1 + 2*(units_in_x_direction -1)))/2.0)
	    //{
	    //pAuxVessel->SetOwnerRank(2*(units_in_x_direction-1)-2*idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(2.0*(double(units_in_x_direction)-1.0)-2.0*double(idx))/3.0));
	   // }
	    //else
	    //{
            //pAuxVessel->SetOwnerRank(units_in_y_direction+idx);
            //pAuxVessel->SetRadius(main_radius/pow(2.0,(double(idx)+double(units_in_y_direction))/3.0));
	   // }
            //pVesselNetwork->AddVessel(pAuxVessel);
    //}

    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    // Remove vessels with both nodes outside of the bounds
    //std::vector<std::shared_ptr<Vessel<DIM> > > vessels = pVesselNetwork->GetVessels();
    //for(unsigned idx=0; idx<vessels.size(); idx++)
    //{
    //    double x_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_0 = vessels[idx]->GetStartNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    double x_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[0];
    //    double y_loc_1 = vessels[idx]->GetEndNode()->rGetLocation().Convert(mReferenceLength)[1];
    //    if(extended_in_x and x_loc_0>width/mReferenceLength and x_loc_1>width/mReferenceLength)
    //    {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
     //   }
     //   else if(extended_in_y and y_loc_0>height/mReferenceLength and y_loc_1>height/mReferenceLength)
     //   {
     //       pVesselNetwork->RemoveVessel(vessels[idx], true);
     //       pVesselNetwork->UpdateAll();
      //  }
    //}

//    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}



template<unsigned DIM>
std::shared_ptr<VesselNetwork<DIM> > VesselNetworkGenerator<DIM>::GenerateForkingNetworkNoCorners(unsigned order,
                                                                                           QLength main_length,
											   QLength input_radius,
											   double twicelambda,
                                                                                           bool fillDomain)
{
    // There will be no heterogeneity in radii between any two daughters
    double alpha = 1.0;

    double dimless_length = 1.0;

    for(unsigned i_aux3=1; i_aux3<order+1; i_aux3++)
    	{
	dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux3-1)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i_aux3-1)));
    	}
    //dimensional horizontal length of the domain
    QLength domain_length = dimless_length*2.0*twicelambda*input_radius;

    // Vessels are laid out on a regular grid in forking pattern
    // There are extra two vessels - input and output
    // Create vessels by looping over the units, x direction is inside loop.

    std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork = VesselNetwork<DIM>::Create();
    std::shared_ptr<Vessel<DIM> >   pAuxVessel;

    for(unsigned i=0; i<order; i++)
    {

    double aux_dimless_length = 1.0;

    for(unsigned i_aux2=0; i_aux2<i; i_aux2++)
    	{
	aux_dimless_length += pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i_aux2)/3.0)-pow(0.9,2.0)*pow(2.0, -2.0*double(i_aux2)));
    	}

    // auxiliary variable calculating x-coordinates for nodes of vessels to be added
    QLength aux_dimensional_length = aux_dimless_length*twicelambda*input_radius;

	    // the following for loop calculates order of the vessels being added
  	    // the order is stored in "count" variable after the for loop is finished, and will be used to assign vessel radii
	    for(unsigned j=0; j<pow(2,i); j++)
	    {
	    unsigned aux = j;
	    unsigned count=0;
	    while(aux>0)
	    {
	    count+=aux%2;
	    aux/=2;
	    }

	    // add vessels to the first half of the domain

	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
  					     VesselNode<DIM>::Create((aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2.0)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
	                                                                     pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));

	    // OwnerRank here will serve to store information on vessel order
	    pAuxVessel->SetOwnerRank(i+1);
	    // radii follow Murray's law
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
	    // distance to previous bifurcation is hardcoded
	    pAuxVessel->SetDistToPrevBif(twicelambda*pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));
            // preference for haematocrit is hardcoded
	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(1);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(0);
	    }

	    pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                             VesselNode<DIM>::Create((aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));

            pAuxVessel->SetOwnerRank(i+1);

            pAuxVessel->SetRadius(pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));

	    pAuxVessel->SetDistToPrevBif(twicelambda*pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,double(i)/3.0));

	    if(j % 2 == 0)
	    {
	    pAuxVessel->SetPreference(0);
	    }
	    else
	    {
	    pAuxVessel->SetPreference(1);
	    }
            pVesselNetwork->AddVessel(pAuxVessel);


	    // add vessels to the second half of the domain


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.5+2.0*double(j))));


            // radii and owner ranks ("orders") are assigned symmetrically to the first half
            pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count)+1.0)*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);


	    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length-aux_dimensional_length,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(1.0+2.0*double(j))),
                                                                           VesselNode<DIM>::Create(domain_length-(aux_dimless_length+pow(2.0,-1/3)*sqrt(pow(2.0,-2.0*double(i)/3.0)-pow(0.9,2)*pow(2.0, -2.0*double(i))))*twicelambda*input_radius,
                                                                                  pow(0.5,double(i)-1.0)*main_length*(0.5+2.0*double(j))));

            pAuxVessel->SetOwnerRank(i+1);
            pAuxVessel->SetRadius(pow(alpha,double(count))*input_radius/pow(1.0+alpha*alpha*alpha,(double(i)+1.0)/3.0));
            pVesselNetwork->AddVessel(pAuxVessel);

            }

    }

    // add input vessel

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(0.0*main_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(twicelambda*input_radius,
                                                                                  2.0*main_length));

    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(input_radius);
    pVesselNetwork->AddVessel(pAuxVessel);

   // add output vessel

    pAuxVessel = Vessel<DIM>::Create(VesselNode<DIM>::Create(domain_length,
                                                                                  2.0*main_length),
                                                                           VesselNode<DIM>::Create(domain_length-twicelambda*input_radius,
                                                                                  2.0*main_length));
    pAuxVessel->SetOwnerRank(0);
    pAuxVessel->SetRadius(input_radius);
    pVesselNetwork->AddVessel(pAuxVessel);



    pVesselNetwork->MergeCoincidentNodes();
    pVesselNetwork->UpdateAll();

    return pVesselNetwork;
}



//Explicit instantiation
template class VesselNetworkGenerator<2> ;
template class VesselNetworkGenerator<3> ;
