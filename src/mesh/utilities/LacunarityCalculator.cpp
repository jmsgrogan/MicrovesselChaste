/*

 Copyright (c) 2005-2015, University of Oxford.
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

#include "VesselSegment.hpp"
#include "ChastePoint.hpp"
#include "GeometryTools.hpp"
#include "LacunarityCalculator.hpp"

template<unsigned DIM>
LacunarityCalculator<DIM>::LacunarityCalculator() :
        AbstractRegularGridDiscreteContinuumSolver<DIM>(), mpNetwork()
{

}

template<unsigned DIM>
boost::shared_ptr<LacunarityCalculator<DIM> > LacunarityCalculator<DIM>::Create()
{
    MAKE_PTR(LacunarityCalculator, pSelf);
    return pSelf;
}

template<unsigned DIM>
LacunarityCalculator<DIM>::~LacunarityCalculator()
{

}

template<unsigned DIM>
void LacunarityCalculator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void LacunarityCalculator<DIM>::Solve()
{
    unsigned extents_x = this->mpRegularGrid->GetExtents()[0];

    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments;
    segments = this->mpNetwork->GetVesselSegments();

    // Get the box widths
    std::vector<unsigned> width_factors;
    width_factors.push_back(1);
    width_factors.push_back(2);
    width_factors.push_back(4);
    width_factors.push_back(8);
    width_factors.push_back(16);

    std::ofstream output_file((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "output.dat").c_str());
    if (output_file.is_open())
    {
        output_file << "Lacunarity, Box\n";

    }

    for (unsigned width_index = 0; width_index < width_factors.size(); width_index++)
    {
        double box_size = double(extents_x - 1) / double(width_factors[width_index])
                * this->mpRegularGrid->GetSpacing()/this->mpRegularGrid->GetReferenceLengthScale();
        double q1 = 0.0;
        double q2 = 0.0;

        unsigned z_extent = width_factors[width_index];
        z_extent = 1;

        for (unsigned kdx = 0; kdx < z_extent; kdx++)
        {
            for (unsigned jdx = 0; jdx < width_factors[width_index]; jdx++)
            {
                for (unsigned idx = 0; idx < width_factors[width_index]; idx++)
                {
                    c_vector<double, DIM> box_location;
                    box_location[0] = double(idx) * box_size + box_size / 2.0;
                    box_location[1] = double(jdx) * box_size + box_size / 2.0;
                    box_location[2] = double(kdx) * box_size + box_size / 2.0;

                    double vessel_length = 0.0;
                    for (unsigned seg_index = 0; seg_index < segments.size(); seg_index++)
                    {
                        vessel_length += LengthOfLineInBox<DIM>(segments[seg_index]->GetNode(0)->rGetLocation().rGetLocation(),
                                                                segments[seg_index]->GetNode(1)->rGetLocation().rGetLocation(),
                                                                box_location, box_size);
                    }
                    q1 += vessel_length;
                    q2 += vessel_length * vessel_length;

                }
            }
        }
        unsigned num_boxes = width_factors[width_index] * width_factors[width_index] * z_extent;
        double lacunarity = double(num_boxes) * q2 / (q1 * q1);

        if (output_file.is_open())
        {
            output_file << lacunarity << ", " << box_size << " \n";
        }
    }
    output_file.close();
}

// Explicit instantiation
template class LacunarityCalculator<3> ;
