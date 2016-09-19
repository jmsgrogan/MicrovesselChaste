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
#include <vtkBox.h>
#include "VesselSegment.hpp"
#include "ChastePoint.hpp"
#include "DensityMap.hpp"

template<unsigned DIM>
DensityMap<DIM>::DensityMap()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>()
{

}

template<unsigned DIM>
boost::shared_ptr<DensityMap<DIM> > DensityMap<DIM>::Create()
{
    MAKE_PTR(DensityMap<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
DensityMap<DIM>::~DensityMap()
{

}

template<unsigned DIM>
bool DensityMap<DIM>::IsPointInBox(c_vector<double, DIM> point, c_vector<double, DIM> location, double spacing)
{
    bool point_in_box = false;
    if(point[0] >= location[0] -spacing/2.0 && point[0] <= location [0] + spacing/2.0)
    {
        if(point[1] >= location[1] -spacing/2.0 && point[1] <= location [1] + spacing/2.0)
        {
            if(DIM == 3)
            {
                if(point[2] >= location[2] -spacing/2.0 && point[2] <= location [2] + spacing/2.0)
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        }
    }
    return point_in_box;
}

template<unsigned DIM>
double DensityMap<DIM>::LengthOfLineInBox(c_vector<double, DIM> start_point, c_vector<double, DIM> end_point, c_vector<double, DIM> location, double spacing)
{
    if(DIM==2)
    {
        EXCEPTION("Line in box method is currently 3D only");
    }

    // If the line is fully in the box return its length
    bool point1_in_box = IsPointInBox(start_point, location, spacing);
    bool point2_in_box = IsPointInBox(end_point, location, spacing);
    if(point1_in_box && point2_in_box)
    {
        return norm_2(end_point - start_point);
    }
    else
    {
        c_vector<double,2*DIM> bounds;
        bounds[0] = location[0] - spacing/2.0;
        bounds[1] = location[0] + spacing/2.0;
        bounds[2] = location[1] - spacing/2.0;
        bounds[3] = location[1] + spacing/2.0;
        if(DIM==3)
        {
            bounds[4] = location[2] - spacing/2.0;
            bounds[5] = location[2] + spacing/2.0;
        }

        double t1;
        double t2;
        int plane1;
        int plane2;
        c_vector<double,DIM> intercept_1;
        c_vector<double,DIM> intercept_2;

        int in_box = vtkBox::IntersectWithLine(&bounds[0], &start_point[0], &end_point[0], t1, t2, &intercept_1[0], &intercept_2[0], plane1, plane2);

        if(point1_in_box)
        {
            return norm_2(intercept_2 - start_point);
        }

        if(point2_in_box)
        {
            return norm_2(intercept_1 - end_point);
        }

        if(in_box)
        {
            return norm_2(intercept_2 - intercept_1);
        }
        else
        {
            return 0.0;
        }
    }
}

template<unsigned DIM>
void DensityMap<DIM>::Solve()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    unsigned number_of_points = this->mpRegularGrid->GetNumberOfPoints();
    unsigned extents_x = this->mpRegularGrid->GetExtents()[0];
    unsigned extents_y = this->mpRegularGrid->GetExtents()[1];
    unsigned extents_z = this->mpRegularGrid->GetExtents()[2];
    double spacing = this->mpRegularGrid->GetSpacing()/this->mpRegularGrid->GetReferenceLengthScale();

    std::vector<double> vessel_solution(number_of_points, 0.0);
    std::vector<boost::shared_ptr<VesselSegment<DIM> > > segments;
    if (this->mpNetwork)
    {
        segments = this->mpNetwork->GetVesselSegments();
        for (unsigned i = 0; i < extents_z; i++) // Z
        {
            for (unsigned j = 0; j < extents_y; j++) // Y
            {
                for (unsigned k = 0; k < extents_x; k++) // X
                {
                    unsigned grid_index = this->mpRegularGrid->Get1dGridIndex(k, j, i);
                    c_vector<double, DIM> location = this->mpRegularGrid->GetLocation(k ,j, i).rGetLocation();

                    for (unsigned idx = 0; idx <  segments.size(); idx++)
                    {
                        vessel_solution[grid_index] += LengthOfLineInBox(segments[idx]->GetNode(0)->rGetLocation().rGetLocation(),
                                                                         segments[idx]->GetNode(1)->rGetLocation().rGetLocation(),
                                                                         location, spacing);
                    }
                    vessel_solution[grid_index] /= (std::pow(spacing,3));
                }
            }
        }
    }

    this->UpdateSolution(vessel_solution);

    if (this->mWriteSolution)
    {
        this->Write();
    }
}

// Explicit instantiation
template class DensityMap<2> ;
template class DensityMap<3> ;
