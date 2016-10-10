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

#include <math.h>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkBox.h>
#include <vtkTetra.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include "RandomNumberGenerator.hpp"
#include "Polygon.hpp"

#include "VoronoiGenerator.hpp"

// Hang Si's tetgen.

#define REAL double
#define VOID void
#include "tetgen.h"
#undef REAL
#undef VOID

struct triangulateio;

template<unsigned DIM>
VoronoiGenerator<DIM>::VoronoiGenerator()
{
}

template<unsigned DIM>
VoronoiGenerator<DIM>::~VoronoiGenerator()
{

}

template<unsigned DIM>
boost::shared_ptr<Part<DIM> > VoronoiGenerator<DIM>::Generate(boost::shared_ptr<Part<DIM> > pPart,
                                                              std::vector<boost::shared_ptr<DimensionalChastePoint<DIM> > > seeds,
                                                              unsigned numSeeds)
{
    if(DIM==2)
    {
        EXCEPTION("This generator works in 3D only");
    }

    boost::shared_ptr<Part<DIM> > p_tesselation = Part<DIM>::Create();
    c_vector<double, 2*DIM> extents = pPart->GetBoundingBox();
    units::quantity<unit::length> length_scale = pPart->GetReferenceLengthScale();

    // If no seeds have been provided generate some random ones
    if(seeds.size() == 0)
    {
        for(unsigned idx = 0; idx< numSeeds; idx++)
        {
            c_vector<double, DIM> location;
            for(unsigned jdx=0; jdx<DIM; jdx++)
            {
                location[jdx] = extents[2*jdx] + (extents[2*jdx + 1] - extents[2*jdx])*RandomNumberGenerator::Instance()->ranf();
            }
            seeds.push_back(DimensionalChastePoint<DIM>::Create(location, length_scale));
        }
    }

    // Use tetgen to make the tessellation
    class tetgen::tetgenio mesher_input, mesher_output;

    mesher_input.initialize();
    mesher_output.initialize();
    mesher_input.numberofpoints = seeds.size();
    mesher_input.pointlist = new double[mesher_input.numberofpoints * DIM];
    for (unsigned idx=0; idx<seeds.size(); idx++)
    {
        for (unsigned jdx=0; jdx < DIM; jdx++)
        {
            mesher_input.pointlist[DIM*idx + jdx] = double((*seeds[idx])[jdx]);
        }
    }

    tetgen::tetrahedralize((char*)"veeQ", &mesher_input, &mesher_output);

    // Create 2-point polygons corresponding to each edge
    std::vector<boost::shared_ptr<Polygon<DIM> > > polygons;
    for (int n=0; n<mesher_output.numberofvedges; ++n)
    {
      tetgen::tetgenio::voroedge e =  mesher_output.vedgelist[n];
      int n0 = e.v1;
      int n1 = e.v2;
      double* u = &mesher_output.vpointlist[DIM*n0];
      double* v = n1 == -1 ? e.vnormal : &mesher_output.vpointlist[DIM*n1]; // -1 indicates ray
      boost::shared_ptr<DimensionalChastePoint<DIM> > p_vertex1 = DimensionalChastePoint<DIM>::Create(u[0], u[1], u[2], length_scale);
      boost::shared_ptr<DimensionalChastePoint<DIM> > p_vertex2 = DimensionalChastePoint<DIM>::Create(v[0], v[1], v[2], length_scale);

      // Only include edges that are inside the domain - use the bounding box.
      double zero_location_tol = 20.0;
      double t1;
      double t2;
      int plane1;
      int plane2;
      c_vector<double,DIM> intercept_1;
      c_vector<double,DIM> intercept_2;
      int in_box = vtkBox::IntersectWithLine(&extents[0], &p_vertex1->rGetLocation(length_scale)[0],
                                             &p_vertex2->rGetLocation(length_scale)[0], t1, t2, &intercept_1[0], &intercept_2[0], plane1, plane2);

      // If the line is not outside the box
      if(in_box!=0)
      {
          // if the line does not intersect the box it must be fully inside
          if(in_box==-1)
          {
              // Check that we haven't hit the zero location and remove very short edges
              if((norm_2(p_vertex1->rGetLocation(length_scale)) > zero_location_tol) && (norm_2(p_vertex2->rGetLocation(length_scale)) > zero_location_tol))
              {
                  if(norm_2(p_vertex1->rGetLocation(length_scale) - p_vertex2->rGetLocation(length_scale)) > 2.0 * zero_location_tol)
                  {
                      boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(p_vertex1);
                                p_polygon->AddVertex(p_vertex2);
                                polygons.push_back(p_polygon);
                  }
              }
          }
          else
          {
              // Find where the points are
              bool vert1_inside = true;
              bool vert2_inside = true;
              if(p_vertex1->rGetLocation(length_scale)[0] < extents[0] || p_vertex1->rGetLocation(length_scale)[0] > extents[1])
              {
                  vert1_inside = false;
              }
              if(p_vertex1->rGetLocation(length_scale)[1] < extents[2] || p_vertex1->rGetLocation(length_scale)[1] > extents[3])
              {
                  vert1_inside = false;
              }
              if(p_vertex2->rGetLocation(length_scale)[0] < extents[0] || p_vertex2->rGetLocation(length_scale)[0] > extents[1])
              {
                  vert2_inside = false;
              }
              if(p_vertex2->rGetLocation(length_scale)[1] < extents[2] || p_vertex2->rGetLocation(length_scale)[1] > extents[3])
              {
                  vert2_inside = false;
              }
              if(DIM==3)
              {
                  if(p_vertex1->rGetLocation(length_scale)[2] < extents[4] || p_vertex1->rGetLocation(length_scale)[2] > extents[5])
                  {
                      vert1_inside = false;
                  }
                  if(p_vertex2->rGetLocation(length_scale)[2] < extents[4] || p_vertex2->rGetLocation(length_scale)[2] > extents[5])
                  {
                      vert2_inside = false;
                  }
              }

              if(vert1_inside && !vert2_inside)
              {
                  // move vert 2 to intsersection point
                  c_vector<double,DIM> gap = intercept_1-p_vertex2->rGetLocation(length_scale);
                  p_vertex2->Translate(gap);
              }
              else if(vert2_inside && !vert1_inside)
              {
                  // move vert 1 to intersection point
                  c_vector<double,DIM> gap = intercept_1-p_vertex1->rGetLocation(length_scale);
                  p_vertex1->Translate(gap);
              }
              else
              {
                  // Otherwise both points are either on or outside
                  c_vector<double,DIM> tv1 = intercept_1-p_vertex1->rGetLocation(length_scale);
                  c_vector<double,DIM> tv2 = intercept_2-p_vertex2->rGetLocation(length_scale);
                  if(norm_2(tv1) > zero_location_tol)
                  {
                      p_vertex1->Translate(tv1);
                  }
                  if(norm_2(tv2) > zero_location_tol)
                  {
                      p_vertex2->Translate(tv2);
                  }
              }

              // Check that we haven't hit the zero location and remove very short edges
              if((norm_2(p_vertex1->rGetLocation(length_scale)) > zero_location_tol) && (norm_2(p_vertex2->rGetLocation(length_scale)) > zero_location_tol))
              {
                  if(norm_2(p_vertex1->rGetLocation(length_scale) - p_vertex2->rGetLocation(length_scale)) > 2.0 * zero_location_tol)
                  {
                      boost::shared_ptr<Polygon<DIM> > p_polygon = Polygon<DIM>::Create(p_vertex1);
                                p_polygon->AddVertex(p_vertex2);
                                polygons.push_back(p_polygon);
                  }
              }
          }
      }
    }

    // Add the polygons to the part
    for (unsigned idx=0; idx<polygons.size(); idx++)
    {
        p_tesselation->AddPolygon(polygons[idx]);
    }

    return p_tesselation;
}

//Explicit instantiation
template class VoronoiGenerator<2> ;
template class VoronoiGenerator<3> ;
