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



#ifndef TESTMAPPABLEGRIDGENERATOR_HPP_
#define TESTMAPPABLEGRIDGENERATOR_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <math.h>
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "MappableGridGenerator.hpp"
#include "Part.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DimensionalChastePoint.hpp"
#include "VtkMeshWriter.hpp"

class TestMappableGridGenerator : public CxxTest::TestSuite
{

public:

    void TestMakePlane() throw(Exception)
    {
        // Make one with and without end caps
        MappableGridGenerator generator;
        boost::shared_ptr<Part<3> > p_part = generator.GeneratePlane(10, 10);
        TS_ASSERT_EQUALS(p_part->GetVertices().size(), 200u);
        TS_ASSERT_EQUALS(p_part->GetPolygons().size(), 198u);

        boost::shared_ptr<Part<3> > p_part_no_caps = generator.GeneratePlane(10, 10, false);
        TS_ASSERT_EQUALS(p_part_no_caps->GetVertices().size(), 200u);
        TS_ASSERT_EQUALS(p_part_no_caps->GetPolygons().size(), 180u);

        // Make sure the resulting part can be meshed
        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->Update();

        VtkMeshWriter<3, 3> mesh_writer("TestMappableGridGenerator", "Plane", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }

    void TestMakeCylinder() throw(Exception)
    {
        // Make one closed cylinder, one open cylinder and one with too large an angle.
        MappableGridGenerator generator;
        boost::shared_ptr<Part<3> > p_part = generator.GenerateCylinder(1.5, 0.1 , 5.0, 10, 10);
        boost::shared_ptr<Part<3> > p_part_open = generator.GenerateCylinder(1.5, 0.1 , 5.0, 10, 10, M_PI);
        TS_ASSERT_THROWS_ANYTHING(generator.GenerateCylinder(1.5, 0.1 , 5.0, 10, 10, 2.1*M_PI));

        // Make sure the vertices are in the expected locations
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices = p_part->GetVertices();
        for(unsigned idx=0; idx<vertices.size(); idx++)
        {
            double loc_x = vertices[idx]->GetLocation(1.e-6*unit::metres)[0];
            double loc_z = vertices[idx]->GetLocation(1.e-6*unit::metres)[2];
            double distance = std::sqrt(loc_x*loc_x + loc_z*loc_z);
            bool is_inside = (distance < 1.5  + 1.e-6) && (distance > 1.4  - 1.e-6);
            TS_ASSERT(is_inside);
        }

        // Make sure the part can be meshed
        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->Update();
        VtkMeshWriter<3, 3> mesh_writer("TestMappableGridGenerator", "Closed_Cylinder", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }


    void TestMakeHemisphere() throw(Exception)
    {
        // Make one good and two 'bad' hemispheres
        MappableGridGenerator generator;
        boost::shared_ptr<Part<3> > p_part = generator.GenerateHemisphere(1.5, 0.1 , 10, 10, M_PI, 0.5*M_PI);
        TS_ASSERT_THROWS_ANYTHING(generator.GenerateHemisphere(1.5, 0.1 , 10, 10, 2.0*M_PI, 0.5*M_PI));
        TS_ASSERT_THROWS_ANYTHING(generator.GenerateHemisphere(1.5, 0.1 , 10, 10, M_PI, 1.0*M_PI));

        // Make sure the vertices are in the expected locations
        std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > vertices = p_part->GetVertices();
        for(unsigned idx=0; idx<vertices.size(); idx++)
        {
            bool is_inside = (vertices[idx]->GetNorm2()/(1.e-6*unit::metres) < 1.5  + 1.e-6) &&
                    (vertices[idx]->GetNorm2()/(1.e-6*unit::metres) > 1.4  - 1.e-6);
            TS_ASSERT(is_inside);
        }

        // Make sure the part can be meshed
        boost::shared_ptr<DiscreteContinuumMeshGenerator<3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3>::Create();
        p_mesh_generator->SetDomain(p_part);
        p_mesh_generator->Update();
        VtkMeshWriter<3, 3> mesh_writer("TestMappableGridGenerator", "Hemisphere", false);
        mesh_writer.WriteFilesUsingMesh(*(p_mesh_generator->GetMesh()));
    }
};

#endif /*TESTMAPPABLEGRIDGENERATORHPP_*/
