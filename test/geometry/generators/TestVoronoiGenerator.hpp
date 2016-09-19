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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#ifndef TESTVORONOIGENERATOR_HPP_
#define TESTVORONOIGENERATOR_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "VoronoiGenerator.hpp"
#include "OutputFileHandler.hpp"

class TestVoronoiGenerator : public CxxTest::TestSuite
{
public:

    // Voronoi generation with tetgen no longer supported
    void TestSquare()
    {
        boost::shared_ptr<Part<3> > p_part = Part<3>::Create();
        p_part->AddCuboid(3000*1.e-6*unit::metres,
                          1500*1.e-6*unit::metres,
                          200*1.e-6*unit::metres,
                          DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        VoronoiGenerator<3> generator;
//        boost::shared_ptr<Part<3> > p_tesselation = generator.Generate(p_part, std::vector<boost::shared_ptr<Vertex> >(), 200);
//        std::vector<boost::shared_ptr<Vertex> > vertices = p_tesselation->GetVertices();
//        for(unsigned idx=0; idx < vertices.size(); idx++)
//        {
//            vertices[idx]->SetCoordinate(1, 2.0 * vertices[idx]->rGetLocation()[1]);
//        }
//        OutputFileHandler output_file_handler("TestVoronoiNetwork", false);
//        p_tesselation->Write(output_file_handler.GetOutputDirectoryFullPath() + "part.vtp");
    }
};

#endif /*TESTVORONOIGENERATOR_HPP_*/
