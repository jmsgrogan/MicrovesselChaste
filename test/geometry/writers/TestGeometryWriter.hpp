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

#ifndef TESTGEOMETRYWRITER_HPP_
#define TESTGEOMETRYWRITER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "SmartPointers.hpp"
#include "Polygon.hpp"
#include "Part.hpp"
#include "OutputFileHandler.hpp"
#include "GeometryWriter.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestGeometryWriter : public CxxTest::TestSuite
{

public:

    void TestWriteCuboid()
    {
        Part<3> part = Part<3>();
        part.AddCuboid(1.e-6*unit::metres, 1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        OutputFileHandler output_file_handler("TestGeometryWriter/TestWriteCuboid");

        GeometryWriter writer;
        writer.SetInput(part.GetVtk());
        writer.SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.vtp");
        writer.SetOutputFormat(GeometryFormat::VTP);
        writer.Write();

        writer.SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.stl");
        writer.SetOutputFormat(GeometryFormat::STL);
        writer.Write();
    }

};

#endif /*TESTGEOMETRYWRITER_HPP_*/
