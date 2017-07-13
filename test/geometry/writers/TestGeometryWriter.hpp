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

    void TestWriteCuboid() throw(Exception)
    {
        Part<3> part;
        part.AddCuboid(1_um, 1_um, 1_um);
        part.GetPolygons()[0]->AddAttribute("Polygon Number", 1.0);
        part.GetPolygons()[1]->AddAttribute("Polygon Number", 2.0);
        part.GetPolygons()[2]->AddAttribute("Polygon Number", 3.0);
        part.GetPolygons()[3]->AddAttribute("Polygon Number", 4.0);
        part.GetPolygons()[4]->AddAttribute("Polygon Number", 5.0);
        part.GetPolygons()[5]->AddAttribute("Polygon Number", 6.0);
        part.GetPolygons()[0]->AddAttributeToAllEdges("Parent Polygon", 0.0);
        part.GetPolygons()[1]->AddAttributeToAllEdges("Parent Polygon", 1.0);
        part.GetPolygons()[2]->AddAttributeToAllEdges("Parent Polygon", 2.0);
        part.GetPolygons()[3]->AddAttributeToAllEdges("Parent Polygon", 3.0);
        part.GetPolygons()[4]->AddAttributeToAllEdges("Parent Polygon", 4.0);
        part.GetPolygons()[5]->AddAttributeToAllEdges("Parent Polygon", 5.0);

        OutputFileHandler output_file_handler("TestGeometryWriter/TestWriteCuboid");

        auto p_writer = GeometryWriter::Create();
        TS_ASSERT_THROWS_THIS(p_writer->Write(), "An input geometry is not set.");

        p_writer->AddInput(part.GetVtk());
        TS_ASSERT_THROWS_THIS(p_writer->Write(), "No file name set for the GeometryWriter.");
        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.vtp");
        p_writer->SetOutputFormat(GeometryFormat::VTP);
        p_writer->Write();

        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.stl");
        p_writer->SetOutputFormat(GeometryFormat::STL);
        p_writer->Write();

        part.Write(output_file_handler.GetOutputDirectoryFullPath() + "cube_alone.vtp", GeometryFormat::VTP, false);
    }

    void TestWriteTwoParts() throw(Exception)
    {
        Part<3> part1;
        part1.AddCuboid(1_um, 1_um, 1_um);

        Part<3> part2;
        part2.AddCuboid(3_um, 3_um, 3_um);

        OutputFileHandler output_file_handler("TestGeometryWriter/TestWriteTwoPart");
        auto p_writer = GeometryWriter::Create();
        p_writer->AddInput(part1.GetVtk());
        p_writer->AddInput(part2.GetVtk());
        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.vtp");
        p_writer->SetOutputFormat(GeometryFormat::VTP);
        p_writer->Write();

        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.stl");
        p_writer->SetOutputFormat(GeometryFormat::STL);
        p_writer->Write();
    }

};

#endif /*TESTGEOMETRYWRITER_HPP_*/
