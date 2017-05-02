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
#include "Debug.hpp"

class TestGeometryWriter : public CxxTest::TestSuite
{

public:

    void TestWriteCuboid()
    {
        Part<3> part = Part<3>();
        part.AddCuboid(1.e-6*unit::metres, 1.e-6*unit::metres, 1.e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        part.GetFacets()[0]->SetLabel("Facet 0");
        part.GetFacets()[1]->SetLabel("Facet 1");
        part.GetFacets()[2]->SetLabel("Facet 2");
        part.GetFacets()[3]->SetLabel("Facet 3");
        part.GetFacets()[4]->SetLabel("Facet 4");
        part.GetFacets()[5]->SetLabel("Facet 5");
        part.GetPolygons()[0]->LabelAllEdges("Edge 0");
        part.GetPolygons()[1]->LabelAllEdges("Edge 1");
        part.GetPolygons()[2]->LabelAllEdges("Edge 2");
        part.GetPolygons()[3]->LabelAllEdges("Edge 3");
        part.GetPolygons()[4]->LabelAllEdges("Edge 4");
        part.GetPolygons()[5]->LabelAllEdges("Edge 5");

        OutputFileHandler output_file_handler("TestGeometryWriter/TestWriteCuboid");
        part.Write(output_file_handler.GetOutputDirectoryFullPath() + "cube_alone_edges.vtp",
                 GeometryFormat::VTP, true);

        boost::shared_ptr<GeometryWriter> p_writer = GeometryWriter::Create();
        TS_ASSERT_THROWS_THIS(p_writer->Write(), "An input geometry is not set.");

        p_writer->SetInput(part.GetVtk());
        TS_ASSERT_THROWS_THIS(p_writer->Write(), "No file name set for the GeometryWriter.");
        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.vtp");
        p_writer->SetOutputFormat(GeometryFormat::VTP);
        p_writer->Write();

        p_writer->SetFileName(output_file_handler.GetOutputDirectoryFullPath() + "cube.stl");
        p_writer->SetOutputFormat(GeometryFormat::STL);
        p_writer->Write();

        part.Write(output_file_handler.GetOutputDirectoryFullPath() + "cube_alone.vtp",
                 GeometryFormat::VTP, false);
    }

};

#endif /*TESTGEOMETRYWRITER_HPP_*/
