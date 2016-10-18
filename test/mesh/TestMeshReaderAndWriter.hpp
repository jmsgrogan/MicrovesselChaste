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

#ifndef TESTMESHREADERANDWRITER_HPP_
#define TESTMESHREADERANDWRITER_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "MeshReader.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"

class TestMeshReaderAndWriter : public CxxTest::TestSuite
{
public:

    void TestMeshIO()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestMeshReaderAndWriter/");
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/bifurcation_mesh.vtu", RelativeTo::ChasteSourceRoot);

        // Read the file
        boost::shared_ptr<MeshReader> p_mesh_reader = MeshReader::Create();
        p_mesh_reader->SetFilename(finder.GetAbsolutePath());
        p_mesh_reader->Read();

        // Write it out in vtk format
        MultiFormatMeshWriter<3> mesh_writer;
        mesh_writer.SetFilename(file_handler1.GetOutputDirectoryFullPath()+"bifurcation_mesh");
        mesh_writer.SetMesh(p_mesh_reader->GetMesh());
        mesh_writer.SetOutputFormat(MeshFormat::VTU);
        mesh_writer.Write();

        // Write it out in dolfin format
//        mesh_writer.SetOutputFormat(MeshFormat::DOLFIN);
//        mesh_writer.Write();

        // Write it out in stl format
        mesh_writer.SetOutputFormat(MeshFormat::STL);
        mesh_writer.Write();
    }

    void Test2dMeshIO()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestMeshReaderAndWriter2d/");
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/retinal_2d.vtu", RelativeTo::ChasteSourceRoot);

        // Read the file
        MeshReader mesh_reader;
        mesh_reader.SetFilename(finder.GetAbsolutePath());
        mesh_reader.Read();

        // Write it out in vtk format
        MultiFormatMeshWriter<2> mesh_writer;
        mesh_writer.SetFilename(file_handler1.GetOutputDirectoryFullPath()+"retinal_2d");
        mesh_writer.SetMesh(mesh_reader.GetMesh());
        mesh_writer.Write();

        // Write it out in dolfin format
//        mesh_writer.SetOutputFormat(MeshFormat::DOLFIN);
//        mesh_writer.Write();
    }
};
#endif
