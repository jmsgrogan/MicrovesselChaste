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

#ifndef TESTIMAGETOMESH_HPP_
#define TESTIMAGETOMESH_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "ImageToMesh.hpp"
#include "ImageReader.hpp"
#include "RegularGridWriter.hpp"
#include "SurfaceCleaner.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "VtkMeshWriter.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "Part.hpp"

class TestImageToMesh : public CxxTest::TestSuite
{
public:

    void Test2dMesh()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestImageToMesh/", false);
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/median.tif", RelativeTo::ChasteSourceRoot);

        ImageReader reader = ImageReader();
        reader.SetFilename(finder.GetAbsolutePath());
        reader.SetImageResizeFactors(0.5, 0.5, 1.0);
        reader.Read();

        RegularGridWriter writer = RegularGridWriter();
        writer.SetFilename(file_handler1.GetOutputDirectoryFullPath() + "/resized_image.vti");
        writer.SetImage(reader.GetImage());
        writer.Write();

        // Do the meshing
        boost::shared_ptr<ImageToMesh<2> > p_image_mesher = ImageToMesh<2>::Create();

        TS_ASSERT_THROWS_THIS(p_image_mesher->GetMesh(), "No mesh set. Did you run 'Update()' ?");
        p_image_mesher->SetInput(reader.GetImage());
        p_image_mesher->SetElementSize(2.e6*units::pow<3>(1.e-6*unit::metres));
        p_image_mesher->Update();

        VtkMeshWriter<2, 2> mesh_writer("TestImageToMesh", "Image2d", false);
        mesh_writer.WriteFilesUsingMesh(*(p_image_mesher->GetMesh()));
    }

    void DontTest2dMeshTissue()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestImageToMesh/", false);
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/median.tif", RelativeTo::ChasteSourceRoot);

        ImageReader reader = ImageReader();
        reader.SetFilename(finder.GetAbsolutePath());
        reader.SetImageResizeFactors(0.5, 0.5, 1.0);
        reader.Read();

        RegularGridWriter writer = RegularGridWriter();
        writer.SetFilename(file_handler1.GetOutputDirectoryFullPath() + "/resized_image.vti");
        writer.SetImage(reader.GetImage());
        writer.Write();

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        c_vector<double, 2> origin;
        origin[0] = 200.0;
        origin[1] = 200.0;
        p_domain->AddRectangle(2000.0*unit::metres, 1400.0*unit::metres, DimensionalChastePoint<2>(origin, 1.e-6*unit::metres));

        // Do the meshing
        ImageToMesh<2> image_mesher = ImageToMesh<2>();
        image_mesher.SetInput(reader.GetImage());
        image_mesher.SetElementSize(2.e6*units::pow<3>(1.e-6*unit::metres));
        image_mesher.SetTissueDomain(p_domain);
        image_mesher.Update();

        VtkMeshWriter<2, 2> mesh_writer("TestImageToMesh", "Image2dTissue", false);
        mesh_writer.WriteFilesUsingMesh(*(image_mesher.GetMesh()));

        MultiFormatMeshWriter<2> dolfin_mesh_writer;
        dolfin_mesh_writer.SetMesh(image_mesher.GetMesh());
        dolfin_mesh_writer.SetFilename(file_handler1.GetOutputDirectoryFullPath() + "/RetinalTissue");
//        dolfin_mesh_writer.SetOutputFormat(MeshFormat::DOLFIN);
        dolfin_mesh_writer.SetOutputFormat(MeshFormat::STL);
        dolfin_mesh_writer.Write();

//        LabelDolfinMesh<2> labeller;
//        labeller.SetInputFilename(file_handler1.GetOutputDirectoryFullPath() + "/RetinalTissue.xml");
//        labeller.SetOutputFilename(file_handler1.GetOutputDirectoryFullPath() + "/RetinalTissueLabel");
//        labeller.SetElementAttributes(image_mesher.GetMesh2d()->GetElementRegionMarkers());
//        labeller.Update();
    }
};
#endif
