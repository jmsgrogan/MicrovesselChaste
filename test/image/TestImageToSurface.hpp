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

#ifndef TESTIMAGETOSURFACE_HPP_
#define TESTIMAGETOSURFACE_HPP_

#include <cxxtest/TestSuite.h>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkXMLImageDataWriter.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"
#include "ImageToSurface.hpp"
#include "ImageReader.hpp"
#include "SurfaceCleaner.hpp"
#include "GeometryWriter.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"

class TestImageToSurface : public CxxTest::TestSuite
{
public:

    void TestDefaultExtraction() throw(Exception)
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestImageToSurface/");
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/median.tif", RelativeTo::ChasteSourceRoot);

        ImageReader reader = ImageReader();
        reader.SetFilename(finder.GetAbsolutePath());
        reader.SetImageResizeFactors(0.5, 0.5, 1.0);
        reader.Read();

        vtkSmartPointer<vtkXMLImageDataWriter> p_writer1 = vtkSmartPointer<vtkXMLImageDataWriter>::New();
        p_writer1->SetFileName((file_handler1.GetOutputDirectoryFullPath()+"image.vti").c_str());
        #if VTK_MAJOR_VERSION <= 5
            p_writer1->SetInput(reader.GetImage());
        #else
            p_writer1->SetInputData(reader.GetImage());
        #endif
        p_writer1->Write();

        // Extract the surface
        std::shared_ptr<ImageToSurface> p_surface_extract = ImageToSurface::Create();

        TS_ASSERT_THROWS_THIS(p_surface_extract->GetOutput(), "No output set. Did you run 'Update()' ?");
        p_surface_extract->SetInput(reader.GetImage());
        p_surface_extract->SetThreshold(1.0, false);
        p_surface_extract->Update();

        // Write the surface to file
        std::shared_ptr<GeometryWriter> p_writer = GeometryWriter::Create();
        p_writer->SetFileName((file_handler1.GetOutputDirectoryFullPath()+"surface.vtp").c_str());
        p_writer->AddInput(p_surface_extract->GetOutput());
        p_writer->Write();
        p_writer->ClearInputs();

        // Clean the surface
        std::shared_ptr<SurfaceCleaner> p_cleaner = SurfaceCleaner::Create();
        p_cleaner->SetInput(p_surface_extract->GetOutput());
        p_cleaner->SetDecimateTargetReduction(0.995);
        p_cleaner->SetLinearSubdivisionNumber(1);
        p_cleaner->Update();

        p_writer->SetFileName((file_handler1.GetOutputDirectoryFullPath()+"surface_cleaned.vtp").c_str());
        p_writer->AddInput(p_cleaner->GetOutput());
        p_writer->Write();

        p_writer->SetFileName((file_handler1.GetOutputDirectoryFullPath()+"surface_cleaned.stl").c_str());
        p_writer->SetOutputFormat(GeometryFormat::STL);
        p_writer->Write();
        p_writer->ClearInputs();

        // Use marching cubes
        p_surface_extract->SetUseMarchingCubes(true);
        p_surface_extract->Update();
        p_writer->SetFileName((file_handler1.GetOutputDirectoryFullPath()+"surface_marching_cubes.vtp").c_str());
        p_writer->AddInput(p_surface_extract->GetOutput());
        p_writer->Write();
    }
};
#endif
