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



#ifndef TESTIMAGEREADERANDWRITER_HPP_
#define TESTIMAGEREADERANDWRITER_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "ImageReader.hpp"
#include "RegularGridWriter.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"

class TestImageReaderAndWriter : public CxxTest::TestSuite
{
public:

    void TestImageIO()
    {
        // Read the image from file
        OutputFileHandler file_handler1 = OutputFileHandler("TestImageReaderAndWriter/");
        FileFinder finder = FileFinder("projects/MicrovesselChaste/test/data/median.tif", RelativeTo::ChasteSourceRoot);

        // Read the file in tif format
        boost::shared_ptr<ImageReader> p_image_reader = ImageReader::Create();
        p_image_reader->SetImageResizeFactors(0.2, 0.2, 1.0);

        TS_ASSERT_THROWS_THIS(p_image_reader->GetImage(), "No image data has been set.");
        TS_ASSERT_THROWS_THIS(p_image_reader->Read(), "Input file not specified for image reader");
        p_image_reader->SetFilename(finder.GetAbsolutePath());
        p_image_reader->Read();

        // Write it out in VTI format
        RegularGridWriter image_writer;
        image_writer.SetFilename(file_handler1.GetOutputDirectoryFullPath()+"image_vtk_format.vti");
        image_writer.SetImage(p_image_reader->GetImage());
        image_writer.Write();
    }
};
#endif
