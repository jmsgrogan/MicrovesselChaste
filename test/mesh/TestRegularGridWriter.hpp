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

#ifndef TESTREGULARGRIDWRITER_HPP_
#define TESTREGULARGRIDWRITER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkImageData.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "OutputFileHandler.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestRegularGridWriter : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestWriteSimpleGrid() throw(Exception)
    {
        OutputFileHandler output_file_handler("TestRegularGridWriter", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("SimpleGrid.vti");

        // Set up a 3d grid
        std::shared_ptr<RegularGrid<3> > p_grid_3d = RegularGrid<3>::Create();
        c_vector<unsigned, 3> dimensions;
        dimensions[0] = 5;
        dimensions[1] = 7;
        dimensions[2] = 4;
        p_grid_3d->SetDimensions(dimensions);

        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(3,1,2), 78u)
        TS_ASSERT_EQUALS(p_grid_3d->GetGlobalGridIndex(0,3,3), 120u)

        std::shared_ptr<RegularGridWriter> p_writer = RegularGridWriter::Create();
        TS_ASSERT_THROWS_THIS(p_writer->Write(), "Output file not specified for image writer.");
        p_writer->SetFilename(output_filename);
        p_writer->SetImage(vtkImageData::SafeDownCast(p_grid_3d->GetGlobalVtkGrid()));
        p_writer->Write();
    }

};

#endif /*TESTREGULARGRIDWRITER_HPP_*/
