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

#ifndef TESTREGULARGRIDWRITER_HPP_
#define TESTREGULARGRIDWRITER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "OutputFileHandler.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestRegularGridWriter : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestWriteSimpleGrid() throw(Exception)
    {
        // Set up a 3d grid
        boost::shared_ptr<RegularGrid<3> > p_grid_3d = RegularGrid<3>::Create();
        std::vector<unsigned> extents_3d(3);
        extents_3d[0] = 5;
        extents_3d[1] = 7;
        extents_3d[2] = 4;
        p_grid_3d->SetExtents(extents_3d);

        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(0,0,0), 0u)
        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(3,1,2), 78u)
        TS_ASSERT_EQUALS(p_grid_3d->Get1dGridIndex(0,3,3), 120u)

        OutputFileHandler output_file_handler("TestRegularGridWriter", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("SimpleGrid.vti");
        RegularGridWriter writer;
        writer.SetFilename(output_filename);
        writer.SetImage(p_grid_3d->GetVtkGrid());
        writer.Write();
    }

};

#endif /*TESTREGULARGRIDWRITER_HPP_*/
