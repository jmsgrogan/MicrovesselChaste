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

#include "Exception.hpp"
#include <boost/filesystem.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkXMLPImageDataWriter.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkVersion.h>
#include <vtkTrivialProducer.h>
#include <vtkProgrammableFilter.h>
#include "PetscTools.hpp"
#include "RegularGridWriter.hpp"

RegularGridWriter::RegularGridWriter()
    : mpVtkImage(vtkSmartPointer<vtkImageData>::New()),
      mFilepath(),
      mWholeExtents()
{

}

boost::shared_ptr<RegularGridWriter> RegularGridWriter::Create()
{
    MAKE_PTR(RegularGridWriter, pSelf);
    return pSelf;
}

RegularGridWriter::~RegularGridWriter()
{

}

void RegularGridWriter::SetImage(vtkSmartPointer<vtkImageData> pImage)
{
    mpVtkImage = pImage;
}

void RegularGridWriter::SetFilename(const std::string& filename)
{
    mFilepath = filename;
}

void RegularGridWriter::SetWholeExtents(std::vector<unsigned> wholeExtents)
{
    mWholeExtents = wholeExtents;
}

// Helper method to correctly set parallel output extents
void SetParallelExtents(void* arguments)
{
    vtkProgrammableFilter* p_programmable_filter =
          static_cast<vtkProgrammableFilter*>(arguments);
    vtkImageData* p_input_image = static_cast<vtkImageData*>(p_programmable_filter->GetInput());

    int local_extents[6];
    unsigned piece_size = unsigned(std::floor(p_input_image->GetDimensions()[0]/PetscTools::GetNumProcs()));
    unsigned local_piece_size = piece_size;
    if (PetscTools::AmTopMost())
    {
        local_piece_size = (p_input_image->GetDimensions()[0] - piece_size*PetscTools::GetNumProcs()) + piece_size;
    }

    if(PetscTools::AmMaster())
    {
        local_extents[0] = PetscTools::GetMyRank()*piece_size;
    }
    else
    {
        local_extents[0] = PetscTools::GetMyRank()*piece_size-1;
    }

    local_extents[1] = PetscTools::GetMyRank()*piece_size + local_piece_size-1;
    local_extents[2] = 0;
    local_extents[3] = p_input_image->GetDimensions()[1]-1;
    local_extents[4] = 0;
    local_extents[5] = p_input_image->GetDimensions()[2]-1;

    p_programmable_filter->GetOutput()->ShallowCopy(p_programmable_filter->GetInput());
    p_programmable_filter->GetOutput()->Crop(local_extents);
}
void RegularGridWriter::Write()
{
    if(mFilepath == "")
    {
        EXCEPTION("Output file not specified for image writer.");
    }

    if(!mpVtkImage)
    {
        EXCEPTION("Output image not set for image writer.");
    }

//    vtkSmartPointer<vtkTrivialProducer> tp = vtkSmartPointer<vtkTrivialProducer>::New();
//    tp->SetOutput(mpVtkImage);
    if(PetscTools::IsSequential())
    {
        vtkSmartPointer<vtkXMLImageDataWriter> p_writer1 = vtkSmartPointer<vtkXMLImageDataWriter>::New();
        p_writer1->SetFileName(mFilepath.c_str());
        #if VTK_MAJOR_VERSION <= 5
            p_writer1->SetInput(mpVtkImage);
        #else
            p_writer1->SetInputData(mpVtkImage);
        #endif
        p_writer1->Write();
    }
    else
    {
//        if(mWholeExtents.size()!=6)
//        {
//            EXCEPTION("Writing in parallel requires the whole extents to be set.");
//        }
        mpVtkImage->SetExtent(mWholeExtents[0], mWholeExtents[1], mWholeExtents[2],
                mWholeExtents[3], mWholeExtents[4], mWholeExtents[5]);
//        tp->SetWholeExtent(mWholeExtents[0], mWholeExtents[1], mWholeExtents[2],
//                mWholeExtents[3], mWholeExtents[4], mWholeExtents[5]);

        vtkSmartPointer<vtkProgrammableFilter> p_programmable_filter =
            vtkSmartPointer<vtkProgrammableFilter>::New();
        p_programmable_filter->SetInputData(mpVtkImage);
        p_programmable_filter->SetExecuteMethod(SetParallelExtents, p_programmable_filter);

        vtkSmartPointer<vtkXMLPImageDataWriter> p_writer1 = vtkSmartPointer<vtkXMLPImageDataWriter>::New();

        p_writer1->SetFileName(mFilepath.c_str());

    //        #if VTK_MAJOR_VERSION <= 5
    //            p_writer1->SetInput(mpVtkImage);
    //        #else
    //            p_writer1->SetInputData(mpVtkImage);
    //        #endif
        p_writer1->SetNumberOfPieces(PetscTools::GetNumProcs());
        p_writer1->SetEndPiece(PetscTools::GetMyRank());
        p_writer1->SetStartPiece(PetscTools::GetMyRank());
        p_writer1->SetInputConnection(p_programmable_filter->GetOutputPort());
        p_writer1->Write();
    }
}
