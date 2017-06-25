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

#define _BACKWARD_BACKWARD_WARNING_H 1
#include <vtkXMLPolyDataWriter.h>
#include <vtkSTLWriter.h>
#include <vtkVersion.h>
#include <vtkTriangleFilter.h>
#include <vtkCleanPolyData.h>
#include "Exception.hpp"
#include "PetscTools.hpp"
#include "GeometryWriter.hpp"

GeometryWriter::GeometryWriter() :
    mpInputGeometries(),
    mFilename(),
    mFormat(GeometryFormat::VTP)
{

}

GeometryWriter::~GeometryWriter()
{

}

std::shared_ptr<GeometryWriter > GeometryWriter::Create()
{
    return std::make_shared<GeometryWriter>();
}

void GeometryWriter::SetFileName(const std::string& rFileName)
{
    mFilename = rFileName;
}

void GeometryWriter::AddInput(vtkSmartPointer<vtkPolyData> pSurface)
{
    mpInputGeometries.push_back(pSurface);
}

void GeometryWriter::ClearInputs()
{
    mpInputGeometries.clear();
}

void GeometryWriter::SetOutputFormat(GeometryFormat::Value format)
{
    mFormat = format;
}

void GeometryWriter::Write(bool masterOnly)
{
    if(mpInputGeometries.size()==0)
    {
        EXCEPTION("An input geometry is not set.");
    }

    if(mFilename.empty())
    {
        EXCEPTION("No file name set for the GeometryWriter.");
    }

    if(mFormat == GeometryFormat::STL)
    {
        if((masterOnly and PetscTools::AmMaster()) or !masterOnly)
        {

            vtkSmartPointer<vtkTriangleFilter> p_tri_filter = vtkSmartPointer<vtkTriangleFilter>::New();
            if(mpInputGeometries.size()==1)
            {
                #if VTK_MAJOR_VERSION <= 5
                p_tri_filter->SetInput(mpInputGeometries[0]);
                #else
                p_tri_filter->SetInputData(mpInputGeometries[0]);
                #endif
            }
            else
            {
                vtkSmartPointer<vtkAppendPolyData> p_append = vtkSmartPointer<vtkAppendPolyData>::New();
                for(unsigned idx=0;idx<mpInputGeometries.size();idx++)
                {
                    #if VTK_MAJOR_VERSION <= 5
                        p_append->AddInput(mpInputGeometries[idx]);
                    #else
                        p_append->AddInputData(mpInputGeometries[idx]);
                    #endif
                }
                p_tri_filter->SetInputConnection(p_append->GetOutputPort());
            }

            vtkSmartPointer<vtkCleanPolyData> p_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New();
            p_clean_filter->SetInputConnection(p_tri_filter->GetOutputPort());

            vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
            writer->SetFileName(mFilename.c_str());
            writer->SetInputConnection(p_clean_filter->GetOutputPort());
            writer->SetFileTypeToASCII();
            writer->Write();
        }
    }
    else
    {
        if((masterOnly and PetscTools::AmMaster()) or !masterOnly)
        {
            vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
            writer->SetFileName(mFilename.c_str());

            if(mpInputGeometries.size()==1)
            {
                #if VTK_MAJOR_VERSION <= 5
                    writer->SetInput(mpInputGeometries[0]);
                #else
                    writer->SetInputData(mpInputGeometries[0]);
                #endif
            }
            else
            {
                vtkSmartPointer<vtkAppendPolyData> p_append = vtkSmartPointer<vtkAppendPolyData>::New();
                for(unsigned idx=0;idx<mpInputGeometries.size();idx++)
                {
                    #if VTK_MAJOR_VERSION <= 5
                        p_append->AddInput(mpInputGeometries[idx]);
                    #else
                        p_append->AddInputData(mpInputGeometries[idx]);
                    #endif
                }
                writer->SetInputConnection(p_append->GetOutputPort());
            }
            writer->Write();
        }
    }
}
