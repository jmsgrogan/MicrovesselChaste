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

#ifdef CHASTE_VTK
#define _BACKWARD_BACKWARD_WARNING_H 1
#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>
#endif // CHASTE_VTK
#include "SmartPointers.hpp"
#include "Exception.hpp"
#include "PetscTools.hpp"
#include "VesselNetworkWriter.hpp"

template <unsigned DIM>
VesselNetworkWriter<DIM>::VesselNetworkWriter() :
    mpVesselNetwork(),
    mpVtkVesselNetwork(vtkSmartPointer<vtkPolyData>::New()),
    mIsVtkNetworkUpToDate(false),
    mFilename(),
    mReferenceLength(1_um)
{

}

template <unsigned DIM>
VesselNetworkWriter<DIM>::~VesselNetworkWriter()
{

}

template <unsigned DIM>
std::shared_ptr<VesselNetworkWriter<DIM> > VesselNetworkWriter<DIM>::Create()
{
    return std::make_shared<VesselNetworkWriter<DIM> >();

}

template <unsigned DIM>
void VesselNetworkWriter<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpVesselNetwork = pNetwork;
    mIsVtkNetworkUpToDate = false;
}

template <unsigned DIM>
void VesselNetworkWriter<DIM>::SetReferenceLengthScale(QLength rReferenceLength)
{
    mReferenceLength = rReferenceLength;
}

template <unsigned DIM>
vtkSmartPointer<vtkPolyData> VesselNetworkWriter<DIM>::GetOutput()
{
    if(!mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for the vtk writer.");
    }

    if(mReferenceLength == 0.0 * unit::metres)
    {
        EXCEPTION("A non zero reference length scale is required for the vtk writer.");
    }

    mpVtkVesselNetwork = vtkSmartPointer<vtkPolyData>::New();

    if(mpVesselNetwork->GetNumberOfVessels()>0)
    {
        // Set up the vessel data arrays.
    	std::vector<std::shared_ptr<Vessel<DIM> > > vessels = mpVesselNetwork->GetVessels();
        std::vector<vtkSmartPointer<vtkDoubleArray> > pVesselInfoVector;
        std::map<std::string, double>::iterator vessel_map_iterator;
        std::map<std::string, double> vessel_data_map = vessels[0]->GetOutputData();
        for(vessel_map_iterator = vessel_data_map.begin(); vessel_map_iterator != vessel_data_map.end(); vessel_map_iterator++)
        {
            vtkSmartPointer<vtkDoubleArray> pVesselInfo = vtkSmartPointer<vtkDoubleArray>::New();
            pVesselInfo->SetNumberOfComponents(1);
            pVesselInfo->SetNumberOfTuples(vessels.size());
            pVesselInfo->SetName((*vessel_map_iterator).first.c_str());
            pVesselInfoVector.push_back(pVesselInfo);
        }

        // Set up node data arrays
        std::vector<std::shared_ptr<VesselNode<DIM> > > nodes = mpVesselNetwork->GetNodes();
        std::vector<vtkSmartPointer<vtkDoubleArray> > pNodeInfoVector;
        std::map<std::string, double>::iterator vtk_node_map_iterator;
        std::map<std::string, double> vtk_node_map = vessels[0]->GetStartNode()->GetOutputData();
        for(vtk_node_map_iterator = vtk_node_map.begin(); vtk_node_map_iterator != vtk_node_map.end(); vtk_node_map_iterator++)
        {
            vtkSmartPointer<vtkDoubleArray> pNodeInfo = vtkSmartPointer<vtkDoubleArray>::New();
            pNodeInfo->SetNumberOfComponents(1);
            pNodeInfo->SetNumberOfTuples(nodes.size());
            pNodeInfo->SetName((*vtk_node_map_iterator).first.c_str());
            pNodeInfoVector.push_back(pNodeInfo);
        }

        // Create the geometric data
        vtkSmartPointer<vtkPoints> pPoints= vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> pLines = vtkSmartPointer<vtkCellArray>::New();
        unsigned vessel_index=0;
        for(unsigned idx=0; idx<nodes.size(); idx++)
        {
            nodes[idx]->SetId(idx);
            if(DIM == 2)
            {
                pPoints->InsertNextPoint(nodes[idx]->rGetLocation().GetLocation(mReferenceLength)[0],
                                         nodes[idx]->rGetLocation().GetLocation(mReferenceLength)[1], 0.0);
            }
            else
            {
                pPoints->InsertNextPoint(nodes[idx]->rGetLocation().GetLocation(mReferenceLength)[0],
                                         nodes[idx]->rGetLocation().GetLocation(mReferenceLength)[1],
                                         nodes[idx]->rGetLocation().GetLocation(mReferenceLength)[2]);
            }

            std::map<std::string, double> vtk_node_data = nodes[idx]->GetOutputData();
            for(unsigned jdx=0; jdx < pNodeInfoVector.size(); jdx++)
            {
                std::string key = pNodeInfoVector[jdx]->GetName();
                // check if the key exists in the data map, if yes populate the vtk node data array
                if(vtk_node_data.count(key) == 1)
                {
                    pNodeInfoVector[jdx]->SetValue(idx, vtk_node_data[key]);
                }
            }
        }

        // Create the vessels
        typename std::vector<std::shared_ptr<Vessel<DIM> > >::iterator it;
        for(it = vessels.begin(); it < vessels.end(); it++)
        {
            vtkSmartPointer<vtkLine> pLine = vtkSmartPointer<vtkLine>::New();
            std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = (*it)->GetSegments();
            for(unsigned i = 0; i < segments.size(); i++)
            {
                pLine->GetPointIds()->InsertId(i, segments[i]->GetNode(0)->GetId());

                // Do an extra insert for the last node in the segment
                if (i == segments.size() - 1)
                {
                    pLine->GetPointIds()->InsertId(i + 1, segments[i]->GetNode(1)->GetId());
                }
            }
            pLines->InsertNextCell(pLine);

            // Add the vessel data
            std::map<std::string, double> vtk_vessel_data = (*it)->GetOutputData();
            for(unsigned idx=0; idx < pVesselInfoVector.size(); idx++)
            {
                std::string key = pVesselInfoVector[idx]->GetName();
                // If the key is in the vtk data use it
                if(vtk_vessel_data.count(key) == 1)
                {
                    pVesselInfoVector[idx]->SetValue(vessel_index, vtk_vessel_data[key]);
                }
            }
            vessel_index++;
        }
        mpVtkVesselNetwork->SetPoints(pPoints);
        mpVtkVesselNetwork->SetLines(pLines);

        for (unsigned i = 0; i < pVesselInfoVector.size(); i++)
        {
        	mpVtkVesselNetwork->GetCellData()->AddArray(pVesselInfoVector[i]);
        }

        for (unsigned i = 0; i < pNodeInfoVector.size(); i++)
        {
        	mpVtkVesselNetwork->GetPointData()->AddArray(pNodeInfoVector[i]);
        }
    }
    mIsVtkNetworkUpToDate = true;
    return mpVtkVesselNetwork;
}

template <unsigned DIM>
void VesselNetworkWriter<DIM>::SetFileName(const std::string& rFileName)
{
    mFilename = rFileName;
}

template <unsigned DIM>
void VesselNetworkWriter<DIM>::Write(bool masterOnly)
{
    if(mFilename.empty())
    {
        EXCEPTION("No file name set for VesselNetworkWriter");
    }

    if(PetscTools::AmMaster() or !masterOnly)
    {
        vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
        writer->SetFileName(mFilename.c_str());

        #if VTK_MAJOR_VERSION <= 5
            writer->SetInput(this->GetOutput());
        #else
            writer->SetInputData(this->GetOutput());
        #endif
        writer->Write();
    }
}

// Explicit instantiation
template class VesselNetworkWriter<2>;
template class VesselNetworkWriter<3>;
