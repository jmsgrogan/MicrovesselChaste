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
#include "CsvVesselNetworkReader.hpp"
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

template<unsigned DIM>
CsvVesselNetworkReader<DIM>::CsvVesselNetworkReader()
    : mFileName()
{
}

template<unsigned DIM>
CsvVesselNetworkReader<DIM>::~CsvVesselNetworkReader()
{
}

template <unsigned DIM>
boost::shared_ptr<CsvVesselNetworkReader<DIM> > CsvVesselNetworkReader<DIM>::Create()
{
    MAKE_PTR(CsvVesselNetworkReader<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<VesselNetwork<DIM> > CsvVesselNetworkReader<DIM>::Read()
{
    if(mFileName.empty())
    {
        EXCEPTION("File name not set in vessel network reader");
    }

    std::ifstream in(mFileName.c_str());
    if(!in.is_open())
    {
        EXCEPTION("Problem during input file opening in vessel network reader.");
    }

    // Create an empty vessel network
    boost::shared_ptr<VesselNetwork<DIM> > p_network = VesselNetwork<DIM>::Create();

    typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
    std::vector<std::string> vec;
    std::string line;

    unsigned counter = 0;
    while (std::getline(in, line))
    {
        // Skip the header
        if(counter == 0)
        {
            counter ++;
            continue;
        }
        Tokenizer tok(line);
        vec.assign(tok.begin(),tok.end());

        if(vec.size()<10)
        {
            EXCEPTION("CSV File not in expected format");
        }

        double pos_x_1 = boost::lexical_cast<double>(vec[3]);
        double pos_y_1 = boost::lexical_cast<double>(vec[4]);
        double pos_z_1 = boost::lexical_cast<double>(vec[5]);
        double pos_x_2 = boost::lexical_cast<double>(vec[6]);
        double pos_y_2 = boost::lexical_cast<double>(vec[7]);
        double pos_z_2 = boost::lexical_cast<double>(vec[8]);

        p_network->AddVessel(Vessel<DIM>::Create(VesselNode<DIM>::Create(pos_x_1, pos_y_1 ,pos_z_1),
                                                 VesselNode<DIM>::Create(pos_x_2, pos_y_2 ,pos_z_2)));

        counter ++;
    }

    // Need to flip in 'y' direction for consistency with VTK tools
    double y_max = p_network->GetExtents()[1].second;
    for(unsigned idx=0; idx<p_network->GetNumberOfNodes();idx++)
    {
        c_vector<double, DIM> current_location = p_network->GetNode(idx)->rGetLocation();
        c_vector<double, DIM> new_location;
        new_location[0] = current_location[0];
        new_location[1] = y_max - current_location[1];
        if(DIM==3)
        {
            new_location[2] = current_location[2];
        }
        p_network->GetNode(idx)->SetLocation(new_location);
    }

    return p_network;
}

template<unsigned DIM>
void CsvVesselNetworkReader<DIM>::SetFileName(const std::string& rFileName)
{
    mFileName = rFileName;
}

//Explicit instantiation
template class CsvVesselNetworkReader<2> ;
template class CsvVesselNetworkReader<3> ;
