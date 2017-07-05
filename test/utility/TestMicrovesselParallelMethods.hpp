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

#ifndef TESTMICROVESSELPARALLELMETHODS_HPP
#define TESTMICROVESSELPARALLELMETHODS_HPP

#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "PetscTools.hpp"
#include "ObjectCommunicator.hpp"
#include "BaseParameterInstance.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestMicrovesselParallelMethods : public CxxTest::TestSuite
{

public:

    void TestSendReceiveParameterInstance()
    {
        // Send and receive a parameter instance
        std::shared_ptr<BaseParameterInstance> p_my_parameter = std::shared_ptr<BaseParameterInstance>(new BaseParameterInstance);
        p_my_parameter->SetShortDescription("Base Parameter");
        p_my_parameter->SetName("Base_" + boost::lexical_cast<std::string>(PetscTools::GetMyRank()));
        p_my_parameter->SetBibliographicInformation("J. Smith et al., (2003).");

        if(PetscTools::GetNumProcs()>1)
        {
            MPI_Status status;
            ObjectCommunicator<BaseParameterInstance> communicator;
            unsigned com_tag = 456;
            std::shared_ptr<BaseParameterInstance> p_neighour_parameter;

            if (!PetscTools::AmTopMost())
            {
                p_neighour_parameter = communicator.SendRecvObject(p_my_parameter, PetscTools::GetMyRank() + 1, com_tag, PetscTools::GetMyRank() + 1, com_tag, status);
            }
            if (!PetscTools::AmMaster())
            {
                p_neighour_parameter = communicator.SendRecvObject(p_my_parameter, PetscTools::GetMyRank() - 1, com_tag, PetscTools::GetMyRank() - 1, com_tag, status);
            }

            std::cout << "Proc: " << PetscTools::GetMyRank() << " received param: " << p_neighour_parameter->GetName() << std::endl;
        }
    }
};

#endif // TESTMICROVESSELPARALLELMETHODS_HPP
