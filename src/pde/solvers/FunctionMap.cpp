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

#include "FunctionMap.hpp"

template<unsigned DIM>
FunctionMap<DIM>::FunctionMap()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>()
{

}

template<unsigned DIM>
boost::shared_ptr<FunctionMap<DIM> > FunctionMap<DIM>::Create()
{
    MAKE_PTR(FunctionMap, pSelf);
    return pSelf;
}

template<unsigned DIM>
FunctionMap<DIM>::~FunctionMap()
{

}

template<unsigned DIM>
void FunctionMap<DIM>::Solve()
{

}

template<unsigned DIM>
void FunctionMap<DIM>::UpdateFunctionSolution(std::vector<double>& data)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    vtkSmartPointer<vtkDoubleArray> pPointData = vtkSmartPointer<vtkDoubleArray>::New();
    pPointData->SetNumberOfComponents(1);
    pPointData->SetNumberOfTuples(data.size());
    pPointData->SetName(this->GetLabel().c_str());
    std::cout << "fn in" << std::endl;
    std::cout << this->GetLabel() << std::endl;
    for (unsigned i = 0; i < data.size(); i++)
    {
        pPointData->SetValue(i, data[i]);
    }
    this->mpVtkSolution->GetPointData()->AddArray(pPointData);

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mSolution = std::vector<double>(data.size(), 0.0);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mSolution[i] = data[i];
    }

    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(data.size(), 0.0*this->mReferenceConcentration);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mConcentrations[i] = data[i]*this->mReferenceConcentration;
    }
}

// Explicit instantiation
template class FunctionMap<2> ;
template class FunctionMap<3> ;
