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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include "vtkXMLPolyDataWriter.h"
#include "vtkPolyData.h"
#include "SimpleCellPopulation.hpp"

/**
 * Helper class to see if a cell exists in a vector
 */
template<unsigned DIM>
class ExistsInVector
{

    /**
     * the vector
     */
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > m_vec;

public:

    /**
     * Does it exist in vector
     * @param vec the test vector
     */
    ExistsInVector(std::vector<boost::shared_ptr<SimpleCell<DIM> > > vec)
        : m_vec(vec)
    {

    }
    /**
     * Overloaded operator
     * @param i each entry
     * @return is it in the vector
     */
    bool operator() (boost::shared_ptr<SimpleCell<DIM> > i)
    {
        return (std::find(m_vec.begin(), m_vec.end(), i) != m_vec.end());
    }
};

template<unsigned DIM>
SimpleCellPopulation<DIM>::SimpleCellPopulation() :
        mCells(),
        mReferenceLength(1.e-6 * unit::metres)
{

}

template<unsigned DIM>
boost::shared_ptr<SimpleCellPopulation<DIM> > SimpleCellPopulation<DIM>::Create()
{
    MAKE_PTR(SimpleCellPopulation<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
SimpleCellPopulation<DIM>::~SimpleCellPopulation()
{

}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::BooleanWithVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > remove_cells;
    double tolerance = 1.e-6;
    for(unsigned idx=0; idx<mCells.size();idx++)
    {
        std::pair<boost::shared_ptr<VesselSegment<DIM> >, units::quantity<unit::length> > seg_pair = pNetwork->GetNearestSegment(mCells[idx]->rGetLocation());
        if(seg_pair.second / seg_pair.first->GetNode(0)->GetReferenceLengthScale() < tolerance)
        {
            remove_cells.push_back(mCells[idx]);
        }
    }

    mCells.erase( std::remove_if(mCells.begin(), mCells.end(), ExistsInVector<DIM>(remove_cells)), mCells.end());
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::GenerateCellsAtPoints(std::vector<c_vector<double, DIM> > points)
{
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells;
    for(unsigned idx = 0; idx < points.size();idx++)
    {
        cells.push_back(SimpleCell<DIM>::Create(points[idx]));
    }
    AddCells(cells);
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::GenerateCellsOnGrid(unsigned xDim, unsigned yDim, unsigned zDim,
                                                    units::quantity<unit::length> spacing, c_vector<double, DIM> origin)
{
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells;
    for (unsigned idx = 0; idx < zDim;idx++)
    {
        for (unsigned jdx = 0; jdx < yDim ;jdx++)
        {
            for (unsigned kdx = 0; kdx < xDim;kdx++)
            {
                double x_coord = (double(kdx) * spacing + origin[0] * mReferenceLength)/mReferenceLength;
                double y_coord = (double(jdx) * spacing + origin[1] * mReferenceLength)/mReferenceLength;
                double z_coord = (double(idx) * spacing + origin[2] * mReferenceLength)/mReferenceLength;
                cells.push_back(SimpleCell<DIM>::Create(x_coord, y_coord, z_coord));
            }
        }
    }
    AddCells(cells);
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::GenerateCellsOnGrid(boost::shared_ptr<Part<DIM> > pPart, units::quantity<unit::length> spacing)
{
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells;

    // Get the bounding box of the part
    c_vector<double,2*DIM> bbox = pPart->GetBoundingBox();
    unsigned num_x = double(bbox[1] - bbox[0]) * mReferenceLength  / spacing + 1;
    unsigned num_y = double(bbox[3] - bbox[2]) * mReferenceLength  / spacing + 1;
    unsigned num_z = 1;
    if(DIM==3)
    {
        num_z = double(bbox[5] - bbox[4]) * mReferenceLength  / spacing + 1;
    }

    for (unsigned idx = 0; idx < num_z;idx++)
    {
        for (unsigned jdx = 0; jdx < num_y ;jdx++)
        {
            for (unsigned kdx = 0; kdx < num_x;kdx++)
            {
                c_vector<double, DIM> location;
                location[0] = (bbox[0] * mReferenceLength + double(kdx) * spacing)/mReferenceLength;
                location[1] = (bbox[2] * mReferenceLength + double(jdx) * spacing)/mReferenceLength;
                if(DIM==3)
                {
                    location[2] = (bbox[4] * mReferenceLength + double(idx) * spacing)/mReferenceLength;
                }

                if(pPart->IsPointInPart(location))
                {
                    cells.push_back(SimpleCell<DIM>::Create(location));
                }
            }
        }
    }
    AddCells(cells);
}

template<unsigned DIM>
std::vector<boost::shared_ptr<SimpleCell<DIM> > > SimpleCellPopulation<DIM>::GetCells()
{
    return mCells;
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::AddCell(boost::shared_ptr<SimpleCell<DIM> > pCell)
{
    mCells.push_back(pCell);
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::AddCells(std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells)
{
    mCells.insert(mCells.end(), cells.begin(), cells.end());
}

template<unsigned DIM>
vtkSmartPointer<vtkPoints> SimpleCellPopulation<DIM>::GetVtk()
{
    vtkSmartPointer<vtkPoints> p_vertices = vtkSmartPointer<vtkPoints>::New();

    p_vertices->SetNumberOfPoints(mCells.size());
    for (vtkIdType idx = 0; idx < vtkIdType(mCells.size()); idx++)
    {
        c_vector<double, DIM> location = mCells[idx]->rGetLocation();
        if(DIM==3)
        {
            p_vertices->SetPoint(idx, location[0], location[1], location[2]);
        }
        else
        {
            p_vertices->SetPoint(idx, location[0], location[1], 0.0);
        }
    }
    return p_vertices;
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::SetReferenceLengthScale(units::quantity<unit::length> lenthScale)
{
    mReferenceLength = lenthScale;
}

template<unsigned DIM>
void SimpleCellPopulation<DIM>::Write(const std::string& rFileName)
{
    vtkSmartPointer<vtkPolyData> p_part_data = vtkSmartPointer<vtkPolyData>::New();
    p_part_data->SetPoints(GetVtk());
    vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName(rFileName.c_str());
    #if VTK_MAJOR_VERSION <= 5
        writer->SetInput(p_part_data);
    #else
        writer->SetInputData(p_part_data);
    #endif
    writer->Write();
}

//template class SimpleCellPopulation<1> ;
template class SimpleCellPopulation<2> ;
template class SimpleCellPopulation<3> ;
