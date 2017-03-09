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

#include <vector>
#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include "VesselNode.hpp"
#include "DimensionalChastePoint.hpp"

/**
 * Convert Python Iterables to C++
 *
 */
struct PythonIterableToStl
{

    template <typename Container>
    PythonIterableToStl&
    from_python()
    {
        boost::python::converter::registry::push_back(
                &PythonIterableToStl::convertible,
                &PythonIterableToStl::construct<Container>,
                boost::python::type_id<Container>());

        // Support chaining.
        return *this;
    }

    // Check if PyObject is iterable.
    static void* convertible(PyObject* object)
    {
        return PyObject_GetIter(object) ? object : NULL;
    }

    template <typename Container>
    static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        namespace python = boost::python;
        python::handle<> handle(python::borrowed(object));

        typedef python::converter::rvalue_from_python_storage<Container> storage_type;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;
        typedef python::stl_input_iterator<typename Container::value_type>iterator;

        new (storage) Container(
                iterator(python::object(handle)), // begin
                iterator());// end
        data->convertible = storage;
    }
};

// Make the module
BOOST_PYTHON_MODULE(_chaste_project_MicrovesselChaste_preload)
{
    // Iterators
    PythonIterableToStl()
      .from_python<std::vector<boost::shared_ptr<VesselNode<2> > > >()
      .from_python<std::vector<boost::shared_ptr<VesselNode<3> > > >()
      .from_python<std::vector<boost::shared_ptr<DimensionalChastePoint<2> > > >()
      .from_python<std::vector<boost::shared_ptr<DimensionalChastePoint<3> > > >()
      ;
}
