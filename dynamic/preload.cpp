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
