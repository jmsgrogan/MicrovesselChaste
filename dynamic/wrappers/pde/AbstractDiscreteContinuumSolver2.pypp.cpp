// This file has been generated by Py++.


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

#include "boost/python.hpp"
#include "wrapper_header_collection.hpp"
#include "AbstractDiscreteContinuumSolver2.pypp.hpp"

namespace bp = boost::python;

struct AbstractDiscreteContinuumSolver_less__2__greater__wrapper : AbstractDiscreteContinuumSolver< 2 >, bp::wrapper< AbstractDiscreteContinuumSolver< 2 > > {

    AbstractDiscreteContinuumSolver_less__2__greater__wrapper( )
    : AbstractDiscreteContinuumSolver<2>( )
      , bp::wrapper< AbstractDiscreteContinuumSolver< 2 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations(  ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetConcentrations(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations(  ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetConcentrations( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > pGrid ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations( pGrid );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetConcentrations( pGrid );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > pGrid ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetConcentrations( pGrid );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations( pSamplePoints );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetConcentrations( pSamplePoints );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetConcentrations( pSamplePoints );
    }

    virtual ::std::vector< double > GetSolution(  ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetSolution(  );
        }
    }
    
    ::std::vector< double > default_GetSolution(  ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetSolution( );
    }

    virtual ::std::vector< double > GetSolution( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution( pSamplePoints );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetSolution( pSamplePoints );
        }
    }
    
    ::std::vector< double > default_GetSolution( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetSolution( pSamplePoints );
    }

    virtual ::std::vector< double > GetSolution( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > pGrid ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution( pGrid );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetSolution( pGrid );
        }
    }
    
    ::std::vector< double > default_GetSolution( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > pGrid ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetSolution( pGrid );
    }

    virtual ::std::vector< double > GetSolutionP( ::vtkPoints * pSamplePoints ) {
        if( bp::override func_GetSolutionP = this->get_override( "GetSolutionP" ) )
            return func_GetSolutionP( boost::python::ptr(pSamplePoints) );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetSolutionP( boost::python::ptr(pSamplePoints) );
        }
    }
    
    ::std::vector< double > default_GetSolutionP( ::vtkPoints * pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetSolutionP( boost::python::ptr(pSamplePoints) );
    }

    virtual ::vtkSmartPointer< vtkDataSet > GetVtkSolution(  ) {
        if( bp::override func_GetVtkSolution = this->get_override( "GetVtkSolution" ) )
            return func_GetVtkSolution(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 2 >::GetVtkSolution(  );
        }
    }
    
    ::vtkSmartPointer< vtkDataSet > default_GetVtkSolution(  ) {
        return AbstractDiscreteContinuumSolver< 2 >::GetVtkSolution( );
    }

    virtual void Setup(  ){
        bp::override func_Setup = this->get_override( "Setup" );
        func_Setup(  );
    }

    virtual void Solve(  ){
        bp::override func_Solve = this->get_override( "Solve" );
        func_Solve(  );
    }

    virtual void Update(  ){
        bp::override func_Update = this->get_override( "Update" );
        func_Update(  );
    }

    virtual void UpdateCellData(  ){
        bp::override func_UpdateCellData = this->get_override( "UpdateCellData" );
        func_UpdateCellData(  );
    }

    virtual void UpdateSolution( ::std::vector< double > const & rData ) {
        if( bp::override func_UpdateSolution = this->get_override( "UpdateSolution" ) )
            func_UpdateSolution( boost::ref(rData) );
        else{
            this->AbstractDiscreteContinuumSolver< 2 >::UpdateSolution( boost::ref(rData) );
        }
    }
    
    void default_UpdateSolution( ::std::vector< double > const & rData ) {
        AbstractDiscreteContinuumSolver< 2 >::UpdateSolution( boost::ref(rData) );
    }

    virtual void UpdateSolution( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & rData ) {
        if( bp::override func_UpdateSolution = this->get_override( "UpdateSolution" ) )
            func_UpdateSolution( boost::ref(rData) );
        else{
            this->AbstractDiscreteContinuumSolver< 2 >::UpdateSolution( boost::ref(rData) );
        }
    }
    
    void default_UpdateSolution( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & rData ) {
        AbstractDiscreteContinuumSolver< 2 >::UpdateSolution( boost::ref(rData) );
    }

    virtual void Write(  ){
        bp::override func_Write = this->get_override( "Write" );
        func_Write(  );
    }

};

void register_AbstractDiscreteContinuumSolver2_class(){

    bp::class_< AbstractDiscreteContinuumSolver_less__2__greater__wrapper, boost::noncopyable >( "AbstractDiscreteContinuumSolver2", bp::init< >() )    
        .def( 
            "AddBoundaryCondition"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< DiscreteContinuumBoundaryCondition< 2 > > ))( &::AbstractDiscreteContinuumSolver< 2 >::AddBoundaryCondition )
            , ( bp::arg("pBoundaryCondition") ) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )(  ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetConcentrations) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > ))(&::AbstractDiscreteContinuumSolver< 2 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetConcentrations)
            , ( bp::arg("pGrid") ) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<2>::* )( ::vtkSmartPointer< vtkPoints > ))(&::AbstractDiscreteContinuumSolver< 2 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::vtkSmartPointer< vtkPoints > ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetConcentrations)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetDensityMap"
            , (::boost::shared_ptr< DensityMap< 2 > > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))( &::AbstractDiscreteContinuumSolver< 2 >::GetDensityMap ) )    
        .def( 
            "GetLabel"
            , (::std::string const & ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))( &::AbstractDiscreteContinuumSolver< 2 >::GetLabel )
            , bp::return_internal_reference< >() )    
        .def( 
            "GetPde"
            , (::boost::shared_ptr< AbstractDiscreteContinuumPde< 2, 2 > > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))( &::AbstractDiscreteContinuumSolver< 2 >::GetPde ) )    
        .def( 
            "GetReferenceConcentration"
            , (::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))( &::AbstractDiscreteContinuumSolver< 2 >::GetReferenceConcentration ) )    
        .def( 
            "GetReferenceLength"
            , (::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))( &::AbstractDiscreteContinuumSolver< 2 >::GetReferenceLength ) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::GetSolution)
            , (::std::vector< double > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )(  ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetSolution) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<2>::* )( ::vtkSmartPointer< vtkPoints > ))(&::AbstractDiscreteContinuumSolver< 2 >::GetSolution)
            , (::std::vector< double > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::vtkSmartPointer< vtkPoints > ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetSolution)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > ))(&::AbstractDiscreteContinuumSolver< 2 >::GetSolution)
            , (::std::vector< double > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetSolution)
            , ( bp::arg("pGrid") ) )    
        .def( 
            "GetSolutionP"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<2>::* )( ::vtkPoints * ))(&::AbstractDiscreteContinuumSolver< 2 >::GetSolutionP)
            , (::std::vector< double > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::vtkPoints * ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetSolutionP)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetVtkSolution"
            , (::vtkSmartPointer< vtkDataSet > ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::GetVtkSolution)
            , (::vtkSmartPointer< vtkDataSet > ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )(  ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_GetVtkSolution) )    
        .def( 
            "SetCellPopulation"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::AbstractCellPopulation< 2, 2 > &,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double >,::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetCellPopulation )
            , ( bp::arg("rCellPopulation"), bp::arg("cellPopulationReferenceLength"), bp::arg("cellPopulationReferenceConcentration") ) )    
        .def( 
            "SetDensityMap"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< DensityMap< 2 > > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetDensityMap )
            , ( bp::arg("pDensityMap") ) )    
        .def( 
            "SetFileHandler"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< OutputFileHandler > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetFileHandler )
            , ( bp::arg("pOutputFileHandler") ) )    
        .def( 
            "SetFileName"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::std::string const & ))( &::AbstractDiscreteContinuumSolver< 2 >::SetFileName )
            , ( bp::arg("rFilename") ) )    
        .def( 
            "SetGrid"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 2, 2 > > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetGrid )
            , ( bp::arg("pGrid") ) )    
        .def( 
            "SetLabel"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::std::string const & ))( &::AbstractDiscreteContinuumSolver< 2 >::SetLabel )
            , ( bp::arg("rLabel") ) )    
        .def( 
            "SetPde"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumPde< 2, 2 > > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetPde )
            , ( bp::arg("pPde") ) )    
        .def( 
            "SetReferenceConcentration"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::units::quantity< boost::units::unit< boost::units::list< boost::units::dim< boost::units::length_base_dimension, boost::units::static_rational< -3, 1 > >, boost::units::list< boost::units::dim< boost::units::amount_base_dimension, boost::units::static_rational< 1, 1 > >, boost::units::dimensionless_type > >, boost::units::homogeneous_system< boost::units::list< boost::units::si::meter_base_unit, boost::units::list< boost::units::scaled_base_unit< boost::units::cgs::gram_base_unit, boost::units::scale< 10, boost::units::static_rational< 3 > > >, boost::units::list< boost::units::si::second_base_unit, boost::units::list< boost::units::si::ampere_base_unit, boost::units::list< boost::units::si::kelvin_base_unit, boost::units::list< boost::units::si::mole_base_unit, boost::units::list< boost::units::si::candela_base_unit, boost::units::list< boost::units::angle::radian_base_unit, boost::units::list< boost::units::angle::steradian_base_unit, boost::units::dimensionless_type > > > > > > > > > >, void >, double > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetReferenceConcentration )
            , ( bp::arg("referenceConcentration") ) )    
        .def( 
            "SetVesselNetwork"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::boost::shared_ptr< VesselNetwork< 2 > > ))( &::AbstractDiscreteContinuumSolver< 2 >::SetVesselNetwork )
            , ( bp::arg("pNetwork") ) )    
        .def( 
            "SetWriteSolution"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( bool ))( &::AbstractDiscreteContinuumSolver< 2 >::SetWriteSolution )
            , ( bp::arg("write")=(bool)(true) ) )    
        .def( 
            "Setup"
            , bp::pure_virtual( (void ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::Setup) ) )    
        .def( 
            "Solve"
            , bp::pure_virtual( (void ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::Solve) ) )    
        .def( 
            "Update"
            , bp::pure_virtual( (void ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::Update) ) )    
        .def( 
            "UpdateCellData"
            , bp::pure_virtual( (void ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::UpdateCellData) ) )    
        .def( 
            "UpdateSolution"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::std::vector< double > const & ))(&::AbstractDiscreteContinuumSolver< 2 >::UpdateSolution)
            , (void ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::std::vector< double > const & ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_UpdateSolution)
            , ( bp::arg("rData") ) )    
        .def( 
            "UpdateSolution"
            , (void ( ::AbstractDiscreteContinuumSolver<2>::* )( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & ))(&::AbstractDiscreteContinuumSolver< 2 >::UpdateSolution)
            , (void ( AbstractDiscreteContinuumSolver_less__2__greater__wrapper::* )( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & ))(&AbstractDiscreteContinuumSolver_less__2__greater__wrapper::default_UpdateSolution)
            , ( bp::arg("rData") ) )    
        .def( 
            "Write"
            , bp::pure_virtual( (void ( ::AbstractDiscreteContinuumSolver<2>::* )(  ))(&::AbstractDiscreteContinuumSolver< 2 >::Write) ) );

}