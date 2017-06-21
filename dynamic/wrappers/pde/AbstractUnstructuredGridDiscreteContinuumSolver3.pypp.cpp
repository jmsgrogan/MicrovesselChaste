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
#include "AbstractUnstructuredGridDiscreteContinuumSolver3.pypp.hpp"

namespace bp = boost::python;

struct AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper : AbstractUnstructuredGridDiscreteContinuumSolver< 3 >, bp::wrapper< AbstractUnstructuredGridDiscreteContinuumSolver< 3 > > {

    AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper( )
    : AbstractUnstructuredGridDiscreteContinuumSolver<3>( )
      , bp::wrapper< AbstractUnstructuredGridDiscreteContinuumSolver< 3 > >(){
        // null constructor
    
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrationsAtCentroids(  ) {
        if( bp::override func_GetConcentrationsAtCentroids = this->get_override( "GetConcentrationsAtCentroids" ) )
            return func_GetConcentrationsAtCentroids(  );
        else{
            return this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::GetConcentrationsAtCentroids(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrationsAtCentroids(  ) {
        return AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::GetConcentrationsAtCentroids( );
    }

    virtual void Setup(  ) {
        if( bp::override func_Setup = this->get_override( "Setup" ) )
            func_Setup(  );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Setup(  );
        }
    }
    
    void default_Setup(  ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Setup( );
    }

    virtual void Solve(  ){
        bp::override func_Solve = this->get_override( "Solve" );
        func_Solve(  );
    }

    virtual void Update(  ) {
        if( bp::override func_Update = this->get_override( "Update" ) )
            func_Update(  );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Update(  );
        }
    }
    
    void default_Update(  ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Update( );
    }

    virtual void UpdateCellData(  ) {
        if( bp::override func_UpdateCellData = this->get_override( "UpdateCellData" ) )
            func_UpdateCellData(  );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateCellData(  );
        }
    }
    
    void default_UpdateCellData(  ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateCellData( );
    }

    virtual void UpdateElementSolution( ::std::vector< double > const & rData ) {
        if( bp::override func_UpdateElementSolution = this->get_override( "UpdateElementSolution" ) )
            func_UpdateElementSolution( boost::ref(rData) );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateElementSolution( boost::ref(rData) );
        }
    }
    
    void default_UpdateElementSolution( ::std::vector< double > const & rData ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateElementSolution( boost::ref(rData) );
    }

    virtual void UpdateSolution( ::std::vector< double > const & rData ) {
        if( bp::override func_UpdateSolution = this->get_override( "UpdateSolution" ) )
            func_UpdateSolution( boost::ref(rData) );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution( boost::ref(rData) );
        }
    }
    
    void default_UpdateSolution( ::std::vector< double > const & rData ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution( boost::ref(rData) );
    }

    virtual void UpdateSolution( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & rData ) {
        if( bp::override func_UpdateSolution = this->get_override( "UpdateSolution" ) )
            func_UpdateSolution( boost::ref(rData) );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution( boost::ref(rData) );
        }
    }
    
    void default_UpdateSolution( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & rData ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution( boost::ref(rData) );
    }

    virtual void Write(  ) {
        if( bp::override func_Write = this->get_override( "Write" ) )
            func_Write(  );
        else{
            this->AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Write(  );
        }
    }
    
    void default_Write(  ) {
        AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Write( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations(  ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetConcentrations(  );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations(  ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetConcentrations( );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > pGrid ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations( pGrid );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetConcentrations( pGrid );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > pGrid ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetConcentrations( pGrid );
    }

    virtual ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > GetConcentrations( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        if( bp::override func_GetConcentrations = this->get_override( "GetConcentrations" ) )
            return func_GetConcentrations( pSamplePoints );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetConcentrations( pSamplePoints );
        }
    }
    
    ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > default_GetConcentrations( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetConcentrations( pSamplePoints );
    }

    virtual ::std::vector< double > GetSolution(  ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetSolution(  );
        }
    }
    
    ::std::vector< double > default_GetSolution(  ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetSolution( );
    }

    virtual ::std::vector< double > GetSolution( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution( pSamplePoints );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetSolution( pSamplePoints );
        }
    }
    
    ::std::vector< double > default_GetSolution( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetSolution( pSamplePoints );
    }

    virtual ::std::vector< double > GetSolution( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > pGrid ) {
        if( bp::override func_GetSolution = this->get_override( "GetSolution" ) )
            return func_GetSolution( pGrid );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetSolution( pGrid );
        }
    }
    
    ::std::vector< double > default_GetSolution( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > pGrid ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetSolution( pGrid );
    }

    virtual ::std::vector< boost::numeric::ublas::c_vector<double, 3> > GetSolutionGradients( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        if( bp::override func_GetSolutionGradients = this->get_override( "GetSolutionGradients" ) )
            return func_GetSolutionGradients( pSamplePoints );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetSolutionGradients( pSamplePoints );
        }
    }
    
    ::std::vector< boost::numeric::ublas::c_vector<double, 3> > default_GetSolutionGradients( ::vtkSmartPointer< vtkPoints > pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetSolutionGradients( pSamplePoints );
    }

    virtual ::std::vector< double > GetSolutionP( ::vtkPoints * pSamplePoints ) {
        if( bp::override func_GetSolutionP = this->get_override( "GetSolutionP" ) )
            return func_GetSolutionP( boost::python::ptr(pSamplePoints) );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetSolutionP( boost::python::ptr(pSamplePoints) );
        }
    }
    
    ::std::vector< double > default_GetSolutionP( ::vtkPoints * pSamplePoints ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetSolutionP( boost::python::ptr(pSamplePoints) );
    }

    virtual ::vtkSmartPointer< vtkDataSet > GetVtkSolution(  ) {
        if( bp::override func_GetVtkSolution = this->get_override( "GetVtkSolution" ) )
            return func_GetVtkSolution(  );
        else{
            return this->AbstractDiscreteContinuumSolver< 3 >::GetVtkSolution(  );
        }
    }
    
    ::vtkSmartPointer< vtkDataSet > default_GetVtkSolution(  ) {
        return AbstractDiscreteContinuumSolver< 3 >::GetVtkSolution( );
    }

};

void register_AbstractUnstructuredGridDiscreteContinuumSolver3_class(){

    bp::class_< AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper, bp::bases< AbstractDiscreteContinuumSolver< 3 > >, boost::noncopyable >( "AbstractUnstructuredGridDiscreteContinuumSolver3", bp::init< >() )    
        .def( 
            "GetConcentrationsAtCentroids"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::GetConcentrationsAtCentroids)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetConcentrationsAtCentroids) )    
        .def( 
            "Setup"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Setup)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_Setup) )    
        .def( 
            "Solve"
            , bp::pure_virtual( (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Solve) ) )    
        .def( 
            "Update"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Update)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_Update) )    
        .def( 
            "UpdateCellData"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateCellData)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_UpdateCellData) )    
        .def( 
            "UpdateElementSolution"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )( ::std::vector< double > const & ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateElementSolution)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::std::vector< double > const & ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_UpdateElementSolution)
            , ( bp::arg("rData") ) )    
        .def( 
            "UpdateSolution"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )( ::std::vector< double > const & ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::std::vector< double > const & ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_UpdateSolution)
            , ( bp::arg("rData") ) )    
        .def( 
            "UpdateSolution"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::UpdateSolution)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > const & ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_UpdateSolution)
            , ( bp::arg("rData") ) )    
        .def( 
            "Write"
            , (void ( ::AbstractUnstructuredGridDiscreteContinuumSolver<3>::* )(  ))(&::AbstractUnstructuredGridDiscreteContinuumSolver< 3 >::Write)
            , (void ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_Write) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<3>::* )(  ))(&::AbstractDiscreteContinuumSolver< 3 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetConcentrations) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ))(&::AbstractDiscreteContinuumSolver< 3 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetConcentrations)
            , ( bp::arg("pGrid") ) )    
        .def( 
            "GetConcentrations"
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::vtkSmartPointer< vtkPoints > ))(&::AbstractDiscreteContinuumSolver< 3 >::GetConcentrations)
            , (::std::vector< boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, boost::units::static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::vtkSmartPointer< vtkPoints > ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetConcentrations)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<3>::* )(  ))(&::AbstractDiscreteContinuumSolver< 3 >::GetSolution)
            , (::std::vector< double > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetSolution) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::vtkSmartPointer< vtkPoints > ))(&::AbstractDiscreteContinuumSolver< 3 >::GetSolution)
            , (::std::vector< double > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::vtkSmartPointer< vtkPoints > ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetSolution)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetSolution"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ))(&::AbstractDiscreteContinuumSolver< 3 >::GetSolution)
            , (::std::vector< double > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::boost::shared_ptr< AbstractDiscreteContinuumGrid< 3, 3 > > ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetSolution)
            , ( bp::arg("pGrid") ) )    
        .def( 
            "GetSolutionGradients"
            , (::std::vector< boost::numeric::ublas::c_vector<double, 3> > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::vtkSmartPointer< vtkPoints > ))(&::AbstractDiscreteContinuumSolver< 3 >::GetSolutionGradients)
            , (::std::vector< boost::numeric::ublas::c_vector<double, 3> > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::vtkSmartPointer< vtkPoints > ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetSolutionGradients)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetSolutionP"
            , (::std::vector< double > ( ::AbstractDiscreteContinuumSolver<3>::* )( ::vtkPoints * ))(&::AbstractDiscreteContinuumSolver< 3 >::GetSolutionP)
            , (::std::vector< double > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )( ::vtkPoints * ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetSolutionP)
            , ( bp::arg("pSamplePoints") ) )    
        .def( 
            "GetVtkSolution"
            , (::vtkSmartPointer< vtkDataSet > ( ::AbstractDiscreteContinuumSolver<3>::* )(  ))(&::AbstractDiscreteContinuumSolver< 3 >::GetVtkSolution)
            , (::vtkSmartPointer< vtkDataSet > ( AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::* )(  ))(&AbstractUnstructuredGridDiscreteContinuumSolver_less__3__greater__wrapper::default_GetVtkSolution) );

}
