#include "BaseUnits.hpp"
#include "UnitCollection.hpp"
#include "ParameterCollection.hpp"
#include "BaseParameterInstance.hpp"
#include "ChastePoint.hpp"

//// Typdef in this namespace so that pyplusplus uses the nicer typedef'd name for the class
//namespace pyplusplus{
//namespace aliases{
//
//typedef ParameterInstance<unit::mass> ParameterInstanceMass;
////typedef ParameterInstance<unit::time> ParameterInstanceTime;
////typedef ParameterInstance<unit::dynamic_viscosity> ParameterInstanceDynamicViscosity;
////typedef ParameterInstance<unit::pressure> ParameterInstancePressure;
////typedef ParameterInstance<unit::length> ParameterInstanceLength;
//}
//}//pyplusplus::aliases
//

//namespace chaste{
//template class ParameterInstance<unit::mass>;
//}

//template class ChastePoint<3>;

//inline int Instantiation()
//{
//    return  sizeof(pyplusplus::aliases::ParameterInstanceMass);
////            sizeof(ParameterInstance<unit::mass>) +
////            sizeof(ParameterInstance<unit::time>) +
////            sizeof(ParameterInstance<unit::dynamic_viscosity>) +
////            sizeof(ParameterInstance<unit::pressure>) +
////            sizeof(ParameterInstance<unit::length>);
//}
////template class PressureUnit;
////template class PressureQuantity;
//
////typedef unit::kg_instance_t< true > kg;
//
