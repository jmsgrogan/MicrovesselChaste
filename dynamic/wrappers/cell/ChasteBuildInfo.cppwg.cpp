#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Version.hpp"

#include "ChasteBuildInfo.cppwg.hpp"

namespace py = pybind11;
typedef ChasteBuildInfo ChasteBuildInfo;
;

void register_ChasteBuildInfo_class(py::module &m){
py::class_<ChasteBuildInfo    >(m, "ChasteBuildInfo")
        .def_static(
            "GetLicenceText", 
            (::std::string(*)()) &ChasteBuildInfo::GetLicenceText, 
            " " )
        .def_static(
            "GetRootDir", 
            (char const *(*)()) &ChasteBuildInfo::GetRootDir, 
            " " )
        .def_static(
            "GetVersionString", 
            (::std::string(*)()) &ChasteBuildInfo::GetVersionString, 
            " " )
        .def_static(
            "GetMajorReleaseNumber", 
            (unsigned int(*)()) &ChasteBuildInfo::GetMajorReleaseNumber, 
            " " )
        .def_static(
            "GetMinorReleaseNumber", 
            (unsigned int(*)()) &ChasteBuildInfo::GetMinorReleaseNumber, 
            " " )
        .def_static(
            "GetRevisionNumber", 
            (unsigned int(*)()) &ChasteBuildInfo::GetRevisionNumber, 
            " " )
        .def_static(
            "IsWorkingCopyModified", 
            (bool(*)()) &ChasteBuildInfo::IsWorkingCopyModified, 
            " " )
        .def_static(
            "GetBuildTime", 
            (char const *(*)()) &ChasteBuildInfo::GetBuildTime, 
            " " )
        .def_static(
            "GetCurrentTime", 
            (char const *(*)()) &ChasteBuildInfo::GetCurrentTime, 
            " " )
        .def_static(
            "GetBuilderUnameInfo", 
            (char const *(*)()) &ChasteBuildInfo::GetBuilderUnameInfo, 
            " " )
        .def_static(
            "GetBuildInformation", 
            (char const *(*)()) &ChasteBuildInfo::GetBuildInformation, 
            " " )
        .def_static(
            "GetCompilerType", 
            (char const *(*)()) &ChasteBuildInfo::GetCompilerType, 
            " " )
        .def_static(
            "GetCompilerVersion", 
            (char const *(*)()) &ChasteBuildInfo::GetCompilerVersion, 
            " " )
        .def_static(
            "GetCompilerFlags", 
            (char const *(*)()) &ChasteBuildInfo::GetCompilerFlags, 
            " " )
        .def_static(
            "GetXsdVersion", 
            (char const *(*)()) &ChasteBuildInfo::GetXsdVersion, 
            " " )
        .def_static(
            "rGetProjectVersions", 
            (::std::map<std::basic_string<char>, std::basic_string<char>, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, std::basic_string<char> > > > const &(*)()) &ChasteBuildInfo::rGetProjectVersions, 
            " " )
        .def_static(
            "rGetIfProjectsModified", 
            (::std::map<std::basic_string<char>, std::basic_string<char>, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, std::basic_string<char> > > > const &(*)()) &ChasteBuildInfo::rGetIfProjectsModified, 
            " " )
        .def_static(
            "GetProvenanceString", 
            (::std::string(*)()) &ChasteBuildInfo::GetProvenanceString, 
            " " )
    ;
}
