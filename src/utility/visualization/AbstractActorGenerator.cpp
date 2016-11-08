/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is Abstract of Chaste.

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
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A AbstractICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#if VTK_MAJOR_VERSION > 5
    #include <vtkNamedColors.h>
#endif
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkGlyph2D.h>
#include <vtkCubeAxesActor2D.h>
#include <vtkImageData.h>
#include <vtkGeometryFilter.h>
#include <vtkTubeFilter.h>
#include <vtkExtractEdges.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkPolygon.h>
#include <vtkIdList.h>
#include <vtkFeatureEdges.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "BaseUnits.hpp"

#include "AbstractActorGenerator.hpp"

template<unsigned DIM>
AbstractActorGenerator<DIM>::AbstractActorGenerator()
    : mpColorTransferFunction(vtkSmartPointer<vtkColorTransferFunction>::New()),
      mLengthScale(BaseUnits::Instance()->GetReferenceLengthScale()),
      mShowEdges(true),
      mShowPoints(false),
      mShowVolume(true),
      mEdgeColor(zero_vector<double>(3)),
      mPointColor(unit_vector<double>(3, 2)),
      mVolumeColor(unit_vector<double>(3, 0)),
      mViridisColorMap(),
      mVolumeOpacity(0.6),
      mPointSize(1.0),
      mEdgeSize(0.5),
      mDataLabel()
{
    mPointColor*=255.0;
    mVolumeColor*=255.0;

    double viridis_colors[256][3] =
    { {0.0267004, 0.004874, 0.329415},
      {0.0268510, 0.009605, 0.335427},
      {0.0269944, 0.014625, 0.341379},
      {0.0271305, 0.019942, 0.347269},
      {0.0272594, 0.025563, 0.353093},
      {0.0273809, 0.031497, 0.358853},
      {0.0274952, 0.037752, 0.364543},
      {0.0276022, 0.044167, 0.370164},
      {0.0277018, 0.050344, 0.375715},
      {0.0277941, 0.056324, 0.381191},
      {0.0278791, 0.062145, 0.386592},
      {0.0279566, 0.067836, 0.391917},
      {0.0280267, 0.073417, 0.397163},
      {0.0280894, 0.078907, 0.402329},
      {0.0281446, 0.084320, 0.407414},
      {0.0281924, 0.089666, 0.412415},
      {0.0282327, 0.094955, 0.417331},
      {0.0282656, 0.100196, 0.422160},
      {0.0282910, 0.105393, 0.426902},
      {0.0283091, 0.110553, 0.431554},
      {0.0283197, 0.115680, 0.436115},
      {0.0283229, 0.120777, 0.440584},
      {0.0283187, 0.125848, 0.444960},
      {0.0283072, 0.130895, 0.449241},
      {0.0282884, 0.135920, 0.453427},
      {0.0282623, 0.140926, 0.457517},
      {0.0282290, 0.145912, 0.461510},
      {0.0281887, 0.150881, 0.465405},
      {0.0281412, 0.155834, 0.469201},
      {0.0280868, 0.160771, 0.472899},
      {0.0280255, 0.165693, 0.476498},
      {0.0279574, 0.170599, 0.479997},
      {0.0278826, 0.175490, 0.483397},
      {0.0278012, 0.180367, 0.486697},
      {0.0277134, 0.185228, 0.489898},
      {0.0276194, 0.190074, 0.493001},
      {0.0275191, 0.194905, 0.496005},
      {0.0274128, 0.199721, 0.498911},
      {0.0273006, 0.204520, 0.501721},
      {0.0271828, 0.209303, 0.504434},
      {0.0270595, 0.214069, 0.507052},
      {0.0269308, 0.218818, 0.509577},
      {0.0267968, 0.223549, 0.512008},
      {0.0266580, 0.228262, 0.514349},
      {0.0265145, 0.232956, 0.516599},
      {0.0263663, 0.237631, 0.518762},
      {0.0262138, 0.242286, 0.520837},
      {0.0260571, 0.246922, 0.522828},
      {0.0258965, 0.251537, 0.524736},
      {0.0257322, 0.256130, 0.526563},
      {0.0255645, 0.260703, 0.528312},
      {0.0253935, 0.265254, 0.529983},
      {0.0252194, 0.269783, 0.531579},
      {0.0250425, 0.274290, 0.533103},
      {0.0248629, 0.278775, 0.534556},
      {0.0246811, 0.283237, 0.535941},
      {0.0244972, 0.287675, 0.537260},
      {0.0243113, 0.292092, 0.538516},
      {0.0241237, 0.296485, 0.539709},
      {0.0239346, 0.300855, 0.540844},
      {0.0237441, 0.305202, 0.541921},
      {0.0235526, 0.309527, 0.542944},
      {0.0233603, 0.313828, 0.543914},
      {0.0231674, 0.318106, 0.544834},
      {0.0229739, 0.322361, 0.545706},
      {0.0227802, 0.326594, 0.546532},
      {0.0225863, 0.330805, 0.547314},
      {0.0223925, 0.334994, 0.548053},
      {0.0221989, 0.339161, 0.548752},
      {0.0220057, 0.343307, 0.549413},
      {0.0218130, 0.347432, 0.550038},
      {0.0216210, 0.351535, 0.550627},
      {0.0214298, 0.355619, 0.551184},
      {0.0212395, 0.359683, 0.551710},
      {0.0210503, 0.363727, 0.552206},
      {0.0208623, 0.367752, 0.552675},
      {0.0206756, 0.371758, 0.553117},
      {0.0204903, 0.375746, 0.553533},
      {0.0203063, 0.379716, 0.553925},
      {0.0201239, 0.383670, 0.554294},
      {0.0199430, 0.387607, 0.554642},
      {0.0197636, 0.391528, 0.554969},
      {0.0195860, 0.395433, 0.555276},
      {0.0194100, 0.399323, 0.555565},
      {0.0192357, 0.403199, 0.555836},
      {0.0190631, 0.407061, 0.556089},
      {0.0188923, 0.410910, 0.556326},
      {0.0187231, 0.414746, 0.556547},
      {0.0185556, 0.418570, 0.556753},
      {0.0183898, 0.422383, 0.556944},
      {0.0182256, 0.426184, 0.557120},
      {0.0180629, 0.429975, 0.557282},
      {0.0179019, 0.433756, 0.557430},
      {0.0177423, 0.437527, 0.557565},
      {0.0175841, 0.441290, 0.557685},
      {0.0174274, 0.445044, 0.557792},
      {0.0172719, 0.448791, 0.557885},
      {0.0171176, 0.452530, 0.557965},
      {0.0169646, 0.456262, 0.558030},
      {0.0168126, 0.459988, 0.558082},
      {0.0166617, 0.463708, 0.558119},
      {0.0165117, 0.467423, 0.558141},
      {0.0163625, 0.471133, 0.558148},
      {0.0162142, 0.474838, 0.558140},
      {0.0160665, 0.478540, 0.558115},
      {0.0159194, 0.482237, 0.558073},
      {0.0157729, 0.485932, 0.558013},
      {0.0156270, 0.489624, 0.557936},
      {0.0154815, 0.493313, 0.557840},
      {0.0153364, 0.497000, 0.557724},
      {0.0151918, 0.500685, 0.557587},
      {0.0150476, 0.504369, 0.557430},
      {0.0149039, 0.508051, 0.557250},
      {0.0147607, 0.511733, 0.557049},
      {0.0146180, 0.515413, 0.556823},
      {0.0144759, 0.519093, 0.556572},
      {0.0143343, 0.522773, 0.556295},
      {0.0141935, 0.526453, 0.555991},
      {0.0140536, 0.530132, 0.555659},
      {0.0139147, 0.533812, 0.555298},
      {0.0137770, 0.537492, 0.554906},
      {0.0136408, 0.541173, 0.554483},
      {0.0135066, 0.544853, 0.554029},
      {0.0133743, 0.548535, 0.553541},
      {0.0132444, 0.552216, 0.553018},
      {0.0131172, 0.555899, 0.552459},
      {0.0129933, 0.559582, 0.551864},
      {0.0128729, 0.563265, 0.551229},
      {0.0127568, 0.566949, 0.550556},
      {0.0126453, 0.570633, 0.549841},
      {0.0125394, 0.574318, 0.549086},
      {0.0124395, 0.578002, 0.548287},
      {0.0123463, 0.581687, 0.547445},
      {0.0122606, 0.585371, 0.546557},
      {0.0121831, 0.589055, 0.545623},
      {0.0121148, 0.592739, 0.544641},
      {0.0120565, 0.596422, 0.543611},
      {0.0120092, 0.600104, 0.542530},
      {0.0119738, 0.603785, 0.541400},
      {0.0119512, 0.607464, 0.540218},
      {0.0119423, 0.611141, 0.538982},
      {0.0119483, 0.614817, 0.537692},
      {0.0119699, 0.618490, 0.536347},
      {0.0120081, 0.622161, 0.534946},
      {0.0120638, 0.625828, 0.533488},
      {0.0121380, 0.629492, 0.531973},
      {0.0122312, 0.633153, 0.530398},
      {0.0123444, 0.636809, 0.528763},
      {0.0124780, 0.640461, 0.527068},
      {0.0126326, 0.644107, 0.525311},
      {0.0128087, 0.647749, 0.523491},
      {0.0130067, 0.651384, 0.521608},
      {0.0132268, 0.655014, 0.519661},
      {0.0134692, 0.658636, 0.517649},
      {0.0137339, 0.662252, 0.515571},
      {0.0140210, 0.665859, 0.513427},
      {0.0143303, 0.669459, 0.511215},
      {0.0146616, 0.673050, 0.508936},
      {0.0150148, 0.676631, 0.506589},
      {0.0153894, 0.680203, 0.504172},
      {0.0157851, 0.683765, 0.501686},
      {0.0162016, 0.687316, 0.499129},
      {0.0166383, 0.690856, 0.496502},
      {0.0170948, 0.694384, 0.493803},
      {0.0175707, 0.697900, 0.491033},
      {0.0180653, 0.701402, 0.488189},
      {0.0185783, 0.704891, 0.485273},
      {0.0191090, 0.708366, 0.482284},
      {0.0196571, 0.711827, 0.479221},
      {0.0202219, 0.715272, 0.476084},
      {0.0208030, 0.718701, 0.472873},
      {0.0214000, 0.722114, 0.469588},
      {0.0220124, 0.725509, 0.466226},
      {0.0226397, 0.728888, 0.462789},
      {0.0232815, 0.732247, 0.459277},
      {0.0239374, 0.735588, 0.455688},
      {0.0246070, 0.738910, 0.452024},
      {0.0252899, 0.742211, 0.448284},
      {0.0259857, 0.745492, 0.444467},
      {0.0266941, 0.748751, 0.440573},
      {0.0274149, 0.751988, 0.436601},
      {0.0281477, 0.755203, 0.432552},
      {0.0288921, 0.758394, 0.428426},
      {0.0296479, 0.761561, 0.424223},
      {0.0304148, 0.764704, 0.419943},
      {0.0311925, 0.767822, 0.415586},
      {0.0319809, 0.770914, 0.411152},
      {0.0327796, 0.773980, 0.406640},
      {0.0335885, 0.777018, 0.402049},
      {0.0344074, 0.780029, 0.397381},
      {0.0352360, 0.783011, 0.392636},
      {0.0360741, 0.785964, 0.387814},
      {0.0369214, 0.788888, 0.382914},
      {0.0377779, 0.791781, 0.377939},
      {0.0386433, 0.794644, 0.372886},
      {0.0395174, 0.797475, 0.367757},
      {0.0404001, 0.800275, 0.362552},
      {0.0412913, 0.803041, 0.357269},
      {0.0421908, 0.805774, 0.351910},
      {0.0430983, 0.808473, 0.346476},
      {0.0440137, 0.811138, 0.340967},
      {0.0449368, 0.813768, 0.335384},
      {0.0458674, 0.816363, 0.329727},
      {0.0468053, 0.818921, 0.323998},
      {0.0477504, 0.821444, 0.318195},
      {0.0487026, 0.823929, 0.312321},
      {0.0496615, 0.826376, 0.306377},
      {0.0506271, 0.828786, 0.300362},
      {0.0515992, 0.831158, 0.294279},
      {0.0525776, 0.833491, 0.288127},
      {0.0535621, 0.835785, 0.281908},
      {0.0545524, 0.838039, 0.275626},
      {0.0555484, 0.840254, 0.269281},
      {0.0565498, 0.842430, 0.262877},
      {0.0575563, 0.844566, 0.256415},
      {0.0585678, 0.846661, 0.249897},
      {0.0595839, 0.848717, 0.243329},
      {0.0606045, 0.850733, 0.236712},
      {0.0616293, 0.852709, 0.230052},
      {0.0626579, 0.854645, 0.223353},
      {0.0636902, 0.856542, 0.216620},
      {0.0647257, 0.858400, 0.209861},
      {0.0657642, 0.860219, 0.203082},
      {0.0668054, 0.861999, 0.196293},
      {0.0678489, 0.863742, 0.189503},
      {0.0688944, 0.865448, 0.182725},
      {0.0699415, 0.867117, 0.175971},
      {0.0709898, 0.868751, 0.169257},
      {0.0720391, 0.870350, 0.162603},
      {0.0730889, 0.871916, 0.156029},
      {0.0741388, 0.873449, 0.149561},
      {0.0751884, 0.874951, 0.143228},
      {0.0762373, 0.876424, 0.137064},
      {0.0772852, 0.877868, 0.131109},
      {0.0783315, 0.879285, 0.125405},
      {0.0793760, 0.880678, 0.120005},
      {0.0804182, 0.882046, 0.114965},
      {0.0814576, 0.883393, 0.110347},
      {0.0824940, 0.884720, 0.106217},
      {0.0835270, 0.886029, 0.102646},
      {0.0845561, 0.887322, 0.099702},
      {0.0855810, 0.888601, 0.097452},
      {0.0866013, 0.889868, 0.095953},
      {0.0876168, 0.891125, 0.095250},
      {0.0886271, 0.892374, 0.095374},
      {0.0896320, 0.893616, 0.096335},
      {0.0906311, 0.894855, 0.098125},
      {0.0916242, 0.896091, 0.100717},
      {0.0926106, 0.897330, 0.104071},
      {0.0935904, 0.898570, 0.108131},
      {0.0945636, 0.899815, 0.112838},
      {0.0955300, 0.901065, 0.118128},
      {0.0964894, 0.902323, 0.123941},
      {0.0974417, 0.903590, 0.130215},
      {0.0983868, 0.904867, 0.136897},
      {0.0993248, 0.906157, 0.143936}};

    for(unsigned idx=0; idx<256;idx++)
    {
        mpColorTransferFunction->AddRGBPoint(double(idx)/255.0,
                                               viridis_colors[idx][0],
                                               viridis_colors[idx][1],
                                               viridis_colors[idx][2]);
    }
}

template<unsigned DIM>
AbstractActorGenerator<DIM>::~AbstractActorGenerator()
{

}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowEdges(bool show)
{
    mShowEdges = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowPoints(bool show)
{
    mShowPoints = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetPointSize(double size)
{
    mPointSize = size;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetEdgeSize(double size)
{
    mEdgeSize = size;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowVolume(bool show)
{
    mShowVolume = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetEdgeColor(const c_vector<double, 3>& rColor)
{
    mEdgeColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetPointColor(const c_vector<double, 3>& rColor)
{
    mPointColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetVolumeColor(const c_vector<double, 3>& rColor)
{
    mVolumeColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetVolumeOpacity(double opacity)
{
    mVolumeOpacity = opacity;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetDataLabel(const std::string& rLabel)
{
    mDataLabel = rLabel;
}


template class AbstractActorGenerator<2>;
template class AbstractActorGenerator<3>;
