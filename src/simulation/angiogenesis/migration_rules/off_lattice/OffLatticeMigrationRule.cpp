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

#include "GeometryTools.hpp"
#include "OffLatticeMigrationRule.hpp"
#include "RandomNumberGenerator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
OffLatticeMigrationRule<DIM>::OffLatticeMigrationRule()
    : AbstractMigrationRule<DIM>(),
      mGlobalX(unit_vector<double>(DIM,0)),
      mGlobalY(unit_vector<double>(DIM,0)),
      mGlobalZ(zero_vector<double>(DIM)),
      mMeanAngles(std::vector<double>(DIM, 0.0)),
      mSdvAngles(std::vector<double>(DIM, M_PI/18.0)),
      mVelocity(20.0 *(1.e-6/3600.0) * unit::metre_per_second), // um/hr
      mChemotacticStrength(1.0),
      mAttractionStrength(1.0),
      mProbeLength(5.0 * 1.e-6 * unit::metres),
      mIsSprouting(false)
{
    if(DIM==3)
    {
        mGlobalZ = unit_vector<double>(DIM,2);
    }
}

template <unsigned DIM>
boost::shared_ptr<OffLatticeMigrationRule<DIM> > OffLatticeMigrationRule<DIM>::Create()
{
    MAKE_PTR(OffLatticeMigrationRule<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
OffLatticeMigrationRule<DIM>::~OffLatticeMigrationRule()
{

}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetSproutingVelocity(units::quantity<unit::velocity> velocity)
{
    mVelocity = velocity;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetChemotacticStrength(double strength)
{
    mChemotacticStrength = strength;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetAttractionStrength(double strength)
{
    mAttractionStrength = strength;
}

template<unsigned DIM>
void OffLatticeMigrationRule<DIM>::SetIsSprouting(bool isSprouting)
{
    this->mIsSprouting = isSprouting;
}

template<unsigned DIM>
std::vector<c_vector<double, DIM> > OffLatticeMigrationRule<DIM>::GetDirections(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if (this->mIsSprouting)
    {
        return GetDirectionsForSprouts(rNodes);
    }
    else
    {
        std::vector<c_vector<double, DIM> > movement_vectors(rNodes.size(), zero_vector<double>(DIM));

        // We want to probe the PDE solution all at once first if needed, as this is an expensive operation if done node-by-node.
        // Every node has 5 probes in 2D and 7 in 3D.
        std::vector<units::quantity<unit::concentration> > probed_solutions;
        unsigned probes_per_node = 2*DIM+1;
        std::vector<DimensionalChastePoint<DIM> > probe_locations(probes_per_node*rNodes.size());

        if(this->mpSolver)
        {
            double normalized_probe_length = mProbeLength/BaseUnits::Instance()->GetReferenceLengthScale();
            for(unsigned idx=0; idx<rNodes.size(); idx++)
            {
                probe_locations[idx*probes_per_node] = rNodes[idx]->rGetLocation();
                probe_locations[idx*probes_per_node+1] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() +
                                                                                   normalized_probe_length * unit_vector<double>(DIM,0));
                probe_locations[idx*probes_per_node+2] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() -
                                                                                   normalized_probe_length * unit_vector<double>(DIM,0));
                probe_locations[idx*probes_per_node+3] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() +
                                                                                   normalized_probe_length * unit_vector<double>(DIM,1));
                probe_locations[idx*probes_per_node+4] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() -
                                                                                   normalized_probe_length * unit_vector<double>(DIM,1));
                if(DIM==3)
                {
                    probe_locations[idx*probes_per_node+5] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() +
                                                                                       normalized_probe_length * unit_vector<double>(DIM,2));
                    probe_locations[idx*probes_per_node+6] = DimensionalChastePoint<DIM>(rNodes[idx]->rGetLocation().rGetLocation() -
                                                                                       normalized_probe_length * unit_vector<double>(DIM,2));
                }
            }
            if(probe_locations.size()>0)
            {
                probed_solutions = this->mpSolver->GetConcentrations(probe_locations);
            }
        }

        for(unsigned idx=0; idx<rNodes.size(); idx++)
        {
            double angle_x = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[0], mSdvAngles[0]);
            double angle_y = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[1], mSdvAngles[1]);
            double angle_z = 0.0;
            if(DIM==3)
            {
                angle_z = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[2], mSdvAngles[2]);
            }
            c_vector<double, DIM> currentDirection  =
                    -rNodes[idx]->GetSegments()[0]->GetOppositeNode(rNodes[idx])->rGetLocation().rGetLocation() + rNodes[idx]->rGetLocation().rGetLocation();
            currentDirection /= norm_2(currentDirection);

            c_vector<double, DIM> new_direction_z = RotateAboutAxis<DIM>(currentDirection, mGlobalZ, angle_z);
            c_vector<double, DIM> new_direction_y = RotateAboutAxis<DIM>(new_direction_z, mGlobalY, angle_y);
            c_vector<double, DIM> new_direction = RotateAboutAxis<DIM>(new_direction_y, mGlobalX, angle_x);
            new_direction /= norm_2(new_direction);

            // Solution dependent contribution
            if(this->mpSolver)
            {
                // Get the gradients
                std::vector<units::quantity<unit::concentration_gradient> > gradients(probes_per_node-1, 0.0*unit::mole_per_metre_pow_4);
                if(probed_solutions.size()>=idx*probes_per_node+probes_per_node)
                {
                    for(unsigned jdx=1; jdx<probes_per_node; jdx++)
                    {
                        gradients.push_back((probed_solutions[idx*probes_per_node+jdx] - probed_solutions[idx*probes_per_node]) / mProbeLength);
                    }
                }

                // Get the index of the max gradient
                units::quantity<unit::concentration_gradient> max_grad = 0.0 * unit::mole_per_metre_pow_4;
                int index = -1;

                for(unsigned jdx = 0; jdx<gradients.size(); jdx++)
                {
                    if(gradients[jdx]>max_grad)
                    {
                        max_grad = gradients[jdx];
                        index = jdx;
                    }
                }

                if(index == 0)
                {
                    new_direction += mChemotacticStrength* unit_vector<double>(DIM,0);
                }
                else if(index == 1)
                {
                    new_direction += mChemotacticStrength*-unit_vector<double>(DIM,0);
                }
                else if(index == 2)
                {
                    new_direction += mChemotacticStrength*unit_vector<double>(DIM,1);
                }
                else if(index == 3)
                {
                    new_direction += mChemotacticStrength*-unit_vector<double>(DIM,1);
                }
                else if(index == 4)
                {
                    new_direction += mChemotacticStrength*unit_vector<double>(DIM,2);
                }
                else if(index == 5)
                {
                    new_direction += mChemotacticStrength*-unit_vector<double>(DIM,2);
                }
            }

            new_direction /= norm_2(new_direction);

            // Get the closest node in the search cone
            std::vector<boost::shared_ptr<VesselNode<DIM> > > nodes = this->mpVesselNetwork->GetNodes();

            double min_distance = 1.e12;
            c_vector<double, DIM> min_direction = zero_vector<double>(DIM);
            for(unsigned jdx=0; jdx<nodes.size(); jdx++)
            {
                if(IsPointInCone<DIM>(nodes[jdx]->rGetLocation().rGetLocation(), rNodes[idx]->rGetLocation().rGetLocation(), rNodes[idx]->rGetLocation().rGetLocation() + currentDirection * 100.0, M_PI/3.0))
                {
                    double distance = norm_2(rNodes[idx]->rGetLocation().rGetLocation() - nodes[jdx]->rGetLocation().rGetLocation());
                    if(distance < min_distance)
                    {
                        min_distance = distance;
                        min_direction = nodes[jdx]->rGetLocation().rGetLocation() - rNodes[idx]->rGetLocation().rGetLocation();
                        min_direction /= norm_2(min_direction);
                    }
                }
            }

            double strength;
            double crictical_distance = 100.0;
            if(min_distance >= crictical_distance)
            {
                strength = 0.0;
            }
            else
            {
                strength = mAttractionStrength *  (1.0 - (min_distance * min_distance) / (crictical_distance * crictical_distance));
            }
            new_direction += strength * min_direction;
            new_direction /= norm_2(new_direction);
            movement_vectors[idx] = new_direction * double(mVelocity*(BaseUnits::Instance()->GetReferenceTimeScale()/BaseUnits::Instance()->GetReferenceLengthScale()));
        }
        return movement_vectors;
    }
}

template<unsigned DIM>
std::vector<c_vector<double, DIM> > OffLatticeMigrationRule<DIM>::GetDirectionsForSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    std::vector<c_vector<double, DIM> > movement_vectors(rNodes.size(), zero_vector<double>(DIM));
    for(unsigned idx = 0; idx < rNodes.size(); idx++)
    {
        c_vector<double, DIM> sprout_direction;
        c_vector<double, DIM> cross_product = VectorProduct(rNodes[idx]->GetSegments()[0]->GetUnitTangent(),
                                                            rNodes[idx]->GetSegments()[1]->GetUnitTangent());
        double sum = 0.0;
        for(unsigned jdx=0; jdx<DIM; jdx++)
        {
            sum += cross_product[jdx];
        }
        if (sum<=1.e-6)
        {
            // parallel segments, chose any normal to the first tangent
            c_vector<double, DIM> normal;
            c_vector<double, DIM> tangent = rNodes[idx]->GetSegments()[0]->GetUnitTangent();
            if(DIM==2 or tangent[2]==0.0)
            {
                if(tangent[1] == 0.0)
                {
                    normal[0] = 0.0;
                    normal[1] = 1.0;
                }
                else
                {
                    normal[0] = 1.0;
                    normal[1] = -tangent[0] /tangent[1];
                }
            }
            else
            {
                if(std::abs(tangent[0]) + std::abs(tangent[1]) == 0.0)
                {
                    normal[0] = 1.0;
                    normal[1] = 1.0;
                }
                else
                {
                    normal[0] = 1.0;
                    normal[1] = 1.0;
                    normal[2] = -(tangent[0] + tangent[1])/tangent[2];
                }
            }
            if(RandomNumberGenerator::Instance()->ranf()>=0.5)
            {
                sprout_direction = normal/norm_2(normal);
            }
            else
            {
                sprout_direction = -normal/norm_2(normal);
            }
        }
        else
        {
            // otherwise the direction is out of the plane of the segment tangents
            if(RandomNumberGenerator::Instance()->ranf()>=0.5)
            {
                sprout_direction = cross_product/norm_2(cross_product);
            }
            else
            {
                sprout_direction = -cross_product/norm_2(cross_product);
            }
        }
        double angle = RandomNumberGenerator::Instance()->NormalRandomDeviate(mMeanAngles[0], mSdvAngles[0]);
        c_vector<double, DIM> new_direction = RotateAboutAxis<DIM>(sprout_direction, rNodes[idx]->GetSegments()[0]->GetUnitTangent(), angle);
        new_direction /= norm_2(new_direction);
        movement_vectors[idx] = new_direction * double(mVelocity * (BaseUnits::Instance()->GetReferenceTimeScale()/BaseUnits::Instance()->GetReferenceLengthScale()));
    }
    return movement_vectors;
}

// Explicit instantiation
template class OffLatticeMigrationRule<2> ;
template class OffLatticeMigrationRule<3> ;
