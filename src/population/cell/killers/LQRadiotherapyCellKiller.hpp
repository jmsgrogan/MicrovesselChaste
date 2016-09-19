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

#ifndef LQRADIOTHERAPYCELLKILLER_HPP_
#define LQRADIOTHERAPYCELLKILLER_HPP_

#include "AbstractCellKiller.hpp"
#include "RandomNumberGenerator.hpp"

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>

/**
 * A cell killer that kills cells based on the LQ Model in Radiotherapy
 */
template<unsigned DIM>
class LQRadiotherapyCellKiller : public AbstractCellKiller<DIM>
{
    /**
     * Radiosensitivity of a cancerous cell.
     */
    double cancerousLinearRadiosensitivity;
    double cancerousQuadraticRadiosensitivity;

    /**
     * Radiosensitivity of a normal cell
     */
    double normalLinearRadiosensitivity;
    double normalQuadraticRadiosensitivity;

    /**
     * Dose of Radiation injected
     */
    double mDose;

    /**
     * Time at which radiotherapy is used
     */
    std::vector<double> mRadiationTimes;

    double mOerAlphaMax;

    double mOerAlphaMin;

    double mOerBetaMax;

    double mOerBetaMin;

    double mKOer;

    double mAlphaMax;

    double mBetaMax;

    bool mUseOer;

private:

    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Archive the object.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellKiller<DIM> >(*this);

        // Make sure the random number generator is also archived
        SerializableSingleton<RandomNumberGenerator>* p_rng_wrapper =
                RandomNumberGenerator::Instance()->GetSerializationWrapper();
        archive & p_rng_wrapper;
    }

public:

    /**
     * Default constructor.
     *
     * @param pCellPopulation pointer to the cell population
     * @param probabilityOfDeath
     */
    LQRadiotherapyCellKiller(AbstractCellPopulation<DIM>* pCellPopulation);

    /**
     * Sets doseInjected, the radiation dose injected
     */
    void SetDoseInjected(double d);

    /**
     * Sets timeOfRadiation, the time at which radiation occurs
     */
    void SetTimeOfRadiation(std::vector<double> t);

    /**
     * Sets cancerousLinearRadiosensitivity and cancerousQuadraticRadiosensitivity to specified concentration.
     *
     * @param alpha and beta to which cancerousLinearRadiosensitivity and cancerousQuadraticRadiosensitivity should be set.
     */
    void SetCancerousRadiosensitivity(double alpha, double beta);

    /**
     * Sets normalLinearRadiosensitivity and normalQuadraticRadiosensitivity to specified concentration.
     *
     * @param alpha and beta to which normalLinearRadiosensitivity and normalQuadraticRadiosensitivity should be set.
     */
    void SetNormalRadiosensitivity(double alpha, double beta);

    double GetNormalLinearRadiosensitivity();
    double GetNormalQuadraticRadiosensitivity();
    double GetCancerousLinearRadiosensitivity();
    double GetCancerousQuadraticRadiosensitivity();
    double GetDoseInjected();

    void SetOerAlphaMax(double value);

    void SetOerAlphaMin(double value);

    void SetOerBetaMax(double value);

    void SetOerBetaMin(double value);

    void SetOerConstant(double value);

    void SetAlphaMax(double value);

    void SetBetaMax(double value);

    void UseOer(bool useOer);

    /**
     * Overridden method to test a given cell for apoptosis.
     *
     * @param pCell the cell to test for apoptosis
     */
    void CheckAndLabelSingleCellForApoptosis(CellPtr pCell);

    /**
     * Loop over cells and start apoptosis randomly, based on the user-set
     * probability.
     */
    void CheckAndLabelCellsForApoptosisOrDeath();

    /**
     * Overridden OutputCellKillerParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputCellKillerParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(LQRadiotherapyCellKiller)

namespace boost
{
namespace serialization
{
/**
 * Serialize information required to construct a LQRadiotherapyCellKiller.
 */
template<class Archive, unsigned DIM>
inline void save_construct_data(Archive & ar, const LQRadiotherapyCellKiller<DIM> * t, const unsigned int file_version)
{
    // Save data required to construct instance
    const AbstractCellPopulation<DIM>* const p_cell_population = t->GetCellPopulation();
    ar << p_cell_population;
}

/**
 * De-serialize constructor parameters and initialise a LQRadiotherapyCellKiller.
 */
template<class Archive, unsigned DIM>
inline void load_construct_data(Archive & ar, LQRadiotherapyCellKiller<DIM> * t, const unsigned int file_version)
{
    // Retrieve data from archive required to construct new instance
    AbstractCellPopulation<DIM>* p_cell_population;
    ar >> p_cell_population;

    // Invoke inplace constructor to initialise instance
    ::new (t) LQRadiotherapyCellKiller<DIM>(p_cell_population);
}
}
} // namespace ...

#endif /*LQRADIOTHERAPYCELLKILLER_HPP_*/
