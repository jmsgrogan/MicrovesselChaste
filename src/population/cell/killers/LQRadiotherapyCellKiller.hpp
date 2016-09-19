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
     * Linear component of radiosensitivity of a cancerous cell.
     */
    double cancerousLinearRadiosensitivity;

    /**
     * Quadratic component of radiosensitivity of a cancerous cell.
     */
    double cancerousQuadraticRadiosensitivity;

    /**
     * Linear component of radiosensitivity of a normal cell.
     */
    double normalLinearRadiosensitivity;

    /**
     * Quadratic component of radiosensitivity of a normal cell.
     */
    double normalQuadraticRadiosensitivity;

    /**
     * Dose of Radiation injected
     */
    double mDose;

    /**
     * Times at which radiotherapy is used
     */
    std::vector<double> mRadiationTimes;

    /**
     * alpha_max OER term
     */
    double mOerAlphaMax;

    /**
     * alpha_min OER term
     */
    double mOerAlphaMin;

    /**
     * beta_max OER term
     */
    double mOerBetaMax;

    /**
     * beta_min OER term
     */
    double mOerBetaMin;

    /**
     * Oer constant K term
     */
    double mKOer;

    /**
     * Radiotherapy alpha_max term
     */
    double mAlphaMax;

    /**
     * Radiotherapy beta_max term
     */
    double mBetaMax;

    /**
     * Whether to use an oxygen enhancement ratio
     */
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
        SerializableSingleton<RandomNumberGenerator>* p_rng_wrapper = RandomNumberGenerator::Instance()->GetSerializationWrapper();
        archive & p_rng_wrapper;
    }

public:

    /**
     * Default constructor.
     *
     * @param pCellPopulation pointer to the cell population
     */
    LQRadiotherapyCellKiller(AbstractCellPopulation<DIM>* pCellPopulation);

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


    double GetNormalLinearRadiosensitivity();
    double GetNormalQuadraticRadiosensitivity();
    double GetCancerousLinearRadiosensitivity();
    double GetCancerousQuadraticRadiosensitivity();
    double GetDoseInjected();

    /**
     * Overridden OutputCellKillerParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputCellKillerParameters(out_stream& rParamsFile);

    /**
     * Sets doseInjected, the radiation dose injected
     * @param d dose delivered
     */
    void SetDoseInjected(double d);

    /**
     * Sets timeOfRadiation, the time at which radiation occurs
     * @param t the radiation times
     */
    void SetTimeOfRadiation(std::vector<double> t);

    /**
     * Sets cancerousLinearRadiosensitivity and cancerousQuadraticRadiosensitivity to specified concentration.
     *
     * @param alpha linear radiosensitivity for cancer cells
     * @param beta quadratic radiosensitivity for cancer cells
     */
    void SetCancerousRadiosensitivity(double alpha, double beta);

    /**
     * Sets normalLinearRadiosensitivity and normalQuadraticRadiosensitivity to specified concentration.
     *
     * @param alpha linear radiosensitivity for normal cells
     * @param beta quadratic radiosensitivity for normal cells
     */
    void SetNormalRadiosensitivity(double alpha, double beta);

    /**
     * Sets alpha_max OER value
     * @param value alpha_max OER value
     */
    void SetOerAlphaMax(double value);

    /**
     * Sets alpha_min OER value
     * @param value alpha_min OER value
     */
    void SetOerAlphaMin(double value);

    /**
     * Sets beta_max OER value
     * @param value beta_max OER value
     */
    void SetOerBetaMax(double value);

    /**
     * Sets beta_min OER value
     * @param value beta_min OER value
     */
    void SetOerBetaMin(double value);

    /**
     * Sets K OER value
     * @param value K OER value
     */
    void SetOerConstant(double value);

    /**
     * Sets alpha_max radiotherapy value
     * @param value alpha_max radiotherapy value
     */
    void SetAlphaMax(double value);

    /**
     * Sets beta_max radiotherapy value
     * @param value beta_max radiotherapy value
     */
    void SetBetaMax(double value);

    /**
     * Whether to use OER
     * @param useOer Whether to use OER
     */
    void UseOer(bool useOer);

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
