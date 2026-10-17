#ifndef ATDATAMANIP_H
#define ATDATAMANIP_H
#include <Math/IFunctionfwd.h>

#include <cmath>
#include <memory>
#include <optional>
class TF1;
class AtHit;
class AtDigiPar;
/**
 * Namespace for helper functions manipulating things in AtData (and others).
 * Will take in the parameter class to use. If nullptr then looks for it in the runtime DB
 * N.B. FairRoot implentations mean only the primary thread can find things
 * (the underlying static variable referenced FairRun::Instance() is thread_local).
 */
namespace AtTools {

/**
 * @brief Get charge as a function of TB.
 */
std::unique_ptr<TF1> GetHitFunctionTB(const AtHit &hit, const AtDigiPar *par = nullptr);

/**
 * @brief A hit's charge as a function of TB: amp * exp(-0.5 * ((tb - mean) / sigma)^2).
 */
struct GaussianTB {
   double amp;
   double mean;
   double sigma;

   double operator()(double tb) const
   {
      auto u = (tb - mean) / sigma;
      return amp * std::exp(-0.5 * u * u);
   }
};

/**
 * @brief Get the charge of a hit as a function of TB, without building a TF1 (which is slow to do for every hit).
 * Returns nullopt if the hit has no z extent.
 */
std::optional<GaussianTB> GetHitGaussianTB(const AtHit &hit, const AtDigiPar *par = nullptr);

/**
 * @brief Minimize fcn with Minuit2's Migrad, starting from the fcn.NDim() values in par.
 * Returns false if the minimization failed. Otherwise par holds the minimum and, if given, minValue the value there.
 * Set up like ROOT::Fit::Fitter (see the definition), with the same results, but safe to call from many threads.
 */
bool MinimizeMigrad(const ROOT::Math::IMultiGenFunction &fcn, double *par, double *minValue = nullptr);

/**
 * @brief Get charge as a function of z (mm).
 */
std::unique_ptr<TF1> GetHitFunction(const AtHit &hit, const AtDigiPar *par = nullptr);

/**
 * @brief Get TB that corresponds to the passed z position [mm].
 */
double GetTB(double z, const AtDigiPar *par = nullptr);

/**
 * @brief Get TB that corresponds to a drift of distance d [mm].
 */
double GetDriftTB(double d, const AtDigiPar *par = nullptr);

};     // namespace AtTools
#endif // ATDATAMANIP_H
