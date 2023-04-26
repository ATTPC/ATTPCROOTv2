#ifndef ATDATAMANIP_H
#define ATDATAMANIP_H
#include <memory>
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
