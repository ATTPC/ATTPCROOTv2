#ifndef ATDATAMANIP_H
#define ATDATAMANIP_H
#include <memory>
class TF1;
class AtHit;
/**
 * Namespace for helper functions manipulating things in AtData (and others).
 */
namespace AtTools {

/**
 * @brief Get charge as a function of TB.
 */
std::unique_ptr<TF1> GetHitFunctionTB(const AtHit &hit);

/**
 * @brief Get charge as a function of z (mm).
 */
std::unique_ptr<TF1> GetHitFunction(const AtHit &hit);

/**
 * @brief Get TB that corresponds to the passed z position [mm].
 */
double GetTB(double z);

/**
 * @brief Get TB that corresponds to a drift of distance d [mm].
 */
double GetDriftTB(double d);

};     // namespace AtTools
#endif // ATDATAMANIP_H
