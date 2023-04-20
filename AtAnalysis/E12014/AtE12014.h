#ifndef ATE12014_H
#define ATE12014_H
#include <memory>
#include <set>
#include <string> // for string
#include <vector>

class AtMap;
class TH1;
class AtHit;
class AtRawEvent;

/**
 * Namespace deticated for useful functions specific to the E12014 fission experiment.
 * (now a class becuase ROOT disctionaries are stupid and cause double definitions of variables in the namespace)
 * I'm intentionally breaking the At... naming pattern as this is a class outside the normal framework.
 */
class E12014 {
public:
   /**
    * Mapping to use when executing functions in the E12014 namespace.
    */
   static std::shared_ptr<AtMap> fMap; //!

   /**
    * Create and set fMap to the mapping for this experiment including setting the pad pads.
    */
   static void CreateMap();

   /**
    * Fill the historgram with charge information from the pads associated with the passed hits
    *@param[in/out] hist Histrogram to clear and fill.
    *@param[in] hits Add the charge from the hits associated with these hits.
    *@param[in] event Event containing the pads to pull charge from.
    *@param[in] threshold Only add TBs that are above this threshold.
    *@param[in] qName Name of the pad augment that contains the charge.
    */
   static void FillChargeSum(TH1 *hist, const std::vector<AtHit *> &hits, AtRawEvent &event, int threshold = 0,
                             std::string qName = "Qreco");

   /**
    * Fill the historgram with charge information from the passed hits assuming the charge is gauss distributed.
    *@param[in/out] hist Histrogram to clear and fill.
    *@param[in] hits Add the charge from the hits associated with these hits.
    *@return The pads associated with the hits used to fill the histogram.
    */
   static std::set<int> FillHitSum(TH1 &hist, const std::vector<AtHit *> &hits, int threshold = 0,
                                   float saturationThreshold = std::numeric_limits<float>::max());

   /**
    * Fill the array with charge information from the passed hits assuming the charge is gauss distributed.
    *@param[in/out] vec vector to clear and fill.
    *@param[in] hits Add the charge from the hits associated with these hits.
    *@return The pads associated with the hits used to fill the histogram.
    */
   static std::set<int> FillHitSum(std::vector<double> &vec, const std::vector<AtHit *> &hits, int threshold = 0,
                                   float saturationThreshold = std::numeric_limits<float>::max());

}; // namespace E12014

#endif //#ifndef ATE12014_H
