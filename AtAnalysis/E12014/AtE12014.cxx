#include "AtE12014.h"

#include "AtCSVReader.h"
#include "AtContainerManip.h"
#include "AtDataManip.h"
#include "AtHit.h"
#include "AtMap.h"
#include "AtPad.h" // for AtPad
#include "AtPadArray.h"
#include "AtPadReference.h" // for AtPadReference
#include "AtRawEvent.h"
#include "AtTpcMap.h"

#include <FairLogger.h>

#include <TF1.h>
#include <TH1.h>
#include <TString.h>

#include <algorithm> // for fill_n, max
#include <cstdlib>   // for getenv
#include <iosfwd>    // for ifstream
#include <set>
std::shared_ptr<AtMap> E12014::fMap;

void E12014::CreateMap()
{
   fMap = std::make_shared<AtTpcMap>();
   auto mapFile = TString(getenv("VMCWORKDIR")) + "/scripts/e12014_pad_mapping.xml";
   fMap->ParseXMLMap(mapFile.Data());

   // Add the inhibited pads
   AtPadReference ref{.cobo = 2, .asad = 3, .aget = 0, .ch = 0};
   for (ref.aget = 0; ref.aget < 4; ++ref.aget) {
      for (ref.ch = 0; ref.ch < 68; ++ref.ch) {
         fMap->InhibitPad(ref, AtMap::InhibitType::kBadPad);
      }
   }
   std::vector<AtPadReference> badPads = {{0, 1, 1, 6},  {0, 1, 1, 7},  {0, 1, 1, 9},  {0, 1, 1, 10},
                                          {0, 1, 1, 12}, {0, 1, 1, 39}, {0, 1, 1, 40}, {0, 1, 1, 41},
                                          {0, 1, 1, 44}, {0, 1, 1, 43}, {0, 1, 1, 46}, {0, 1, 3, 13}};

   for (auto &badPad : badPads)
      fMap->InhibitPad(badPad, AtMap::InhibitType::kBadPad);

   // Add the smart-zap region to the map
   std::ifstream file("/mnt/projects/hira/e12014/tpcSharedInfo/e12014_zap.csv");
   if (!file.is_open())
      LOG(error) << "Failed to open smart zap file";

   // Clear out the header
   std::string temp;
   std::getline(file, temp);
   std::getline(file, temp);

   for (auto &row : CSVRange<int>(file)) {
      LOG(debug) << "Inhibiting " << row[4];
      fMap->InhibitPad(row[4], AtMap::InhibitType::kLowGain);
   }
}

void E12014::FillChargeSum(TH1 *hist, const std::vector<AtHit *> &hits, AtRawEvent &event, int threshold,
                           std::string qName)
{
   if (fMap == nullptr)
      LOG(fatal) << "The map (E12014::fMap) was never set! Please call E12014::CreateMap()";

   std::set<int> usedPads;
   for (auto &hit : hits) {

      // Skip hit if the pad is inhibited or we've already added the charge
      if (usedPads.count(hit->GetPadNum()) != 0 || fMap->IsInhibited(hit->GetPadNum()) != AtMap::InhibitType::kNone)
         continue;
      usedPads.insert(hit->GetPadNum());

      auto pad = event.GetPad(hit->GetPadNum());
      if (pad == nullptr)
         continue;

      const auto charge = pad->GetAugment<AtPadArray>(qName);
      if (charge == nullptr)
         continue;

      // If we pass all the checks, add the charge to the histogram
      for (int tb = 20; tb < 500; ++tb)
         if (charge->GetArray(tb) > threshold)
            hist->Fill(tb + 0.5, charge->GetArray(tb));
   }
}

std::set<int> E12014::FillHitSum(TH1 &hist, const std::vector<AtHit *> &hits, int threshold, float satThresh)
{
   std::vector<double> charge;
   auto pads = FillHitSum(charge, hits, threshold, satThresh);
   ContainerManip::SetHistFromData(hist, charge);
   return pads;
}

std::set<int>
E12014::FillHitSum(std::vector<double> &vec, const std::vector<AtHit *> &hits, int threshold, float satThresh)
{
   vec.clear();
   vec.resize(512);
   std::fill_n(vec.begin(), 512, 0);
   std::set<int> goodPads;

   if (fMap == nullptr)
      LOG(fatal) << "The map (E12014::fMap) was never set!";

   for (auto &hit : hits) {
      if (fMap->IsInhibited(hit->GetPadNum()) != AtMap::InhibitType::kNone)
         continue;
      if (hit->GetCharge() > satThresh)
         continue;

      auto func = AtTools::GetHitFunctionTB(*hit);
      if (func == nullptr)
         continue;

      // Add the charge to the array
      for (int tb = 0; tb < vec.size(); ++tb) {
         auto val = func->Eval(tb);
         if (val > threshold)
            vec[tb] += val;
      }
      // Add the pad to the return list
      goodPads.insert(hit->GetPadNum());
   }

   return goodPads;
}

void E12014::FillSimHitSum(TH1 &hist, const std::vector<AtHit *> &hits, const std::set<int> &goodPads, int threshold,
                           float satThresh)
{
   std::vector<double> charge;
   FillSimHitSum(charge, hits, goodPads, threshold, satThresh);
   ContainerManip::SetHistFromData(hist, charge);
}

void E12014::FillSimHitSum(std::vector<double> &vec, const std::vector<AtHit *> &hits, const std::set<int> &goodPads,
                           int threshold, float satThresh)
{
   vec.clear();
   vec.resize(512);
   std::fill_n(vec.begin(), 512, 0);

   for (auto &hit : hits) {
      if (goodPads.find(hit->GetPadNum()) == goodPads.end()) {
         LOG(debug) << "Skipping pad " << hit->GetPadNum() << " because not good";
         continue;
      }
      if (hit->GetCharge() > satThresh)
         continue;

      auto func = AtTools::GetHitFunctionTB(*hit);
      if (func == nullptr)
         continue;

      // Add the charge to the array
      LOG(debug) << "Adding pad " << hit->GetPadNum();

      for (int tb = 0; tb < vec.size(); ++tb) {
         auto val = func->Eval(tb);
         if (val > threshold)
            vec[tb] += val;
      }
   }
}
