#include "AtCutHEIST.h"

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TCollection.h> // for TIter
#include <TCutG.h>
#include <TFile.h>
#include <TList.h>   // for TList
#include <TObject.h> // for TObject

#include <utility> // for tuple_element<>::type

class TTree;

AtCutHEIST::AtCutHEIST(TString treeFile, TString cutFile, TString treeName)
   : fFile(TFile::Open(treeFile)), fReader(treeName, fFile.get()), fMusic(fReader, "MUSIC"), fUsMcp(fReader, "USMCP"),
     fDsMcp(fReader, "DSMCP"), fCutFile(TFile::Open(cutFile))
{
   if (fReader.GetTree() == nullptr)
      LOG(fatal) << "Could not find " << treeName << " in " << treeFile;
   if (fCutFile == nullptr)
      LOG(fatal) << "Could not find cut file " << cutFile;
}

AtCutHEIST::AtCutHEIST(TTree *tree, TString cutFile)
   : fFile(nullptr), fReader(tree), fMusic(fReader, "MUSIC"), fUsMcp(fReader, "USMCP"), fDsMcp(fReader, "DSMCP"),
     fCutFile(TFile::Open(cutFile))
{
   if (fReader.GetTree() == nullptr)
      LOG(fatal) << "Passed tree is null!";
   if (fCutFile == nullptr)
      LOG(fatal) << "Could not find cut file " << cutFile;
}

/**
 * @brief Add a species to accept in the cut.
 * @return true if the species has been added, false if we couldn't find or add it.
 */
bool AtCutHEIST::AddSpecies(std::string species)
{
   // Return early if this species has already been added
   if (fSpecies.find(species) != fSpecies.end())
      return true;

   auto *cut = static_cast<TCutG *>(fCutFile->GetObjectChecked(species.data(), "TCutG"));
   if (cut == nullptr) {
      LOG(error) << "Could not find cut " << species << " in file";
      return false;
   }

   fSpecies[species] = cut;
   if (fOutFile) {
      fOutFile->cd();
      cut->Write();
   }
   return true;
}

void AtCutHEIST::AddAllSpecies()
{
   auto keys = fCutFile->GetListOfKeys();
   for (auto key : *keys)
      AddSpecies(key->GetName());
}

bool AtCutHEIST::operator()()
{
   fReader.SetEntry(FairRootManager::Instance()->GetEntryNr());
   auto energy = GetEnergy();
   auto tof = GetToF();
   LOG(debug) << "Energy: " << energy << " ToF: " << tof;

   for (auto &[name, cut] : fSpecies) {
      bool inCut = cut->IsInside(tof, energy);

      if (inCut) {
         LOG(info) << "Event is " << name;
         return true;
      }
   }
   return false;
}

std::string AtCutHEIST::GetSpecies()
{
   fReader.SetEntry(FairRootManager::Instance()->GetEntryNr());
   auto energy = GetEnergy();
   auto tof = GetToF();

   for (auto &[name, cut] : fSpecies) {
      bool inCut = cut->IsInside(tof, energy);

      if (inCut) {
         return name;
      }
   }
   return "";
}

double AtCutHEIST::GetEnergy()
{
   double en = 0;
   for (int i = 0; i < 9; ++i)
      en += fMusic->GetEnergy(i);
   return 11.17 * en;
}

double AtCutHEIST::GetToF()
{
   if (fUsMcp->GetTimeAnode().size() > 0)
      return fDsMcp->GetTimeAnode()[0] - fUsMcp->GetTimeAnode()[0] + 728;
   else
      return 0;
}
