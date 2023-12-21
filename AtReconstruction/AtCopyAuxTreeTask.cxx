#include "AtCopyAuxTreeTask.h"

#include "AtBaseEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairTask.h>

#include <TChain.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TObject.h>
#include <TTree.h>

bool AtCopyAuxTreeTask::SetInputTree(TString fileName, TString treeName)
{
   if (fInputTree != nullptr) {
      LOG(error) << "Input tree was already set! Add more trees using the "
                 << "function AddInputTree.";
      return false;
   }
   // Add the tree to the input
   fInputTree = std::make_unique<TChain>(treeName);

   // If there is a tree with the correct name in the file
   return AddInputFile(fileName);
}

bool AtCopyAuxTreeTask::AddInputFile(TString fileName)
{
   if (fInputTree == nullptr) {
      LOG(fatal) << "Input TChain was not initialized using the "
                 << "function SetInputTree.";
      return false;
   }

   return (fInputTree->Add(fileName, -1) == 1);
}

InitStatus AtCopyAuxTreeTask::Init()
{
   // Register the input and output branches with the IO manager
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fCheckEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fCheckEventBranchName));
   if (fCheckEventArray == nullptr) {
      LOG(fatal) << "Cannot find AtBaseEvent array in branch " << fCheckEventBranchName << "!";
      return kFATAL;
   }

   if (fInputTree == nullptr) {
      LOG(fatal) << "Input tree was never initialized!";
      return kFATAL;
   }

   // Create the clone of the input tree in the new file
   fOutputFile = new TFile(fOutputFileName, "RECREATE"); // NOLINT (owned by ROOT)
   if (fOutputFile->IsZombie()) {
      LOG(fatal) << "Failed to open output file: " << fOutputFileName;
      return kERROR;
   }
   fOutputFile->cd();
   fOutputTree = fInputTree->CloneTree(0);
   LOG(info) << "Initialized output tree in " << fOutputFileName;

   return kSUCCESS;
}

void AtCopyAuxTreeTask::Exec(Option_t *opt)
{
   fInputTree->GetEntry(FairRootManager::Instance()->GetEntryNr());

   auto fEvent = dynamic_cast<AtBaseEvent *>(fCheckEventArray->At(0));

   if (fEvent->IsGood())
      fOutputTree->Fill();
}

void AtCopyAuxTreeTask::Finish()
{
   fOutputFile->cd();
   fOutputTree->Write();
   fOutputFile->Close();
}
