#include "AtCopyAuxTreeTask.h"

#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootFileSink.h>
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairSink.h> // for FairSink
#include <FairTask.h>

#include <TChain.h>
#include <TClonesArray.h>
#include <TObject.h>

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

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray == nullptr) {
      LOG(fatal) << "Cannot find AtRawEvent array in branch " << fRawEventBranchName << "!";
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

   auto fEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

   if (fEvent->IsGood())
      fOutputTree->Fill();
}

void AtCopyAuxTreeTask::Finish()
{
   fOutputFile->cd();
   fOutputTree->Write();
   fOutputFile->Close();
}
