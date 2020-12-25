//////////////////////////////////////////////////////////
// This class has been automatically generated on
// Thu May  4 14:01:25 2017 by ROOT version 6.06/02
// from TChain bgtracks/
//////////////////////////////////////////////////////////

#ifndef anaClassBg_h
#define anaClassBg_h

#include <TROOT.h>
#include <TChain.h>
#include <TFile.h>
#include "TClonesArray.h"
#include "TMath.h"
#include "TRandom3.h"
#include "TVector3.h"
#include <TGraph2D.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <TF2.h>
#include <TH1.h>
#include <TH2.h>

// Header file for the classes stored in the TTree if any.

class anaClassBg {
public :
   TTree          *fChain;   //!pointer to the analyzed TTree or TChain
   Int_t           fCurrent; //!current Tree number in a TChain

// Fixed size dimensions of array or collections stored in the TTree if any.

   // Declaration of leaf types
   Double_t        Elosshibg;
   Double_t        Rangelowbg;
   Double_t        Rangehibg;
   Double_t        Thetabg;
   Double_t        Phibg;
   Double_t        Av_elossbg;
   Bool_t          Goodfitbg;
   Double_t        Vertexbg;
   Int_t           ContaBg;

   // List of branches
   TBranch        *b_Elosshibg;   //!
   TBranch        *b_Rangelowbg;   //!
   TBranch        *b_Rangehibg;   //!
   TBranch        *b_Thetabg;   //!
   TBranch        *b_Phibg;   //!
   TBranch        *b_Av_elossbg;   //!
   TBranch        *b_Goodfitbg;   //!
   TBranch        *b_Vertexbg;   //!
   TBranch        *b_ContaBg;   //!

   anaClassBg(TTree *tree=0);
   virtual ~anaClassBg();
   virtual Int_t    Cut(Long64_t entry);
   virtual Int_t    GetEntry(Long64_t entry);
   virtual Long64_t LoadTree(Long64_t entry);
   virtual void     Init(TTree *tree);
   virtual void     Loop();
   virtual Bool_t   Notify();
   virtual void     Show(Long64_t entry = -1);
};

#endif

#ifdef anaClassBg_cxx
anaClassBg::anaClassBg(TTree *tree) : fChain(0) 
{
// if parameter tree is not specified (or zero), connect the file
// used to generate this class and read the Tree.
   if (tree == 0) {

#ifdef SINGLE_TREE
      // The following code should be used if you want this class to access
      // a single tree instead of a chain
      TFile *f = (TFile*)gROOT->GetListOfFiles()->FindObject("Memory Directory");
      if (!f || !f->IsOpen()) {
         f = new TFile("Memory Directory");
      }
      f->GetObject("bgtracks",tree);

#else // SINGLE_TREE

      // The following code should be used if you want this class to access a chain
      // of trees.
      TChain * chain = new TChain("bgtracks","");
      chain->Add("hist_d2He_12CBgonly_1atm_20.root/He2_reconstr_resol/bgtracks");
      tree = chain;
#endif // SINGLE_TREE

   }
   Init(tree);
}

anaClassBg::~anaClassBg()
{
   if (!fChain) return;
   delete fChain->GetCurrentFile();
}

Int_t anaClassBg::GetEntry(Long64_t entry)
{
// Read contents of entry.
   if (!fChain) return 0;
   return fChain->GetEntry(entry);
}
Long64_t anaClassBg::LoadTree(Long64_t entry)
{
// Set the environment to read one entry
   if (!fChain) return -5;
   Long64_t centry = fChain->LoadTree(entry);
   if (centry < 0) return centry;
   if (fChain->GetTreeNumber() != fCurrent) {
      fCurrent = fChain->GetTreeNumber();
      Notify();
   }
   return centry;
}

void anaClassBg::Init(TTree *tree)
{
   // The Init() function is called when the selector needs to initialize
   // a new tree or chain. Typically here the branch addresses and branch
   // pointers of the tree will be set.
   // It is normally not necessary to make changes to the generated
   // code, but the routine can be extended by the user if needed.
   // Init() will be called many times when running on PROOF
   // (once per file to be processed).

   // Set branch addresses and branch pointers
   if (!tree) return;
   fChain = tree;
   fCurrent = -1;
   fChain->SetMakeClass(1);

   fChain->SetBranchAddress("Elosshibg", &Elosshibg, &b_Elosshibg);
   fChain->SetBranchAddress("Rangelowbg", &Rangelowbg, &b_Rangelowbg);
   fChain->SetBranchAddress("Rangehibg", &Rangehibg, &b_Rangehibg);
   fChain->SetBranchAddress("Thetabg", &Thetabg, &b_Thetabg);
   fChain->SetBranchAddress("Phibg", &Phibg, &b_Phibg);
   fChain->SetBranchAddress("Av_elossbg", &Av_elossbg, &b_Av_elossbg);
   fChain->SetBranchAddress("Goodfitbg", &Goodfitbg, &b_Goodfitbg);
   fChain->SetBranchAddress("Vertexbg", &Vertexbg, &b_Vertexbg);
   fChain->SetBranchAddress("ContaBg", &ContaBg, &b_ContaBg);
   Notify();
}

Bool_t anaClassBg::Notify()
{
   // The Notify() function is called when a new file is opened. This
   // can be either for a new TTree in a TChain or when when a new TTree
   // is started when using PROOF. It is normally not necessary to make changes
   // to the generated code, but the routine can be extended by the
   // user if needed. The return value is currently not used.

   return kTRUE;
}

void anaClassBg::Show(Long64_t entry)
{
// Print contents of entry.
// If entry is not specified, print current entry
   if (!fChain) return;
   fChain->Show(entry);
}
Int_t anaClassBg::Cut(Long64_t entry)
{
// This function may be called from Loop.
// returns  1 if entry is accepted.
// returns -1 otherwise.
   return 1;
}
#endif // #ifdef anaClassBg_cxx
