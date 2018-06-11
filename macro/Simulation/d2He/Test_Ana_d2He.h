//////////////////////////////////////////////////////////
// This class has been automatically generated on
// Fri Sep  9 12:01:24 2016 by ROOT version 6.06/02
// from TChain cbmsim/
//////////////////////////////////////////////////////////

#ifndef Test_Ana_d2He_h
#define Test_Ana_d2He_h

#include <TROOT.h>
#include <TChain.h>
#include <TFile.h>

// Header file for the classes stored in the TTree if any.
#include "TClonesArray.h"
#include "TObject.h"
#include "invalid"
#include "FairMultiLinkedData_Interface.h"
#include "invalid"
#include "invalid"
#include "invalid"
#include "TNamed.h"
#include "TGeoAtt.h"
#include "TAttLine.h"
#include "TAttMarker.h"
#include "TVirtualGeoTrack.h"
#include "TGeoTrack.h"

class Test_Ana_d2He {
public :
   TTree          *fChain;   //!pointer to the analyzed TTree or TChain
   Int_t           fCurrent; //!current Tree number in a TChain

// Fixed size dimensions of array or collections stored in the TTree if any.
   const Int_t kMaxcbmroot_Stack_MCTrack = 4;
   const Int_t kMaxcbmroot_AtTpc_AtTpcPoint = 742;
   const Int_t kMaxcbmroot_Event_MCEventHeader = 1;
   const Int_t kMaxcbmroot_MCGeoTrack_GeoTracks = 4;

   // Declaration of leaf types
   Int_t           MCTrack_;
   UInt_t          MCTrack_fUniqueID[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   UInt_t          MCTrack_fBits[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Int_t           MCTrack_fPdgCode[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Int_t           MCTrack_fMotherId[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fPx[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fPy[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fPz[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fStartX[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fStartY[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fStartZ[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Double32_t      MCTrack_fStartT[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Int_t           MCTrack_fNPoints[kMaxcbmroot_Stack_MCTrack];   //[cbmroot.Stack.MCTrack_]
   Int_t           AtTpcPoint_;
   UInt_t          AtTpcPoint_fUniqueID[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   UInt_t          AtTpcPoint_fBits[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Int_t           AtTpcPoint_fTrackID[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   UInt_t          AtTpcPoint_fEventId[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPx[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPy[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPz[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fTime[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fLength[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fELoss[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Int_t           AtTpcPoint_fDetectorID[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fX[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fY[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fZ[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fX_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fY_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fZ_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPx_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPy_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double32_t      AtTpcPoint_fPz_out[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Int_t           AtTpcPoint_fDetCopyID[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   TString         AtTpcPoint_fVolName[kMaxcbmroot_AtTpc_AtTpcPoint];
   Double_t        AtTpcPoint_fEnergyIni[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Double_t        AtTpcPoint_fAngleIni[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Int_t           AtTpcPoint_fAiso[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   Int_t           AtTpcPoint_fZiso[kMaxcbmroot_AtTpc_AtTpcPoint];   //[cbmroot.AtTpc.AtTpcPoint_]
   FairMCEventHeader *MCEventHeader_;
   UInt_t          MCEventHeader_TNamed_fUniqueID;
   UInt_t          MCEventHeader_TNamed_fBits;
   TString         MCEventHeader_TNamed_fName;
   TString         MCEventHeader_TNamed_fTitle;
   UInt_t          MCEventHeader_fRunId;
   UInt_t          MCEventHeader_fEventId;
   Double32_t      MCEventHeader_fX;
   Double32_t      MCEventHeader_fY;
   Double32_t      MCEventHeader_fZ;
   Double32_t      MCEventHeader_fT;
   Double32_t      MCEventHeader_fB;
   Int_t           MCEventHeader_fNPrim;
   Bool_t          MCEventHeader_fIsSet;
   Double32_t      MCEventHeader_fRotX;
   Double32_t      MCEventHeader_fRotY;
   Double32_t      MCEventHeader_fRotZ;
   Int_t           GeoTracks_;
   UInt_t          GeoTracks_fUniqueID[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   UInt_t          GeoTracks_fBits[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   UInt_t          GeoTracks_fGeoAtt[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Short_t         GeoTracks_fLineColor[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Short_t         GeoTracks_fLineStyle[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Short_t         GeoTracks_fLineWidth[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Short_t         GeoTracks_fMarkerColor[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Short_t         GeoTracks_fMarkerStyle[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Float_t         GeoTracks_fMarkerSize[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Int_t           GeoTracks_fPDG[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Int_t           GeoTracks_fId[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Int_t           GeoTracks_fPointsSize[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Int_t           GeoTracks_fNpoints[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot.MCGeoTrack.GeoTracks_]
   Double_t       *GeoTracks_fPoints[kMaxcbmroot_MCGeoTrack_GeoTracks];   //[cbmroot_MCGeoTrack_GeoTracks_fNpoints]

   // List of branches
   TBranch        *b_cbmroot_Stack_MCTrack_;   //!
   TBranch        *b_MCTrack_fUniqueID;   //!
   TBranch        *b_MCTrack_fBits;   //!
   TBranch        *b_MCTrack_fPdgCode;   //!
   TBranch        *b_MCTrack_fMotherId;   //!
   TBranch        *b_MCTrack_fPx;   //!
   TBranch        *b_MCTrack_fPy;   //!
   TBranch        *b_MCTrack_fPz;   //!
   TBranch        *b_MCTrack_fStartX;   //!
   TBranch        *b_MCTrack_fStartY;   //!
   TBranch        *b_MCTrack_fStartZ;   //!
   TBranch        *b_MCTrack_fStartT;   //!
   TBranch        *b_MCTrack_fNPoints;   //!
   TBranch        *b_cbmroot_AtTpc_AtTpcPoint_;   //!
   TBranch        *b_AtTpcPoint_fUniqueID;   //!
   TBranch        *b_AtTpcPoint_fBits;   //!
   TBranch        *b_AtTpcPoint_fTrackID;   //!
   TBranch        *b_AtTpcPoint_fEventId;   //!
   TBranch        *b_AtTpcPoint_fPx;   //!
   TBranch        *b_AtTpcPoint_fPy;   //!
   TBranch        *b_AtTpcPoint_fPz;   //!
   TBranch        *b_AtTpcPoint_fTime;   //!
   TBranch        *b_AtTpcPoint_fLength;   //!
   TBranch        *b_AtTpcPoint_fELoss;   //!
   TBranch        *b_AtTpcPoint_fDetectorID;   //!
   TBranch        *b_AtTpcPoint_fX;   //!
   TBranch        *b_AtTpcPoint_fY;   //!
   TBranch        *b_AtTpcPoint_fZ;   //!
   TBranch        *b_AtTpcPoint_fX_out;   //!
   TBranch        *b_AtTpcPoint_fY_out;   //!
   TBranch        *b_AtTpcPoint_fZ_out;   //!
   TBranch        *b_AtTpcPoint_fPx_out;   //!
   TBranch        *b_AtTpcPoint_fPy_out;   //!
   TBranch        *b_AtTpcPoint_fPz_out;   //!
   TBranch        *b_AtTpcPoint_fDetCopyID;   //!
   TBranch        *b_AtTpcPoint_fVolName;   //!
   TBranch        *b_AtTpcPoint_fEnergyIni;   //!
   TBranch        *b_AtTpcPoint_fAngleIni;   //!
   TBranch        *b_AtTpcPoint_fAiso;   //!
   TBranch        *b_AtTpcPoint_fZiso;   //!
   TBranch        *b_cbmroot_Event_MCEventHeader_;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fUniqueID;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fBits;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fName;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fTitle;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fRunId;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fEventId;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fX;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fY;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fZ;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fT;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fB;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fNPrim;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fIsSet;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotX;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotY;   //!
   TBranch        *b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotZ;   //!
   TBranch        *b_cbmroot_MCGeoTrack_GeoTracks_;   //!
   TBranch        *b_GeoTracks_fUniqueID;   //!
   TBranch        *b_GeoTracks_fBits;   //!
   TBranch        *b_GeoTracks_fGeoAtt;   //!
   TBranch        *b_GeoTracks_fLineColor;   //!
   TBranch        *b_GeoTracks_fLineStyle;   //!
   TBranch        *b_GeoTracks_fLineWidth;   //!
   TBranch        *b_GeoTracks_fMarkerColor;   //!
   TBranch        *b_GeoTracks_fMarkerStyle;   //!
   TBranch        *b_GeoTracks_fMarkerSize;   //!
   TBranch        *b_GeoTracks_fPDG;   //!
   TBranch        *b_GeoTracks_fId;   //!
   TBranch        *b_GeoTracks_fPointsSize;   //!
   TBranch        *b_GeoTracks_fNpoints;   //!
   TBranch        *b_GeoTracks_fPoints;   //!

   Test_Ana_d2He(TTree *tree=0);
   virtual ~Test_Ana_d2He();
   virtual Int_t    Cut(Long64_t entry);
   virtual Int_t    GetEntry(Long64_t entry);
   virtual Long64_t LoadTree(Long64_t entry);
   virtual void     Init(TTree *tree);
   virtual void     Loop();
   virtual Bool_t   Notify();
   virtual void     Show(Long64_t entry = -1);
};

#endif

#ifdef Test_Ana_d2He_cxx
Test_Ana_d2He::Test_Ana_d2He(TTree *tree) : fChain(0) 
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
      f->GetObject("cbmsim",tree);

#else // SINGLE_TREE

      // The following code should be used if you want this class to access a chain
      // of trees.
      TChain * chain = new TChain("cbmsim","");
      chain->Add("data/attpcsim_proto.root/cbmsim");
      tree = chain;
#endif // SINGLE_TREE

   }
   Init(tree);
}

Test_Ana_d2He::~Test_Ana_d2He()
{
   if (!fChain) return;
   delete fChain->GetCurrentFile();
}

Int_t Test_Ana_d2He::GetEntry(Long64_t entry)
{
// Read contents of entry.
   if (!fChain) return 0;
   return fChain->GetEntry(entry);
}
Long64_t Test_Ana_d2He::LoadTree(Long64_t entry)
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

void Test_Ana_d2He::Init(TTree *tree)
{
   // The Init() function is called when the selector needs to initialize
   // a new tree or chain. Typically here the branch addresses and branch
   // pointers of the tree will be set.
   // It is normally not necessary to make changes to the generated
   // code, but the routine can be extended by the user if needed.
   // Init() will be called many times when running on PROOF
   // (once per file to be processed).

   // Set object pointer
   MCEventHeader_ = 0;
   // Set array pointer
   for(int i=0; i<kMaxcbmroot_MCGeoTrack_GeoTracks; ++i) GeoTracks_fPoints[i] = 0;

   // Set branch addresses and branch pointers
   if (!tree) return;
   fChain = tree;
   fCurrent = -1;
   fChain->SetMakeClass(1);

   fChain->SetBranchAddress("MCTrack", &MCTrack_, &b_cbmroot_Stack_MCTrack_);
   fChain->SetBranchAddress("MCTrack.fUniqueID", MCTrack_fUniqueID, &b_MCTrack_fUniqueID);
   fChain->SetBranchAddress("MCTrack.fBits", MCTrack_fBits, &b_MCTrack_fBits);
   fChain->SetBranchAddress("MCTrack.fPdgCode", MCTrack_fPdgCode, &b_MCTrack_fPdgCode);
   fChain->SetBranchAddress("MCTrack.fMotherId", MCTrack_fMotherId, &b_MCTrack_fMotherId);
   fChain->SetBranchAddress("MCTrack.fPx", MCTrack_fPx, &b_MCTrack_fPx);
   fChain->SetBranchAddress("MCTrack.fPy", MCTrack_fPy, &b_MCTrack_fPy);
   fChain->SetBranchAddress("MCTrack.fPz", MCTrack_fPz, &b_MCTrack_fPz);
   fChain->SetBranchAddress("MCTrack.fStartX", MCTrack_fStartX, &b_MCTrack_fStartX);
   fChain->SetBranchAddress("MCTrack.fStartY", MCTrack_fStartY, &b_MCTrack_fStartY);
   fChain->SetBranchAddress("MCTrack.fStartZ", MCTrack_fStartZ, &b_MCTrack_fStartZ);
   fChain->SetBranchAddress("MCTrack.fStartT", MCTrack_fStartT, &b_MCTrack_fStartT);
   fChain->SetBranchAddress("MCTrack.fNPoints", MCTrack_fNPoints, &b_MCTrack_fNPoints);
   fChain->SetBranchAddress("AtTpcPoint", &AtTpcPoint_, &b_cbmroot_AtTpc_AtTpcPoint_);
   fChain->SetBranchAddress("AtTpcPoint.fUniqueID", AtTpcPoint_fUniqueID, &b_AtTpcPoint_fUniqueID);
   fChain->SetBranchAddress("AtTpcPoint.fBits", AtTpcPoint_fBits, &b_AtTpcPoint_fBits);
   fChain->SetBranchAddress("AtTpcPoint.fTrackID", AtTpcPoint_fTrackID, &b_AtTpcPoint_fTrackID);
   fChain->SetBranchAddress("AtTpcPoint.fEventId", AtTpcPoint_fEventId, &b_AtTpcPoint_fEventId);
   fChain->SetBranchAddress("AtTpcPoint.fPx", AtTpcPoint_fPx, &b_AtTpcPoint_fPx);
   fChain->SetBranchAddress("AtTpcPoint.fPy", AtTpcPoint_fPy, &b_AtTpcPoint_fPy);
   fChain->SetBranchAddress("AtTpcPoint.fPz", AtTpcPoint_fPz, &b_AtTpcPoint_fPz);
   fChain->SetBranchAddress("AtTpcPoint.fTime", AtTpcPoint_fTime, &b_AtTpcPoint_fTime);
   fChain->SetBranchAddress("AtTpcPoint.fLength", AtTpcPoint_fLength, &b_AtTpcPoint_fLength);
   fChain->SetBranchAddress("AtTpcPoint.fELoss", AtTpcPoint_fELoss, &b_AtTpcPoint_fELoss);
   fChain->SetBranchAddress("AtTpcPoint.fDetectorID", AtTpcPoint_fDetectorID, &b_AtTpcPoint_fDetectorID);
   fChain->SetBranchAddress("AtTpcPoint.fX", AtTpcPoint_fX, &b_AtTpcPoint_fX);
   fChain->SetBranchAddress("AtTpcPoint.fY", AtTpcPoint_fY, &b_AtTpcPoint_fY);
   fChain->SetBranchAddress("AtTpcPoint.fZ", AtTpcPoint_fZ, &b_AtTpcPoint_fZ);
   fChain->SetBranchAddress("AtTpcPoint.fX_out", AtTpcPoint_fX_out, &b_AtTpcPoint_fX_out);
   fChain->SetBranchAddress("AtTpcPoint.fY_out", AtTpcPoint_fY_out, &b_AtTpcPoint_fY_out);
   fChain->SetBranchAddress("AtTpcPoint.fZ_out", AtTpcPoint_fZ_out, &b_AtTpcPoint_fZ_out);
   fChain->SetBranchAddress("AtTpcPoint.fPx_out", AtTpcPoint_fPx_out, &b_AtTpcPoint_fPx_out);
   fChain->SetBranchAddress("AtTpcPoint.fPy_out", AtTpcPoint_fPy_out, &b_AtTpcPoint_fPy_out);
   fChain->SetBranchAddress("AtTpcPoint.fPz_out", AtTpcPoint_fPz_out, &b_AtTpcPoint_fPz_out);
   fChain->SetBranchAddress("AtTpcPoint.fDetCopyID", AtTpcPoint_fDetCopyID, &b_AtTpcPoint_fDetCopyID);
   fChain->SetBranchAddress("AtTpcPoint.fVolName", AtTpcPoint_fVolName, &b_AtTpcPoint_fVolName);
   fChain->SetBranchAddress("AtTpcPoint.fEnergyIni", AtTpcPoint_fEnergyIni, &b_AtTpcPoint_fEnergyIni);
   fChain->SetBranchAddress("AtTpcPoint.fAngleIni", AtTpcPoint_fAngleIni, &b_AtTpcPoint_fAngleIni);
   fChain->SetBranchAddress("AtTpcPoint.fAiso", AtTpcPoint_fAiso, &b_AtTpcPoint_fAiso);
   fChain->SetBranchAddress("AtTpcPoint.fZiso", AtTpcPoint_fZiso, &b_AtTpcPoint_fZiso);
   fChain->SetBranchAddress("MCEventHeader.", &MCEventHeader_, &b_cbmroot_Event_MCEventHeader_);
   fChain->SetBranchAddress("MCEventHeader.TNamed.fUniqueID", &MCEventHeader_TNamed_fUniqueID, &b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fUniqueID);
   fChain->SetBranchAddress("MCEventHeader.TNamed.fBits", &MCEventHeader_TNamed_fBits, &b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fBits);
   fChain->SetBranchAddress("MCEventHeader.TNamed.fName", &MCEventHeader_TNamed_fName, &b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fName);
   fChain->SetBranchAddress("MCEventHeader.TNamed.fTitle", &MCEventHeader_TNamed_fTitle, &b_MCEventHeader_cbmroot_Event_MCEventHeader_TNamed_fTitle);
   fChain->SetBranchAddress("MCEventHeader.fRunId", &MCEventHeader_fRunId, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fRunId);
   fChain->SetBranchAddress("MCEventHeader.fEventId", &MCEventHeader_fEventId, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fEventId);
   fChain->SetBranchAddress("MCEventHeader.fX", &MCEventHeader_fX, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fX);
   fChain->SetBranchAddress("MCEventHeader.fY", &MCEventHeader_fY, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fY);
   fChain->SetBranchAddress("MCEventHeader.fZ", &MCEventHeader_fZ, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fZ);
   fChain->SetBranchAddress("MCEventHeader.fT", &MCEventHeader_fT, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fT);
   fChain->SetBranchAddress("MCEventHeader.fB", &MCEventHeader_fB, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fB);
   fChain->SetBranchAddress("MCEventHeader.fNPrim", &MCEventHeader_fNPrim, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fNPrim);
   fChain->SetBranchAddress("MCEventHeader.fIsSet", &MCEventHeader_fIsSet, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fIsSet);
   fChain->SetBranchAddress("MCEventHeader.fRotX", &MCEventHeader_fRotX, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotX);
   fChain->SetBranchAddress("MCEventHeader.fRotY", &MCEventHeader_fRotY, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotY);
   fChain->SetBranchAddress("MCEventHeader.fRotZ", &MCEventHeader_fRotZ, &b_MCEventHeader_cbmroot_Event_MCEventHeader_fRotZ);
   fChain->SetBranchAddress("GeoTracks", &GeoTracks_, &b_cbmroot_MCGeoTrack_GeoTracks_);
   fChain->SetBranchAddress("GeoTracks.fUniqueID", GeoTracks_fUniqueID, &b_GeoTracks_fUniqueID);
   fChain->SetBranchAddress("GeoTracks.fBits", GeoTracks_fBits, &b_GeoTracks_fBits);
   fChain->SetBranchAddress("GeoTracks.fGeoAtt", GeoTracks_fGeoAtt, &b_GeoTracks_fGeoAtt);
   fChain->SetBranchAddress("GeoTracks.fLineColor", GeoTracks_fLineColor, &b_GeoTracks_fLineColor);
   fChain->SetBranchAddress("GeoTracks.fLineStyle", GeoTracks_fLineStyle, &b_GeoTracks_fLineStyle);
   fChain->SetBranchAddress("GeoTracks.fLineWidth", GeoTracks_fLineWidth, &b_GeoTracks_fLineWidth);
   fChain->SetBranchAddress("GeoTracks.fMarkerColor", GeoTracks_fMarkerColor, &b_GeoTracks_fMarkerColor);
   fChain->SetBranchAddress("GeoTracks.fMarkerStyle", GeoTracks_fMarkerStyle, &b_GeoTracks_fMarkerStyle);
   fChain->SetBranchAddress("GeoTracks.fMarkerSize", GeoTracks_fMarkerSize, &b_GeoTracks_fMarkerSize);
   fChain->SetBranchAddress("GeoTracks.fPDG", GeoTracks_fPDG, &b_GeoTracks_fPDG);
   fChain->SetBranchAddress("GeoTracks.fId", GeoTracks_fId, &b_GeoTracks_fId);
   fChain->SetBranchAddress("GeoTracks.fPointsSize", GeoTracks_fPointsSize, &b_GeoTracks_fPointsSize);
   fChain->SetBranchAddress("GeoTracks.fNpoints", GeoTracks_fNpoints, &b_GeoTracks_fNpoints);
   fChain->SetBranchAddress("GeoTracks.fPoints", GeoTracks_fPoints, &b_GeoTracks_fPoints);
   Notify();
}

Bool_t Test_Ana_d2He::Notify()
{
   // The Notify() function is called when a new file is opened. This
   // can be either for a new TTree in a TChain or when when a new TTree
   // is started when using PROOF. It is normally not necessary to make changes
   // to the generated code, but the routine can be extended by the
   // user if needed. The return value is currently not used.

   return kTRUE;
}

void Test_Ana_d2He::Show(Long64_t entry)
{
// Print contents of entry.
// If entry is not specified, print current entry
   if (!fChain) return;
   fChain->Show(entry);
}
Int_t Test_Ana_d2He::Cut(Long64_t entry)
{
// This function may be called from Loop.
// returns  1 if entry is accepted.
// returns -1 otherwise.
   return 1;
}
#endif // #ifdef Test_Ana_d2He_cxx
