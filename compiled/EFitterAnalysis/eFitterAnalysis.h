#include "AtEvent.h"
#include "AtFitter.h"
#include "AtFormat.h"
#include "AtGenfit.h"
#include "AtHit.h"
#include "AtKinematics.h"
#include "AtMCPoint.h"
#include "AtPad.h"
#include "AtParsers.h"
#include "AtPatternEvent.h"
#include "AtTrack.h"
#include "AtTrackTransformer.h"
#include "AtVirtualTerminal.h"

#include <boost/filesystem.hpp>

#include "AbsFitterInfo.h"
#include "AbsKalmanFitter.h"
#include "ConstField.h"
#include "DAF.h"
#include "EventDisplay.h"
#include "Exception.h"
#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FieldManager.h"
#include "FitStatus.h"
#include "KalmanFitStatus.h"
#include "KalmanFitterInfo.h"
#include "KalmanFitterRefTrack.h"
#include "MaterialEffects.h"
#include "MeasuredStateOnPlane.h"
#include "MeasurementFactory.h"
#include "MeasurementOnPlane.h"
#include "MeasurementProducer.h"
#include "TCanvas.h"
#include "TClonesArray.h"
#include "TFile.h"
#include "TGeoManager.h"
#include "TGeoMaterialInterface.h"
#include "TH1F.h"
#include "TH2F.h"
#include "TSpectrum.h"
#include "TStopwatch.h"
#include "TString.h"
#include "TSystem.h"
#include "TTree.h"
#include "TTreePlayer.h"
#include "TTreeReader.h"
#include "TTreeReaderValue.h"
#include "TrackPoint.h"
#include <ncurses.h>

#include <chrono>
#include <future>
#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

struct trackSegment {
   Double_t eLoss;
   TVector3 iniPos;
   TVector3 deltaMom;
   TVector3 deltaPos;
   Double_t theta;
   Double_t phi;
   UInt_t id;

   friend std::ostream &operator<<(std::ostream &os, const trackSegment &ts);
};

std::ostream &operator<<(std::ostream &os, const trackSegment &ts)
{
   os << "\n";
   os << " Track segment : " << ts.id << " - Momentum:  " << ts.deltaMom.X() << " - " << ts.deltaMom.Y() << " - "
      << ts.deltaMom.Z() << " - Energy Loss : " << ts.eLoss << "\n";
   os << " =============   - Position :  " << ts.iniPos.X() << " - " << ts.iniPos.Y() << " - " << ts.iniPos.Z()
      << " . Mag Dir : " << ts.iniPos.Mag() << "\n";
   os << " =============   - Position direction :  " << ts.deltaPos.X() << " - " << ts.deltaPos.Y() << " - "
      << ts.deltaPos.Z() << " . Mag Dir : " << ts.deltaPos.Mag() << "\n";
   os << " =============   - Theta    :  " << ts.theta * TMath::RadToDeg() << " - Phi : " << ts.phi * TMath::RadToDeg()
      << "\n";
   return os;
}

struct firstOrbit {
   Double_t POCA;
   Double_t Z;
   Double_t phi;
   Double_t length;
   Double_t eLoss;
};

class FitManager {

public:
   FitManager();
   FitManager(std::string list);
   ~FitManager();

   // Event display
   Bool_t EnableGenfitDisplay();
   genfit::EventDisplay *GetEventDisplay() { return display; }

   // Setters
   Bool_t SetFitters(Bool_t simConv);
   Bool_t SetGeometry(TString file, Float_t field, Float_t density);
   Bool_t SetInputFile(TString &file, std::size_t firstEve, std::size_t lastEve);
   Bool_t SetOutputFile(TString &file);
   void SetFitDirection(Int_t direction) { fFitDirection = direction; }

   // Getters
   std::shared_ptr<TTreeReader> GetReader() { return fReader; }
   AtPatternEvent *GetPatternEve() { return (AtPatternEvent *)fPatternEveArray->Get()->At(0); }
   AtEvent *GetEve() { return (AtEvent *)fEveArray->Get()->At(0); }
   void GetAuxiliaryChannels(const std::vector<AtAuxPad> &auxPadArray);

   // File management
   void ClearTree();
   void FillTree() { fOutputTree->Fill(); }
   void WriteTree() { fOutputTree->Write(); }
   void ChangeToOutputFile() { fOutputFile->cd(); }
   void CloseOutputFile() { fOutputFile->Close(); }

   // Fit management
   Bool_t FitTracks(std::vector<AtTrack> &tracks);
   void EnableMerging(Bool_t merging) { fEnableMerging = merging; }
   void EnableSingleVertexTrack(Bool_t singletrack) { fEnableSingleVertexTrack = singletrack; }
  void EnableReclustering(Bool_t reclustering, Double_t clusterRadius,Double_t clusterSize) {fEnableReclustering = reclustering;fClusterRadius=clusterRadius;fClusterSize=clusterSize;}
  
   // TODO: Move to tools and AtFitter
   Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test);
   Double_t GetMaximum(double *adc);

private:
   Int_t fVerbosity;
   Bool_t fSimulationConv;
   Float_t fMagneticField;
   Float_t fGasDensity;
   Bool_t fEnableMerging;
   Bool_t fEnableSingleVertexTrack;
   Bool_t fEnableReclustering;
   Double_t fClusterSize{0};
   Double_t fClusterRadius{0};
   Int_t fFitDirection;
   std::vector<AtTools::IonFitInfo> *ionList;
   AtTools::AtParsers fParser;
   std::unique_ptr<AtTools::AtTrackTransformer> fTrackTransformer;

   genfit::EventDisplay *display;

   TString fWorkDir;
   std::shared_ptr<TTreeReader> fReader;
   std::shared_ptr<TFile> fInputFile;
   std::shared_ptr<TFile> fOutputFile;
   std::shared_ptr<TTree> fOutputTree;
   std::shared_ptr<TTreeReaderValue<TClonesArray>> fPatternEveArray;
   std::shared_ptr<TTreeReaderValue<TClonesArray>> fEveArray;
   std::vector<AtFITTER::AtFitter *> fFitters;
   AtFITTER::AtFitter *fFitter;
   std::shared_ptr<AtTools::AtKinematics> fKinematics;
   firstOrbit GetFirstOrbit(genfit::Track *track, genfit::AbsTrackRep *rep, TVector3 vertex);
   void ConstructTrack(const genfit::StateOnPlane *prevState, const genfit::StateOnPlane *state,
                       const genfit::AbsTrackRep *rep, std::vector<TVector3> &track,
                       std::vector<trackSegment> &segments);
   Bool_t CompareTracks(AtTrack *trA, AtTrack *trB);
   Bool_t CheckOverlap(AtTrack *trA, AtTrack *trB);

public:
   // Output tree format (TODO: To be moved to other src file)
   //  Output file
   Float_t EFit;
   Float_t AFit;
   Float_t PhiFit;
   Float_t EPRA;
   Float_t APRA;
   Float_t PhiPRA;
   Float_t Ex;
   Float_t xiniFit;
   Float_t yiniFit;
   Float_t ziniFit;
   Float_t xiniPRA;
   Float_t yiniPRA;
   Float_t ziniPRA;
   Float_t pVal;
   Float_t IC;
   Float_t EFitXtr;
   Float_t ExXtr;
   Float_t xiniFitXtr;
   Float_t yiniFitXtr;
   Float_t ziniFitXtr;
   Float_t distXtr;
   Float_t trackLength;
   Float_t POCAXtr;
   Int_t trackID;
   Int_t ICMult;
   Int_t particleQ;
   Int_t evMult;
   Int_t praMult;

   std::vector<Float_t> EFitVec;
   std::vector<Float_t> AFitVec;
   std::vector<Float_t> PhiFitVec;
   std::vector<Float_t> EPRAVec;
   std::vector<Float_t> APRAVec;
   std::vector<Float_t> PhiPRAVec;
   std::vector<Float_t> ExVec;
   std::vector<Float_t> xiniFitVec;
   std::vector<Float_t> yiniFitVec;
   std::vector<Float_t> ziniFitVec;
   std::vector<Float_t> xiniPRAVec;
   std::vector<Float_t> yiniPRAVec;
   std::vector<Float_t> ziniPRAVec;
   std::vector<Float_t> pValVec;
   std::vector<Float_t> ICVec;
   std::vector<Int_t> ICTimeVec;
   std::vector<Float_t> EFitXtrVec;
   std::vector<Float_t> ExXtrVec;
   std::vector<Float_t> xiniFitXtrVec;
   std::vector<Float_t> yiniFitXtrVec;
   std::vector<Float_t> ziniFitXtrVec;
   std::vector<Float_t> distXtrVec;
   std::vector<Float_t> trackLengthVec;
   std::vector<Float_t> POCAXtrVec;
   std::vector<Int_t> trackIDVec;
   std::vector<Float_t> fChi2Vec;
   std::vector<Float_t> bChi2Vec;
   std::vector<Float_t> fNdfVec;
   std::vector<Float_t> bNdfVec;
   std::vector<Float_t> ICEVec;
   std::vector<Int_t> particleQVec;
   std::vector<Float_t> POCAOrbZVec;
   std::vector<Float_t> firstOrbZVec;
   std::vector<Float_t> phiOrbZVec;
   std::vector<Float_t> lengthOrbZVec;
   std::vector<Float_t> eLossOrbZVec;
   std::vector<Float_t> brhoVec;
   std::vector<Float_t> eLossADC;
   std::vector<Float_t> dEdxADC;
   std::vector<std::string> pdgVec;
   std::vector<Int_t> trackPointsVec;
   std::vector<Bool_t> fitConvergedVec;
};
