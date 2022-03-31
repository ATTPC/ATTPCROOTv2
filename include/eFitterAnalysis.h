#include <ncurses.h>

#include "AtFitter.h"
#include "AtGenfit.h"
#include "AtFormat.h"
#include "AtParsers.h"
#include "AtKinematics.h"
#include "AtVirtualTerminal.h"
#include "AtTpcPoint.h"
#include "AtEvent.h"
#include "AtPad.h"
#include "AtHit.h"
#include "AtTrack.h"
#include "AtPatternEvent.h"

#include "FairRootManager.h"
#include "FairLogger.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include <memory>
#include <chrono>
#include <thread>
#include <thread>
#include <future>
#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>
#include <sstream>


#include "AbsKalmanFitter.h"
#include "KalmanFitterRefTrack.h"
#include "DAF.h"
#include "ConstField.h"
#include "FieldManager.h"
#include "MaterialEffects.h"
#include "TGeoMaterialInterface.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"
#include "EventDisplay.h"
#include "KalmanFitStatus.h"
#include "FitStatus.h"
#include "AbsFitterInfo.h"
#include "KalmanFitterInfo.h"
#include "MeasuredStateOnPlane.h"
#include "MeasurementOnPlane.h"
#include "TrackPoint.h"
#include "Exception.h"


#include "TClonesArray.h"
#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TTreeReader.h"
#include "TTreePlayer.h"
#include "TTreeReaderValue.h"
#include "TSystem.h"
#include "TH1F.h"
#include "TH2F.h"
#include "TCanvas.h"
#include "TStopwatch.h"
#include "TGeoManager.h"
#include "TSpectrum.h"


#include <boost/filesystem.hpp>

class FitManager{

public:
  FitManager();
  FitManager(std::string list);
  ~FitManager();

  //Event display
  Bool_t EnableGenfitDisplay();

  //Setters
  Bool_t SetFitters(Bool_t simConv);
  Bool_t SetGeometry(TString file,Float_t field,Float_t density);  
  Bool_t SetInputFile(TString& file,std::size_t firstEve, std::size_t lastEve);
  Bool_t SetOutputFile(TString& file);

  //Getters
  std::shared_ptr<TTreeReader> GetReader() { return fReader;}
  AtPatternEvent* GetPatternEve() {(AtPatternEvent *)fPatternEveArray->Get()->At(0);}
  AtEvent* GetEve() {(AtEvent *)fEveArray->Get()->At(0);}
  void GetAuxiliaryChannels(std::vector<AtPad> *auxPadArray);

  //File management
  void ClearTree();
  void FillTree() {fOutputTree->Fill();}
  void WriteTree() {fOutputTree->Write();}
  void ChangeToOutputFile() {fOutputFile->cd();}
  void CloseOutputFile() {fOutputFile->Close();}

  //Fit management
  Bool_t FitTracks(std::vector<AtTrack> &tracks);
  
  //TODO: Move to tools and AtFitter
  Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test);
  Double_t GetMaximum(double *adc);
  void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius);
  
  
private:

  Int_t fVerbosity;
  Bool_t fSimulationConv;
  Float_t fMagneticField;
  Float_t fGasDensity;
  std::vector<AtTools::IonFitInfo>* ionList;
  AtTools::AtParsers fParser;
  
  genfit::EventDisplay *display;

  TString fWorkDir;
  std::shared_ptr<TTreeReader> fReader;
  std::shared_ptr<TFile> fInputFile;
  std::shared_ptr<TFile> fOutputFile;
  std::shared_ptr<TTree> fOutputTree;
  std::shared_ptr<TTreeReaderValue<TClonesArray>> fPatternEveArray;
  std::shared_ptr<TTreeReaderValue<TClonesArray>> fEveArray;
  std::vector<AtFITTER::AtFitter*> fFitters;
  AtFITTER::AtFitter* fFitter;
  
public:
  //Output tree format (TODO: To be moved to other src file)
  // Output file
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
  
  
};  
