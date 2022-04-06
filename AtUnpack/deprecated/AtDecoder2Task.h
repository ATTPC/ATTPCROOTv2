
#ifndef _AtDECODER2TASK_H_
#define _AtDECODER2TASK_H_

#include <FairTask.h>
#include <FairLogger.h>

#include "AtCore2.h"
#include "AtTpcMap.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"

#include "AtDigiPar.h"

// ROOT classes
#include <TClonesArray.h>
#include <TString.h>

// STL
#include <vector>

using std::vector;

class AtDecoder2Task : public FairTask {
private:
   AtCore2 *fDecoder; ///< AtConverter pointer

   vector<TString> fDataList[40]; ///< Raw data file list
   Int_t fDataNum;                ///< Set which number in data list to be decoded

   Double_t fFPNPedestalRMS; ///< RMS cut of baseline matching part selection

   Bool_t fExternalNumTbs; ///< Flag for checking if the number of time buckets is set by the user.
   Int_t fNumTbs;          ///< The number of time buckets

   Bool_t fIsPersistence; ///< Persistence check variable

   std::shared_ptr<AtMap> fMap;

   AtDigiPar *fPar;              ///< Parameter read-out class pointer
   TClonesArray *fRawEventArray; ///< STRawEvent container
   AtRawEvent *fRawEvent;        ///< Current raw event for run

   Bool_t fOldData;         ///< Set to decode old data
   Bool_t fIsSeparatedData; ///< Set to use separated data files
   Bool_t fIsPositive;
   Bool_t fIsPseudoTopology;

   Long64_t fEventIDLast; ///< Last event ID
   Long64_t fEventID;     ///< Event ID for STSource

   Long64_t fInternalID;

   Int_t fNumCobo;
   Int_t fMask;

public:
   /// Constructor
   AtDecoder2Task();
   /// Destructor
   ~AtDecoder2Task();

   /// Setting the number of time buckets used when taking data
   void SetNumTbs(Int_t numTbs);
   /// Adding raw data file to the list
   void AddData(TString filename, Int_t coboIdx = 0);
   /// Setting which data to be decoded
   void SetData(Int_t value);
   /// Setting the FPN RMS value cut for pedestal region taking
   void SetFPNPedestal(Double_t rms);
   /// Setting use gain calibration data file. If there's no file specified by user using two methods below, it'll use
   /// the one in parameter files.
   // void SetUseGainCalibration(Bool_t value = kTRUE);
   /// Setting gain calibration data file. If not set, gain is not calibrated.
   // void SetGainCalibrationData(TString filename);
   /// Setting gain calibration reference.
   // void SetGainReference(Double_t constant, Double_t linear, Double_t quadratic = 0.);
   /// Setting to decode old data file
   void SetOldData(Bool_t oldData = kTRUE);
   /// Setting to use not merged data files
   void SetUseSeparatedData(Bool_t value = kTRUE);
   /// Setting event id for STSource
   void SetEventID(Long64_t eventid = -1);

   void SetMap(std::shared_ptr<AtMap> map) { fMap = map; }
   void SetPseudoTopologyFrame(Bool_t value);
   void SetPTFMask(Int_t mask);

   Long64_t GetEventID();

   /// If set, decoded raw data is written in ROOT file with AtRawEvent class.
   void SetPersistence(Bool_t value = kTRUE);
   void SetPositivePolarity(Bool_t value = kFALSE);

   /// Initializing the task. This will be called when Init() method invoked from FairRun.
   virtual InitStatus Init();
   /// Setting parameter containers. This will be called inbetween Init() and Run().
   virtual void SetParContainers();
   /// Running the task. This will be called when Run() method invoked from FairRun.
   virtual void Exec(Option_t *opt);
   /// Finishing the event.
   virtual void FinishEvent();

   /// Read event for AtSource
   Int_t ReadEvent(Int_t eventID);

   void SetNumCobo(Int_t numCobo);

   ClassDef(AtDecoder2Task, 2);
};

#endif
