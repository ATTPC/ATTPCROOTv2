
#ifndef _AtDECODERSpecMATTASK_H_
#define _AtDECODERSpecMATTASK_H_

#include "FairLogger.h"
#include "FairTask.h"

#include "AtCoreSpecMAT.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"
#include "AtSpecMATMap.h"

#include "AtDigiPar.h"

// ROOT classes
#include "TClonesArray.h"
#include "TString.h"

// STL
#include <vector>

using std::vector;

class AtDecoderSpecMATTask : public FairTask {

private:
   std::unique_ptr<AtCoreSpecMAT> fDecoder; ///< AtConverter pointer

   vector<TString> fDataList[40]; ///< Raw data file list
   Int_t fDataNum;                ///< Set which number in data list to be decoded

   Double_t fFPNPedestalRMS; ///< RMS cut of baseline matching part selection

   Bool_t fExternalNumTbs; ///< Flag for checking if the number of time buckets
                           ///< is set by the user.
   Int_t fNumTbs;          ///< The number of time buckets

   Bool_t fIsPersistence; ///< Persistence check variable

   /* Old map style
   Int_t fOpt;
   TString fGeoFile;
   TString fProtoMapFile;
   Char_t const *fMap;
   TString fIniMap;
   TString fLowgMap;
   TString fXtalkMap;
   std::vector<Int_t> fAuxChannels; /// Auxiliary channels in GET electronics
   */
   // New map style
   std::shared_ptr<AtSpecMATMap> fMap;

   AtDigiPar *fPar;              ///< Parameter read-out class pointer
   TClonesArray *fRawEventArray; ///< STRawEvent container
   AtRawEvent *fRawEvent;        ///< Current raw event for run

   Bool_t fOldData;         ///< Set to decode old data
   Bool_t fIsSeparatedData; ///< Set to use separated data files
   Bool_t fIsPositive[4];
   Bool_t fIsPseudoTopology;

   Long64_t fEventIDLast; ///< Last event ID
   Long64_t fEventID;     ///< Event ID for STSource

   Long64_t fInternalID;

   Int_t fNumCobo;
   Bool_t fIsCoboPadPlane[4];
   Int_t fMask;

public:
   /// Constructor
   AtDecoderSpecMATTask();
   /// Destructor
   ~AtDecoderSpecMATTask() = default;

   /// Setting the number of time buckets used when taking data
   void SetNumTbs(Int_t numTbs);
   /// Adding raw data file to the list
   void AddData(TString filename, Int_t coboIdx = 0);
   /// Setting which data to be decoded
   void SetData(Int_t value);
   /// Setting the FPN RMS value cut for pedestal region taking
   void SetFPNPedestal(Double_t rms);
   /// Setting to decode old data file
   void SetOldData(Bool_t oldData = kTRUE);
   /// Setting to use not merged data files
   void SetUseSeparatedData(Bool_t value = kTRUE);
   /// Setting event id for STSource
   void SetEventID(Long64_t eventid = -1);

   /* old map style
   void SetProtoMap(TString mapfile); // only for prototype
   void SetMap(Char_t const *map);
   void SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap);
   void SetGeo(TString geofile); // only for prototype
   void SetMapOpt(Int_t value);
   */
   // New map style
   void SetMap(std::shared_ptr<AtSpecMATMap> map) { fMap = map; }
   void SetPseudoTopologyFrame(Bool_t value);

   void SetPTFMask(Int_t mask);

   Long64_t GetEventID();

   /// If set, decoded raw data is written in ROOT file with AtRawEvent class.
   void SetPersistence(Bool_t value = kTRUE);
   void SetPositivePolarity(Bool_t *value);

   /// Initializing the task. This will be called when Init() method invoked from
   /// FairRun.
   virtual InitStatus Init();
   /// Setting parameter containers. This will be called inbetween Init() and
   /// Run().
   virtual void SetParContainers();
   /// Running the task. This will be called when Run() method invoked from
   /// FairRun.
   virtual void Exec(Option_t *opt);
   /// Finishing the event.
   virtual void FinishEvent();

   /// Read event for AtSource
   Int_t ReadEvent(Int_t eventID);

   // Setting Auxiliary channels
   void SetAuxChannels(std::vector<Int_t> AuxCh);

   void SetNumCobo(Int_t numCobo);
   void SetIsCoboPadPlane(Bool_t *IsPadPlane);

   ClassDef(AtDecoderSpecMATTask, 1);
};

#endif
