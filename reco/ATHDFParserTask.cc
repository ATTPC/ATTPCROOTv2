#include "ATHDFParserTask.hh"

#include "FairRootManager.h"
#include "FairRunOnline.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHDFParserTask);

ATHDFParserTask::ATHDFParserTask():
  AtPadCoordArr(boost::extents[10240][3][2])
{
  fLogger = FairLogger::GetLogger();
  fIsPersistence = kFALSE;
  fRawEventArray = new TClonesArray("ATRawEvent");
  fEventID = 0;
  fIniEventID = 0;
  fTimestampIndex = 1;
  fRawEvent = new ATRawEvent();
  fIsOldFormat  = kFALSE;
  fIsBaseLineSubtraction = kFALSE;  
  kOpt = 0;
  
  fIsProtoGeoSet = kFALSE;
  fIsProtoMapSet = kFALSE;
  if(kOpt==0) fAtMapPtr = new AtTpcMap();
  else if(kOpt==1) fAtMapPtr = new AtTpcProtoMap();
  else std::cout << "== ATHDFParserTask Initialization Error : Option not found. Current available options: ATTPC Map 0 / Prototype Map 1" << std::endl;
  
}

ATHDFParserTask::ATHDFParserTask(Int_t opt):
  AtPadCoordArr(boost::extents[10240][3][2]),kOpt(0)
{
  fLogger = FairLogger::GetLogger();
  fIsPersistence = kFALSE;
  fIsOldFormat  = kFALSE;
  fIsBaseLineSubtraction = kFALSE;
  fRawEventArray = new TClonesArray("ATRawEvent");
  fEventID = 0;
  fIniEventID = 0;
  fTimestampIndex = 1;
  fRawEvent = new ATRawEvent();

  kOpt = opt;

  fIsProtoGeoSet = kFALSE;
  fIsProtoMapSet = kFALSE;

  if(kOpt==0) fAtMapPtr = new AtTpcMap();
  else if(kOpt==1) fAtMapPtr = new AtTpcProtoMap();
  else std::cout << "== ATHDFParserTask Initialization Error : Option not found. Current available options: ATTPC Map 0 / Prototype Map 1" << std::endl;
  
}

ATHDFParserTask::~ATHDFParserTask()
{
	delete fRawEventArray;
	delete fRawEvent;
	delete fAtMapPtr;
}

void ATHDFParserTask::SetPersistence(Bool_t value){ fIsPersistence = value; }

Bool_t  ATHDFParserTask::SetOldFormat(Bool_t oldF)
{ 
  fIsOldFormat = oldF;
  return kTRUE; 
}

Bool_t ATHDFParserTask::SetBaseLineSubtraction(Bool_t value)
{
  fIsBaseLineSubtraction = value;
  return kTRUE;
}


bool   ATHDFParserTask::SetAuxChannel(uint32_t hash, std::string channel_name)
{
  auto value = fAuxTable.emplace(hash,channel_name);

  std::cout<<cGREEN<<" Auxiliary channel added "<<channel_name<<" - Hash "<<hash<<cNORMAL<<"\n";

  return value.second;

}

std::pair<bool,std::string>  ATHDFParserTask::FindAuxChannel(uint32_t hash)
{
  fAuxTableIt = fAuxTable.find(hash);
 
  
  if (fAuxTableIt != fAuxTable.end())
  {
    //std::cout << "Element Found - ";
    //std::cout << fAuxTableIt->first << "::" << fAuxTableIt->second << std::endl;
    return std::make_pair(true,fAuxTableIt->second);
  }
  else
  {
    //std::cout << "Element Not Found" << std::endl;
    return std::make_pair(false,"not_found");
  }
 
  

}

bool ATHDFParserTask::SetATTPCMap(Char_t const *lookup){

  if(kOpt==0) fAtMapPtr->GenerateATTPC();
  Bool_t MapIn = fAtMapPtr->ParseXMLMap(lookup);
  if(!MapIn) return false;

  std::cout<<cGREEN<<" Open lookup table "<<lookup<<cNORMAL<<"\n";

  //AtPadCoordArr = fAtMapPtr->GetPadCoordArr();//TODO Use a pointer to a simpler container
  //**** For debugging purposes only! ******//
  //fAtMapPtr->SetGUIMode();
  //fAtMapPtr->GetATTPCPlane();
  return true;

}

Bool_t ATHDFParserTask::SetProtoGeoFile(TString geofile){

   if(kOpt==1){

  fIsProtoGeoSet = fAtMapPtr->SetGeoFile(geofile);
        return fIsProtoGeoSet;

   }else{
   std::cout << "== ATHDFParserTask::SetProtoGeoMap. This method must be used only with Prototype mapping (kOpt=1)!" << std::endl;
         return kFALSE;
   }


}


Bool_t ATHDFParserTask::SetProtoMapFile(TString mapfile){

  if(kOpt==1)
  {
    
    fIsProtoMapSet = fAtMapPtr->SetProtoMap(mapfile);
    return fIsProtoMapSet;
    
  } else
  {
    std::cout << "== ATHDFParserTask::SetProtoMapFile. This method must be used only with Prototype mapping (kOpt=1)!" << std::endl;
    return kFALSE;
  }
}

Bool_t ATHDFParserTask::SetInitialEvent(std::size_t inievent)
{
  fIniEventID = inievent; 
}


InitStatus ATHDFParserTask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");  
    return kERROR;
  }
  
  //Need try-catch
  HDFParser = new ATHDFParser();
  fNumEvents = HDFParser->open(fFileName.c_str());
  std::cout<<" Number of events : " << fNumEvents << std::endl;
  
  auto numUniqueEvents = HDFParser->getFirstEvent() - HDFParser->getLastEvent();
  
  if(fIniEventID > numUniqueEvents)
  {
    fLogger -> Fatal(MESSAGE_ORIGIN, "Exceeded the valid range of event numbers");
    return kERROR;
  }
  else
    fEventID = fIniEventID + HDFParser->getFirstEvent();
  
  ioMan -> Register("ATRawEvent", "ATTPC", fRawEventArray, fIsPersistence);
  return kSUCCESS;
}

void ATHDFParserTask::SetParContainers()
{
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger->Fatal(MESSAGE_ORIGIN, "No analysis run!");
  
  FairRuntimeDb *db = run->GetRuntimeDb();
  if (!db)
    fLogger->Fatal(MESSAGE_ORIGIN, "No runtime database!");
  
  fPar = (ATDigiPar *) db->getContainer("ATDigiPar");
  if (!fPar)
    fLogger->Fatal(MESSAGE_ORIGIN, "Cannot find ATDigiPar!");
}

void ATHDFParserTask::Exec(Option_t *opt)
{
  fRawEventArray -> Delete();
  fRawEvent->Clear();

  
  if (fEventID > HDFParser->getLastEvent())
  {
    fLogger -> Fatal(MESSAGE_ORIGIN, "Tried to unpack an event that was too large!");
    return;
  }
  
  //Get the names of the header and data
  TString event_name  = TString::Format("evt%d_data", fEventID);
  TString header_name = TString::Format("evt%d_header", fEventID);

  auto header = HDFParser->get_header(header_name.Data());
  
  fRawEvent->SetEventID(header.at(0));
  fRawEvent->SetTimestamp(header.at(fTimestampIndex));

  std::size_t npads = HDFParser->n_pads(event_name.Data());
    
  //std::cout << "Found at " << fEventID << " event " << fRawEvent->GetEventID() << " Event name " << event_name
  //<< " with timestamp " << fRawEvent->GetTimestamp() << std::endl;
    
  for(auto ipad = 0; ipad < npads; ++ipad)
  {
      
    std::vector<int16_t> rawadc = HDFParser->pad_raw_data(ipad);
      
    int iCobo = rawadc[0];
    int iAsad = rawadc[1];
    int iAget = rawadc[2];
    int iCh   = rawadc[3];
    int iPad  = rawadc[4];

    std::vector<int> PadRef = {iCobo,iAsad,iAget,iCh};
    int PadRefNum = fAtMapPtr->GetPadNum(PadRef);

    //std::cout<<iCobo<<" "<<iAsad<<" "<<iAget<<" "<<iCh<<" "<<iPad<<"  "<<PadRefNum<<"\n";

    std::vector<Float_t> PadCenterCoord;
    PadCenterCoord.reserve(2);
    PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);

    ATPad *pad = new ATPad(PadRefNum);
    pad->SetPadXCoord(PadCenterCoord[0]);
    pad->SetPadYCoord(PadCenterCoord[1]);
    
    auto hash  = CalculateHash(uint32_t(iCobo),uint32_t(iAsad),uint32_t(iAget),uint32_t(iCh)); 
    std::pair<bool,std::string> isAux = FindAuxChannel(hash);
    
    if(isAux.first)
    {
      pad->SetIsAux(true);
      pad->SetAuxName(isAux.second);
    }
    
    
    //std::cout<<PadCenterCoord[0]<<" "<<PadCenterCoord[1]<<"\n";
    
    //Baseline subtraction
    double adc[512] = {0};
    double baseline =0;
    
    if(fIsBaseLineSubtraction)
    {
      for (Int_t iTb = 5; iTb < 25; iTb++)
	baseline+=rawadc[iTb];
      
      baseline/=20.0;
    }
    
    for (Int_t iTb = 0; iTb < 512; iTb++)
    {
      pad -> SetRawADC(iTb, rawadc.at(iTb+5));
      adc[iTb] = (double)rawadc[iTb+5] - baseline;
      //std::cout<<" iTb "<<iTb<<" rawadc "<<rawadc[iTb]<<"	"<<adc[iTb]<<"\n";
      pad -> SetADC(iTb, adc[iTb]);
    }          
    
    pad -> SetPedestalSubtracted(kTRUE);
    
    fRawEvent -> SetIsGood(kTRUE);
    fRawEvent -> SetPad(pad);
    delete pad;
    
      
  }// End loop over pads 
  
  new ((*fRawEventArray)[0]) ATRawEvent(fRawEvent);
    
  ++fEventID;
  
}


void ATHDFParserTask::FinishEvent()
{
  
  
    //fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
    //FairRootManager::Instance() -> SetFinishRun();
    
}
