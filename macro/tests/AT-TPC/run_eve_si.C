/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairLogger.h"
#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/

void run_eve_si(int runNum = 29, TString OutputDataFile = "./data/output.reco_display.root")
{
   TString InputDataFile = TString::Format("./data/run_%04d.root", runNum);
   std::cout << "Opening: " << InputDataFile << std::endl;

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";

   TString InputDataPath = InputDataFile;
   TString OutputDataPath = OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;
   TString mapDir = dir + "/scripts/" + mapFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy.root");
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   eveMan->AddTab(std::move(tabMain));

   auto tab1903 = std::make_unique<AtTabPad>(2, 4, "1903");
   tab1903->DrawRawGenTrace(1,0,0);
   tab1903->DrawRawGenTrace(2,0,1);
   tab1903->DrawRawGenTrace(3,0,2);
   tab1903->DrawRawGenTrace(4,0,3);

   tab1903->DrawRawGenTrace(5,1,0);
   tab1903->DrawRawGenTrace(6,1,1);
   tab1903->DrawRawGenTrace(7,1,2);
   tab1903->DrawRawGenTrace(8,1,3);
   eveMan->AddTab(std::move(tab1903));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
