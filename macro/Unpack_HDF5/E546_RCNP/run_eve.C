/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairLogger.h"
#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/

//void run_eve(int runNum = 174, TString OutputDataFile = "./data/output.reco_display.root")
void run_eve(int runNum = 52, TString OutputDataFile = "./run_0052_display.root")  
{
  //  TString OutputDataFile = "./run_0038_display.root";
  TString InputDataFile = TString::Format("./decode_data/run_%04d.root", runNum);
  std::cout << "Opening: " << InputDataFile << std::endl;

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_C4H10_57_7torr_geomanager.root";
   //   TString mapFile = "RCNP2025.xml";
   TString mapFile = "rcnp_map.xml";   

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

   auto tabBraggCurve = std::make_unique<AtTabBraggCurve>();
   tabBraggCurve->SetMultiHit(100);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabBraggCurve));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
