/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairLogger.h"
#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/

void run_eve_simple(TString OutputDataFile = "./data/output.sim_display.root")
{
   TString InputDataFile = "./data/output_digi.root";
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

   AtEventDrawTask *eve = new AtEventDrawTask();
   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   eveMan->AddTab(std::move(tabMain));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
