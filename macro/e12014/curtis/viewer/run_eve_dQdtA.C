/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "HEISTpid3.h"
#include "ADCsumPlot.h"
#include "QsumPlot.h"
#include "DrawAuxPad.h"
#include "TestTPC.h"

void run_eve_dQdtA(int runNum = 210, TString OutputDataFile = "./data/output.reco_display.root")
{
   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString InputDataFile = "/mnt/analysis/e12014/TPC/fission_linked/run_0279.root";
   TString evtInputDataFile = "/mnt/analysis/e12014/TPC/fission_linked/evtRun_0279.root";
   //TString InputDataFile = TString::Format("../unpacking/data/linked/run_%04d.root", runNum);
   std::cout << "Opening: " << InputDataFile << std::endl;
   //TString evtInputDataFile = TString::Format("../unpacking/data/linked/evtRun_%04d.root", runNum);
   std::cout << "Opening: " << evtInputDataFile << std::endl;
   TString inputCuts = "/mnt/projects/hira/e12014/tpcSharedInfo/PIDcuts/Run323Yield.root"; // 150 torr

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";
   TString parameterFile = "ATTPC.e12014.par";

   TString InputDataPath = InputDataFile;
   TString OutputDataPath = OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;
   TString mapDir = dir + "/scripts/" + mapFile;
   TString digiParFile = dir + "/parameters/" + parameterFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   //FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy.root");
   //rtdb->setFirstInput(parIo1);

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   rtdb->getContainer("AtDigiPar");

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());

   AtViewerManager *eveMan = new AtViewerManager(fMap);
   eveMan->SetCheckBranch(eveMan->GetRawEventName());

   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile("respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();

   auto threshold = 45;
   auto filterOrder = 6;
   auto cutoff = 75;
   auto cycles = 4;

   auto psa = std::make_unique<AtPSAIterDeconv>();
   psa->SetResponse(*respAvgEvent);
   psa->SetFilterOrder(filterOrder);
   psa->SetCutoffFreq(cutoff);
   psa->SetIterations(cycles);
   // psa->SetThreshold(threshold);
   auto sidePSA = new AtSidebarPSAIterDeconv(eveMan->GetSidebar());
   sidePSA->SetPSA(psa.get());
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventSub");
   psaTask->SetOutputBranch("AtEventIter");
   psaTask->SetPersistence(kTRUE);
   eveMan->GetSidebar()->AddSidebarFrame(sidePSA);

   AtCutHEIST *cut = new AtCutHEIST(evtInputDataFile, inputCuts);
   cut->AddAllSpecies();
   //cut->AddSpecies("Pb197");

   auto boolMacEvt = new AtTabInfoFairRoot<AtRawEvent>(eveMan->GetRawEventName());
   auto boolFunc = TestTPC;

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEventSub");
   reduceTask->SetReductionFunction([cut, boolFunc, boolMacEvt]() { return (*cut)() && boolFunc(boolMacEvt); });

   auto sideSpec = new AtSidebarInfoMacro(eveMan->GetCurrentEntry(), eveMan->GetSidebar());
   sideSpec->SetLabel("Species");
   sideSpec->SetFunction([cut]() { return cut->GetSpecies();} );
   eveMan->GetSidebar()->AddSidebarFrame(sideSpec);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   string auxName = "IC";
   auto auxFunc = DrawAuxPad;

   auto sumInfo = std::make_unique<AtTabInfoFairRoot<AtRawEvent>>(eveMan->GetRawEventName());
   auto tabSum = std::make_unique<AtTabMacro>(2, 2, "Pad Sums");
   tabSum->GetTabInfo()->AddAugment(std::move(sumInfo));
   tabSum->SetDrawEventFunction(ADCsumPlot, 0, 0);
   tabSum->SetDrawEventFunction(QsumPlot, 0, 1);
   tabSum->SetDrawEventFunction([auxName, auxFunc](AtTabInfo *info){auxFunc(info, auxName); }, 1, 1);

   /*auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawAuxADC("IC", 1, 0);
   tabPad->DrawArrayAug("Qreco", 1, 1);*/

   auto heistTree =
      std::make_shared<AtTabInfoTree>("E12014", evtInputDataFile, AtViewerManager::Instance()->GetCurrentEntry());
   auto heistInfo = std::make_shared<AtTabInfoBranch<HTMusicIC>>(heistTree, "MUSIC");
   auto tabMac = std::make_unique<AtTabMacro>();
   tabMac->GetTabInfo()->AddAugment(heistInfo, "MusicIC");
   tabMac->GetTabInfo()->AddAugment(heistTree);
   tabMac->SetDrawTreeFunction(PlotPID);

   eveMan->AddTask(reduceTask);
   eveMan->AddTask(psaTask);
   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabSum));
   eveMan->AddTab(std::move(tabMac));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
