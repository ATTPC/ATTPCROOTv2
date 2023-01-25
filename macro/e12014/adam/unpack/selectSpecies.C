#include <TString.h>

void selectSpecies()
{
   TString input = "/mnt/analysis/e12014/TPC/fission_linked/run_0206.root";
   TString inputEvt = "/mnt/analysis/e12014/TPC/fission_linked/evtRun_0206.root";
   TString inputCuts = "/mnt/projects/hira/e12014/tpcSharedInfo/PIDcuts/Run323Yield.root"; // 150 torr
   TString output = "./data/output.root";

   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";
   TString parFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   AtRunAna *fRun = new AtRunAna();
   fRun->SetSource(new FairFileSource(input));
   fRun->SetSink(new FairRootFileSink(output));
   fRun->SetGeomFile(dir + "/geometry/" + geoFile);

   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(dir + "/parameters/" + parFile, "in");
   fRun->GetRuntimeDb()->setFirstInput(parIo1);
   fRun->GetRuntimeDb()->getContainer("AtDigiPar");

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(dir + "/scripts/" + mapFile);

   AtCutHEIST cut(inputEvt, inputCuts);
   cut.AddAllSpecies();
   cut.AddSpecies("Pb197");

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEventSub");
   reduceTask->SetReductionFunction([&cut]() { return cut(); });
   // fRun->AddTask(reduceTask);

   AtCopyTreeTask *task = new AtCopyTreeTask();
   fRun->AddTask(task);

   fRun->Init();
   fRun->Run(0, 10);
}
