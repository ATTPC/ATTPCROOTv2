#include "TxtEvent.h"

void unpack_linked(int tpcRunNum = 206)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString evtInputDir = "/mnt/analysis/e12014/HiRAEVT/mapped";
   TString outDir = "/mnt/analysis/e12014/TPC/fission_linked";
   TString evtOutDir = outDir;
   TString sharedInfoDir = "/mnt/projects/hira/e12014/tpcSharedInfo/";

   /**** Should not have to change code between this line and the next star comment ****/

   EventMap eventMap(sharedInfoDir + "RunMap.csv");
   int nsclRunNum = eventMap.GetNsclRunNum(tpcRunNum);
   if (nsclRunNum == -1)
      throw std::invalid_argument("No matting NSCL run number");

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", tpcRunNum);
   TString outputFile = outDir + TString::Format("/run_%04d.root", tpcRunNum);
   TString evtOutputFile = evtOutDir + TString::Format("/evtRun_%04d.root", tpcRunNum);
   TString evtInputFile = evtInputDir + TString::Format("/mappedRun-%d.root", nsclRunNum);

   std::cout << "Unpacking run " << tpcRunNum << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;
   std::cout << "Linking to run " << nsclRunNum << " from: " << evtInputFile << std::endl;
   std::cout << "Saving in: " << evtOutputFile << std::endl;

   // Set the mapping for the TPC
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   TString parameterFile = "ATTPC.e12014.par";

   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";

   // Create a run
   AtRunAna *run = new AtRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   rtdb->getContainer("AtDigiPar");

   // Create the detector map
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapDir.Data());
   mapping->GeneratePadPlane();

   /**** Should not have to change code between this line and the above star comment ****/
   mapping->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   mapping->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   mapping->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   mapping->AddAuxPad({10, 0, 2, 34}, "IC");

   // Create the unpacker task
   auto unpacker = std::make_unique<AtHDFUnpacker>(mapping);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);
   run->AddTask(unpackTask);

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   TxtEvents events;
   events.AddTxtFile(TString::Format(sharedInfoDir + "/EventLabels/fissionEventsRun_%04d", tpcRunNum).Data());
   reduceTask->SetReductionFunction(events);
   run->AddTask(reduceTask);

   AtLinkDAQTask *linker = new AtLinkDAQTask(); //< Must run after the data reduction task!!!
   auto success = linker->SetInputTree(evtInputFile, "E12014");
   linker->SetEvtOutputFile(evtOutputFile);
   linker->SetEvtTimestamp("tstamp");
   linker->SetTpcTimestampIndex(1);
   linker->SetSearchMean(1);
   linker->SetSearchRadius(2);
   linker->SetCorruptedSearchRadius(1000);
   run->AddTask(linker);

   auto threshold = 45;

   auto filterSub = new AtFilterSubtraction(mapping);
   filterSub->SetThreshold(threshold);
   filterSub->SetIsGood(false); // Save events event if
   AtFilterTask *subTask = new AtFilterTask(filterSub);
   subTask->SetPersistence(true);
   subTask->SetFilterAux(true);
   subTask->SetOutputBranch("AtRawEventSub");

   run->AddTask(subTask);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(threshold);
   AtPSAtask *psaTask = new AtPSAtask(psa->Clone());
   psaTask->SetInputBranch("AtRawEventSub");
   psaTask->SetOutputBranch("AtEvent");
   psaTask->SetPersistence(true);
   run->AddTask(psaTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 500;

   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   // return;
   run->Run(0, numEvents);

   std::cout << std::endl << std::endl;
   std::cout << "Done unpacking events" << std::endl << std::endl;
   std::cout << "- Output file : " << outputFile << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------

   return 0;
}
