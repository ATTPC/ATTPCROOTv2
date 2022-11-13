class TxtEvents {
   std::vector<int> fEventNumbers; // Sorted event of event numbers
   int fLastAcces = 0;             // Index of the last accessed event

public:
   void AddTxtFile(std::string fileName)
   {
      std::ifstream file(fileName);
      if (!file.is_open())
         throw std::invalid_argument("file does not exist " + fileName);
      while (!file.eof()) {
         int evt = -1;
         file >> evt;
         if (evt != -1)
            fEventNumbers.push_back(evt);
      }
      std::sort(fEventNumbers.begin(), fEventNumbers.end());
      fLastAcces = 0;
      /*      std::cout << "Events to keep: " << std::endl;
            for (auto evt : fEventNumbers)
               std::cout << evt << std::endl;
      */
   }
   bool operator()(AtRawEvent *event)
   {

      int eventNum = event->GetEventID();
      // std::cout << "Searching for " << eventNum << " at index " << fLastAcces << " " << fEventNumbers[fLastAcces]
      //         << std::endl;
      //  Check if the event number is larger than our last accessed
      if (fEventNumbers[fLastAcces] > eventNum) {
         if (fLastAcces == 0)
            return false;
         fLastAcces--;
         return (*this)(event);
      }
      while (fEventNumbers[fLastAcces] < eventNum)
         fLastAcces++;

      return fEventNumbers[fLastAcces] == eventNum;
   }
};

// Requires the TPC run number
void unpack_toH5(int runNumber = 210)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString outDir = "./data";

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", runNumber);
   TString outputFile = outDir + TString::Format("/run_%04dFissionAndBeam.root", runNumber);

   std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;

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
   FairRunAna *run = new FairRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   std::cout << "Getting containers..." << std::endl;
   // We must get the container before initializing a run
   rtdb->getContainer("AtDigiPar");

   // Create the detector map
   auto fAtMapPtr = std::make_shared<AtTpcMap>();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GeneratePadPlane();

   fAtMapPtr->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   fAtMapPtr->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   fAtMapPtr->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   fAtMapPtr->AddAuxPad({10, 0, 2, 34}, "IC");

   // Create the unpacker task
   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);

   // Create data reduction task
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   TxtEvents events;
   // events.AddTxtFile("testEvent.txt");
   // events.AddTxtFile("/mnt/projects/hira/e12014/tpcSharedInfo/labeledEventRuns/fissionEventsRun_0210");
   // events.AddTxtFile("/mnt/projects/hira/e12014/tpcSharedInfo/labeledEventRuns/beamEventsRun_0210");
   events.AddTxtFile("/mnt/projects/hira/e12014/tpcSharedInfo/labeledEventRuns/otherEventsRun_0210");
   reduceTask->SetReductionFunction(events);

   auto threshold = 45;

   AtFilterSubtraction *filter = new AtFilterSubtraction(fAtMapPtr);
   filter->SetThreshold(threshold);
   filter->SetIsGood(false);

   AtFilterTask *filterTask = new AtFilterTask(filter);
   filterTask->SetPersistence(kTRUE);
   filterTask->SetFilterAux(true);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(threshold);

   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetOutputBranch("AtEventFiltered");
   psaTask->SetPersistence(kTRUE);

   auto *wHDF = new AtHDF5WriteTask("data/outputOther.h5", "AtEventFiltered");
   wHDF->SetUseEventNum(true);

   // Add unpacker to the run
   run->AddTask(unpackTask);
   run->AddTask(reduceTask);
   run->AddTask(filterTask);
   run->AddTask(psaTask);
   run->AddTask(wHDF);

   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 7000;

   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   // return;
   std::cout << "starting run" << std::endl;
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
