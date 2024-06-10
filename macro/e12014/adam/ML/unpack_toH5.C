
// Takes a ROOT file and a branch name for an AtEvent.
// Saves the hists to an H5 file for DavidsonML group.
void unpack_toH5(TString inputFilePath, TString branchName)
{
   inputFilePath = "/home/faculty/aanthony/fission/data/e12014/unpacked/Bi200Sim.root";
   inputFilePath = "/home/faculty/aanthony/attpcroot/macro/e12014/adam/ML/data/output_digi01.root";
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString outDir = "./data";
   TString outFileName = "Bi200Sim";
   // Set the in/out files
   TString inputFile(inputFilePath);
   TString outputFile = outDir + "/" + outFileName + ".root";

   std::cout << "Unpacking file: " << inputFile << std::endl;
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
   run->SetSource(new FairFileSource(inputFile));
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

   auto *wHDF = new AtHDF5WriteTask("data/" + outFileName + ".h5", branchName);
   wHDF->SetUseEventNum(false);

   run->AddTask(wHDF);

   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = run->GetSource()->CheckMaxEventNo() - 1;

   // numEvents = 1700;//217;
   // numEvents = 10;

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
