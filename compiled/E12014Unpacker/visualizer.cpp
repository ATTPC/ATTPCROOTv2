#include "SAMain.h"

int main(int argc, char* argv[])
{

  if (argc < 2)
  {
    std::cout << "Not enough arguments!" << std::endl;
    std::cout << "Usage: " << std::endl
	      << "SAMain file" << std::endl
	      << "file:\tFile to unpack" << std::endl;
    return -1;
  }
  
  //Keep TApplication from trying to process argv
  TApplication app("app", &argc, argv, 0, -1);   

  //Load the library for unpacking and reconstruction
  gSystem->Load("libATTPCReco.so");
  
  TStopwatch timer;
  timer.Start();
  
  int runNumber = 260;
   
  //Set the input file
  TString inputFile(argv[1]);

  //Do some stuff to try to extract the run number
  //run_0123.h5 -> run number is len-7 to len-4
  TString runNumStr(inputFile(inputFile.Length()-7, 4));
  runNumber = runNumStr.Atoi();

  //Set the output file
  TString outputFile = TString::Format("run_%04d.root", runNumber);
  
  std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

   
  //Set the mapping for the TPC
  TString scriptfile = "Lookup20150611.xml";
  TString parameterFile = "ATTPC.e15250.par";
   
  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";


  //Create a run
  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);
   
  //Set the parameter file
  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  rtdb -> setSecondInput(parIo1);

  //Create the unpacker task
  ATHDFParserTask* HDFParserTask = new ATHDFParserTask();
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetFileName(inputFile.Data());
  HDFParserTask->SetOldFormat(false);
  HDFParserTask->SetTimestampIndex(2);

  //Add unpacker to the run
  run -> AddTask(HDFParserTask);
  run -> Init();

  //Get the number of events and unpack the whole run
  auto numEvents = HDFParserTask->GetNumEvents()/2;
   
  std::cout << "Unpacking " << numEvents << " events. " << std::endl;
  run->Run(0,numEvents);


  std::cout << "Done unpacking events." << std::endl;
  return 0;
}
