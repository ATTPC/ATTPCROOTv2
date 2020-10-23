//Unpacks tpc files from /mnt/daqtesting/e18008_attpc_transfer to /mnt/analysis/e18008/rootMerg/

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m\e[1m"
#define WHITE   "\033[37m"


// Requires the TPC run number
void re_process(int runNumberS800, int runNumberATTPC)
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libATTPCReco.so");
  gSystem->Load("libS800.so");
  gSystem->Load("libXMLParser.so");

  TStopwatch timer;
  timer.Start();

  //Set the input file
  //TString inputFile = TString::Format("hdf5Files/run_%04d.h5", runNumber);
  //TString inputFile = TString::Format("/mnt/daqtesting/e18008_attpc_transfer/h5/run_%04d.h5", runNumberATTPC);
  // TString inputFile = "/home/juan/NSCL/run_2060_0060.root";
  TString InputDataFile = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800, runNumberATTPC);
  // TString inputFile = TString::Format("/mnt/simulations/ceclub/giraud/attpc/ATTPCROOTv2/macro/Unpack_HDF5/hdf5Files/run_0002.h5", runNumberATTPC);

  //Set the output file
  // TString outputFile = TString::Format("/mnt/analysis/e18008/rootMerg/run_2%03d_%04d.root", runNumberS800, runNumberATTPC);
    TString outputFile = TString::Format("reprocess_run_%04d_%04d.root", runNumberS800, runNumberATTPC);
  // TString outputFile = TString::Format("/projects/ceclub/giraud/attpc/ATTPCROOTv2/macro/Simulation/d2He/Analyis_d2He/merged_run0002_48Ca.root", runNumberS800, runNumberATTPC);

  //std::cout << "Unpacking AT-TPC run " << runNumberATTPC << " from: " << inputFile << std::endl;
  //std::cout << "Saving in: " << outputFile << std::endl;

  //Set the mapping for the TPC
  TString scriptfile = "e12014_pad_mapping.xml";//"Lookup20150611.xml";
 // TString scriptfile = "Lookup20150611.xml";//"Lookup20150611.xml";
  //TString parameterFile = "ATTPC.testnew.par";
  TString parameterFile = "ATTPC.d2He.par";

  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  TString digiParFile = dir + "/parameters/" + parameterFile;
  //TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";
  //TString geoManFile  = dir + "/geometry/ATTPC_He1bar.root";
  TString geoManFile  = dir + "/geometry/ATTPC_d2He_07atm.root";


  //Create a run
  FairRunAna* run = new FairRunAna();
  run -> SetInputFile(inputFile);
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);

  //Set the parameter file
  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  rtdb -> setSecondInput(parIo1);



  ATRansacTask *RandTask = new ATRansacTask();
  RandTask ->SetPersistence(kTRUE);
  RandTask ->SetIsReprocess(kTRUE);
  //RandTask ->SetModelType(1);
  //RandTask ->SetFullMode();
  RandTask->SetTiltAngle(0.0);
  RandTask->SetDistanceThreshold(12.0);//15.
  RandTask->SetMinHitsLine(10);//7
  RandTask->SetAlgorithm(1); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;
  RandTask->SetRanSamMode(3);// 0=Uniform; 1=Gaussian; 2=Weighted; 3=Gaussian+Weighted
  RandTask->SetChargeThreshold(200); //for removing electron hits


  //Add unpacker to the run
  run -> AddTask(RandTask);

  run -> Init();


  TFile* file = new TFile(inputFile,"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");
  Int_t nEvents = tree -> GetEntries();




  //return;
  run->Run(0,nEvents);
  //run->Run(0,100);



  //std::cout << "Done unpacking events"  << std::endl << std::endl;
  //std::cout << "- Output file : " << outputFile << std::endl << std::endl;
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
