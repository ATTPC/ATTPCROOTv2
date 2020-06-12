#include "ATTPCFissionGeneratorV3.h"

// Default constructor
ATTPCFissionGeneratorV3::ATTPCFissionGeneratorV3():
  nTracks(0)
{
  
}

// Generator that takes in a file that specifies the expected distribution of
// fission particles. 
ATTPCFissionGeneratorV3::ATTPCFissionGeneratorV3(const char *name,
						 TString ionList,
						 TString
						 fissionDistro):nTracks(0)
{

  //Get the working directory
  TString workingDir = std::getenv("VCMWORKDIR");

  //Look for the file defining the fission mass distrobution
  std::ifstream fileIn(ionList.Data());

  if (!fileIn.is_open())
    Fatal("ATTPCFissionGeneratorV3",
	  Form("Cannot open input file: %s", ionList.Data()));

  // Read the file until all of the ions have been generated
  std::set < std::vector < int >>ionSet;

  while (!fileIn.eof()) {

    int A = 0, Z = 0;
    fileIn >> Z >> A;

    // If this wasn't a valid ion, then look to make sure we've hit the end of the ion section
    if (A == 0 && Z == 0) {
      char strIn[100];
      fileIn.getline(strIn, 100);
      std::cout << "Invalid ion (A,Z): (" << A << ", " << Z << ") "
	  << " next line: " << strIn << std::endl;
      break;
    }

    ionSet.emplace(std::vector < int >( {
				       Z, A}));
    std::cout << "Found ion: " << Z << " " << A << std::endl;

    //Add the ion to the set of found ions
  }				// end loop through ion list

  // Get the run instance and register the ions
  auto run = FairRunSim::Instance();
 for (auto it:ionSet) {
    FairIon *ion = new FairIon(TString::Format("Ion_%d_%d", it.at(0), it.at(1)),
			       it.at(0), it.at(1), 0);
    run->AddNewIon(ion);
  }

  //Load the tree
  TFile *treeFile = new TFile(fissionDistro, "READ");
  if (treeFile->IsZombie())
    std::cout << "No file of fission events found!" << std::endl;

  fissionEvents = (TTree *) treeFile->Get("trEvents");
  fissionEvents->SetBranchAddress("nTracks", &nTracks);
  fissionEvents->SetBranchAddress("Aout", Aout);
  fissionEvents->SetBranchAddress("Zout", Zout);
  fissionEvents->SetBranchAddress("pX", pX);
  fissionEvents->SetBranchAddress("pY", pY);
  fissionEvents->SetBranchAddress("pZ", pZ);
  fissionEvents->SetBranchAddress("pT", pT);

  nEvents = fissionEvents->GetEntries();
  event = 0;
}

// Deep copy constructor
ATTPCFissionGeneratorV3::ATTPCFissionGeneratorV3(ATTPCFissionGeneratorV3 & rhs)
{

}

ATTPCFissionGeneratorV3::~ATTPCFissionGeneratorV3()
{

}

Bool_t ATTPCFissionGeneratorV3::ReadEvent(FairPrimaryGenerator * primeGen)
{
  //If this is a beam-like event don't do anything
  if (gATVP->GetDecayEvtCnt() % 2 == 0) {
    std::cout << "ATTPCFissionGeneratorV3: Skipping beam-like event" << std::
	endl;
    gATVP->IncDecayEvtCnt();
    return true;
  }
  else
    std::cout << "ATTPCFissionGeneratorV3: Runing reaction-like event" << std::endl;

  gATVP->IncDecayEvtCnt();

  auto fPDG = TDatabasePDG::Instance();
  auto stack = (AtStack *) gMC->GetStack();

  //Read this event and get the current vertex
  Double_t fVx = gATVP->GetVx();
  Double_t fVy = gATVP->GetVy();
  Double_t fVz = gATVP->GetVz();

  fissionEvents->GetEntry(event);

  for (int i = 0; i < nTracks; ++i) {
    // Create the particle
    Int_t pdgType = 0;
    TString partName = TString::Format("Ion_%d_%d", Zout[i], Aout[i]);
    auto part = fPDG->GetParticle(partName);

    if (!part)
      std::cout << "Couldn't find particle " << partName << " in database!"
	  << std::endl;
    else
      pdgType = part->PdgCode();

    auto px = pX[i] / 1000;	//Change to MeV
    auto py = pY[i] / 1000;
    auto pz = pZ[i] / 1000;

    std::
	cout << "ATTPCFissionGeneratorV3: Generating ion of type " << partName
	<< " with momentum (" << px << ", " << py << ", " << pz <<
	") GeV/c at vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm." <<
	std::endl;

    primeGen->AddTrack(pdgType, px, py, pz, fVx, fVy, fVz);

  }				//End loop over tracks

  std::cout << "Wrote tracks for fission event: " << event << std::endl;
  event++;

}

//ClassImp(ATTPCFissionGeneratorV3);
