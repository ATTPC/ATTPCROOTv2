#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>
#include <sstream>

void root_to_pcl()
{
	TString dir = getenv("VMCWORKDIR");
	TString dataDir = dir + "/data/";

	TString inputFile = dataDir + "noisy_output.root";
	TString outputFileName = dataDir + "output";

	std::cout << "Converting " << inputFile << "..." << std::endl;

	TFile file(inputFile);
	TTree *tree = nullptr;
	file.GetObject("cbmsim", tree);

	TClonesArray *events = nullptr;
	TBranch *eventsBranch = tree->GetBranch("ATEventH");
	eventsBranch->SetAddress(&events);

	Int_t eventCount = tree->GetEntries();
	for (Int_t i = 0; i < eventCount; ++i) {
		std::cout << "# Reading event " << (i + 1) << " of " << eventCount << std::endl;
		eventsBranch->GetEvent(i);
		ATEvent *event = (ATEvent*)events->At(0);

		std::vector<ATHit> const *hitArray = event->GetHitArray();
		std::cout << "HitArray Size: " << hitArray->size() << std::endl;

		Int_t const hitCount = hitArray->size();
		
		std::ostringstream oss;
		oss << outputFileName << "." << i << ".pcl";

		std::ofstream outputFile(oss.str());
		outputFile << "VERSION 0.7" << std::endl
			<< "FIELDS x y z time intensity" << std::endl
			<< "SIZE 4 4 4 4 4" << std::endl
			<< "TYPE F F F U F" << std::endl
			<< "WIDTH " << hitCount << std::endl
			<< "HEIGHT 1" << std::endl
			<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
			<< "POINTS " << hitCount << std::endl
			<< "DATA ascii" << std::endl;

		for (Int_t j = 0; j < hitCount; ++j) {
			std::cout << "# Reading point " << (j + 1) << " of " << hitCount << std::endl;
			ATHit const &hit = (*hitArray)[j];
			TVector3 position = hit.GetPosition();

			outputFile << position.X() << " "
				<< position.Y() << " "
				<< position.Z() << " "
				<< hit.GetTimeStamp() << " "
				<< hit.GetCharge() << std::endl;
		}
	}
}
