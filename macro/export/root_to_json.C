#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>
#include <sstream>

void root_to_json()
{
	TString dir = getenv("VMCWORKDIR");
	TString dataDir = dir + "/data/";

	TString inputFile = dataDir + "noisy_output.root";
	TString outputFileName = dataDir + "output.json";

	std::cout << "Converting " << inputFile << "..." << std::endl;

	TFile file(inputFile);
	TTree *tree = nullptr;
	file.GetObject("cbmsim", tree);

	TClonesArray *events = nullptr;
	TBranch *eventsBranch = tree->GetBranch("ATEventH");
	eventsBranch->SetAddress(&events);

	std::ofstream outputFile(outputFileName);
	outputFile << "[" << std::endl;

	Int_t eventCount = tree->GetEntries();
	for (Int_t i = 0; i < eventCount; ++i)
	{
		std::cout << "# Reading event " << (i + 1) << " of " << eventCount << std::endl;
		eventsBranch->GetEvent(i);
		ATEvent *event = (ATEvent*)events->At(0);

		std::vector<ATHit> const *hitArray = event->GetHitArray();
		std::cout << "HitArray Size: " << hitArray->size() << std::endl;

		Int_t const hitCount = hitArray->size();

		outputFile << "    {" << std::endl;
		outputFile << "        \"ATEventH\": {" << std::endl;
		outputFile << "            \"HitArray\": [" << std::endl;

		for (Int_t j = 0; j < hitCount; ++j)
		{
			std::cout << "# Reading point " << (j + 1) << " of " << hitCount << std::endl;
			ATHit const &hit = (*hitArray)[j];
			TVector3 position = hit.GetPosition();

			outputFile << "                { \"X\": " << position.X() << ", "
				<< "\"Y\": " << position.Y() << ", "
				<< "\"Z\": " << position.Z() << ", "
				<< "\"TimeStamp\": " << hit.GetTimeStamp() << ", "
				<< "\"Charge\": " << hit.GetCharge() << " }";

			if (j < (hitCount - 1))
				outputFile << ",";

			outputFile << std::endl;
		}

		outputFile << "            ]" << std::endl;
		outputFile << "        }" << std::endl;
		outputFile << "    }";

		if (i < (eventCount - 1))
			outputFile << ",";

		outputFile << std::endl;
	}

	outputFile << "]" << std::endl;
}
