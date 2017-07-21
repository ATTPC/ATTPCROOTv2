#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>
#include <sstream>

void root_to_pcl()
{
	TString dataDir = "../data/";
	TString mcFileName = dataDir + "attpcsim_2.root";
	TString outputFileName = dataDir + "output";

	std::cout << "Analysis of simulation file " << mcFileName << "..." << std::endl;

	TFile file(mcFileName.Data());
	TTree *tree = nullptr;
	file.GetObject("cbmsim", tree);

	TClonesArray *pointArray = nullptr;
	tree->SetBranchAddress("AtTpcPoint", &pointArray);

	Int_t eventCount = tree->GetEntries();
	for (Int_t i = 0; i < eventCount; ++i) {
		std::cout << "# Reading event " << (i + 1) << " of " << eventCount << std::endl;
		tree->GetEvent(i);
		Int_t pointCount = pointArray->GetEntries();
		
		std::ostringstream oss;
		oss << outputFileName << "." << i << ".pcl";

		std::ofstream outputFile(oss.str());
		outputFile << "VERSION 0.7" << std::endl
			<< "FIELDS x y z time intensity" << std::endl
			<< "SIZE 4 4 4 4 4" << std::endl
			<< "TYPE F F F U F" << std::endl
			<< "WIDTH " << pointCount << std::endl
			<< "HEIGHT 1" << std::endl
			<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
			<< "POINTS " << pointCount << std::endl
			<< "DATA ascii" << std::endl;

		for (Int_t j = 0; j < pointCount; ++j) {
			std::cout << "# Reading point " << (j + 1) << " of " << pointCount << std::endl;
			AtTpcPoint *point = (AtTpcPoint*)(*pointArray)[j];

			outputFile << point->GetXIn() << " "
				<< point->GetYIn() << " "
				<< point->GetZIn() << " "
				<< point->GetMassNum() << " " // FIXME
				<< point->GetAIni() << std::endl; // FIXME
		}
	}
}
