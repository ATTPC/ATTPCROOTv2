void run_reco_hierachical_clustering(Int_t firstEvent = 0, Int_t eventCount = 1)
{
	// Timer
	TStopwatch timer;
	timer.Start();

	// Set file names
	TString dir = getenv("VMCWORKDIR");
	TString dataDir = dir + "/data/";

	TString loggerFile = dataDir + "ATTPCLog_Reco.log";
	TString inputFile = dataDir + "attpcsim_2.root";
	TString outputFile = dataDir + "output.root";

	// Logger
	FairLogger *fLogger = FairLogger::GetLogger();
	fLogger->SetLogFileName(loggerFile);
	fLogger->SetLogToScreen(kTRUE);
	fLogger->SetLogToFile(kTRUE);
	fLogger->SetLogVerbosityLevel("HIGH");

	// read pointArray
	TFile file(inputFile.Data());
	TTree *tree = nullptr;
	file.GetObject("cbmsim", tree);

	TClonesArray *pointArray = nullptr;
	tree->SetBranchAddress("AtTpcPoint", &pointArray);

	// Run task
	ATHierarchicalClusteringTask hierarchicalClusteringTask;

	// optional: set different parameters
	hierarchicalClusteringTask.SetBestClusterDistanceDelta(2.91713f);
	hierarchicalClusteringTask.SetCleanupMinTriplets(20);
	hierarchicalClusteringTask.SetCloudScaleModifier(0.281718f);
	hierarchicalClusteringTask.SetGenTripletsMaxError(0.0103171f);
	hierarchicalClusteringTask.SetGenTripletsNnKandidates(14);
	hierarchicalClusteringTask.SetGenTripletsNBest(2);
	hierarchicalClusteringTask.SetSmoothRadius(0.818581f);

	Int_t lastEvent = (eventCount < (tree->GetEntries() - firstEvent) ? eventCount : (tree->GetEntries() - firstEvent)) + firstEvent;
	for (Int_t i = firstEvent; i < lastEvent; ++i) {
		std::cout << "# Analyzing event " << (i + 1) << " of " << tree->GetEntries() << std::endl;
		tree->GetEvent(i);

		// analyze
		hierarchicalClusteringTask.AnalyzePointArray(pointArray);
		// TODO
	}

	// Finish
	timer.Stop();

	std::cout << "Reconstruction macro finished succesfully." << std::endl
		<< "(Real time: " << timer.RealTime() << " s, CPU time: " << timer.CpuTime() << " s)" << std::endl
		<< "Output file: " << outputFile << std::endl;
}
