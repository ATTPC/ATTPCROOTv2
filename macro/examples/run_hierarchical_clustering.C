void run_hierarchical_clustering(Int_t firstEvent = 0, Int_t eventCount = 9999)
{
	// Timer
	TStopwatch timer;
	timer.Start();

	// Set file names
	TString dir = getenv("VMCWORKDIR");
	TString dataDir = dir + "/data/";

	TString loggerFile = dataDir + "ATTPCLog_Reco.log";
	TString inputFile = dataDir + "run0218_output.root";
	TString outputFile = dataDir + "output";

	// Logger
	FairLogger *fLogger = FairLogger::GetLogger();
	fLogger->SetLogFileName(loggerFile);
	fLogger->SetLogToScreen(kTRUE);
	fLogger->SetLogToFile(kTRUE);
	fLogger->SetLogVerbosityLevel("HIGH");

	// read pointArray
	TFile file(inputFile);
	TTree *tree = nullptr;
	file.GetObject("cbmsim", tree);

	TClonesArray *events = nullptr;
	TBranch *eventsBranch = tree->GetBranch("ATEventH");
	eventsBranch->SetAddress(&events);

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

	Int_t const lastEvent = (eventCount < (tree->GetEntries() - firstEvent) ? eventCount : (tree->GetEntries() - firstEvent)) + firstEvent;
	for (Int_t i = firstEvent; i < lastEvent; ++i) {
		std::cout << "# Analyzing event " << (i + 1) << " of " << tree->GetEntries() << std::endl;
		eventsBranch->GetEvent(i);
		ATEvent *event = (ATEvent*)events->At(0);

		std::vector<ATHit> *hitArray = event->GetHitArray();
		std::cout << "HitArray Size: " << hitArray->size() << std::endl;

		// analyze
		ATHierarchicalClusteringCluster hierarchicalClusteringCluster = hierarchicalClusteringTask.AnalyzePointArray(*hitArray);
		std::vector<std::vector<size_t>> clusters = hierarchicalClusteringCluster.GetClusters();

		// work with results
		ostringstream oss;
		oss << outputFile << "." << i << ".cluster";
		hierarchicalClusteringCluster.Save(oss.str());
		std::cout << "Generated file: " << oss.str() << std::endl;

		/*
		for (std::vector<size_t> const &cluster : clusters)
		{
			std::cout << "# CLUSTER" << std::endl;

			for (size_t hitIndex : cluster)
			{
				ATHit const &hit = (*hitArray)[hitIndex];

				std::cout << "Point (" << hit.GetPosition().X() << " " << hit.GetPosition().Y() << " " << hit.GetPosition().Z() << ")" << std::endl;
			}
		}
		*/
	}

	// Finish
	timer.Stop();

	std::cout << "Reconstruction macro finished succesfully." << std::endl
		<< "(Real time: " << timer.RealTime() << " s, CPU time: " << timer.CpuTime() << " s)" << std::endl;
}
