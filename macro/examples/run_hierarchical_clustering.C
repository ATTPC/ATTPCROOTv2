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
	hierarchicalClusteringTask.SetSplineTangentScale(0.5f);
	hierarchicalClusteringTask.SetSplineMinControlPointDistance(20.0f);
	hierarchicalClusteringTask.SetSplineJump(1);

	Int_t const lastEvent = (eventCount < (tree->GetEntries() - firstEvent) ? eventCount : (tree->GetEntries() - firstEvent)) + firstEvent;
	for (Int_t i = firstEvent; i < lastEvent; ++i) {
		std::cout << "# Analyzing event " << (i + 1) << " of " << tree->GetEntries() << std::endl;
		eventsBranch->GetEvent(i);
		ATEvent *event = (ATEvent*)events->At(0);

		std::vector<ATHit> *hitArray = event->GetHitArray();
		std::cout << "HitArray Size: " << hitArray->size() << std::endl;

		// analyze
		std::vector<ATTrajectory> trajectories = hierarchicalClusteringTask.AnalyzePointArray(*hitArray);

		// work with results
		for (ATTrajectory const &trajectory : trajectories)
		{
			std::cout << "# TRAJECTORY" << std::endl;

			// std::cout << "    hits:" << std::endl;
			// for (ATHit const &hit : trajectory.GetHits())
			// {
			// 	std::cout << "        hit: " << hit.GetPosition().X() << " " << hit.GetPosition().Y() << " " << hit.GetPosition().Z() << std::endl;
			// }

			ATCubicSplineFit const &cubicSplineFit = trajectory.GetCubicSplineFit();

			float const splineStartPosition = cubicSplineFit.GetStartPosition();
			float const splineEndPosition = cubicSplineFit.GetEndPosition();
			float const trajectoryLength = cubicSplineFit.CalculateArcLength(splineStartPosition, splineEndPosition, 10000);
			float const averageCurvature = cubicSplineFit.CalculateAverageCurvature(splineStartPosition, splineEndPosition, 10000, 0.1f);
			Eigen::Vector3f centroidPoint = trajectory.GetCentroidPoint();
			Eigen::Vector3f mainDirection = trajectory.GetMainDirection();

			std::cout << "    startHitIndex: " << trajectory.GetStartHitIndex() << std::endl;
			std::cout << "    startHitPosition: " << trajectory.GetPositionOnMainDirection(trajectory.GetStartHitVector()) << std::endl;
			std::cout << "    splineStartPosition: " << splineStartPosition << std::endl;
			std::cout << "    endHitIndex: " << trajectory.GetEndHitIndex() << std::endl;
			std::cout << "    endHitPosition: " << trajectory.GetPositionOnMainDirection(trajectory.GetEndHitVector()) << std::endl;
			std::cout << "    splineEndPosition: " << splineEndPosition << std::endl;
			std::cout << "    approximateTrajectoryLength: " << trajectory.GetApproximateTrajectoryLength() << std::endl;
			std::cout << "    trajectoryLength: " << trajectoryLength << std::endl;
			std::cout << "    averageCurvature: " << averageCurvature << std::endl;
			std::cout << "    centroidPoint: " << centroidPoint(0) << " " << centroidPoint(1) << " " << centroidPoint(2) << std::endl;
			std::cout << "    mainDirection: " << mainDirection(0) << " " << mainDirection(1) << " " << mainDirection(2) << std::endl;
		}
	}

	// Finish
	timer.Stop();

	std::cout << "Reconstruction macro finished succesfully." << std::endl
		<< "(Real time: " << timer.RealTime() << " s, CPU time: " << timer.CpuTime() << " s)" << std::endl;
}
