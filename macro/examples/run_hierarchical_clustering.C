void run_hierarchical_clustering(Int_t firstEvent = 0, Int_t eventCount = std::numeric_limits<Int_t>::max())
{
	// Timer
	TStopwatch timer;
	timer.Start();

	// Set file names
	TString dir = getenv("VMCWORKDIR");
	TString dataDir = dir + "/data/";

	TString loggerFile = dataDir + "ATTPCLog_Reco.log";
	// TString inputFile = dataDir + "attpcsim_alpha.root";
	TString inputFile = dataDir + "run0202.root";
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

	bool isSimulation = false;
	TClonesArray *events = nullptr;
	TBranch *eventsBranch = tree->GetBranch("ATEventH");

	if (eventsBranch == nullptr)
	{
		isSimulation = true;
		eventsBranch = tree->GetBranch("AtTpcPoint");
	}

	eventsBranch->SetAddress(&events);

	std::cout << "This is " << (isSimulation ? "" : "not ") << "a simulation." << std::endl;

	// Run task
	ATHierarchicalClusteringTask hierarchicalClusteringTask;

	// optional: set different parameters
	hierarchicalClusteringTask.SetBestClusterDistanceDelta(2.0f);
	hierarchicalClusteringTask.SetCleanupMinTriplets(4);
	hierarchicalClusteringTask.SetCloudScaleModifier(0.2f);
	hierarchicalClusteringTask.SetGenTripletsMaxError(0.01f);
	hierarchicalClusteringTask.SetGenTripletsNnKandidates(10);
	hierarchicalClusteringTask.SetGenTripletsNBest(2);
	hierarchicalClusteringTask.SetSmoothRadius(0.25f);
	hierarchicalClusteringTask.SetSplineTangentScale(0.5f);
	hierarchicalClusteringTask.SetSplineMinControlPointDistance(30.0f);
	hierarchicalClusteringTask.SetSplineJump(1);

	// remember visualizer, so we can reuse it
	// `viewer` might get overwritten in the process
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = nullptr;

	Int_t const lastEvent = (eventCount < (eventsBranch->GetEntries() - firstEvent) ? eventCount : (eventsBranch->GetEntries() - firstEvent)) + firstEvent - (isSimulation ? 1 : 0);
	for (Int_t i = firstEvent; i < lastEvent; i += (isSimulation ? 2 : 1)) {
		std::cout << "# Analyzing event " << (i + 1) << " of " << eventsBranch->GetEntries() << std::endl;

		std::vector<ATHit> hitArray;

		if (isSimulation)
		{
			// first the beam is simulated,
			// then the recoil and scatter.
			// Here we merge them into one.
			for (size_t j = 0; j < 2; ++j)
			{
				eventsBranch->GetEvent(i + j);

				// here we convert the simulated points into ATHit
				// we scale with a factor of 10 because the simulations
				// are in cm, but the experiments are in mm
				for (size_t k = 0; k < events->GetEntries(); ++k)
				{
					AtTpcPoint const &point = *((AtTpcPoint*)events->At(k));
					hitArray.push_back(ATHit(
						point.GetTrackID(),
						point.GetXIn() * 10.0,
						point.GetYIn() * 10.0,
						point.GetZIn() * 10.0,
						0.0
					));
				}
			}
		}
		else
		{
			eventsBranch->GetEvent(i);
			// TODO: make this const
			ATEvent &event = *((ATEvent*)events->At(0));
			hitArray = *event.GetHitArray();
		}

		std::cout << "  HitArray Size: " << hitArray.size() << std::endl;

		try
		{
			// a place for no-matches
			// (points that don't belong to any trajectory)
			std::vector<ATHit> noMatch;

			// analyze
			std::vector<ATTrajectory> trajectories = hierarchicalClusteringTask.AnalyzePointArray(hitArray, &noMatch);

			// work with results
			for (ATTrajectory const &trajectory : trajectories)
			{
				std::cout << "  ## TRAJECTORY ##" << std::endl;

				std::cout << "    hits: " << trajectory.GetHits().size() << std::endl;
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
				std::cout << "    radius: " << (1.0f / averageCurvature) << std::endl;
				std::cout << "    centroidPoint: " << centroidPoint(0) << " " << centroidPoint(1) << " " << centroidPoint(2) << std::endl;
				std::cout << "    mainDirection: " << mainDirection(0) << " " << mainDirection(1) << " " << mainDirection(2) << std::endl;
			}

			// remember visualizer, so we can reuse it
			// `viewer` might get overwritten in the process
			hierarchicalClusteringTask.Visualize(trajectories, noMatch, viewer);
		}
		catch (std::runtime_error e)
		{
			std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
		}
	}

	// Finish
	timer.Stop();

	std::cout << "Reconstruction macro finished succesfully." << std::endl
		<< "(Real time: " << timer.RealTime() << " s, CPU time: " << timer.CpuTime() << " s)" << std::endl;
}
