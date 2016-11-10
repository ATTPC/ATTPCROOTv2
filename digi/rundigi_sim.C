void rundigi_sim
(TString mcFile = "~/fair_install_ROOT6/ATTPCROOTv2_October2016/digi/data/attpcsim_2.root")
{



  // __ Run ____________________________________________
  FairRunAna* fRun = new FairRunAna();
              fRun -> SetInputFile(mcFile);
              fRun -> SetOutputFile("~/fair_install_ROOT6/ATTPCROOTv2_October2016/macro/Unpack_GETDecoder2/output.root");

  // __ AT digi tasks___________________________________

  ATClusterizeTask* clusterizer = new ATClusterizeTask();
                clusterizer -> SetPersistence(kFALSE);

  ATPulseTask* pulse = new ATPulseTask();
      pulse -> SetPersistence(kTRUE);

      ATPSATask *psaTask = new ATPSATask();
      psaTask -> SetPersistence(kTRUE);
      psaTask -> SetThreshold(20);
      psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
      //psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
      psaTask -> SetMaxFinder();
      psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
      psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak


  fRun -> AddTask(clusterizer);
  fRun -> AddTask(pulse);
  fRun -> AddTask(psaTask);

  // __ Init and run ___________________________________

  fRun -> Init();
  fRun -> Run(0,100);
}
