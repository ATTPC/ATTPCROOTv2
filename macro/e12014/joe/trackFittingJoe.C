void trackFittingJoe()
{
	return 0;
}


void FitTracks(int cutLimit = 150, int tpc_run_num = 211)
{
  TChain tpc_tree("cbmsim");
  //tpc_tree.Add(TString::Format("/mnt/analysis/hira_collaboration/e12014/Joe/UnpackedRunsDataReduced/run-%d.root", run_num));
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/trackTesting/th450run_%04d.root", tpc_run_num));
   
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "AtRansac");
  TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
  TTreeReaderValue<TClonesArray> eventH(reader, "AtEventH");
  
   //std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  

	TGraph *plotVertex = new TGraph();
	TGraph *gTrack1x = new TGraph();
	TGraph *gTrack1y = new TGraph();
	TGraph *gTrack2x = new TGraph();
	TGraph *gTrack2y = new TGraph();
			
    TF1 *f1x = new TF1("f1x", "pol1");
    TF1 *f1y = new TF1("f1y", "pol1");  		
    TF1 *f2x = new TF1("f2x", "pol1");  			
    TF1 *f2y = new TF1("f2y", "pol1");
   			
	vector<double> track1x, track1y, track2x, track2y;
	vector<double> track1z, track2z;		
			


	
   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   
   //for (int i = 0; i < tpc_nevents; ++i) {
	for (int i = 0; i < 17; ++i) {  
	reader.Next();
	if(i<16){continue;}
	
	for (int cut = 0; cut < cutLimit; ++cut){
	   int countTrack1 = 0;
	   int countTrack2 = 0;
			
	   f1x->SetParameters(1,1);		
	   f1y->SetParameters(1,1);		
	   f2x->SetParameters(1,1);
	   f2y->SetParameters(1,1);

	   track1x.clear(); track1y.clear(); track2x.clear(); track2y.clear();
	   track1z.clear(); track2z.clear();
	
	   if (ransac->GetEntries() > 0) {
	      auto atransac = (AtRANSACN::AtRansac *)(ransac->At(0));
	      auto ransac_ary = atransac->GetTrackCand();

	      std::vector<int> validTrackIndex;
	      for (int i = 0; i < ransac_ary.size(); ++i) {
		 double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
		 if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
		    validTrackIndex.push_back(i);
	      }

	      if (validTrackIndex.size() > 1)
		 numFission++;

	      // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
	      if (validTrackIndex.size() == 2) {
		 // Get four vectors describing the lines
		 TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
			     0);
		 TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
			     1);
		 TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
			     0);
		 TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
			     1);
		 TVector3 n = d1.Cross(d2);
		 TVector3 n1 = d1.Cross(n);
		 TVector3 n2 = d2.Cross(n);
			

		 TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
		 TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

		 auto vertex = (c1.Z() + c2.Z()) / 2.0;
		 if (vertex < 1000) {
		    numVertex++;
		 }
			
			
		 // Get the angle between vectors
		 TVector3 u1 = p1 + d1 - c1;
		 TVector3 u2 = p2 + d2 - c2;

		 // Print out the RANSAC fits in parametric form
		 //std::cout << "RANSAC Track 1: {(" << p1[0] << ") + (" << d1[0] << ")*z ,(" << p1[1] << ") + (" << d1[1] << ")*z}"<< std::endl;
		 //std::cout << "RANSAC Track 2: {(" << p2[0] << ") + (" << d2[0] << ")*z ,(" << p2[1] << ") + (" << d2[1] << ")*z}"<< std::endl;
		 //std::cout << "Vertex at " << vertex << " mm (z)" << std::endl;

			
		 //Getting x line
		 auto hits1 = ransac_ary[validTrackIndex[0]].GetHitArray();
		 auto hits2 = ransac_ary[validTrackIndex[1]].GetHitArray();

			
		 int j1 = 0;
		 int j2 = 0;
			
		 for(const auto & hit : *hits1)
		 {
		    //Adding cuts
		    //Only fit parts of track that are more than 10cm from center of detector
		    if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut)
		       break;
			
		    track1x.push_back(hit.GetPosition().X());
		    track1y.push_back(hit.GetPosition().Y());
		    track1z.push_back(hit.GetPosition().Z()); //hit.z()/100?	

		    gTrack1x->SetPoint(j1, track1z[j1], track1x[j1]);
		    gTrack1y->SetPoint(j1, track1z[j1], track1y[j1]);
		    j1++;
		    countTrack1++;
		 }

		 for(const auto & hit : *hits2)
		 {
				
		    if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut)
		       break;
				
				
		    track2x.push_back(hit.GetPosition().X());
		    track2y.push_back(hit.GetPosition().Y());
		    track2z.push_back(hit.GetPosition().Z()); //hit.z()/100?		
			
		    gTrack2x->SetPoint(j2, track2z[j2], track2x[j2]);
		    gTrack2y->SetPoint(j2, track2z[j2], track2y[j2]);
		    j2++;
		    countTrack2++;
		 }			
			
	      }

	   } // End loop over ransac




		gTrack1x->Fit(f1x,"Q");	  
		gTrack1y->Fit(f1y,"Q");   
		gTrack2x->Fit(f2x,"Q");   
		gTrack2y->Fit(f2y,"Q");   
   
            TVector3 jp1(f1x->GetParameter(0), f1y->GetParameter(0), 0);
            TVector3 jd1(f1x->GetParameter(1), f1y->GetParameter(1), 1);
            TVector3 jp2(f2x->GetParameter(0), f2y->GetParameter(0), 0);
            TVector3 jd2(f2x->GetParameter(1), f2y->GetParameter(1), 1);
			
            TVector3 jn = jd1.Cross(jd2);
            TVector3 jn1 = jd1.Cross(jn);
            TVector3 jn2 = jd2.Cross(jn);
			
            TVector3 jc1 = jp1 + (jp2 - jp1).Dot(jn2) / jd1.Dot(jn2) * jd1;
            TVector3 jc2 = jp2 + (jp1 - jp2).Dot(jn1) / jd2.Dot(jn1) * jd2;

            auto jvertex = (jc1.Z() + jc2.Z()) / 2.0;   

		std::cout << "For Cut Radius of " << cut << " mm, User Vertex at " << jvertex << " mm (z)" << std::endl;
	
		plotVertex->SetPoint(cut,cut,jvertex);
	
		for (int i1 = 0; i1 < countTrack1+1; ++i1)
		{
			gTrack1x->RemovePoint(i1);
			gTrack1y->RemovePoint(i1);
		}
	
		for (int i2 = 0; i2 < countTrack2+1; ++i2)
		{
			gTrack2x->RemovePoint(i2);
			gTrack2y->RemovePoint(i2);
		}

   } 
	
   } // end loop over events
   
   plotVertex->Draw(""); plotVertex->SetTitle("Vertex Location (z, mm) From Fitting Events Outside Cut Range");
   plotVertex->GetYaxis()->SetTitle("Vertex (z, mm)"); plotVertex->GetYaxis()->CenterTitle(true);
   plotVertex->GetXaxis()->SetTitle("Cut Radius (mm)"); plotVertex->GetXaxis()->CenterTitle(true);
   
}

void FitTracksCheck(int cut_radius = 0, int tpc_run_num = 206)
{
  TChain tpc_tree("cbmsim");
  //tpc_tree.Add(TString::Format("/mnt/analysis/hira_collaboration/e12014/Joe/UnpackedRunsDataReduced/run-%d.root", run_num));
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/trackTesting/th450run_%04d.root", tpc_run_num));
   
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "AtRansac");
  TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
  TTreeReaderValue<TClonesArray> eventH(reader, "AtEventH");
  
   //std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  
			TGraph *gTrack1x = new TGraph();
			TGraph *gTrack1y = new TGraph();
			TGraph *gTrack2x = new TGraph();
			TGraph *gTrack2y = new TGraph();
  
  
   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   //for (int i = 0; i < tpc_nevents; ++i) {
	for (int i = 0; i < 5; ++i) {  
		
	reader.Next();
	if(i<4){continue;}
	
      if (ransac->GetEntries() > 0) {
         auto atransac = (AtRANSACN::AtRansac *)(ransac->At(0));
         auto ransac_ary = atransac->GetTrackCand();

         std::vector<int> validTrackIndex;
         for (int i = 0; i < ransac_ary.size(); ++i) {
            double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
            if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
               validTrackIndex.push_back(i);
         }

         if (validTrackIndex.size() > 1)
            numFission++;

         // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
         if (validTrackIndex.size() == 2) {
            // Get four vectors describing the lines
            TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
                        0);
            TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
                        1);
            TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
                        0);
            TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
                        1);
            TVector3 n = d1.Cross(d2);
            TVector3 n1 = d1.Cross(n);
            TVector3 n2 = d2.Cross(n);
			

            TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
            TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

            auto vertex = (c1.Z() + c2.Z()) / 2.0;
            if (vertex < 1000) {
               numVertex++;
            }
			
			
            // Get the angle between vectors
            TVector3 u1 = p1 + d1 - c1;
            TVector3 u2 = p2 + d2 - c2;

			// Print out the RANSAC fits in parametric form
			//std::cout << "RANSAC Track 1: {(" << p1[0] << ") + (" << d1[0] << ")*z ,(" << p1[1] << ") + (" << d1[1] << ")*z}"<< std::endl;
			//std::cout << "RANSAC Track 2: {(" << p2[0] << ") + (" << d2[0] << ")*z ,(" << p2[1] << ") + (" << d2[1] << ")*z}"<< std::endl;
			//std::cout << "Vertex at " << vertex << " mm (z)" << std::endl;

			
					//Getting x line
			auto hits1 = ransac_ary[validTrackIndex[0]].GetHitArray();
			auto hits2 = ransac_ary[validTrackIndex[1]].GetHitArray();
			vector<double> track1x, track1y, track2x, track2y;
			vector<double> track1z, track2z;
			int j1 = 0;
			int j2 = 0;
			
			for(const auto & hit : *hits1)
			{
				//Adding cuts
				//Only fit parts of track that are more than 10cm from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut_radius)
					break;
			
			track1x.push_back(hit.GetPosition().X());
			track1y.push_back(hit.GetPosition().Y());
			track1z.push_back(hit.GetPosition().Z()); //hit.z()/100?	

			gTrack1x->SetPoint(j1, track1z[j1], track1x[j1]);
			gTrack1y->SetPoint(j1, track1z[j1], track1y[j1]);
			j1++;
			}

			for(const auto & hit : *hits2)
			{
				
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut_radius)
					break;
				
				
			track2x.push_back(hit.GetPosition().X());
			track2y.push_back(hit.GetPosition().Y());
			track2z.push_back(hit.GetPosition().Z()); //hit.z()/100?		
			
			gTrack2x->SetPoint(j2, track2z[j2], track2x[j2]);
			gTrack2y->SetPoint(j2, track2z[j2], track2y[j2]);
			j2++;
			}			
			
			//auto Coeff1Track1 = Fit(track1x,track1z);
			//auto Coeff2Track1 = Fit(track1y,track1z);
			
			
			
			
         }

      } // End loop over ranscal

   } // end loop over events
   
   TF1 *f1x = new TF1("f1x", "pol1");
   gTrack1x->Fit(f1x, "Q");	   

   TF1 *f1y = new TF1("f1y", "pol1");
   gTrack1y->Fit(f1y, "Q");  
  
   TF1 *f2x = new TF1("f2x", "pol1");
   gTrack2x->Fit(f2x, "Q");
   
   TF1 *f2y = new TF1("f2y", "pol1");
   gTrack2y->Fit(f2y, "Q");   
 
   
            TVector3 jp1(f1x->GetParameter(0), f1y->GetParameter(0), 0);
            TVector3 jd1(f1x->GetParameter(1), f1y->GetParameter(1), 1);
            TVector3 jp2(f2x->GetParameter(0), f2y->GetParameter(0), 0);
            TVector3 jd2(f2x->GetParameter(1), f2y->GetParameter(1), 1);
			
            TVector3 jn = jd1.Cross(jd2);
            TVector3 jn1 = jd1.Cross(jn);
            TVector3 jn2 = jd2.Cross(jn);
			
            TVector3 jc1 = jp1 + (jp2 - jp1).Dot(jn2) / jd1.Dot(jn2) * jd1;
            TVector3 jc2 = jp2 + (jp1 - jp2).Dot(jn1) / jd2.Dot(jn1) * jd2;

            auto jvertex = (jc1.Z() + jc2.Z()) / 2.0;   

	std::cout << "User Vertex at " << jvertex << " mm (z)" << "For cut radius of " << cut_radius << " mm (z)" << std::endl;
}

void FitTracksPlotAngle(int cutLimit = 150, int tpc_run_num = 206)
{
  TChain tpc_tree("cbmsim");
  //tpc_tree.Add(TString::Format("/mnt/analysis/hira_collaboration/e12014/Joe/UnpackedRunsDataReduced/run-%d.root", run_num));
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/trackTesting/th450run_%04d.root", tpc_run_num));
   
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "AtRansac");
  TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
  TTreeReaderValue<TClonesArray> eventH(reader, "AtEventH");

  
   //std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  

	TGraph *plotVertex = new TGraph();
	TGraph *hAngle = new TGraph();
	TGraph *gTrack1x = new TGraph();
	TGraph *gTrack1y = new TGraph();
	TGraph *gTrack2x = new TGraph();
	TGraph *gTrack2y = new TGraph();
			
    TF1 *f1x = new TF1("f1x", "pol1");
    TF1 *f1y = new TF1("f1y", "pol1");  		
    TF1 *f2x = new TF1("f2x", "pol1");  			
    TF1 *f2y = new TF1("f2y", "pol1");
   			
	vector<double> track1x, track1y, track2x, track2y;
	vector<double> track1z, track2z;		
			


	
   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   
   //for (int i = 0; i < tpc_nevents; ++i) {
	for (int i = 0; i < 1; ++i) {  
	reader.Next();
	//if(i<16){continue;}
	
	for (int cut = 0; cut < cutLimit; ++cut){
	int countTrack1 = 0;
	int countTrack2 = 0;
			
	f1x->SetParameters(1,1);		
	f1y->SetParameters(1,1);		
	f2x->SetParameters(1,1);
	f2y->SetParameters(1,1);

	track1x.clear(); track1y.clear(); track2x.clear(); track2y.clear();
	track1z.clear(); track2z.clear();
	
      if (ransac->GetEntries() > 0) {
         auto atransac = (AtRANSACN::AtRansac *)(ransac->At(0));
         auto ransac_ary = atransac->GetTrackCand();

         std::vector<int> validTrackIndex;
         for (int i = 0; i < ransac_ary.size(); ++i) {
            double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
            if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
               validTrackIndex.push_back(i);
         }

         if (validTrackIndex.size() > 1)
            numFission++;

         // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
         if (validTrackIndex.size() == 2) {
            // Get four vectors describing the lines
            TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
                        0);
            TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
                        1);
            TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
                        0);
            TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
                        1);
            TVector3 n = d1.Cross(d2);
            TVector3 n1 = d1.Cross(n);
            TVector3 n2 = d2.Cross(n);
			

            TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
            TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

            auto vertex = (c1.Z() + c2.Z()) / 2.0;
            if (vertex < 1000) {
               numVertex++;
            }

			
					//Getting x line
			auto hits1 = ransac_ary[validTrackIndex[0]].GetHitArray();
			auto hits2 = ransac_ary[validTrackIndex[1]].GetHitArray();

			
			int j1 = 0;
			int j2 = 0;
			
			for(const auto & hit : *hits1)
			{
				//Adding cuts
				//Only fit parts of track that are more than 10cm from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut)
					break;
			
			track1x.push_back(hit.GetPosition().X());
			track1y.push_back(hit.GetPosition().Y());
			track1z.push_back(hit.GetPosition().Z()); //hit.z()/100?	

			gTrack1x->SetPoint(j1, track1z[j1], track1x[j1]);
			gTrack1y->SetPoint(j1, track1z[j1], track1y[j1]);
			j1++;
			countTrack1++;
			}

			for(const auto & hit : *hits2)
			{
				
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cut)
					break;
				
				
			track2x.push_back(hit.GetPosition().X());
			track2y.push_back(hit.GetPosition().Y());
			track2z.push_back(hit.GetPosition().Z()); //hit.z()/100?		
			
			gTrack2x->SetPoint(j2, track2z[j2], track2x[j2]);
			gTrack2y->SetPoint(j2, track2z[j2], track2y[j2]);
			j2++;
			countTrack2++;
			}			
			
         }

      } // End loop over ransac




		gTrack1x->Fit(f1x,"Q");	  
		gTrack1y->Fit(f1y,"Q");   
		gTrack2x->Fit(f2x,"Q");   
		gTrack2y->Fit(f2y,"Q");   
   
            TVector3 jp1(f1x->GetParameter(0), f1y->GetParameter(0), 0);
            TVector3 jd1(f1x->GetParameter(1), f1y->GetParameter(1), 1);
            TVector3 jp2(f2x->GetParameter(0), f2y->GetParameter(0), 0);
            TVector3 jd2(f2x->GetParameter(1), f2y->GetParameter(1), 1);
			
            TVector3 jn = jd1.Cross(jd2);
            TVector3 jn1 = jd1.Cross(jn);
            TVector3 jn2 = jd2.Cross(jn);
			
            TVector3 jc1 = jp1 + (jp2 - jp1).Dot(jn2) / jd1.Dot(jn2) * jd1;
            TVector3 jc2 = jp2 + (jp1 - jp2).Dot(jn1) / jd2.Dot(jn1) * jd2;

            auto jvertex = (jc1.Z() + jc2.Z()) / 2.0;   
			
			// Get the angle between user fitted tracks
            TVector3 ju1 = jp1 + jd1 - jc1;
            TVector3 ju2 = jp2 + jd2 - jc2;

		std::cout << "For Cut Radius of " << cut << " mm, User Vertex at " << jvertex << " mm (z)" << std::endl;
	
		plotVertex->SetPoint(cut, cut, jvertex);
		hAngle->SetPoint(cut, cut, ju1.Angle(ju2) * TMath::RadToDeg());
	
		for (int i1 = 0; i1 < countTrack1+1; ++i1)
		{
			gTrack1x->RemovePoint(i1);
			gTrack1y->RemovePoint(i1);
		}
	
		for (int i2 = 0; i2 < countTrack2+1; ++i2)
		{
			gTrack2x->RemovePoint(i2);
			gTrack2y->RemovePoint(i2);
		}

   } 
	
   } // end loop over events
   
   TCanvas *c = new TCanvas("c1");
   plotVertex->Draw(""); plotVertex->SetTitle("Vertex Location (z, mm) From Fitting Events Outside Cut Range");
   plotVertex->GetYaxis()->SetTitle("Vertex (z, mm)"); plotVertex->GetYaxis()->CenterTitle(true);
   plotVertex->GetXaxis()->SetTitle("Cut Radius (mm)"); plotVertex->GetXaxis()->CenterTitle(true);

   TCanvas *c2 = new TCanvas("c2");   
   hAngle->Draw(""); hAngle->SetTitle("Angle Between Tracks From Fitting Events Outside Cut Range");
   hAngle->GetYaxis()->SetTitle("Angle Between Tracks"); hAngle->GetYaxis()->CenterTitle(true);
   hAngle->GetXaxis()->SetTitle("Cut Radius (mm)"); hAngle->GetXaxis()->CenterTitle(true);   
}

void FitTracksPlotResiduals(int cutRadius = 0, int tpc_run_num = 206)
{
  TChain tpc_tree("cbmsim");
  //tpc_tree.Add(TString::Format("/mnt/analysis/hira_collaboration/e12014/Joe/UnpackedRunsDataReduced/run-%d.root", run_num));
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/trackTesting/th450run_%04d.root", tpc_run_num));
   
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "AtRansac");
  TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
  TTreeReaderValue<TClonesArray> eventH(reader, "AtEventH");

  
   //std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  

	TGraph *plotResiduals1 = new TGraph();
	TGraph *plotResiduals2 = new TGraph();
	TGraph *gTrack1x = new TGraph();
	TGraph *gTrack1y = new TGraph();
	TGraph *gTrack2x = new TGraph();
	TGraph *gTrack2y = new TGraph();
			
    TF1 *f1x = new TF1("f1x", "pol1");
    TF1 *f1y = new TF1("f1y", "pol1");  		
    TF1 *f2x = new TF1("f2x", "pol1");  			
    TF1 *f2y = new TF1("f2y", "pol1");
   			
	vector<double> checktrack1x, checktrack1y, checktrack2x, checktrack2y;
	vector<double> checktrack1z, checktrack2z;
			
	vector<double> track1x, track1y, track2x, track2y;
	vector<double> track1z, track2z;		
			
	
   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   
   //for (int i = 0; i < tpc_nevents; ++i) {
	for (int i = 0; i < 2; ++i) {  
	reader.Next();
	if(i<1){continue;}
	
	
	int countTrack1 = 0;
	int countTrack2 = 0;
	int checkcountTrack1 = 0;
	int checkcountTrack2 = 0;
			
			
	f1x->SetParameters(1,1);		
	f1y->SetParameters(1,1);		
	f2x->SetParameters(1,1);
	f2y->SetParameters(1,1);

	track1x.clear(); track1y.clear(); track2x.clear(); track2y.clear();
	track1z.clear(); track2z.clear();
	checktrack1x.clear(); checktrack1y.clear(); checktrack2x.clear(); checktrack2y.clear();
	checktrack1z.clear(); checktrack2z.clear();
	
	
      if (ransac->GetEntries() > 0) {
         auto atransac = (AtRANSACN::AtRansac *)(ransac->At(0));
         auto ransac_ary = atransac->GetTrackCand();

         std::vector<int> validTrackIndex;
         for (int i = 0; i < ransac_ary.size(); ++i) {
            double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
            if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
               validTrackIndex.push_back(i);
         }

         if (validTrackIndex.size() > 1)
            numFission++;

         // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
         if (validTrackIndex.size() == 2) {
            // Get four vectors describing the lines
            TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
                        0);
            TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
                        1);
            TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
                        0);
            TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
                        1);
            TVector3 n = d1.Cross(d2);
            TVector3 n1 = d1.Cross(n);
            TVector3 n2 = d2.Cross(n);
			

            TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
            TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

            auto vertex = (c1.Z() + c2.Z()) / 2.0;
            if (vertex < 1000) {
               numVertex++;
            }

			
					//Getting x line
			auto hits1 = ransac_ary[validTrackIndex[0]].GetHitArray();
			auto hits2 = ransac_ary[validTrackIndex[1]].GetHitArray();

			
			int j1 = 0;
			int j2 = 0;
			
			for(const auto & hit : *hits1)
			{
			//Record all hits regardless	
			checktrack1x.push_back(hit.GetPosition().X());
			checktrack1y.push_back(hit.GetPosition().Y());
			checktrack1z.push_back(hit.GetPosition().Z()); //hit.z()/100?
			checkcountTrack1++;

				//Adding cuts
				//Only fit parts of track that are more than 10cm from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cutRadius)
					continue;
			
			track1x.push_back(hit.GetPosition().X());
			track1y.push_back(hit.GetPosition().Y());
			track1z.push_back(hit.GetPosition().Z()); //hit.z()/100?	

			gTrack1x->SetPoint(j1, track1z[j1], track1x[j1]);
			gTrack1y->SetPoint(j1, track1z[j1], track1y[j1]);
			j1++;
			countTrack1++;
			}

			for(const auto & hit : *hits2)
			{
			//Record all hits regardless	
			checktrack2x.push_back(hit.GetPosition().X());
			checktrack2y.push_back(hit.GetPosition().Y());
			checktrack2z.push_back(hit.GetPosition().Z()); //hit.z()/100?
			checkcountTrack2++;

				//Adding cuts
				//Only fit parts of track that are more than 10cm from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cutRadius)
					continue;
				
				
			track2x.push_back(hit.GetPosition().X());
			track2y.push_back(hit.GetPosition().Y());
			track2z.push_back(hit.GetPosition().Z()); //hit.z()/100?		
			
			gTrack2x->SetPoint(j2, track2z[j2], track2x[j2]);
			gTrack2y->SetPoint(j2, track2z[j2], track2y[j2]);
			j2++;
			countTrack2++;
			}			
			
         }

      } // End loop over ransac




		gTrack1x->Fit(f1x,"Q");	  
		gTrack1y->Fit(f1y,"Q");   
		gTrack2x->Fit(f2x,"Q");   
		gTrack2y->Fit(f2y,"Q");   
   
            TVector3 jp1(f1x->GetParameter(0), f1y->GetParameter(0), 0);
            TVector3 jd1(f1x->GetParameter(1), f1y->GetParameter(1), 1);
            TVector3 jp2(f2x->GetParameter(0), f2y->GetParameter(0), 0);
            TVector3 jd2(f2x->GetParameter(1), f2y->GetParameter(1), 1);
			
            TVector3 jn = jd1.Cross(jd2);
            TVector3 jn1 = jd1.Cross(jn);
            TVector3 jn2 = jd2.Cross(jn);
			
            TVector3 jc1 = jp1 + (jp2 - jp1).Dot(jn2) / jd1.Dot(jn2) * jd1;
            TVector3 jc2 = jp2 + (jp1 - jp2).Dot(jn1) / jd2.Dot(jn1) * jd2;

            auto jvertex = (jc1.Z() + jc2.Z()) / 2.0;   
			
			// Get the angle between user fitted tracks
            TVector3 ju1 = jp1 + jd1 - jc1;
            TVector3 ju2 = jp2 + jd2 - jc2;
	
		for (int i1 = 0; i1 < checkcountTrack1+1; ++i1)
		{
			//gTrack1x->RemovePoint(i1);
			//gTrack1y->RemovePoint(i1);
			
			auto x1 = (f1x->GetParameter(0) + (f1x->GetParameter(1))*(checktrack1z[i1]));
			auto y1 = (f1y->GetParameter(0) + (f1y->GetParameter(1))*(checktrack1z[i1]));
			auto d1 = sqrt((x1-checktrack1x[i1])*(x1-checktrack1x[i1]) + (y1-checktrack1y[i1])*(y1-checktrack1y[i1]));
		
			plotResiduals1->SetPoint(i1, checktrack1z[i1], d1);
		}

	
		for (int i2 = 0; i2 < checkcountTrack2+1; ++i2)
		{
			//gTrack2x->RemovePoint(i2);
			//gTrack2y->RemovePoint(i2);
			
			auto x2 = (f2x->GetParameter(0) + (f2x->GetParameter(1))*(checktrack2z[i2]));
			auto y2 = (f2y->GetParameter(0) + (f2y->GetParameter(1))*(checktrack2z[i2]));
			auto d2 = sqrt((x2-checktrack2x[i2])*(x2-checktrack2x[i2]) + (y2-checktrack2y[i2])*(y2-checktrack2y[i2]));
			
			plotResiduals2->SetPoint(i2, checktrack2z[i2], d2);
		}

	std::cout << "This many hits in Track 1: " << checkcountTrack1 << std::endl;
	std::cout << "This many hits in Track 2: " << checkcountTrack2 << std::endl;		
	
   } // end loop over events
	
   TCanvas *c = new TCanvas("c1");
   plotResiduals1->Draw("AP*"); plotResiduals1->SetTitle("Residuals for Track 1");
   plotResiduals1->GetYaxis()->SetTitle("Residuals of User Fit (mm)"); plotResiduals1->GetYaxis()->CenterTitle(true);
   plotResiduals1->GetXaxis()->SetTitle("Beam Z (mm)"); plotResiduals1->GetXaxis()->CenterTitle(true);

   TCanvas *c2 = new TCanvas("c2");
   plotResiduals2->Draw("AP*"); plotResiduals2->SetTitle("Residuals for Track 2");
   plotResiduals2->GetYaxis()->SetTitle("Residuals of User Fit (mm)"); plotResiduals2->GetYaxis()->CenterTitle(true);
   plotResiduals2->GetXaxis()->SetTitle("Beam Z (mm)"); plotResiduals2->GetXaxis()->CenterTitle(true);   
}

void FitTracksPlotDistortion(int cutRadius = 200, int tpc_run_num = 206)
{
  TChain tpc_tree("cbmsim");
  //tpc_tree.Add(TString::Format("/mnt/analysis/hira_collaboration/e12014/Joe/UnpackedRunsDataReduced/run-%d.root", run_num));
  tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/trackTesting/th450run_%04d.root", tpc_run_num));
   
  TTreeReader reader(&tpc_tree);
  TTreeReaderValue<TClonesArray> ransac(reader, "AtRansac");
  TTreeReaderValue<TClonesArray> event(reader, "AtRawEvent");
  TTreeReaderValue<TClonesArray> eventH(reader, "AtEventH");

  
   //std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");
  

	TGraph *plotDistortion1 = new TGraph();
	TGraph *plotDistortion2 = new TGraph();
	TGraph *gTrack1x = new TGraph();
	TGraph *gTrack1y = new TGraph();
	TGraph *gTrack2x = new TGraph();
	TGraph *gTrack2y = new TGraph();
			
    TF1 *f1x = new TF1("f1x", "pol1");
    TF1 *f1y = new TF1("f1y", "pol1");  		
    TF1 *f2x = new TF1("f2x", "pol1");  			
    TF1 *f2y = new TF1("f2y", "pol1");
   			
	vector<double> checktrack1x, checktrack1y, checktrack2x, checktrack2y;
	vector<double> checktrack1z, checktrack2z;
			
	vector<double> track1x, track1y, track2x, track2y;
	vector<double> track1z, track2z;		
			
	
   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   
   //for (int i = 0; i < tpc_nevents; ++i) {
	for (int i = 0; i < 1; ++i) {  
	reader.Next();
	//if(i<1){continue;}
	
	
	int countTrack1 = 0;
	int countTrack2 = 0;
	int checkcountTrack1 = 0;
	int checkcountTrack2 = 0;
			
			
	f1x->SetParameters(1,1);		
	f1y->SetParameters(1,1);		
	f2x->SetParameters(1,1);
	f2y->SetParameters(1,1);

	track1x.clear(); track1y.clear(); track2x.clear(); track2y.clear();
	track1z.clear(); track2z.clear();
	checktrack1x.clear(); checktrack1y.clear(); checktrack2x.clear(); checktrack2y.clear();
	checktrack1z.clear(); checktrack2z.clear();
	
	
      if (ransac->GetEntries() > 0) {
         auto atransac = (AtRANSACN::AtRansac *)(ransac->At(0));
         auto ransac_ary = atransac->GetTrackCand();

         std::vector<int> validTrackIndex;
         for (int i = 0; i < ransac_ary.size(); ++i) {
            double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
            if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
               validTrackIndex.push_back(i);
         }

         if (validTrackIndex.size() > 1)
            numFission++;

         // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
         if (validTrackIndex.size() == 2) {
            // Get four vectors describing the lines
            TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
                        0);
            TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
                        1);
            TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
                        0);
            TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
                        1);
            TVector3 n = d1.Cross(d2);
            TVector3 n1 = d1.Cross(n);
            TVector3 n2 = d2.Cross(n);
			

            TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
            TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

            auto vertex = (c1.Z() + c2.Z()) / 2.0;
            if (vertex < 1000) {
               numVertex++;
            }

			
					//Getting x line
			auto hits1 = ransac_ary[validTrackIndex[0]].GetHitArray();
			auto hits2 = ransac_ary[validTrackIndex[1]].GetHitArray();

			
			int j1 = 0;
			int j2 = 0;
			
			for(const auto & hit : *hits1)
			{
			//Record all hits regardless	
			checktrack1x.push_back(hit.GetPosition().X());
			checktrack1y.push_back(hit.GetPosition().Y());
			checktrack1z.push_back(hit.GetPosition().Z()); //hit.z()/100?
			checkcountTrack1++;

				//Adding cuts
				//Only fit parts of track that are more than "cutRadius mm" (e.g. 200 = 20cm) from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cutRadius)
					continue;
			
			track1x.push_back(hit.GetPosition().X());
			track1y.push_back(hit.GetPosition().Y());
			track1z.push_back(hit.GetPosition().Z()); //hit.z()/100?	

			gTrack1x->SetPoint(j1, track1z[j1], track1x[j1]);
			gTrack1y->SetPoint(j1, track1z[j1], track1y[j1]);
			j1++;
			countTrack1++;
			}

			for(const auto & hit : *hits2)
			{
			//Record all hits regardless	
			checktrack2x.push_back(hit.GetPosition().X());
			checktrack2y.push_back(hit.GetPosition().Y());
			checktrack2z.push_back(hit.GetPosition().Z()); //hit.z()/100?
			checkcountTrack2++;

				//Adding cuts
				//Only fit parts of track that are more than 10cm from center of detector
				if(sqrt(hit.GetPosition().X()*hit.GetPosition().X() + hit.GetPosition().Y()*hit.GetPosition().Y()) < cutRadius)
					continue;
				
				
			track2x.push_back(hit.GetPosition().X());
			track2y.push_back(hit.GetPosition().Y());
			track2z.push_back(hit.GetPosition().Z()); //hit.z()/100?		
			
			gTrack2x->SetPoint(j2, track2z[j2], track2x[j2]);
			gTrack2y->SetPoint(j2, track2z[j2], track2y[j2]);
			j2++;
			countTrack2++;
			}			
			
         }

      } // End loop over ransac




		gTrack1x->Fit(f1x,"Q");	  
		gTrack1y->Fit(f1y,"Q");   
		gTrack2x->Fit(f2x,"Q");   
		gTrack2y->Fit(f2y,"Q");   
   
            TVector3 jp1(f1x->GetParameter(0), f1y->GetParameter(0), 0);
            TVector3 jd1(f1x->GetParameter(1), f1y->GetParameter(1), 1);
            TVector3 jp2(f2x->GetParameter(0), f2y->GetParameter(0), 0);
            TVector3 jd2(f2x->GetParameter(1), f2y->GetParameter(1), 1);
			
            TVector3 jn = jd1.Cross(jd2);
            TVector3 jn1 = jd1.Cross(jn);
            TVector3 jn2 = jd2.Cross(jn);
			
            TVector3 jc1 = jp1 + (jp2 - jp1).Dot(jn2) / jd1.Dot(jn2) * jd1;
            TVector3 jc2 = jp2 + (jp1 - jp2).Dot(jn1) / jd2.Dot(jn1) * jd2;

            auto jvertex = (jc1.Z() + jc2.Z()) / 2.0;   
			
			// Get the angle between user fitted tracks
            TVector3 ju1 = jp1 + jd1 - jc1;
            TVector3 ju2 = jp2 + jd2 - jc2;
	
		for (int i1 = 0; i1 < checkcountTrack1+1; ++i1)
		{
			//gTrack1x->RemovePoint(i1);
			//gTrack1y->RemovePoint(i1);
			
			auto x1 = (f1x->GetParameter(0) + (f1x->GetParameter(1))*(checktrack1z[i1]));
			auto y1 = (f1y->GetParameter(0) + (f1y->GetParameter(1))*(checktrack1z[i1]));
			auto d1 = (((checktrack1x[i1])*(checktrack1x[i1])+(checktrack1y[i1])*(checktrack1y[i1]))-((x1)*(x1)+(y1)*(y1)))/1000000;
		
			plotDistortion1->SetPoint(i1, checktrack1z[i1]/1000, d1);
		}

	
		for (int i2 = 0; i2 < checkcountTrack2+1; ++i2)
		{
			//gTrack2x->RemovePoint(i2);
			//gTrack2y->RemovePoint(i2);
			
			auto x2 = (f2x->GetParameter(0) + (f2x->GetParameter(1))*(checktrack2z[i2]));
			auto y2 = (f2y->GetParameter(0) + (f2y->GetParameter(1))*(checktrack2z[i2]));
			auto d2 = (((checktrack2x[i2])*(checktrack2x[i2])+(checktrack2y[i2])*(checktrack2y[i2]))-((x2)*(x2)+(y2)*(y2)))/1000000;
			
			plotDistortion2->SetPoint(i2, checktrack2z[i2]/1000, d2);
		}

	std::cout << "This many hits in Track 1: " << checkcountTrack1 << std::endl;
	std::cout << "This many hits in Track 2: " << checkcountTrack2 << std::endl;		
	
   } // end loop over events
	
   TCanvas *c = new TCanvas("c1");
   plotDistortion1->Draw("AP*"); plotDistortion1->SetTitle("Distortion for Track 1");
   plotDistortion1->GetYaxis()->SetTitle("Radial Drift Distortion (m^2)"); plotDistortion1->GetYaxis()->CenterTitle(true);
   plotDistortion1->GetXaxis()->SetTitle("Electron Drift Distance in Z (m)"); plotDistortion1->GetXaxis()->CenterTitle(true);

   TCanvas *c2 = new TCanvas("c2");
   plotDistortion2->Draw("AP*"); plotDistortion2->SetTitle("Distortion for Track 2");
   plotDistortion2->GetYaxis()->SetTitle("Radial Drift Distortion (m^2)"); plotDistortion2->GetYaxis()->CenterTitle(true);
   plotDistortion2->GetXaxis()->SetTitle("Electron Drift Distance in Z (m)"); plotDistortion2->GetXaxis()->CenterTitle(true);   
}


