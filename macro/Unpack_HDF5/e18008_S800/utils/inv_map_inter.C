//#include "spline.h"  //cubic interpolation
//#include "TInverseMap.hh"
#include <unistd.h>

float  fBrho;
int    fMass;
int    fCharge;
std::string info;

/*
std::vector <double> get_invmap_vars(TInverseMap *inv_map, double x0, double y0, double afp, double bfp)
	{
		int order;
		double sinb_sina;
		vector <double> outPuts;

		order = 5;

		outPuts.push_back(inv_map->Ata(order, x0, afp, y0, bfp));//ata
		outPuts.push_back(inv_map->Bta(order, x0, afp, y0, bfp) * bta_corr);//bta
		outPuts.push_back(inv_map->Yta(order, x0, afp, y0, bfp) * 1000.);//yta
		outPuts.push_back(inv_map->Dta(order, x0, afp, y0, bfp));//dta
		outPuts.push_back(atan(sqrt(pow(tan(outPuts.at(0)), 2) + pow(tan(outPuts.at(1)), 2))));//theta_lab
		outPuts.push_back(atan(tan(outPuts.at(1))/tan(outPuts.at(0))));//phi
		if (outPuts.at(0) < 0)
		outPuts.at(5) = 3.141592653 + outPuts.at(5);
		else if (outPuts.at(1) < 0)
		outPuts.at(5) = 2*3.141592653 + outPuts.at(5);

		return outPuts;
	}
*/

struct InvMapRow{
      double coefficient;
      int    order;
      int    exp[6];
    };
struct InvMapRow_s{
      TSpline3 *coefficient;
      int    order;
      int    exp[6];
    };

std::map<int,std::vector<InvMapRow> > fMap;
std::map<int,std::vector<InvMapRow_s> > fMap_s;
std::vector< std::map<int,std::vector<InvMapRow> > > fMap_v;

bool ReadMapFile(const char *filename) {
  std::string mapfile = filename;
  /*if(mapfile.length()==0)
    mapfile = TGRUTOptions::Get()->S800InverseMapFile();*/
  if(mapfile.length()==0 || access(mapfile.c_str(),F_OK)==-1) {
    printf("no inverse map loaded and file \"%s\" not found.\n",mapfile.c_str());
    return false;
  }
  //static std::mutex inv_map_mutex;

  std::ifstream infile;
  infile.open(mapfile.c_str());
  std::string line;
  getline(infile,info);
  sscanf(info.c_str(),"S800 inverse map - Brho=%g - M=%d - Q=%d", &fBrho, &fMass, &fCharge);

  int par =0;
  while(getline(infile,line)) {
    if(line.find("----------")!=std::string::npos)
      continue;
    if(line.find("COEFFICIENT")!=std::string::npos){
      par++;
      continue;
    }
    unsigned int index;
    InvMapRow invrow;
    std::map<int,std::vector<InvMapRow> > Map;
    
    std::stringstream ss(line);
    ss >> index;
    //if(fMap_v.size() != 0)if((index-1) != fMap_v.at(idx)[par-1].size()) {
      //problems. 
    //}
    {
      std::string temp;
      ss >> temp;
      invrow.coefficient = std::atof(temp.c_str());
    }
//    ss >> invrow.coefficient;
    ss >> invrow.order;
    ss >> invrow.exp[0];
    ss >> invrow.exp[1];
    ss >> invrow.exp[2];
    ss >> invrow.exp[3];
    ss >> invrow.exp[4];
    ss >> invrow.exp[5];
    fMap[par-1].push_back(invrow);

    //printf("%i\t%s\n",index,line.c_str());

  }
  return true;
}

float MapCalc(int order,int par,float *input,int idx) {
  float cumul         = 0.0;
  float multiplicator = 0.0;
  std::vector<InvMapRow> vec = fMap_v.at(idx).at(par);
  for(unsigned int x=0; x<vec.size();x++) {
    if(order<vec.at(x).order) break;
    multiplicator = 1.0;
    for(int y=0;y<6;y++) {
      if(vec.at(x).exp[y]!=0)
        multiplicator *= pow(input[y],vec.at(x).exp[y]);
    }
    cumul += multiplicator*vec.at(x).coefficient;
  }
  return cumul;
}

float MapCalc_s(int order,int par,float *input,double z) {
  float cumul         = 0.0;
  float multiplicator = 0.0;
  std::vector<InvMapRow_s> vec = fMap_s.at(par);
  for(unsigned int x=0; x<vec.size();x++) {
    if(order<vec.at(x).order) break;
    multiplicator = 1.0;
    for(int y=0;y<6;y++) {
      if(vec.at(x).exp[y]!=0)
        multiplicator *= pow(input[y],vec.at(x).exp[y]);
    }
    cumul += multiplicator*vec.at(x).coefficient->Eval(z);
  }
  return cumul;
}


//void inv_map_inter(int runNumberS800, int runNumberATTPC)
void inv_map_inter()
	{


		//TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800, runNumberATTPC);

		//TFile* file = new TFile(digiFileName,"READ");

		//TTree* tree = (TTree*) file -> Get("cbmsim");
		//Int_t nEvents = tree -> GetEntries();

		//TTreeReader reader("cbmsim", file);
		// TTreeReaderValue<S800Calc> s800Calc(reader, "s800cal");
		//TTreeReaderValue<TClonesArray> ransacArray(reader, "ATRansac");

		TFile* outfile;
		TString  outFileNameHead = "inv_map_funcs.root";
		//TString outFileNameHead = TString::Format("runAnalyzed_%04d_%04d.root", runNumberS800, runNumberATTPC);

		outfile   = TFile::Open(outFileNameHead.Data(),"recreate");
		outfile->cd();
	  	outfile->mkdir("ATA");
	  	outfile->mkdir("YTA");
	  	outfile->mkdir("BTA");
	  	outfile->mkdir("DTA");

		//TH1D *epsilon_pp_reco = new TH1D ("epsilon_pp_reco", "#epsilon_{pp}", 100, 0, 10);
		//TH2D *thetacm_epsilon_pp_reco = new TH2D ("thetacm_epsilon_pp_reco", "#theta_{cm} #epsilon_{pp} #^{2}He", 200, 0, 20, 100, 0, 10);


		TTree *anatree = new TTree("anatree","new TTree");

		//anatree->Branch("ivt",&ivt);


		std::vector< std::string > mapfile_v;
		mapfile_v.push_back("invmap_01.inv");
		mapfile_v.push_back("invmap_02.inv");
		mapfile_v.push_back("invmap_03.inv");
		mapfile_v.push_back("invmap_04.inv");
		mapfile_v.push_back("invmap_05.inv");
		mapfile_v.push_back("invmap_06.inv");
		mapfile_v.push_back("invmap_07.inv");
		mapfile_v.push_back("invmap_08.inv");
		mapfile_v.push_back("invmap_09.inv");
		mapfile_v.push_back("invmap_10.inv");
	//	TInverseMap *inv_map = new TInverseMap(mapfile.c_str());

		//----------------------- Inv map ----------------------------------------------


		 std::cout<<"mapfile size : "<<mapfile_v.size()<<std::endl;
		//std::map<int,std::vector<InvMapRow> > fMap;
		//fMap_v.resize(10);//number of map file
		fMap_v.clear();
		std::vector<Double_t> mapdist_v;
		for(int i=0; i<mapfile_v.size(); i++){
			bool isRead=false;
			fMap.clear();
			isRead = ReadMapFile(mapfile_v.at(i).c_str());
    			fMap_v.push_back(fMap);
			mapdist_v.push_back(0.1*(1+i));//change that, find a way to know the distance pivot-target for each map.
			std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "<<mapfile_v.at(i)<<std::endl;
			if(!isRead) std::cout<<"! Inv map file not read : "<<mapfile_v.at(i)<<std::endl;
		}

    std::cout<<"map test : "<<fMap_v.at(0).at(0).at(0).coefficient<<" "<<fMap_v.at(0).at(1).at(1).coefficient<<std::endl;
		std::vector<InvMapRow> invrow_v;
		std::vector< std::vector<double> > coeff_v;
		Int_t Gsize=0,icoeff=0;
		for(int i=0; i<fMap_v.at(0).size(); i++) for(int j=0; j<fMap_v.at(0).at(i).size(); j++) Gsize++;
 
  		//std::vector<InvMapRow> vec = fMap_v.at(idx).at(par);
//std::cout<<"invrow : "<<fMap_v.at(0).size()<<" "<<fMap_v.size()<<std::endl;

TSpline3 *fSpline[Gsize]; 
TGraph *fGraph[Gsize]; 

		if(fMap_v.size()>0)
    	//	for(int i=0; i<fMap_v.size(); i++){
		for(int j=0; j<fMap_v.at(0).size(); j++){//loop on par//assuming the first map has the same format as the following maps 
			for(int k=0; k<fMap_v.at(0).at(j).size(); k++){//loop on coeff in par
				std::vector<double> buff_v;
    				InvMapRow_s invrow_s;
    				for(int i=0; i<fMap_v.size(); i++){//loop on maps
    			 		std::cout<<"mapcoeff : "<<i<<" "<<j<<" "<<k<<" "<<fMap_v.at(i).at(j).at(k).coefficient<<std::endl;
			 		buff_v.push_back((Double_t)fMap_v.at(i).at(j).at(k).coefficient);
			 //std::cout<<"invrow : "<<j<<" "<<invrow_v.at(j).coefficient<<std::endl;
				}
				switch(j) {
  				case 0:
    					outfile->cd("ATA");
    					break;
  				case 1:
    					outfile->cd("YTA");
    					break;
  				case 2:
    					outfile->cd("BTA");
    					break;
				case 3:
    					outfile->cd("DTA");
    					break;
				}
				coeff_v.push_back(buff_v);
				char fname[20];
      				sprintf(fname,"f_%d",icoeff);
				fSpline[icoeff] = new TSpline3(fname, &mapdist_v[0], &buff_v[0],fMap_v.size()); 
				fGraph[icoeff] = new TGraph(fMap_v.size(),&mapdist_v[0], &buff_v[0]);
				fSpline[icoeff]->Write(); 
				fGraph[icoeff]->Write(); 
			 	std::cout<<"debug  : "<<j<<" "<<k<<" "<<fMap_v.size()<<" "<<mapdist_v.back()<<" "<<fSpline[icoeff]->Eval(0.35)<<std::endl;
				invrow_s.coefficient = fSpline[icoeff];
				invrow_s.order = fMap_v.at(0).at(j).at(k).order;
				invrow_s.exp[0] = fMap_v.at(0).at(j).at(k).exp[0];
				invrow_s.exp[1] = fMap_v.at(0).at(j).at(k).exp[1];
				invrow_s.exp[2] = fMap_v.at(0).at(j).at(k).exp[2];
				invrow_s.exp[3] = fMap_v.at(0).at(j).at(k).exp[3];
				invrow_s.exp[4] = fMap_v.at(0).at(j).at(k).exp[4];
				invrow_s.exp[5] = fMap_v.at(0).at(j).at(k).exp[5];
    				fMap_s[j].push_back(invrow_s);
				icoeff++;
			}
    			//fMap_s[j].push_back(invrow_s);
		}
		std::cout<<"eval func "<<fMap_s[0].at(0).coefficient->Eval(0.5)<<" "<<fSpline[0]->Eval(0.5)<<std::endl;


		TCanvas *c1 = new TCanvas("c1","interpolation",0,0,1000,800);
		c1->Divide(2,2);
		for(int i=138; i<142; i++){
			c1->cd(i+1-138);
			fGraph[i]->SetMarkerStyle(3);
			fGraph[i]->GetXaxis()->SetTitle("Distance Target-Pivot [m]");
   			fGraph[i]->GetXaxis()->CenterTitle();
			fGraph[i]->GetYaxis()->SetTitle(Form("Coeff_%d",i));
   			fGraph[i]->GetYaxis()->CenterTitle();
			fGraph[i]->Draw("AP");
			fSpline[i]->SetLineColor(kRed);
			fSpline[i]->Draw("same");
		}
		//fMap_s[0].at(0).coefficient->Draw("same");
	

  float input[6];
  float ata=0;
  int order=5;
  float xfp=200., afp=0.02, yfp=100., bfp=0.03;
  input[0]  = - xfp / 1000.0;
  input[1]  = - afp;
  input[2]  =   yfp / 1000.0;
  input[3]  =   bfp;
  input[4]  =   0.0;
  input[5]  =   0.0;
  //ata=MapCalc(order, 0, input);

  std::cout<<MapCalc_s(order, 0, input,0.11)<<" "<<MapCalc_s(order, 0, input,0.23)<<" "<<MapCalc_s(order, 0, input,0.98)<<std::endl;

		/// --------------------- Event loop -------------------------------------------
/*
		for(Int_t i=0;i<nEvents;i++){

			 //std::cout<<s800cal->GetTS()<<"   "<<s800cal->GetSCINT(0)->GetDE()<<"  "<<trackCand.size()<<"  "<<s800cal->GetIsInCut()<<std::endl;


				//----------------------- S800 -------------------------------------------------

			std::vector <double> S800_invMapOut = get_invmap_vars(inv_map,S800_x0,S800_y0,S800_afp,S800_bfp);
			S800_ata = S800_invMapOut.at(0);
			S800_bta = S800_invMapOut.at(1);
			S800_yta = S800_invMapOut.at(2);
			S800_dta = S800_invMapOut.at(3);
			S800_thetaLab= S800_invMapOut.at(4);
			S800_phi= S800_invMapOut.at(5);

        		ivt=i;
        		anatree->Fill();
//------- New Bragg ------------------------------
		outfile->cd("Bragg");

//-------------------------------------------------
		}// Event loop
*/

	/// --------------------- End event loop ---------------------------------------
	outfile->cd();
	anatree->Write();

	outfile->Close();

} //end main
