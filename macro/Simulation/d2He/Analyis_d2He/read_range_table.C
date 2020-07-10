Double_t Linter(std::vector <std::vector<Double_t>> RE, Double_t energy, Int_t model)
{
	double range=0.;
	if(energy < RE.at(0).at(0)) { // Linear to zero before first knot
		range = RE.at(0).at(model)/RE.at(0).at(0)*energy;
		return range;
	}

	for(int k = 0; k < RE.size(); k++) {
		if(energy >= RE.at(k).at(0) && energy < RE.at(k+1).at(0)) {
			range = RE.at(k).at(model) + (RE.at(k+1).at(model) - RE.at(k).at(model)) / (RE.at(k+1).at(0) - RE.at(k).at(0))*(energy - RE.at(k).at(0));
			return range;
		}
	}
	if(energy > RE.back().at(0)) { // Linear to zero before first knot
		std::cout<<"! Out of energy (linear interpolation) !"<<std::endl;
		return 0;
	}
}
TGraph *graphtable;
Double_t ERfunc(Double_t *x, Double_t *)
{
  return graphtable->Eval(x[0])*0.738;//0.738 for SRIM data to match with GEANT4
}

Double_t fitf(Double_t *x, Double_t *par)
{
   Double_t fitval = par[0] + par[1]*x[0] + par[2]*pow(x[0],2) + par[3]*pow(x[0],3) + par[4]*pow(x[0],4) + par[5]*pow(x[0],5);
   return fitval;
}

void read_range_table ()
{
std::ifstream fRE("p_in_d_LISE_1atm.dat");
std::vector <std::vector<Double_t>> Range_Energy;
Double_t l1=0, l2=0, l3=0, l4=0, l5=0;

for (string line; getline(fRE, line);) {
  std::stringstream parse_die(line);
  std::vector<Double_t> iRE;
  parse_die >> l1 >> l2 >> l3 >> l4 >> l5;
  iRE.push_back(l1);
  iRE.push_back(l2);
  iRE.push_back(l3);
  iRE.push_back(l4);
  iRE.push_back(l5);
  Range_Energy.push_back(iRE);
}
fRE.close();

for(Int_t i=0; i<300; i++){
  Double_t energy_p=(Double_t)i*0.01;
  std::cout<<"LINTER :: "<<energy_p<<" "<<Linter(Range_Energy,energy_p,1)<<std::endl;
}

Int_t v_size = Range_Energy.size();
Double_t X[v_size];
Double_t Y[v_size];
for(Int_t i=0; i<v_size; i++){
  X[i]=Range_Energy.at(i).at(0);//*1.18 with model 1 to fit the data, but little shift at energy <1 MeV
  Y[i]=Range_Energy.at(i).at(1);
}

//TGraph *Gtofit = (TGraph*)gPad->GetPrimitive("tofit");
TH2D *Gtofit = (TH2D*)gPad->GetPrimitive("Eloss_vs_range_p1__1");

TF1 *f1=new TF1("f1",ERfunc,0,5,0);
graphtable = new TGraph(v_size,X,Y);
f1->Draw("same");
graphtable->Draw("same");

TF1 *func = new TF1("fitf",fitf,0.3,1.5,6);
func->SetParameters(2,50,100,0,0);
func->SetParLimits(0,0,4);
func->SetParLimits(1,0,100);
func->SetParLimits(2,0,200);
func->SetParLimits(3,0,100);
func->SetParLimits(4,0,100);
func->SetParLimits(5,0,10);

// Fit histogram in range defined by function
Gtofit->Fit(func,"r");

//Gtofit->Fit(f1,"V","",0.1,3.0
}
