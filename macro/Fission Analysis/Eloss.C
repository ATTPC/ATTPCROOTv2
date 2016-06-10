#include <iostream>
#include <vector>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

void Eloss()
{
TString dir = getenv("VMCWORKDIR");
TString fFileNamebase;
std::ifstream*  ElossData;
fFileNamebase = dir+"/macro/Simulation/data/109Pd_Eloss.txt";
ElossData = new std::ifstream(fFileNamebase.Data());
  if(ElossData->fail()){
    std::cout<<cRED<<" =  No file found! Please, check the path. Current :"<<fFileNamebase.Data()<<cNORMAL<<std::endl;

  }

  TCanvas *c1 = new TCanvas();
  c1->Draw();
  c1->Divide(1,2);

TH2D *Bragg = new TH2D("Bragg", "Bragg", 1000, 0,500, 1000, 0, 1);
TH2D *Interpolate = new TH2D("Interpolate","Interpolate", 1000, 0,200000, 100, 0, 1);

Double_t Eloss_elec, Eloss_nuc,SRIM_energy;
Double_t x = 0; //microns
Double_t x1 = 0;
Double_t x0 = 0;
Double_t Z = 46;
Double_t A = 109;
Double_t step = 1; //microns
Double_t energy = 190000; //keV
Double_t e1, e2, e3, dE1, dE2, dE3;
Double_t param0, param1, param2,param3,param4,param5,param6;
std::vector<std::tuple<Double_t,Double_t,Double_t>> SRIM_table;

std::cout<<cGREEN<<"Extracting data. Please wait...."<<cNORMAL<<std::endl;
while (!ElossData->eof()){   //Fill vector with datafile
  *ElossData>>SRIM_energy>>Eloss_elec>>Eloss_nuc; //keV >> keV/micron >> keV/micron
  SRIM_table.push_back(std::make_tuple(SRIM_energy, Eloss_elec, Eloss_nuc));
  Interpolate->Fill(SRIM_energy, Eloss_elec);
}//EndWhile

/*for (Int_t n = 0; n < SRIM_table.size(); n++){
    std::cout <<' '<< std::get<0> (SRIM_table.at(n))<<' '<<std::get<1> (SRIM_table.at(n))<<' '<<std::get<2> (SRIM_table.at(n))<<std::endl;

}//EndFor
*/

/*Int_t counter = SRIM_table.size()-3;
Double_t nuc_Loss, elec_Loss, total_Loss;
e1 = std::get<0> (SRIM_table.at(counter));
e2 = std::get<0> (SRIM_table.at(counter-1));
e3 = std::get<0> (SRIM_table.at(counter-2));
dE1 = std::get<1> (SRIM_table.at(counter));
dE2 = std::get<1> (SRIM_table.at(counter-1));
dE3 = std::get<1> (SRIM_table.at(counter-2));
Interpolate->Fill(e1, dE1);
Interpolate->Fill(e2, dE2);
Interpolate->Fill(e3, dE3);
Interpolate->Fit("enFit");
param0 = enFit->GetParameter(0);
param1 = enFit->GetParameter(1);

  while (energy>20){
    if(energy>std::get<0> (SRIM_table.at(counter))){
      elec_Loss = param0*energy+param1;
      x+=step;
      //nuc_Loss = (std::get<2> (SRIM_table.at(counter+1)))*step;
      //elec_Loss = (std::get<1> (SRIM_table.at(counter+1)))*step;
      total_Loss = elec_Loss; //+nuc_Loss;
      energy -= total_Loss;
      Bragg->Fill(x/1000, total_Loss*10);
      std::cout<<energy<<" "<<total_Loss<<" "<<x<<std::endl;
    }
    else{
      counter-=1;
      Interpolate->Reset();
      e1 = std::get<0> (SRIM_table.at(counter));
      e2 = std::get<0> (SRIM_table.at(counter-1));
      e3 = std::get<0> (SRIM_table.at(counter-2));
      dE1 = std::get<1> (SRIM_table.at(counter));
      dE2 = std::get<1> (SRIM_table.at(counter-1));
      dE3 = std::get<1> (SRIM_table.at(counter-2));
      Interpolate->Fill(e1, dE1);
      Interpolate->Fill(e2, dE2);
      Interpolate->Fill(e3, dE3);
      Interpolate->Fit("enFit");
      param0 = enFit->GetParameter(0);
      param1 = enFit->GetParameter(1);
    }
}//EndWhile
*/
TF1 *enFit = new TF1("enFit", "[0]+[1]*x+[2]*pow(x,2)+[3]*pow(x,3)+[4]*pow(x,4)+[5]*pow(x,5)+[6]*pow(x,6)", 0 , 200000);
enFit->SetParameters(1,1,1,1,1,1);
Interpolate->Fit("enFit");

param0 = enFit->GetParameter(0);
param1 = enFit->GetParameter(1);
param2 = enFit->GetParameter(2);
param3 = enFit->GetParameter(3);
param4 = enFit->GetParameter(4);
param5 = enFit->GetParameter(5);
param6 = enFit->GetParameter(6);

Double_t inter,loss;


std::cout<<"Calculating..................."<<std::endl;
while(energy>0){
  x+=step;
  inter = param0+ param1*energy+param2*pow(energy,2)+param3*pow(energy,3)+param4*pow(energy,4)+param5*pow(energy,5)+param6*pow(energy,6);
  loss = inter*step;
  energy -= loss;
  Bragg->Fill(x/1000, loss); //mm, keV

}
std::cout<<cRED<<"Range: "<<x/1000<<" mm"<<cNORMAL<<std::endl;

TF1 *brFit = new TF1("brFit", "[0]+[1]*x+[2]*pow(x,2)+[3]*pow(x,3)+[4]*pow(x,4)+[5]*pow(x,5)+[6]*pow(x,6)", 0 , 500);
enFit->SetParameters(1,1,1,1,1,1);
Bragg->Fit("brFit");


c1->cd(1);
Bragg->Draw();
brFit->Draw("Same");
/*c1->cd(2);
Interpolate->Draw();
enFit->Draw("same");
*/
} //EndMacro
