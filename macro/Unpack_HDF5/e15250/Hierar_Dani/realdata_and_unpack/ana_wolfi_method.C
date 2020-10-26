#include <iostream>
#include <fstream>
#include <set>
#include "TSpectrum2.h"
#include "TH2.h"
#include "TF2.h"
#include "TCanvas.h"
#include "TMath.h"
#include "TROOT.h"


double largest(double arr[], int n)
{
    int i;
      double max = arr[0];

    for (i = 1; i < n; i++)
        if (arr[i] > max)
            max = arr[i];

    return max;
}


void ana_wolfi_method(TString runNumber="_0290")
{
  TF1 *f = new TF1("f1","[0]+[1]*x",0,1000);
  gStyle->SetOptStat(0);
TCanvas *c1 = new TCanvas("c1","c1",500,500);
    c1->Divide(2,1);
   Int_t  npeaks = 5;
    ofstream ofile;
    ofile.open("data_track_hitcond_5000.txt",ios::app);
    TString FileNameHead = "run" + runNumber +""; // "_output_noInhibit";
    TString workdir = getenv("VMCWORKDIR");
    TString FilePath = "/media/dani/ubuntuwindows/GitHub_Yassidfork/ATTPCROOTv2/macro/Unpack_HDF5/e15250/Hierar_Dani/realdata_and_unpack/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;
    //std::cout<<" Opening File : "<<FileName.Data()<<std::endl;

    TChain *chain = new TChain("cbmsim");
    chain->Add(FileName);
    TClonesArray* fATEventArray= NULL;
    chain->SetBranchAddress("ATEventH", &fATEventArray);
    Int_t nentries = chain->GetEntries();
    std::cout<<" Number of events : "<<nentries<<std::endl;
    //TH3D* track_s = new TH3D("tracks","track_s",300,-150,150,300,-150,150,500,0,500);

      //TView *view = TView::CreateView(1);
      //view->SetRange(-250,-250,00,250,250,1000);
        // TPolyMarker3D *m = new TPolyMarker3D(10000);
      //    TPolyMarker *m = new TPolyMarker(10000);

      TSpectrum *s = new TSpectrum(npeaks);
     TH1F *hang = new TH1F("hnew","hnew",90,0,180);


    TVector3 coords;

  
    double distmin=25.0;
    double distmax= 50.0;
    double angle;
    double zmax=0.0;
    Int_t n = 10000;
    Int_t evnt =0;
    Bool_t amp = kTRUE;
    Bool_t pad = kFALSE;
    Int_t nmax=0;
    //Int_t p;
    int m;
    Int_t negative_threshold =-1000;
    Int_t amplitude_threshold = 0;
    std::vector<ATHit> *hitArray;
    double zsign=0.0;

    Float_t *MeshArray;
    int j =0;

double Zvert;
    //LOOP over the number of EVENTS

    for(Int_t i=0; i <190; i++){
      double x[10000]={0.0};
      double y[10000]={0.0};
      double z[10000]={0.0};
      double r[10000]={0.0};
      double rsel=r[0];
      double zsel1[10000]={0.0};
      //double rfull[10000]={0.0};
      double rselected[1000]={0.0};
      //double nmax=0;
      int p=0;

        if (i % 1000 == 0) std::cout<<"Processed " << i<<" events"<<std::endl;
        chain->GetEntry(i);
        evnt++;
        amp = kFALSE;
        pad = kFALSE;
        //padCounter = 0;
        ATEvent *fEvent;
        fEvent = (ATEvent*) fATEventArray->At(0);
        MeshArray = fEvent->GetMesh();
        hitArray = fEvent->GetHitArray();
        Int_t NumHits  = fEvent->GetNumHits();
        //cout<<evnt<<"\t"<<"Number of hits="<<NumHits<<endl;
        n= NumHits;

    //  if (evnt ==5){
      if (NumHits<=1000){
          //  event++;
        for (int j =0; j<NumHits; j++){

                ATHit *Hit = fEvent->GetHit(j);
                coords= Hit->GetPosition();
                x[j] = coords.X();
                y[j]= coords.Y();
                z[j]= coords.Z();

		// m->SetPoint(j,coords.x(),coords.y(),coords.z());


	}
        for (int l=0; l<n;l++){

       for (int k=0;k<n;k++){

		double distxy=sqrt(pow((x[l]-x[k]),2) +pow((y[k]-y[l]),2));
		double distz=sqrt(pow((z[l]-z[k]),2));
    
		double disttot=sqrt(pow(distxy,2)+pow(distz,2));
                if(disttot<=distmax && distmin <= disttot){
                  
		   angle=atan2(distxy,distz)/0.01745;
                  if (angle ==0)
                  {
                    angle = -10000.0;
                  }
                  hang->Fill(angle);

                    r[k]=sqrt(pow(x[k],2)+pow(y[k],2));
                   //cout<<"r="<<r[k]<<endl;

                  }
                  int arr_n = sizeof(r)/sizeof(r[0]);

                  rsel =largest(r,arr_n);
                  //cout<<x[k]<<"\t"<<y[k]<<"\t"<<r[k]<<"\t"<<rsel<<endl;
}

}
//hang->Draw();
Int_t nfound = s->Search(hang,1.0,"",0.08);
//cout<<"peaks found="<<nfound<<endl;
double *peak_pos = s->GetPositionX();
angle = *peak_pos;
hang->Draw();
//hang->Reset("ICESM");


if (nfound ==1 && (angle> *peak_pos-2) && (angle< *peak_pos+2)){
for( int m=0;m<NumHits;m++){

  //nmax=0;
  if (r[m]==rsel) {
    zmax=z[m];
//cout<<"zmax="<<zmax<<endl;
  nmax=m;
// cout<<"nmax"<<"\t"<<nmax << "m=" << m <<" rsel="<<r[m]<<endl;
 break;
}}
//for(m=nmax; m<NumHits; m++){
  for(m=0; m<nmax; m++){

 zsel1[p]=z[m];
 rselected[p]=r[m];
p++;
}


//put back here
   }

}
//double angle = *peak_pos;
TGraph *g = new TGraph(n,x,y);
TGraph *g1 = new TGraph(n,z,x);
      TGraph *g2 = new TGraph(n,z,y);
      TGraph *g4 =  new TGraph(m,zsel1,rselected);
    //  TGraph *g4 =  new TGraph(n,z,r);

      g->SetMarkerColor(2);
      g->SetMarkerStyle(7);
      g1->SetMarkerColor(1);
      g1->SetMarkerStyle(7);
      g2->SetMarkerColor(4);
      g2->SetMarkerStyle(7);
      g4->SetMarkerColor(8);
      g4->SetMarkerStyle(7);
      //g->Draw("AP");

//c1->cd(1);
//g->Draw("AP");
//c1->cd(2);
g4->Draw("AP");
g4->Fit(f);
//cout<<"slope"<<f->GetParameter(1)<<endl;
double slope = f->GetParameter(1);
//cout<<"slope="<<slope<<endl;
if (slope>0){
angle = 180.0-angle;
  }
  else angle = angle;


//angle = angle+90;
double delZ= TMath::Pi()*(rsel*0.5)/tan(angle*0.01745);
double BRho=0.001*2.0*rsel/sin(0.01745*angle); // Using Wolfi's formula
Zvert =zmax+delZ;
//out<<"Zvert-Zmax="<<Zvert-zmax<<endl;
if (angle<90.0){
  cout<<"#############Back scattered protons, Hurray#########"<<endl;
cout<<"event="<<evnt-1<<"\t"<<"r_curvature"<<rsel/2.0<<"\t"<<"angle="<< angle<<"\t"<<Zvert<<endl;
ofile<<evnt-1<<"\t"<<angle<<"\t"<<rsel/2.0<<"\t"<<BRho<<endl;
}
  //c1->Draw();

}


  }
