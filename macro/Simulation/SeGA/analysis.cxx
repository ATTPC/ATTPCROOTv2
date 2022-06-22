// Generic analysis for HELIOS MC simulation
// Y. Ayyad ayyadlim@frib.msu.edu 12/30/2020

void analysis(Int_t num_ev=10000)
{

    TString mcFileNameHead = "./data/SeGA";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    TString outFileNameHead = "SeGAana";
    TString outFileNameTail = ".root";
    TString outFileName     = outFileNameHead + outFileNameTail;

    AtSeGAPoint* point = new AtSeGAPoint();
    TClonesArray *pointArray=0;

    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");


    tree = (TTree*) file -> Get("cbmsim");
    tree -> SetBranchAddress("AtSeGAPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();

    if(nEvents>num_ev) nEvents=num_ev;
//
double x[100], y[100];
   int no = 20;
    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {

	tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
double energyLoss_sca = 0.0;
for(Int_t i=0; i<n; i++) {

            point = (AtSeGAPoint*) pointArray -> At(i);
            auto VolName="germaniumTarget";
            auto trackID = point -> GetTrackID();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            //std::cout<<" Track ID : "<<trackID<<std::endl;

    energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
}
x[iEvent] = energyLoss_sca;
y[iEvent] = n;
     }
   
   auto g = new TGraph(no,y,x);
   g->Draw("AC*");
}
