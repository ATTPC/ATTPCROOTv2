void trajtest()
{
	gSystem->Load("libGeom.so");
        TFile* f = new TFile("heliossim.root"); 
        TTree *t=(TTree*)f->Get("cbmsim"); 
        TClonesArray *fT=new TClonesArray("TGeoTrack"); 
        t->SetBranchAddress("GeoTracks",&fT) ; 
        //TGeoManager *geoMan = (TGeoManager*) f->Get("CBMGeom"); 
        TCanvas* c1 = new TCanvas("c1", "", 100, 100, 800, 800); 
        //geoMan->GetVolume("magnet")->Draw("same"); 
        TGeoTrack *tr; 
        for (Int_t j=0; j< t->GetEntriesFast(); j++) 
        {   
		std::cout<<" Entry "<<j<<"\n";

           t->GetEntry(j); 
           for (Int_t i=0; i<fT->GetEntriesFast(); i++) 
           { 
             std::cout<<" Tracks "<<i<<"\n";    
             tr=(TGeoTrack *)fT->At(i);      
             if(tr)tr->Draw("same"); 
           } 
        }
}
