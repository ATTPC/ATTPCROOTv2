
#include <sstream>
#include <iostream>
#include<fstream>


void save_cut(TString CutName)
 {

	      TString rootname = CutName + ".root";
        TFile *file = new TFile(rootname.Data(), "recreate");
        file->cd();
        gROOT->FindObject(CutName)->Write();
        delete file;



}
