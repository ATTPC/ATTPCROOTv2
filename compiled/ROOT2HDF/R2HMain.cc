#include "R2HMain.hh"

int main(int argc, char *argv[])
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   TString FileName = "~/attpcroot/macro/e12014/simulation/eventGenerator/sym/output_digi.root";

   std::cout << " Opening File : " << FileName.Data() << std::endl;
   TFile *file = new TFile(FileName.Data(), "READ");

   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   // nEvents = 1000;
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtEventH");

   const int RANK = 1;
   const H5std_string FILE_NAME("output.h5");

   H5File *HDFfile = new H5File(FILE_NAME, H5F_ACC_TRUNC);

   for (Int_t i = 0; i < nEvents; i++) {
      // while (Reader1.Next()) {

      Reader1.Next();
      AtEvent *event = (AtEvent *)eventArray->At(0);
      Int_t nHits = event->GetNumHits();
      std::cout << " Event number " << i << " Number of hits " << nHits << "\n";

      H5std_string DATASET_NAME(Form("Event_[%i]", i));

      hsize_t dim[] = {(hsize_t)nHits}; /* Dataspace dimensions */
      DataSpace space(RANK, dim);

      ATHit_t hits[nHits];

      for (Int_t iHit = 0; iHit < nHits; iHit++) {
         auto &hit = event->GetHit(iHit);
         auto hitPos = hit.GetPosition();
         std::vector<AtHit::MCSimPoint> MCPoints = hit.GetMCSimPointArray();


         hits[iHit].x = hitPos.X();
         hits[iHit].y = hitPos.Y();
         hits[iHit].z = hitPos.Z();
         hits[iHit].t = hit.GetTimeStamp();
         hits[iHit].A = hit.GetCharge();

         /*if (MCPoints.size() > 0) { // N.B. Only one MC hit information is saved.
            hits[iHit].trackID = MCPoints.at(0).trackID;
            hits[iHit].pointIDMC = MCPoints.at(0).pointID;
            hits[iHit].energyMC = MCPoints.at(0).energy;
            hits[iHit].elossMC = MCPoints.at(0).eloss;
            hits[iHit].angleMC = MCPoints.at(0).angle;
            hits[iHit].AMC = MCPoints.at(0).A;
            hits[iHit].ZMC = MCPoints.at(0).Z;
	    }*/
         // std::cout<<hits[iHit].x<<"\n";
         // std::cout<<" MC points size "<<MCPoints.size()<<"\n";
      }

      CompType mtype1(sizeof(ATHit_t));
      mtype1.insertMember(MEMBER1, HOFFSET(ATHit_t, x), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER2, HOFFSET(ATHit_t, y), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER3, HOFFSET(ATHit_t, z), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER4, HOFFSET(ATHit_t, t), PredType::NATIVE_INT);
      mtype1.insertMember(MEMBER5, HOFFSET(ATHit_t, A), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER6, HOFFSET(ATHit_t, trackID), PredType::NATIVE_INT);
      mtype1.insertMember(MEMBER7, HOFFSET(ATHit_t, pointIDMC), PredType::NATIVE_INT);
      mtype1.insertMember(MEMBER8, HOFFSET(ATHit_t, energyMC), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER9, HOFFSET(ATHit_t, elossMC), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER10, HOFFSET(ATHit_t, angleMC), PredType::NATIVE_DOUBLE);
      mtype1.insertMember(MEMBER11, HOFFSET(ATHit_t, AMC), PredType::NATIVE_INT);
      mtype1.insertMember(MEMBER12, HOFFSET(ATHit_t, ZMC), PredType::NATIVE_INT);

      DataSet *dataset;
      dataset = new DataSet(HDFfile->createDataSet(DATASET_NAME, mtype1, space));

      dataset->write(hits, mtype1);

      delete dataset;
   }

   delete HDFfile;

   return 0;
}
