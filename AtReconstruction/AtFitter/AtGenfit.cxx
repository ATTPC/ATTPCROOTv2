#include "AtGenfit.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include "TGeoManager.h"


ClassImp(AtFITTER::AtGenfit)

   AtFITTER::AtGenfit::AtGenfit()
{

   fTPCDetID = 0;
   fCurrentDirection = 1;

   fKalmanFitter = new genfit::KalmanFitterRefTrack();
   fKalmanFitter->setMinIterations(5);
   fKalmanFitter->setMaxIterations(20);

   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fHitClusterArray = new TClonesArray("AtHitCluster");

   fMeasurementProducer =
      new genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement>(fHitClusterArray);
   fMeasurementFactory = new genfit::MeasurementFactory<genfit::AbsMeasurement>();
   fMeasurementFactory->addProducer(fTPCDetID, fMeasurementProducer);

   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., 20.)); // TODO kGauss
   genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
   materialEffects->init(new genfit::TGeoMaterialInterface());

   fPDGCandidateArray = new std::vector<Int_t>; // TODO
   fPDGCandidateArray->push_back(2212);

    std::cout<<" AtFITTER::AtGenfit::AtGenfit(): Checking materials that GENFIT will use "<<"\n";

  TGeoManager* gGeoMan  = (TGeoManager*)gROOT->FindObject("FAIRGeom");
  TObjArray* volume_list = gGeoMan->GetListOfVolumes();
  if (!volume_list) {
         std::cout<<cRED<< " Warning! Null list of geometry volumes."<<cNORMAL<<"\n";
   }

  int numVol = volume_list->GetEntries();

   for (int ivol = 0; ivol < numVol; ivol++) {
        TGeoVolume * volume = dynamic_cast <TGeoVolume *>(volume_list->At(ivol));
         if (!volume) {

              std::cout<< "Got a null geometry volume!! Skiping current list element"<<"\n";
            continue;
         }

         std::cout<<cGREEN<<" Volume name : "<<volume->GetName()<<cNORMAL<<"\n";

         TGeoMaterial * mat = volume->GetMedium()->GetMaterial();

         Int_t mat_indx = mat->GetIndex();

          if (mat->IsMixture()) {
            TGeoMixture * mixt = dynamic_cast <TGeoMixture*> (mat);
             int Nelements = mixt->GetNelements();
             std::cout<<cYELLOW<<" - Material : "<<mat->GetName()<<cNORMAL<<"\n";
          }else{
             std::cout<<cYELLOW<<" - Material : "<<mat->GetName()<<cNORMAL<<"\n";
          }
     }

}

AtFITTER::AtGenfit::~AtGenfit()
{
      delete fKalmanFitter;
      delete fGenfitTrackArray;
      delete fHitClusterArray;
      delete fMeasurementProducer;
      delete fMeasurementFactory;
      delete fPDGCandidateArray;

}

void AtFITTER::AtGenfit::Init()
{
    std::cout<<" AtFITTER::AtGenfit::Init() "<<"\n";

   fHitClusterArray->Delete();
   fGenfitTrackArray->Delete();
}

void AtFITTER::AtGenfit::SetMinIterations(Int_t value)
{
   fKalmanFitter->setMinIterations(value);
}
void AtFITTER::AtGenfit::SetMaxIterations(Int_t value)
{
   fKalmanFitter->setMaxIterations(value);
}

TClonesArray* AtFITTER::AtGenfit::GetGenfitTrackArray()
{
  return fGenfitTrackArray;
}


bool AtFITTER::AtGenfit::FitTracks(AtPatternEvent &patternEvent)
{

   std::vector<genfit::Track> genfitTrackArray;

  std::vector<AtTrack>& patternTrackCand =  patternEvent.GetTrackCand();


  std::cout<<" AtFITTER::AtGenfit::FitTracks - Number of candidate tracks : "<<patternTrackCand.size()<<"\n";

  for(auto track : patternTrackCand)
  {
        auto hitClusterArray = track.GetHitClusterArray();

         TVector3 pos_res;
         TVector3 mom_res;
         TMatrixDSym cov_res;



        if(hitClusterArray->size()>3 && !track.GetIsNoise()){// TODO Check minimum number of clusters

                  fHitClusterArray -> Delete();
                  genfit::TrackCand trackCand;

                  std::reverse(hitClusterArray->begin(),hitClusterArray->end()); // TODO: Reverted to adapt it to simulation

                 //Adding clusterized  hits
                 for (auto cluster : *hitClusterArray) {
                    TVector3 pos = cluster.GetPosition();
                    Int_t idx = fHitClusterArray->GetEntriesFast();
                    new ((*fHitClusterArray)[idx]) AtHitCluster(cluster);
                    trackCand.addHit(fTPCDetID, idx);
                    //std::cout<<" Adding  cluster "<<idx<<"\n";
                    //std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<pos.Z()<<"\n";
                 }

		   TVector3 iniPos = hitClusterArray->front().GetPosition(); //TODO Check first cluster is the first in time
                 TVector3 posSeed(iniPos.X(),iniPos.Y(),1000.0-iniPos.Z());
                 posSeed.SetMag(posSeed.Mag()/10.);

                 TMatrixDSym covSeed(6);//TODO Check where COV matrix is defined, likely in AtPattern clusterize (hard coded in AtSpacePoint measurement)
                 TMatrixD covMatrix = hitClusterArray->front().GetCovMatrix();
                  for (Int_t iComp = 0; iComp < 3; iComp++)
                    covSeed(iComp, iComp) = covMatrix(iComp, iComp)/100.;// unit conversion mm2 -> cm2

                  for (Int_t iComp = 3; iComp < 6; iComp++)
                    covSeed(iComp, iComp) = covSeed(iComp - 3, iComp - 3);


                  //Initial parameters from pattern recognition. For the moment, I just hardcode the paramters for a specific case. Momentum will be determined from BRho.
                  //Test case 12Be+p elastic 15A MeV, 60.5 deg, 12 MeV, 2 T, 0.50215 Tm, mometum 0.150541
                  //                                    71.619 deg, 5.00336 MeV, 0.32364 Tm, 0.0970261 MeV/c
                  //                                   75.8167 deg, 3.01889 MeV, 0.25126 Tm, 0.753272 MeV/c
                  Double_t theta  = 75.8167*TMath::DegToRad();//track->GetGeoTheta();
                  Double_t radius = 0.0;//track->GetGeoRadius();
                  Double_t phi    = 0.0*TMath::DegToRad();//track->GetGeoPhi();

                  Double_t brho = 0.25126; //12 MeV proton
                  Double_t p_mass = 1.00727647; //amu
                  Int_t p_Z = 1;

                  std::tuple<Double_t,Double_t> mom_ener = GetMomFromBrho(p_mass,p_Z,brho); //TODO Change to structured bindings when C++17
                  Double_t momSeedMag = std::get<0>(mom_ener);
                  TVector3 momSeed(0., 0., momSeedMag); //
                  momSeed.SetTheta(theta);//TODO: Check angle conventions
                  momSeed.SetPhi(phi); // TODO
                  trackCand.setCovSeed(covSeed);
                  trackCand.setPosMomSeed(posSeed, momSeed,p_Z);
                  trackCand.setPdgCode(2212);
                  //trackCand.Print();


                  genfit::Track *gfTrack = new ((*fGenfitTrackArray)[fGenfitTrackArray -> GetEntriesFast()]) genfit::Track(trackCand, *fMeasurementFactory);
                  gfTrack -> addTrackRep(new genfit::RKTrackRep(2212));// TODO: Forcing proton track representation

                  genfit::RKTrackRep *trackRep = (genfit::RKTrackRep *) gfTrack -> getTrackRep(0);

		     try {
                        fKalmanFitter -> processTrackWithRep(gfTrack, trackRep, false);
                  } catch (genfit::Exception &e) {}


                  genfit::FitStatus *fitStatus;
                  try {
                    fitStatus = gfTrack -> getFitStatus(trackRep);
                    std::cout<<" Is fitted? "<<fitStatus->isFitted()<<"\n";
                    std::cout<<" Is Converged ? "<<fitStatus->isFitConverged()<<"\n";
                    std::cout<<" Is Converged Partially? "<<fitStatus->isFitConvergedPartially()<<"\n";
                    std::cout<<" Is pruned ? "<<fitStatus->isTrackPruned()<<"\n";
                    fitStatus->Print();
                  } catch (genfit::Exception &e) {
                    //return 0;
                  }



                  genfit::MeasuredStateOnPlane fitState;
                  try {
                    fitState = gfTrack -> getFittedState();
                    fitState.Print();
                    //Fit result
                    fitState.getPosMomCov(pos_res,mom_res,cov_res);
                    std::cout<<" Total Momentum : "<<mom_res.Mag()<<" - Position : "<<pos_res.X()<<"  "<<pos_res.Y()<<"  "<<pos_res.Z()<<"\n";
                  } catch (genfit::Exception &e) {}


                  //gfTrack ->Print();



        }//if hitClusterArray


  }//iTrack

    std::cout<<" End of GENFIT "<<"\n";
    std::cout<<"               "<<"\n";

    return 0;


}
