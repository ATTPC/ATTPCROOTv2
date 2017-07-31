#include "ATCalibration.hh"



ClassImp(ATCalibration)

ATCalibration::ATCalibration()
{
fIsGainCalibrated   = kFALSE;
fIsJitterCalibrated = kFALSE;
}


ATCalibration::~ATCalibration()
{
}



Bool_t ATCalibration::IsGainFile()      { return fIsGainCalibrated;}
Bool_t ATCalibration::IsJitterFile()    { return fIsJitterCalibrated;}



void ATCalibration::SetGainFile(TString gainFile){
   fGainFile = gainFile;
   std::ifstream*  gainData;
   gainData = new std::ifstream(fGainFile.Data());
   if(gainData->fail()){
     std::cout<<" =  No Gain Calibration file found! Please, check the path. Current :"<<fGainFile.Data()<<std::endl;
     std::cout<<cRED<<" =  Proceeding with no gain calibration!!"<<cNORMAL<<std::endl;
     fIsGainCalibrated = kFALSE;
   }
   else{
     std::cout<<" == Gain calibration using: "<<cRED<<fGainFile.Data()<<cNORMAL<<std::endl;
     Double_t nPadNum, nCal;
     Int_t intPadNum;
     fGainCalib[10240] = {0};

     while (!gainData->eof()){
       *gainData>> nPadNum>>nCal;
       intPadNum = (int) nPadNum;
       fGainCalib[intPadNum] = nCal;
     }
     gainData->close();
     fIsGainCalibrated = kTRUE;
   }
 }



void ATCalibration::SetJitterFile(TString jitterFile){
   fJitterFile = jitterFile;
   std::ifstream* jitterData;
   jitterData = new std::ifstream(fJitterFile.Data());
   if(jitterData->fail()){
     std::cout<<" = No Jitter Calibration file found! Please check the path. Current :"<<cNORMAL<<fJitterFile.Data()<<std::endl;
     std::cout<<cRED<<" = Proceeding with no jitter calibration!!"<<cNORMAL<<std::endl;
     fIsJitterCalibrated = kFALSE;
   }
   else{
     std::cout<<" == Jitter calibration using: "<<cRED<<fJitterFile.Data()<<cNORMAL<<std::endl;
     fJitterCalib[10240] = {0};
     Int_t jiPadNum, jiCal;
     Double_t jdCal, jdPadNum;

     while (!jitterData->eof()){
       *jitterData>>jiPadNum>>jiCal;
       fJitterCalib[jiPadNum] = jiCal;
     }
     jitterData->close();
     fIsJitterCalibrated = kTRUE;
   }
 }



Double_t *ATCalibration::CalibrateGain(Double_t adc[512], Int_t padNum){
  if(!fIsGainCalibrated){
    return 0;
  }

   fPadNum       = padNum;
   fGnewadc[512] = 0;

   for(Int_t i = 0; i<512; i++) fGnewadc[i] = adc[i]*fGainCalib[fPadNum];

   return  fGnewadc;
}



Double_t *ATCalibration::CalibrateJitter(Double_t adc[512], Int_t padNum){
  if(!fIsJitterCalibrated){
    return 0;
  }

  fPadNum       = padNum;
  fGnewadc[512] = 0;
  Int_t tcorr   = 0;

  for(Int_t j = 0; j<512; j++){
    tcorr = j + fJitterCalib[fPadNum];
    if(tcorr >=0 && tcorr<=512 && adc[j] >0 && adc[j] <4000){
      fGnewadc[tcorr] = adc[j];
      //std::cout<<"fGnewadc: "<< fGnewadc[tcorr]<< std::endl;
    }
  }
  return fGnewadc;
}
