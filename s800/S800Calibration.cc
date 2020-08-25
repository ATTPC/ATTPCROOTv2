#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>

#include "S800Calibration.hh"
#include "S800.hh"
#include "S800defs.h"

#include "lmcurve.h"
#include "lmmin.h"
#include "lmfit.h"
using namespace std;


S800Calibration::S800Calibration(){
  // S800
  fped.resize(2);
  fslope.resize(2);
  foffset.resize(2);
  fbad.resize(2);
  for(int i=0;i<2;i++){
    //Initialize the CRDC vectors to 0s and 1s
    fped[i].resize(S800_FP_CRDC_CHANNELS,0);
    fslope[i].resize(S800_FP_CRDC_CHANNELS,1);
    foffset[i].resize(S800_FP_CRDC_CHANNELS,0);
  }

}


S800Calibration::S800Calibration(S800Settings* setting){
   fSett = setting;

   // S800
   fped.resize(2);
   fslope.resize(2);
   foffset.resize(2);
   fbad.resize(2);
   for(int i=0;i<2;i++){
     //Initialize the CRDC vectors to 0s and 1s
      fped[i].resize(S800_FP_CRDC_CHANNELS,0);
      fslope[i].resize(S800_FP_CRDC_CHANNELS,1);
      foffset[i].resize(S800_FP_CRDC_CHANNELS,0);
   }

   fcrdccal.resize(S800_FP_CRDC_CHANNELS);
   ReadCrdcCalibration(fSett->CalFile(),fSett->PedestalFile());
   ReadCrdcBadPads(fSett->BadFile());

   fICoffset.resize(S800_FP_IC_CHANNELS);
   fICslope.resize(S800_FP_IC_CHANNELS);
   ReadICCalibration(fSett->CalFileIC());
}

S800Calibration::~S800Calibration(){
  fped.clear();
  fslope.clear();
  foffset.clear();
  fbad.clear();
  fcrdccal.clear();
  fICoffset.clear();
  fICslope.clear();
  //std::cout << "destructor" << std::endl;
}

// S800
// CRDC cathode pedestal/gain
void S800Calibration::ReadCrdcCalibration(const char *filename, const char *pedfile){
  TEnv crdcpedestals;//
  Int_t status =crdcpedestals.ReadFile(pedfile,kEnvLocal);
  if (status == -1 ){
    Warning(__FUNCTION__,"Could not read CrdcPedFile %s",pedfile);
  }else{
   Info(__FUNCTION__,"Reading CrdcPedFile %s",pedfile);
  }

  TEnv crdccal;
  status = crdccal.ReadFile(filename,kEnvLocal);
  if ( status == -1 ){
    Warning(__FUNCTION__,"Could not read CrdcCalFile %s",filename);
  }else {
    Info(__FUNCTION__,"Reading CrdcCalFile %s",filename);
  }

   for (int c=0; c<2; c++) {
      for(int p=0; p<S800_FP_CRDC_CHANNELS; p++) {
	 fped[c][p] = crdcpedestals.GetValue(Form("Crdc.%d.Ped.%03d",c,p),0.0);
	 fslope[c][p] = crdccal.GetValue(Form("Crdc.%d.Slope.%03d",c,p),1.0);
	 foffset[c][p] = crdccal.GetValue(Form("Crdc.%d.Offset.%03d",c,p),0.0);
      }
   }

   // for (int p=0;p<S800_FP_CRDC_CHANNELS;p++){
   //   std::cout<<fped[0][p]<<"   "<<fslope[0][p]<<"  "<<foffset[0][p]<<std::endl;
   // }
}


// CRDC cathode bad pad
void S800Calibration::ReadCrdcBadPads(const char *filename){
  TEnv bad;// = new TEnv(filename);
  Int_t status = bad.ReadFile(filename,kEnvLocal);
  if (status == -1 ){
    Warning(__FUNCTION__,"Could not read bad pad file %s",filename);
  }else {
    Info(__FUNCTION__,"Reading bad pad file %s",filename);
  }

   for(UShort_t i=0;i<2;i++){
      fbad[i].resize(bad.GetValue(Form("Crdc.%d.Nofbadpads",i),0));
      for(UShort_t k=0;k<fbad[i].size();k++){
	 fbad[i][k] = bad.GetValue(Form("Crdc.%d.badpad.%d",i,k),0);
	 //std::cout << i << " " << k << " " << fbad[i][k] << std::endl;
      }
   }
}


//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv will be removed vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void S800Calibration::CrdcCal(std::vector<Short_t> channel, std::vector<Short_t> data, Int_t id){
   Short_t index;
   std::vector<Float_t> sum;
   std::vector<Short_t> samples;
   sum.clear();
   sum.resize(S800_FP_CRDC_CHANNELS);
   samples.clear();
   samples.resize(S800_FP_CRDC_CHANNELS);
   fcrdccal.clear();
   fcrdccal.resize(S800_FP_CRDC_CHANNELS);
   if(channel.size() != data.size()){
      std::cerr << " channel ("<<channel.size()<<") and data ("<<data.size()<<") have different sizes " << std::endl;
      return;
   }

   for(UShort_t f=0; f < channel.size(); f++){
      index = channel[f];
      sum[index] += data[f] - fped[id][index];
      samples[index]++;
   }
   for(UShort_t ch=0; ch < S800_FP_CRDC_CHANNELS; ch++){
      if (samples[ch] > 0) {
	 fcrdccal[ch] = sum[ch]; // / fSett->SampleWidth();
	 fcrdccal[ch] *= fslope[id][ch];
	 fcrdccal[ch] += foffset[id][ch];
      } else {
	fcrdccal[ch]=sqrt(-1.0);
      }
   }

   return ;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Will be removed^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

std::vector<Float_t> S800Calibration::GetCalibratedCrdcPads(std::vector<Short_t> channels, std::vector<Short_t> data, Int_t id,vector<Float_t> &PedSubtractedPads){
  //Channels should be a vector holding which pads fired in this event
  //data should be the read values in the same order as the channels
  //NOTE each pad will always have several (3-4) reads in each event
  //these have to be summed together in order to get the actual read

   Short_t index;
   //   std::vector<Float_t> sum;
   std::vector<Float_t> samples;

   //sum.resize(S800_FP_CRDC_CHANNELS);
   samples.resize(S800_FP_CRDC_CHANNELS,0);

   std::vector<Float_t> calibratedPadValues;
   calibratedPadValues.resize(S800_FP_CRDC_CHANNELS,0);
   PedSubtractedPads.resize(S800_FP_CRDC_CHANNELS,0);

   if(channels.size() != data.size()){
     //the length of the channels vector and the data vector should be the same.
      std::cerr << " channel ("<<channels.size()<<") and data ("<<data.size()<<") have different sizes " << std::endl;
      throw 1;
   }

   //First loop over the data that was given and add up all the reads for each pad
   //With the pedistal subtracted
   for(UShort_t f=0; f < channels.size(); f++){
      index = channels[f];

      calibratedPadValues[index] += (data[f] - fped[id][index]);
      PedSubtractedPads  [index] += (data[f] - fped[id][index]);
      samples[index]++;
   }

   //Now for all the channels that had at least 1 read in it
   //apply gain calibrations
   for(UShort_t ch=0; ch < S800_FP_CRDC_CHANNELS; ch++){

      if (samples[ch] > 0) {
	// calibratedPadValues[ch] = calibratedPadValues[ch] / samples[ch]; // / fSett->SampleWidth();
	// PedSubtractedPads[ch] = PedSubtractedPads[ch] / samples[ch]; // / fSett->SampleWidth();

	calibratedPadValues[ch] *= fslope[id][ch];
	calibratedPadValues[ch] += foffset[id][ch];
      } else {
	calibratedPadValues[ch]=sqrt(-1.0);
      }
   }

   return calibratedPadValues;

}



//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv will be removed vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void S800Calibration::SetCrdc(std::vector<Short_t> channel, std::vector<Short_t> data,
			      Float_t tac, Float_t anode, Int_t id) {
   fcrdc.Clear();
   fcrdc.SetID(id);
   this->CrdcCal(channel,data,id);
   fcrdc.SetCal(fcrdccal);

   //  cout<<"ID IS "<<id<<endl;
   // for (int i=0;i<fcrdccal.size();i++){
   //   cout<<fcrdccal[i]<<" ";
   // }cout<<endl;
   // cin.get();

   // debug
   // std::cout << "\n" << id << std::endl;
   // for (int i = 0; i < channel.size(); i++) {
   //    std::cout << channel[i] << " " << data[i] << std::endl;
   // }
   double x = fSett->XOffset(id) + fSett->XSlope(id) * this->CalcX();
   double y = fSett->YOffset(id) + fSett->YSlope(id) * tac;
   fcrdc.SetX(x);
   fcrdc.SetY(y);
   fcrdc.SetTAC(tac);
   fcrdc.SetAnode(anode);
}

Float_t S800Calibration::CalcX(){

   // Cluster search
   Bool_t flg_clstr = kFALSE;
   Int_t  iclstr = -1;
   const Int_t maxclstr = S800_FP_CRDC_CHANNELS;
                             // maximum number of clusters (I know this is too many...)
   Int_t clstr[maxclstr][3]; //clstr[][0] is left edge clstr[][1] right edge clstr[][2] is width
   Float_t maxchg[maxclstr]; //
   Float_t maxpad[maxclstr];
   const Float_t qmax = 25.; // MINUMUM value of max charge
                             // to be considered to form a cluster
   const Float_t qthr =  8.; // MINUMUM value of charge to be considered to be hit
   Float_t tmp_qmax = 0.0;
   Int_t gclstr = 0;

   for (UShort_t i = 0; i < S800_FP_CRDC_CHANNELS; i++) {
      if (IsBad(i,fcrdc.GetID())) continue;
      if ((flg_clstr == kFALSE) && (!std::isnan(fcrdccal[i]))) {
	 flg_clstr = kTRUE;
	 iclstr = iclstr + 1;
	 clstr[iclstr][0] =  i; // leading edge
	 clstr[iclstr][1] = -1; // trailing edge (tentative)
	 maxchg[iclstr] = fcrdccal[i];
	 maxpad[iclstr] = i;//Added by sam 3/19/2015 maybe is a good thing
      } else if ((flg_clstr == kTRUE) && (!std::isnan(fcrdccal[i]))) {
	 if (fcrdccal[i] > maxchg[iclstr]) {
	    maxchg[iclstr] = fcrdccal[i];
	    maxpad[iclstr] = i;
	 }
      } else if ((flg_clstr == kTRUE) && (std::isnan(fcrdccal[i]))) {
	 flg_clstr = kFALSE;
	 clstr[iclstr][1] = i - 1;
	 clstr[iclstr][2] = i - clstr[iclstr][0];
	 //if (maxpad[iclstr] < qmax) {
         if (maxchg[iclstr] < qmax) { // As pointed out by Sasano-san 2015/3/9
	    iclstr = iclstr - 1;
	 }
      }
   }

   // if a cluster is located at the end of the cathode
   if (flg_clstr == kTRUE) {
      clstr[iclstr][1] = S800_FP_CRDC_CHANNELS - 1;
      clstr[iclstr][2] = S800_FP_CRDC_CHANNELS - clstr[iclstr][0];
   }

   if (iclstr == 0) {
     //There is only one possible good cluster.  Use that one
     gclstr = 0;
   } else if (iclstr > 0) {
     //There is more than one possible good cluster.

     /*********************************************************************
       NOTE:   MAY HAVE TO THROW ALWAY EVENTS WITH MORE THAN ONE CLUSTER
     **********************************************************************/

     //Will use the largest one

      tmp_qmax = maxchg[0];
      // look for the GOOD cluster (gclstr) to be used to the analysis
      // (the cluster with the max charge is used)
      for (Int_t i = 0; i < iclstr + 1; i++) {
	 if (maxchg[i] > tmp_qmax) {
	    tmp_qmax = maxchg[i];
	    gclstr = i;
	 }
      }
   } else {
     //NO Clusters have been found return NAN
      return sqrt(-1.0);
   }

   fcrdc.SetMaxChg((Float_t)maxchg[gclstr]);
   fcrdc.SetMaxPad((Float_t)maxpad[gclstr]);
   fcrdc.SetNumClusters(iclstr+1);//iclstr is 0 when there is 1 cluster
   fcrdc.SetMaxClusterWidth(clstr[gclstr][2]);

   Int_t j = 0;
   Double_t xpad[S800_FP_CRDC_CHANNELS], qcal[S800_FP_CRDC_CHANNELS]; // (I know this is too many...)
   Float_t sum_q = 0.0, sum_qx = 0.0, sum_qxx = 0.0;

   for (UShort_t i = clstr[gclstr][0]; i <= clstr[gclstr][1]; i++) {
      if (IsBad(i,fcrdc.GetID())) continue;
      if (fcrdccal[i] < qthr) continue;
      sum_q   += fcrdccal[i];
      sum_qx  += fcrdccal[i] * i;
      sum_qxx += fcrdccal[i] * i * i;
      // for fit
      xpad[j] = (Double_t)i;
      qcal[j] = (Double_t)fcrdccal[i];
      // debug
      // std::cout << j << " " << xpad[j] << " " << qcal[j] << std::endl;
      j++;
   }

   fcrdc.SetXpad(xpad,j);
   fcrdc.SetYpad(qcal,j);

   Double_t xcog  = (Double_t)sum_qx/sum_q;
   Double_t sigma = (Double_t)TMath::Sqrt(sum_qxx/sum_q - (sum_qx/sum_q)*(sum_qx/sum_q));
   if(xcog < 0 || xcog > S800_FP_CRDC_CHANNELS){ // no gravity center found
      xcog = sqrt(-1.0);
      std::cout << "Something strange happens with the CRDC data." << std::endl;
   }
   fcrdc.SetXcog((Float_t)xcog);
   fcrdc.SetCathode((Float_t)sum_q);

   if (fSett->XFit()) {

      // Do fit
      Double_t xfit;
      Double_t par[3];
      Int_t    n_par = 3; // number of parameters in model function f

      lm_status_struct status;
      lm_control_struct control = lm_control_double;
      control.printflags = 0; // monitor status (+1) and parameters (+2)

      // Initial guess
      par[0] = (Double_t)maxchg[gclstr];
      par[1] = xcog;
      par[2] = sigma;

      if (fSett->XFitFunc() == 1) {
	 // Secant Hyperbolic Squared
	 lmcurve_fit( n_par, par, j, xpad, qcal, sechs, &control, &status );
      } else if (fSett->XFitFunc() == 2) {
	 // gaussian
	 // lmcurve_fit( n_par, par, j, xpad, qcal, gauss, &control, &status );
      }

      xfit = par[1];
      fcrdc.SetXfit((Float_t)xfit);
      fcrdc.SetFitPrm(0,(Float_t)par[0]);
      fcrdc.SetFitPrm(1,(Float_t)par[1]);
      fcrdc.SetFitPrm(2,(Float_t)par[2]);
      fcrdc.SetFnorm(status.fnorm);
      return (Float_t)xfit;
   }

   // If we do not do fit, return xcog
   return (Float_t)xcog;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^will be removed^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Float_t S800Calibration::CalcX2(CRDC* theCRDC){

  vector <Float_t> theCalibratedPads = theCRDC->GetCal();
  vector <Float_t> thePedSubtractedPads = theCRDC->GetPedSubtractedPads();
  // Cluster search
  Bool_t flg_clstr = kFALSE;
  Int_t  iclstr = -1;//Counter keeping track of the clusters.  Used as the first index in clstr[][]
  const Int_t maxclstr = S800_FP_CRDC_CHANNELS;
  // maximum number of clusters (I know this is too many...)
  Int_t clstr[maxclstr][3]; //clstr[][0] is left edge clstr[][1] right edge clstr[][2] is width
  Float_t maxchg[maxclstr]; //Array holding the max charge of each cluster
  Float_t maxpad[maxclstr]; //Array holding the pad of the max charge of each cluster
  const Float_t qmax = 25.; // MINUMUM value of max charge
  // to be considered to form a cluster
  const Float_t qthr =  8.; // MINUMUM value of charge to be considered to be hit

  Int_t gclstr = 0;

  for (UShort_t i = 0; i < S800_FP_CRDC_CHANNELS; i++) {
    if (IsBad(i,theCRDC->GetID()))continue;

    if ((flg_clstr == kFALSE) && (!std::isnan(theCalibratedPads[i]))) {
      flg_clstr = kTRUE;
      iclstr = iclstr + 1; //iclstr starts at -1
      clstr[iclstr][0] =  i; // leading edge
      clstr[iclstr][1] = -1; // trailing edge (tentative)
      maxchg[iclstr] = theCalibratedPads[i];
      maxpad[iclstr] = i;//Added by sam 3/19/2015 maybe is a good thing
    } else if ((flg_clstr == kTRUE) && (!std::isnan(theCalibratedPads[i]))) {
      if (theCalibratedPads[i] > maxchg[iclstr]) {
	maxchg[iclstr] = theCalibratedPads[i];
	maxpad[iclstr] = i;
      }
    } else if ((flg_clstr == kTRUE) && (std::isnan(theCalibratedPads[i]))) {
      //the previous point should be the end of the cluster as we have reached another NAN
      flg_clstr = kFALSE;//Reset flag
      clstr[iclstr][1] = i - 1; //set right edge of cluster
      clstr[iclstr][2] = i - clstr[iclstr][0];//Set width of cluster
      //if (maxpad[iclstr] < qmax) {
      if (maxchg[iclstr] < qmax) { // As pointed out by Sasano-san 2015/3/9
	//check to see if this cluster had a large enough maximum charge.
	//if it didn't decrement iclster so that the entry in clstr[][] is
	//overwritten
	iclstr = iclstr - 1;
      }
    }
  }

  // if a cluster is located at the end of the cathode
  if (flg_clstr == kTRUE) {
    //flg_clstr == kTRUE means that the above search never found NAN
    //after it found a cluster.  IE it is at the end of the pad plane
    clstr[iclstr][1] = S800_FP_CRDC_CHANNELS - 1;
    clstr[iclstr][2] = S800_FP_CRDC_CHANNELS - clstr[iclstr][0];
  }


  //gclstr is the index of the "good" cluster in clstr[][]
  if (iclstr == 0) {
    //There is only one possible good cluster.  Use that one
    gclstr = 0;
  } else if (iclstr > 0) {
    //There is more than one possible good cluster.
    Float_t tmp_qmax = 0.0;
    /*********************************************************************
       NOTE:   MAY HAVE TO THROW ALWAY EVENTS WITH MORE THAN ONE CLUSTER
    **********************************************************************/

    //When there is more than one good cluster this function will use the largest
    //one

    tmp_qmax = maxchg[0];
    // look for the GOOD cluster (gclstr) to be used to the analysis
    // (the cluster with the max charge is used)
    for (Int_t i = 0; i < iclstr + 1; i++) {
      if (maxchg[i] > tmp_qmax) {
	tmp_qmax = maxchg[i];
	gclstr = i;
      }
    }
  } else {
    //NO Clusters have been found return NAN
    return sqrt(-1.0);
  }

  theCRDC->SetMaxChg((Float_t)maxchg[gclstr]);
  theCRDC->SetMaxPad((Float_t)maxpad[gclstr]);
  theCRDC->SetNumClusters(iclstr+1);//iclstr is 0 when there is 1 cluster
  theCRDC->SetMaxClusterWidth(clstr[gclstr][2]);

  Int_t j = 0;
  Double_t xpad[S800_FP_CRDC_CHANNELS], qcal[S800_FP_CRDC_CHANNELS]; // (I know this is too many...)
  Float_t sum_q = 0.0, sum_qx = 0.0, sum_qxx = 0.0;

  //clstr[][0] is left edge clstr[][1] right edge clstr[][2] is width
  for (UShort_t i = clstr[gclstr][0]; i <= clstr[gclstr][1]; i++) {
    if (IsBad(i,theCRDC->GetID())) continue;
    if (theCalibratedPads[i] < qthr) continue;
    //    if (thePedSubtractedPads[i] < 90) continue;
    sum_q   += theCalibratedPads[i];
    sum_qx  += theCalibratedPads[i] * i;
    sum_qxx += theCalibratedPads[i] * i * i;
    // for fit
    xpad[j] = (Double_t)i;
    qcal[j] = (Double_t)theCalibratedPads[i];
    // debug
    // std::cout << j << " " << xpad[j] << " " << qcal[j] << std::endl;
    j++;
  }

  // fcrdc.SetXpad(xpad,j);
  // fcrdc.SetYpad(qcal,j);

  Double_t xcog  = (Double_t)sum_qx/sum_q;
  Double_t sigma = (Double_t)TMath::Sqrt(sum_qxx/sum_q - (sum_qx/sum_q)*(sum_qx/sum_q));
  if(xcog < 0 || xcog > S800_FP_CRDC_CHANNELS){ // no gravity center found
    xcog = sqrt(-1.0);
    std::cout << "Something strange happens with the CRDC data." << std::endl;
  }

  theCRDC->SetXcog((Float_t)xcog);
  theCRDC->SetCathode((Float_t)sum_q);

  if (fSett->XFit()) {

    // Do fit
    Double_t xfit;
    Double_t par[3];
    Int_t    n_par = 3; // number of parameters in model function f

    lm_status_struct status;
    lm_control_struct control = lm_control_double;
    control.printflags = 0; // monitor status (+1) and parameters (+2)

    // Initial guess
    par[0] = (Double_t)maxchg[gclstr];
    par[1] = xcog;
    par[2] = sigma;

    //sechs is a function defined in lmfit.h
    //xpad and qcal are pad numbers and charges for the "good" cluster
    if (fSett->XFitFunc() == 1) {
      // Secant Hyperbolic Squared
      lmcurve_fit( n_par, par, j, xpad, qcal, sechs, &control, &status );
    } else if (fSett->XFitFunc() == 2) {
      // gaussian
      // lmcurve_fit( n_par, par, j, xpad, qcal, gauss, &control, &status );
    }

    xfit = par[1];//the [1] parameter is the position
    theCRDC->SetXfit((Float_t)xfit);
    theCRDC->SetFitPrm(0,(Float_t)par[0]);
    theCRDC->SetFitPrm(1,(Float_t)par[1]);
    theCRDC->SetFitPrm(2,(Float_t)par[2]);
    theCRDC->SetFnorm(status.fnorm);
    return (Float_t)xfit;
  }

  // If we do not do fit, return xcog
  return (Float_t)xcog;
}



Float_t S800Calibration::TimeOffset(Float_t time1, Float_t time2){
   return time1 - time2;
}

Float_t S800Calibration::TimeOffset(Float_t time){
   return time - fts800;
}

void S800Calibration::SetTof(GTimeOfFlight *tof){
   ftof.Set(TimeOffset(tof->GetRF()), TimeOffset(tof->GetOBJ()), TimeOffset(tof->GetXFP()));
   ftof.SetTAC(tof->GetTACOBJ(), tof->GetTACXFP());
}


void S800Calibration::ReadICCalibration(const char *filename){
   TEnv *iccal = new TEnv(filename);
   for(int i=0;i<S800_FP_IC_CHANNELS;i++){
      fICoffset[i] = iccal->GetValue(Form("IonChamber.Offset.%d",i),0.0);
      fICslope[i] = iccal->GetValue(Form("IonChamber.Slope.%d",i),1.0);
   }
   fde_slope = iccal->GetValue("IonChamber.Slope.DE",1.0);
   fde_offset = iccal->GetValue("IonChamber.Offset.DE",0.0);
}

std::vector<Float_t> S800Calibration::ICCal(std::vector<int> chan, std::vector<float> raw){
   std::vector<Float_t> cal;
   cal.resize(S800_FP_IC_CHANNELS);
   if(chan.size() != raw.size()){
      std::cerr << " channel ("<<chan.size()<<") and data ("<<raw.size()<<" have different sizes " << std::endl;
      return cal;
   }
   for(unsigned int f=0;f<chan.size();f++){
      if( (chan[f]>-1) && (chan[f]<S800_FP_IC_CHANNELS) ){
	 cal[chan[f]] = raw[f]*fICslope[chan[f]]+fICoffset[chan[f]];
      }
      else{
	 std::cerr << " channel "<<chan[f]<<" not found!" << std::endl;
      }
   }
   return cal;
}
Float_t S800Calibration::ICSum(std::vector<Float_t> cal){
   Short_t ch = 0;
   Float_t sum =0;
   for(UShort_t j=0; j<cal.size(); j++){
      if(cal[j]>0){
	 sum += cal[j];
	 ch++;
      }
   }
   if(ch > 0)
      sum/= ch;
   else
      sum = sqrt(-1.0);

   return sum;

}
Float_t S800Calibration::ICDE(Float_t sum, Float_t x, Float_t y){
   // something needs to be done...
   // if(!isnan(sum) && !isnan(ftrack.GetAFP())){
   //   if(!isnan(y))
   //     sum += sum*fSett->dE_ytilt()*y;
   //   if(!isnan(x) && x < fSett->dE_x0tilt())
   //     sum *= exp(fSett->dE_xtilt()* (fSett->dE_x0tilt() -x) );
   //   fs800valid = 0;
   //   return sum * fde_slope + fde_offset;
   // }
   // else return sqrt(-1.0);
   return sum;
}



void S800Calibration::MakeCalibratedCRDC(CRDC* theCRDC,std::vector<Short_t> channels, std::vector<Short_t> data,Float_t tac,Float_t anode, Int_t id){

  theCRDC->Clear(); //Clear the object being passed in
  theCRDC->SetID(id);
  vector<Float_t> PedSubtractedPads;
  //Sets a vector in the CRDC object that stores the calibrated pad values.
  vector<Float_t> calibratedPads = GetCalibratedCrdcPads(channels,data,id,PedSubtractedPads);

  theCRDC->SetCal( calibratedPads);
  theCRDC->SetPedSubtractedPads(PedSubtractedPads);

  // this->CrdcCal(channel,data,id);
  // fcrdc.SetCal(fcrdccal);

  //  cout<<"ID IS "<<id<<endl;
  // for (int i=0;i<fcrdccal.size();i++){
  //   cout<<fcrdccal[i]<<" ";
  // }cout<<endl;
  // cin.get();

  // debug
  // std::cout << "\n" << id << std::endl;
  // for (int i = 0; i < channel.size(); i++) {
  //    std::cout << channel[i] << " " << data[i] << std::endl;
  // }
  double x = fSett->XOffset(id) + fSett->XSlope(id) * this->CalcX2(theCRDC);
  double y = fSett->YOffset(id) + fSett->YSlope(id) * tac;

  //std::cout <<x<<" "<<y<<" "<<fSett->XOffset(id) << " " << fSett->XSlope(id) << std::endl;

  theCRDC->SetX(x);
  theCRDC->SetY(y);
  theCRDC->SetTAC(tac);
  theCRDC->SetAnode(anode);


}

void S800Calibration::S800Calculate(S800* in, S800Calc* out){
   //s800
   TOF tof;
   SCINT scint[3];
   HODOSCOPE hodoscope[32];
   IC ich;

   ich.Clear();
   tof.Clear();
   bool icgood = false;

   out->Clear();
   out->SetTimeS800(in->GetTrigger()->GetS800());
   SetTS800(in->GetTrigger()->GetS800());

   // timestamp
   out->SetTS(in->GetTS());

   // TOF
   tof.Clear();
   this->SetTof(in->GetTimeOfFlight());
   tof = this->GetTof();

   //Copy over the GTrigger Object
   Trigger tempTrigger;
   tempTrigger.Set(in->GetTrigger()->GetRegistr(),in->GetTrigger()->GetS800(),
		   in->GetTrigger()->GetExternal1(),in->GetTrigger()->GetExternal2(),
		   in->GetTrigger()->GetSecondary());
   out->SetTrigger(tempTrigger);


   // CRDC
   for(UShort_t k=0; k<2; k++) {
     MakeCalibratedCRDC(out->GetCRDC(k),
			in->GetCrdc(k)->GetChannels(),
			in->GetCrdc(k)->GetData(),
			in->GetCrdc(k)->GetTAC(),
			in->GetCrdc(k)->GetAnode(),
			k);
   }

   //SCINTILLATOR
   for(UShort_t s=0; s<3; s++){
      scint[s].Clear();
      scint[s].SetTime(TimeOffset(in->GetScintillator(s)->GetTime_up()),
		       TimeOffset(in->GetScintillator(s)->GetTime_down()));
      scint[s].SetDE(in->GetScintillator(s)->GetDE_up(), in->GetScintillator(s)->GetDE_down());
   }

   //HODOSCOPE
   for (UShort_t s = 0; s < 32; s++) {
      hodoscope[s].Clear();
      hodoscope[s].SetEnergy(in->GetHodoscope(s)->GetEnergy());
   }

   //IC
   ich.SetCal(ICCal(in->GetIonChamber()->GetChannels(),in->GetIonChamber()->GetData() ) );
   ich.SetSum(ICSum(ich.GetCal()));
   if(ich.GetSum() > 400){
      icgood = true;
   }
   //ich.SetDE(ICDE(ich.GetSum(), crdc[0].GetX(), crdc[0].GetY()));

   //set Calculated S800
   out->SetTOF(tof);
   for (UShort_t s = 0; s < 3;  s++) {out->SetSCINT(scint[s],s);}
   for (UShort_t s = 0; s < 32; s++) {out->SetHODOSCOPE(hodoscope[s],s);}
   out->SetIC(ich);


   //There is no calibration to do for the MultiHit TDC
   //Just copy the information from the Raw S800 object
   //into the S800Calc object
   MultiHitTOF temp;

   temp.fE1Up = in->GetMultiHitTOF()->fE1Up;
   temp.fE1Down = in->GetMultiHitTOF()->fE1Down;
   temp.fXf = in->GetMultiHitTOF()->fXf;
   temp.fObj = in->GetMultiHitTOF()->fObj;
   temp.fGalotte= in->GetMultiHitTOF()->fGalotte;
   temp.fRf= in->GetMultiHitTOF()->fRf;
   temp.fHodoscope= in->GetMultiHitTOF()->fHodoscope;

   out->SetMultiHitTOF(temp);

}
