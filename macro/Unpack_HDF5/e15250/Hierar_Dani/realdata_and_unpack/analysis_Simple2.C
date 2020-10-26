// This is a new script to obtain the x,y,z positions and the charge in the pad plane,
//at the end we can be able to analyze the angle in each event

// author_Daniel Regueira Castro
#include "TMath.h"
#include "TMatrixD.h"
#include "TH1D.h"
#include "TVirtualFFT.h"
#include "TF1.h"
#include "TCanvas.h"
#include <vector>


#include "TFile.h"
#include "TTree.h"
#include "TCanvas.h"
#include "TFrame.h"
#include "TH1F.h"
#include "TBenchmark.h"
#include "TRandom.h"
#include "TSystem.h"


#ifdef __MAKECINT__
#pragma link C++ class vector<double>+;
#endif

#include "TVirtualFFT.h"
Double_t* GetPadWaveForm(Int_t padnum,std::vector<ATPad> *padarray)
{
	 auto it =  find_if( padarray->begin(),padarray->end(),[&padnum](ATPad& pad) {return pad.GetPadNum()==padnum;}   );

	 if(it != padarray->end()){
             auto padInd = std::distance<std::vector<ATPad>::const_iterator>(padarray->begin(),it);
             //std::cout<<" Pad found !"<<"\n";

             if(padarray->at(padInd).GetPadNum()>-1){
             	return padarray->at(padInd).GetADC();
         	 }else{
         	 	std::cout<<" Invalid pad number : "<<padarray->at(padInd).GetPadNum()<<"\n";
         	 	return 0;
         	 }
     }else{
      std::cerr<<" Warning! : Padnum not found in Pad Array!"<<"\n";	
      return 0;
     } 
}
Double_t distance2( double x,double y,double z, const double *p)
{

    // distance line point is D= | (xp-x0) cross  ux |
    // where ux is direction of line and x0 is a point in the line (like t = 0) and x1 is in t=1
    XYZVector xp(x,y,z);
    XYZVector x0(p[0], p[2], 0. );
    XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
    XYZVector u = (x1-x0).Unit();
    double d2 = ((xp-x0).Cross(u)).Mag2();
    return d2;
}
struct SumDistance2
      {
          TGraph2D * fGraph;

              SumDistance2(TGraph2D * g) : fGraph(g) {}
                  double operator() (const double * par) {
                  assert(fGraph    != 0);
                  double * x = fGraph->GetX();
                  double * y = fGraph->GetY();
                  double * z = fGraph->GetZ();
                  int npoints = fGraph->GetN();
                  double sum = 0;
                  for (int i  = 0; i < npoints; ++i) {
                    double d = distance2(x[i],y[i],z[i],par);
                    sum += d;
                  }
            #ifdef DEBUG
             if (first) std::cout << "point " << i << "\t"
                << x[i] << "\t"
                << y[i] << "\t"
                << z[i] << "\t"
                << std::sqrt(d) << std::endl;
            #endif

            //if (first)
              //std::cout << "Total Initial distance square = " << sum << std::endl;
              //first = false;
              return sum;
              }
};
Double_t GetAngle(ATTrack* track)
{
	gErrorIgnoreLevel=kFatal;
        Int_t nd = 10000;
	TGraph2D * gr = new TGraph2D();
	std::vector<ATHit> *HitArray = track->GetHitArray();

	 double p0[4] = {0,0,0,0}; //For the moment those are dummy parameters

            for(Int_t N=0;N<HitArray->size();N++){
              ATHit hit = HitArray->at(N);
              TVector3 pos = hit.GetPosition();
              gr->SetPoint(N,pos.X(),pos.Y(),pos.Z());
			}
			ROOT::Fit::Fitter fitter;
            SumDistance2 sdist(gr);
            #ifdef __CINT__
            ROOT::Math::Functor fcn(&sdist,4,"SumDistance2");
            #else
            ROOT::Math::Functor fcn(sdist,4);
            #endif
            // set the function and the initial parameter values
            double pStart[4] = {0,0,0,0};
            fitter.SetFCN(fcn,pStart);
            // set step sizes different than default ones (0.3 times parameter values)
            for (int i = 0; i <4; ++i) fitter.Config().ParSettings(i).SetStepSize(1.);

            bool ok = fitter.FitFCN();
            if (!ok) {
              Error("line3Dfit","Line3D Fit failed");
              return 1;
            }
             const ROOT::Fit::FitResult & result = fitter.Result();
             const ROOT::Math::Minimizer * min = fitter.GetMinimizer();
             double sigma2 = 12.5; //Size of the pad
             double Chi2_min = min->MinValue();
             int NDF = min->NFree();
             int npoints = gr->GetN();
             const double * parFitBuff = result.GetParams();
             std::vector<Double_t> parFit;
             for(Int_t i=0;i<4;i++) parFit.push_back(parFitBuff[i]); //4 parameters per fit
             track->SetFitPar(parFit);
             track->SetMinimum(Chi2_min);
             track->SetNFree(NDF);

             //std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<parFit[2]<<" "<<parFit[3]<<std::endl;
 		     //std::cout<<" Chi2 (Minuit) : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
			 //std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints)<<std::endl;
		//Double_t ang = (atan(sqrt(parFit[1]*parFit[1] +1)/parFit[3] ));
		//return ang;
// track is the branch fTrackCand in ATPatternEvent, with GetFitPar i have access to 
			 std::vector<Double_t> p_f = track->GetFitPar();
			 XYZVector Z_1(0.0,0.0,1.); // Beam direction construindo un vector con x,y,z=(0,0,1)

			 if(p_f.size()>0)
             {
                    			XYZVector L0(p_f[0], p_f[2], 0. );//p2
				   	XYZVector L1(p_f[1], p_f[3], 1. );//d2	
					//XYZVector L3(1., 1., 1. );//d3 
		                        // vectores para calcular los angulos		   	
                                        XYZVector vec1 = Z_1;
				   	XYZVector vec2 = L1;
				    Double_t num = vec1.X()*vec2.X() + vec1.Y()*vec2.Y() + vec1.Z()*vec2.Z() ;
  					Double_t den = TMath::Sqrt(vec1.X()*vec1.X() + vec1.Y()*vec1.Y() + vec1.Z()*vec1.Z())*TMath::Sqrt(vec2.X()*vec2.X() + vec2.Y()*vec2.Y() + vec2.Z()*vec2.Z());
  					Double_t ang = TMath::ACos(num/den);

					return ang;
			}else return 0;
}

double GetMaximum(double *adc)
{
	double max = 0;

	for(int indTB=0;indTB<512;++indTB)
	{
	    //std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
	    if(adc[indTB]>max) max = adc[indTB];
	}             	   			 
	return max;
}

double GetMinimum(double *adc)
{
	double min = 0;

	for(int indTB=0;indTB<512;++indTB)
	{
	    //std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
	    if(adc[indTB]<min) min = adc[indTB];
	}             	   			 
	return min;
}
double GetMaximum2(double adcfiltrada[])
{
	double max = 0;

	for(int indTB=0;indTB<512;++indTB)
	{
	    //std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
	    if(adcfiltrada[indTB]>max) max = adcfiltrada[indTB];
	}             	   			 
	return max;
}
//**************************************************************************////**************************************************************************//
//**************************************************************************////**************************************************************************//
//**************************************************************************////**************************************************************************//
									//MAIN PROGRAM//
//**************************************************************************////**************************************************************************//
//**************************************************************************////**************************************************************************//
//**************************************************************************////**************************************************************************//

void analysis_Simple2()
{
 	TStopwatch timer;
  	timer.Start();

    //Double_t Charaux[10000],xaux[10000],yaux[10000],zaux[10000];
    //

    TFile *f = new TFile("datosfourier.root","RECREATE");
    TTree *T = new TTree("datosfiltrados","data with filter");
    

    std::vector<double> puntox;
    std::vector<double> puntoy;
    std::vector<double> puntoz;
    std::vector<double> TBcarga;


    T->Branch("puntox", &puntox);
    T->Branch("puntoy", &puntoy);
    T->Branch("puntoz", &puntoz);
    T->Branch("TBcarga", &TBcarga);



//  Print the ATTPC geometry in the root file     
 	AtTpcMap* fAtMapPtr = new AtTpcMap();
  	fAtMapPtr->GenerateATTPC();
	TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();
  	fAtMapPtr->Dump();
        std::cout<<" Opening File : "<<std::endl; 
        gSystem->Load("libATTPCReco.so");
    
  FairRunAna* run = new FairRunAna(); //Forcing a dummy run
    
    TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "run_0290";
    TString FilePath ="/media/dani/ubuntuwindows/GitHub_Yassidfork/ATTPCROOTv2/macro/Unpack_HDF5/e15250/Hierar_Dani/realdata_and_unpack/";
    TString FileNameTail = ".root";
    TString FileNameOut  = "_analysis_elastic";
    TString FileName     = FilePath + FileNameHead + FileNameTail;
    TString OutputFileName = FilePath + FileNameHead + FileNameOut + FileNameTail;
	std::ofstream outputFile("outputfile_PadWaveFormaEventos.txt");
	std::ofstream outputData("data/Datafirst.txt");
	std::ofstream PadWaveFormaTotal("data/PadWaveForm.txt");
	std::ofstream PadWaveFormaEventos("data/PadWaveFormEventos.txt");
        std::ofstream outputDataHierar("data/Hierardata.txt");
	std::ofstream outputultimo("data/Datalast.dat");
	std::ofstream outputDataMatlab("data/Data.txt");
    // Print the unpacking file
    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    //std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> patterneventArray(Reader1, "ATPatternEvent");
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Open the draw files   
    //TH2F* ADC = new TH2F("ADC","ADC",200,0,514,200,-300,400);
    TH1F* FourierMag = new TH1F("FourierMag","FourierMag",200,0,600);
    TH1F* Chargeall = new TH1F("Chargeall","Chargeall",200,-100,600);
    TH1F* ChargeHC = new TH1F("ChargeHC","ChargeHC",200,-100,600);
    TH1F* angle = new TH1F("angle","angle",1000,-90,90);
    TH2F* Acumulation = new TH2F("Acumulation","Acumulation",400,-300,300,400,-300,300);
    TH2F* AcumulationHC = new TH2F("AcumulationHC","AcumulationHC",400,-300,300,400,-300,300);
    TH1F* TimeStamp = new TH1F("TimeStamp","TimeStamp",1000,0,511);
    TH1F* mesh = new TH1F("mesh","mesh",512,0,511);
    TH1F* mesh2 = new TH1F("mesh2","mesh2",512,0,511);
    TH1F* ADCFILTRhisto = new TH1F("ADCFILTRhisto","ADCFILTRhisto",512,-350,1100);	
    TH2F* FFTreal = new TH2F("FFTreal","FFTreal",200,0,512,200,-3000,3000);
    TH2F* FFTim = new TH2F("FFTim","FFTim",200,0,512,200,-3000,3000);
    TH2F* FFTordenada = new TH2F("FFTordenada","FFTordenada",200,-255,255,200,0,3000);
    TH2F* FFTconvolute = new TH2F("FFTconvolute","FFTconvolute",200,0,511,200,0,3000);
    TH2F* ADCFILTR = new TH2F("ADCFILTR","ADCFILTR",300,0,512,300,-250,4500);
    TH2F* ADCSINFILTR = new TH2F("ADCSINFILTR","ADCSINFILTR",300,0,512,300,-250,4500);
    TH2F* xztotal = new TH2F("xztotal","xztotal",200,0,1000,200,-300,300);
    TH2F* yztotal = new TH2F("yztotal","yztotal",200,0,1000,200,-300,300);
    TH2F* xz_ordenado_z = new TH2F("xz_ordenado_z","xz_ordenado_z",200,0,1000,200,-300,300);
    TH2F* yz_ordenado_z = new TH2F("yz_ordenado_z","yz_ordenado_z",200,0,1000,200,-300,300);
    TH2F* xzfiltrado = new TH2F("xzfiltrado","xzfiltrado",200,0,1000,200,-300,300);
    TH2F* yzfiltrado = new TH2F("yzfiltrado","yzfiltrado",200,0,1000,200,-300,300);
    TH2F* xyfiltrado = new TH2F("xyfiltrado","xyfiltrado",400,-300,300,400,-300,300);
    
    TH2F* DISTRIBUCIONQ = new TH2F("DISTRIBUCIONQ","DISTRIBUCIONQ",300,0,1000,300,0,110000000);
    TH2F* calor = new TH2F("calor", "Mapa de calor XY",200,-300,300,200,-300,300);


       TClonesArray *events = nullptr;
       TBranch *eventsBranch = tree->GetBranch("ATEventH");
       eventsBranch->SetAddress(&events);
       Int_t firstEvent = 0;
       Int_t eventCount = std::numeric_limits<Int_t>::max();
Int_t const lastEvent = (eventCount < (eventsBranch->GetEntries() - firstEvent) ? eventCount : (eventsBranch->GetEntries() - firstEvent)) + firstEvent - 0;
      std::cout<<" piiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii :: "<<TMath::Pi()<<"\n";
      size_t size = 10000;
      //std::vector<double> calculate(size);
	Double_t z[size];Double_t y[size];Double_t x[size];Double_t r[size];
	//variables para juntar las trazas
	Double_t Charge[size];
	Double_t Qhit[size];
	Double_t Char[size];
	Double_t maxim[size];
	Double_t firstfourier[size];
	Double_t Amplitudefourier[size];
	Double_t phasefourier[size];
	Double_t secondfourier[size];
	Double_t thirdfourier[size];
	Double_t firstphasefourier[size];
	Double_t thirdphasefourier[size];
	Double_t secondphasefourier[size];
	Double_t aconvolute[size];
	Double_t bconvolute[size];
	Double_t adcfil[size];
	Double_t adcfiltrada[size];Double_t aux[size];
	Double_t distance1[size];Double_t distance2[size];Double_t distance3[size];Double_t difcharge[size];
	Double_t minfilt[size];Double_t maxfilt[size];
	Double_t Charaux[size];Double_t xaux[size];Double_t yaux[size];Double_t zaux[size];
	Double_t mediaz[size];

	Int_t counteraux;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							//INICIO DEL BUCLE PRINCIPAL
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
    for(Int_t i=0;i<10;i++){ 
          //while (Reader1.Next()) {//nEvents
               
              //std::cout<<" Event number nº::"<<i<<"\n";
              
              Reader1.Next();
	      mesh->Reset();
		////*************ATEvent --> event****************////////
		////*************ATRawEvent --> rawevent****************////////
              ATEvent* event = (ATEvent*) eventArray->At(0);
              ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);
              Int_t nHits = event->GetNumHits();
              std::vector<ATHit>* hitArray = event->GetHitArray();
	
              event->GetHitArrayObj();
              hitArray->size();

            for(Int_t iHit=0; iHit<nHits; iHit++){// iHit for each event nº
	      //std::cout<<" Hit number nº::"<<iHit<<"\n";
              ATHit hit = event->GetHit(iHit);
              TVector3 hitPos = hit.GetPosition();
              //Int_t bin=  fPadPlane->Fill(hitPos.X(),hitPos.Y(),hit.GetCharge());
	      Acumulation->Fill(hitPos.X(),hitPos.Y());
	      Chargeall->Fill(hit.GetCharge());
	       xztotal->Fill(hitPos.Z(),hitPos.X());
	       yztotal->Fill( hitPos.Z(),hitPos.X());
             } 
	    int counter=0;
	    // we quit the null events and we work with the HC tracks (patterneventArray)
		
            if(rawEvent!=NULL && event!=NULL) 
              { 
              ATPatternEvent* patternEvent = (ATPatternEvent*) patterneventArray->At(0);
		////*************ATPatternEvent --> patternevent****************//////// select a subBranch in the Branch
              std::vector<ATTrack>& tracks = patternEvent->GetTrackCand();
		//We have the size of tracks but we can't see it in a Browser because the information is saved in a diferent way than ATEvent and ATRawEvent (dinamical, we need the libraries to reconstruct the tracks)
	      std::cout<<"**************************************************************************************************************"<<"\n";
	      std::cout<<" Found tracks totales quitando null event :: "<<" "<<i<<" TRAZAS anteriores :: "<<tracks.size()<<"\n";
		////*************ATEvent --> event --> auxPadArray****************////////select a subBranch in the Branch
		////*************ATRawEvent --> rawEvent --> padArray****************////////select a subBranch in the Branch
	      std::vector<ATPad> *padArray = rawEvent->GetPads();
	      std::vector<ATPad> *auxPadArray = event->GetAuxPadArray();
			
	              int firstTBOT = 511;
	              int lastTBOT  = 0;
	              double Qint_nearFirst = 0;
	              double Qint_nearLast = 0;
	              double Qtotal = 0;
		      double allQ = 0;
		      double mean = 0;

//******************************************************************************************************************************************************************
//*****************************************************FFTW Aplico para adc[indTB]*************************************************************************************
//******************************************************************************************************************************************************************
// Aplico FFT a adc[indTB] y convoluciono con la funcion rectángulo para filtrar la señal, una vez convolucionada aplico la transformada inversa y voilà
// Estos son los eventos acumulados en Hit, es decir, todos los eventos acumulados por HC, en total 3728 tracks en 10 eventos con cada uno 512 timebuckets
			for(auto padInd=0;padInd<padArray->size();++padInd)
		
	              {
	              		Double_t *adc =  padArray->at(padInd).GetADC();
				
	              		//std::cout<<" Pad Index "<<padInd<<"	Pad Number "<<padArray->at(padInd).GetPadNum()<<"\n";


	              	if(padArray->at(padInd).GetPadNum()>-1){//Para todos los pads
			//if(padArray->at(padInd).GetPadNum()>1980 && padArray->at(padInd).GetPadNum()<1982){//seleccionar un pad en particular o un rango de pads	
			//if(padArray->at(padInd).GetPadNum()>2870 && padArray->at(padInd).GetPadNum()<2875){//seleccionar un pad en particular o un rango de pads

	              		for(int indTB=0;indTB<511;++indTB)
	              	    {
	              	    				//if(adc[indTB]>30.0){
	              	    				//std::cout<<" TB "<<indTB<<" ADC "<<adc[indTB]<<"\n";
	              	    					if(indTB<firstTBOT) firstTBOT = indTB;
	              	    					if(indTB>lastTBOT)  lastTBOT = indTB;
	              	    					mesh->Fill(indTB,adc[indTB]);
 								ADCSINFILTR->Fill(indTB,adc[indTB]);
	              	    					Qtotal+=adc[indTB];
								mesh2->SetBinContent(indTB,adc[indTB]);
								//////in[i] --> adc[indTB] y x es indTB
							//ADC->Fill(indTB,adc[indTB]);	
	              	    				//}
					//std::cout<<i<<"	"<<indTB<<"	"<<adc[indTB]<<"	"<<"\n";
					PadWaveFormaEventos<<i<<"	"<<indTB<<"	"<<adc[indTB]<<"	"<< tracks.size()<<"\n";
	              	    }
				Double_t maximo= GetMaximum(adc);//aquí obtengo el máximo para cada uno de los tracks de los eventos (512 TimeBuckets) 
				Double_t minimo= GetMinimum(adc);//aquí obtengo el mínimo para cada uno de los tracks de los eventos (512 TimeBuckets) 
				//std::cout<<"MÁXIMO::"<<maximo<<"	"<<"MÍNIMO::"<<minimo<<"\n";

	              	    //std::cout<<" First TBOT "<<firstTBOT<<" Last TBOT "<<lastTBOT<<"\n";		
			//Compute the transform and look at the magnitude of the output
   			  TH1 *fourier =0;
   			 	TVirtualFFT::SetTransform(0);
   			 fourier = mesh2->FFT(fourier, "MAG");
	                 fourier->SetStats(kTRUE);
   			 //Look at the phase of the output
   			 TH1 *phase = 0;
   			 phase = mesh2->FFT(phase, "PH");
  		

			Int_t n_size = 511;
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////// INVERSE FFT ///////////////////////////////////////////////////////////////////
			// Para la transformada inversa lo paso a parte real e imaginaria y se realiza FFT inversa
    			//That's the way to get the current transform object:
   			TVirtualFFT *fft = TVirtualFFT::GetCurrentTransform();
    			//c1_4->cd();
 
    			//Use the following method to get the full output:
    			Double_t *re_full = new Double_t[n_size];
    			Double_t *im_full = new Double_t[n_size];
   			 fft->GetPointsComplex(re_full,im_full);

    			 //Now let's make a backward transform:
   			 TVirtualFFT *fft_back = TVirtualFFT::FFT(1, &n_size, "C2R M K");
   			 fft_back->SetPointsComplex(re_full,im_full);
    		       	 fft_back->Transform();
    			 TH1 *hb = 0;
   			 //Let's look at the output
   			 hb = TH1::TransformHisto(fft_back,hb,"Re");
   			 delete fft_back;
   			 fft_back=0;

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////CONVOLUTE RECT///////////////////////////////////////////////////////////////
			for(int indTB=0;indTB<511;++indTB)
	              	    {
			//std::cout<<i<<"	"<<fourier->GetBinContent(indTB)<<"\n";//Aquí obtengo la magnitud de la transformada de Fourier
			//std::cout<<i<<"	"<<phase->GetBinContent(indTB)<<"\n";//Aquí obtengo la fase de la transformada de Fourier
			firstfourier[indTB]=fourier->GetBinContent(indTB);

///multiplicar por la función rect(w)

			if (indTB<100||indTB>411) Amplitudefourier[indTB]=firstfourier[indTB]*(1/sqrt(n_size));
			else Amplitudefourier[indTB]=Amplitudefourier[indTB]=firstfourier[indTB]*(1/sqrt(n_size));;
			firstphasefourier[indTB]=phase->GetBinContent(indTB);
			phasefourier[indTB]=firstphasefourier[indTB];

			FFTreal->Fill(indTB,re_full[indTB]*(1/sqrt(n_size)));
			FFTim->Fill(indTB,im_full[indTB]*(1/sqrt(n_size)));

///Reordenar la transformada
			if (indTB>255){
			int indFFT=indTB-512;//usado para reordenar la gaussiana
			secondfourier[indTB]=Amplitudefourier[indTB];//usado para reordenar la gaussiana
			secondphasefourier[indTB]=phasefourier[indTB];
			FFTordenada->Fill(indFFT,secondfourier[indTB]);
			}
			else {
				int indFFT=indTB;
				secondfourier[indTB]=Amplitudefourier[indTB];
				secondphasefourier[indTB]=phasefourier[indTB];
				FFTordenada->Fill(indTB,secondfourier[indTB]);
			     }

///obtengo la transformada filtrada en forma binomica para poder realizar la transformada inversa
// Multipilico uno a uno por la funcion rect que es la FFT de la sinc; si quiero otra función para el filtrado se multiplica uno a uno realizando la FFT		
			
			FFTconvolute->Fill(indTB,secondfourier[indTB]);
			//aconvolute[indTB]=Amplitudefourier[indTB]*cos(phasefourier[indTB]);
			//bconvolute[indTB]=Amplitudefourier[indTB]*sin(phasefourier[indTB]);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////FILTRADO///////////////////////////////////////////////////////////////

// Se utiliza la función cuadrado en el espacio dual por simplicidad (FFT convolute -->sinc(t)*func(t) -->rect(w)func(w)
			if (indTB<17) { //Así mantiene la integridad del pulso
		        aconvolute[indTB]=re_full[indTB]*(1/sqrt(n_size));
			bconvolute[indTB]=im_full[indTB]*(1/sqrt(n_size));
				}
				else {  aconvolute[indTB]=0.;
					bconvolute[indTB]=0.;
					}
			
		//std::cout<<"a:"<<"	"<<aconvolute[indTB]<<"	"<<"refull:"<<"	"<<re_full[indTB]<<"	"<<"b:"<<"	"<<bconvolute[indTB]<<"	"<<"im:"<<"	"<<im_full[indTB]<<"\n";	
			    }	
	
    			//NOTE: for "real" frequencies you have to divide the x-axes range with the range of your function
   			 //(in this case 4*Pi); y-axes has to be rescaled by a factor of 1/SQRT(n) to be right: this is not done automatically!
 			//Use the following method to get the full output:
    			
    			TVirtualFFT *fft_own = TVirtualFFT::FFT(1, &n_size, "C2R M K");
   			 //fft_own->GetPointsComplex(aconvolute,bconvolute);
    			if (!fft_own) return;
    			 fft_own->SetPointsComplex(aconvolute,bconvolute);
    			fft_own->Transform();
  			   //Copy all the output points:
    			//fft_own->GetPoints(adc);
   		 	//Draw the real part of the output
    			//c1_6->cd();
			
    			TH1 *transformadainversa = 0;
    			transformadainversa = TH1::TransformHisto(fft_own, transformadainversa, "Re");
    			 delete fft_own;
   			 fft_own=0;
 			double maximo2=0;
			double minimo2=0;
    			//myc->cd();
			//std::cout<<"MÁXIMO::"<<maximo<<"	"<<"MÍNIMO::"<<TMath::Abs(minimo)<<"\n";
			Double_t absmin=TMath::Abs(minimo);
			for(int indTB=0;indTB<511;++indTB)
	              	    {
			adcfil[indTB]=transformadainversa->GetBinContent(indTB);
			
			adcfiltrada[indTB]=adcfil[indTB]*(1/sqrt(n_size));
			//adcfiltrada[indTB]=adcfil[indTB]*(1/sqrt(n_size));
			//std::cout<<i<< "	"<<indTB<< "	"<<"adcfiltrada::"<<adcfiltrada[indTB]<<"	"<<"\n";
			//Si quiero obtener la transformada para toda la acumulación
			ADCFILTR->Fill(indTB,adcfiltrada[indTB]);
			ADCFILTRhisto->Fill(adcfiltrada[indTB]);
			/*if(adcfiltrada[indTB]>maximo2) maximo2 =adcfiltrada[indTB] ;
			else maximo2 = maximo2;
			if(adcfiltrada[indTB]<minimo2) minimo2 =adcfiltrada[indTB] ;
			else minimo2 = minimo2;*/
			    }
			
			/* counter=counter+1;
			 minfilt[counter]=minimo2;
			 maxfilt[counter]=maximo2;
			std::cout<<"MÁXIMO FILTRADO::"<<maximo2<<"MÍNIMO FILTRADO::"<<minfilt[counter]<<"COUNTER::"<<counter<<"	"<<"\n";*/
				
	             	}//if
			
	             }//for
			
		      //std::cout<<" Raw event number : "<<i<<"\n";
	              std::cout<<" Number of pads Totales:: "<<padArray->size()<<"\n";


			//indTB es dt en la transformada
			
				//PadWaveFormaEventos<<"     "  <<std::endl;
//******************************************************************************************************************************************************************
//***************************************************** FIN FFTW para adc[indTB]*************************************************************************************
//******************************************************************************************************************************************************************
 
// pongo este booleano para seleccionar los datos aceptados
		 bool isValid = true;
    
		
		Double_t sumaQHit=0;
		
		Int_t numerotrack=0;
                 for(auto  track : tracks)
	              {
// Aquí obtengo la respuesta de la mesh para distintas trazas dentro de un evento
			double Qtot = 0;
			int firstTBOT = 511;
	              	int lastTBOT  = 0;
			numerotrack=numerotrack+1;
			
///////////////////////////////////////////////ANALISIS NUMB PUNTOS/////////////////////////////////////////////////////////			

					//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	 "<<"\n";
	              		        //std::cout<<" TRACK SIZE : "<<tracks.size()<<"\n";
	    		
// Defino hits en track (patternevent) like a vector. Selecciono la Branch fGetHitArray
			std::vector<ATHit>*  hits = track.GetHitArray();
					Double_t sumaeachtrack=0.;
					Double_t cuenta=0.;
				Double_t sumacarga=0;		
// Selecciono los eventos fit con HC con éste for
				for(auto hit : *hits)
	              		{//
		
				
				//Aquí solo tengo los TimeBuckets que el programa me ha identificado como trazas
				for(int indTB=0;indTB<511;++indTB)
			  {	

				sumacarga+=ADCFILTRhisto->GetBinContent(indTB);
					
				//ADCFILTR->Fill(indTB,adcfiltrada[indTB]);
			        //ADCFILTRhisto->Fill(adcfiltrada[indTB]);
				
				//std::cout<<i<< "	"<<indTB<< "	"<<"adcfiltrada::"<<adcfiltrada[indTB]<<"	"<<"\n";


				outputFile<<indTB<<"	"<<ADCFILTRhisto->GetBinContent(indTB)<<"	"<<ADCFILTRhisto->GetBinError(indTB)<<"  "<<i<<"		"<<tracks.size()<<"		"<<"\n";
				
			  }	
				 //Double_t maximo2= GetMaximum2(adcfiltrada);//aquí obtengo el máximo para cada uno de los tracks de los eventos (512 TimeBuckets) 
			      
			      //  std::cout<<"MÁXIMO FILTRADO::"<<maximo2<<"	"<<sumacarga<<"\n";
				
				// Si hay mas de 5 tracks por hit y el numero de tracks separados es 'tracks.size()' encontrados por evento ******************************************************************************************************************************************************************
//con este if me quito la mayoría del cross-talcking y window. Si pongo hit.GetCharge puedo recuperar por donde va el haz y algo de cross-talking en trazas; con track.GetHitArray()->size()<150 me quito la mayoría de eventos 'ruido', track.GetHitArray()->size() son el numero de tracks en una traza identificada por HC
// Con hit.GetCharge() pongo una primera cota al ruido fino teniendo un compromiso entre el ruido que me cargo y el número de puntos en los tracks
//hitGetCharge es el máximo del pulso de cada pad activado con cada track encontrado
//hitGetQHit es la señal integrada para cada TimeBucket

                                
				
				if( track.GetHitArray()->size()>5  && track.GetHitArray()->size()<200 ){//( )|| hit.GetCharge()<0 para ver el haz // && track.GetHitArray()->size()>5//     &&  hit.GetCharge()>0 
//tengo que realizar otro if si calculatezgeo(1) es menor que el rango máximo del 22Mg en ATTPC
				//Double_t angDeg = ang*180.0/TMath::Pi();
				Double_t ang = GetAngle(&track);
				Double_t angDeg = ang*180.0/TMath::Pi();
        	                angle->Fill(angDeg);
				TimeStamp->Fill(hit.GetTimeStamp());
	              	        TVector3 pos = hit.GetPosition();
//pongo un contador a cada evento para poder poner condiciones a cada punto	
                                counter=counter+1;
				//std::cout<<"Counter:"<<"	"<<counter<<"\n";
	              			//std::cout<<pos.X()<<"	"<<pos.Y()<<"	"<<pos.Z()<<"	"<<hit.GetCharge()<<"\n";
	              		//fPadPlane->Fill(pos.X(), pos.Y(), hit.GetCharge());
	              				//std::cout<<" Hit pad num "<<hit.GetHitPadNum()<<" X "<<pos.X()<<" Y "<<pos.Y()<<" Z "<<pos.Z()<<"\n";

					        //std::cout<<" Signox "<<TMath::Sign(1, pos.X())<<"\n";
						//std::cout<<" Signoy "<<TMath::Sign(1, pos.Y())<<"\n";
				AcumulationHC->Fill(pos.X(),pos.Y());
				ChargeHC->Fill(hit.GetCharge());
				Double_t time_drift = 1000. - pos.Z()/0.3382;
//Calculate z :  (fNumTbs - peakIdx)*fTBTime*fDriftVelocity/100.;
//Calculatezgeo :  fZk - (fEntTB - peakIdx)*fTBTime*fDriftVelocity/100.;
/*fNumTbs = fPar -> GetNumTbs(); # Beam position at detector entrance in TB fijo
fTBTime = fPar -> GetTBTime();
fDriftVelocity = fPar -> GetDriftVelocity();
fMaxDriftLength = fPar -> GetDriftLength();
fTB0  =  fPar->GetTB0();
fZk   = fPar->GetZPadPlane();
fEntTB   = (Int_t) fPar->GetTBEntrance();--->Fijo En lo que sea*/

				Double_t CalculateZgeo=((TMath::Abs(511- hit.GetTimeStamp()))*98*2.07/100.);
				
				//Redefino las variables para poder trabajar con ellas mas adelante en el filtrado
				z[counter]=((TMath::Abs(511- hit.GetTimeStamp()))*98*2.07/100.);//el primer valor depende del experimento, es donde se empieza a contar el tiempo, Beam position at detector entrance in TB, un delay??
				x[counter]=pos.X();
				y[counter]=pos.Y();
				Charge[counter]=hit.GetCharge();
				//Qhit[counter]=maximo2;
				Char[counter]=sumacarga;

				if (z[counter]>1.){
				cuenta=cuenta+1;
				sumaeachtrack+=z[counter]/cuenta;
				}
				
				xz_ordenado_z->Fill( CalculateZgeo,pos.X());
				yz_ordenado_z->Fill( CalculateZgeo,pos.Y());
				outputData <<i<< "	"<< pos.X()<<"       "<< pos.Y() <<"       "<< pos.Z() <<"       "<< CalculateZgeo << "       "<< hit.GetTimeStamp() <<"       "<<"       "<< counter<<"       "<< sumacarga<<"       "<< Char[counter]<< "\n";	
	        
				Double_t* adc = GetPadWaveForm(hit.GetHitPadNum(),padArray);
				
				//PadWaveForma <<i<< "	"<< adc[i]<<"	"<< hit.GetHitPadNum()<<"       "<< track.GetHitArray()->size()<<"       "<< tracks.size()<< "\n";
		 	
                 		}//end if charge
				
				
				outputData<<"     "  <<std::endl;
				PadWaveFormaTotal<<"     "  <<std::endl;
 				
				}//end of HITS event
				//mediaz[numerotrack]=sumaeachtrack/cuenta;
			//if (cuenta>0.) std::cout<< "número de pads::"<<cuenta<<"media:::"<<mediaz[numerotrack]<<"	"<<numerotrack<<"\n";
			//z[counter+6]=z[counter];
			//for (int j=1;j<=numerotrack;j++){}
                 }//end of for Tracks

	      }//end of if NULLevents

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////FILTRADO 2////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			        x[-9]=x[10];x[-8]=x[9];x[-7]=x[8];x[-6]=x[7];x[-5]=x[6];x[-4]=x[5];x[-3]=x[4];x[-2]=x[3];x[-1]=x[2];x[0]=x[1];
				y[-9]=y[10];y[-8]=y[9];y[-7]=y[8];y[-6]=y[7];y[-5]=y[6];y[-4]=y[5];y[-3]=y[4];y[-2]=y[3];y[-1]=y[2];y[0]=y[1];
				z[-9]=z[10];z[-8]=z[9];z[-7]=z[8];z[-6]=z[7];z[-5]=z[6];z[-4]=z[5];z[-3]=z[4];z[-2]=z[3];z[-1]=z[2];z[0]=z[1];
				Char[0]=Char[2];difcharge[0]=difcharge[2];
				//z[0]=z[10];z[-1]=z[9];z[-2]=z[8];z[-3]=z[7];z[-4]=z[6];z[-5]=z[5];z[-6]=z[4];z[-7]=z[3];z[-8]=z[2];z[-9]=z[1];
                        counteraux=0;
			int numtrazas=0;

			puntox.clear();
			puntoy.clear();
			puntoz.clear();
			TBcarga.clear();
			for (int j=0;j<counter;++j){
				
		// Aquí realizo otro filtrado por distancias, otra vez manteniendo el compromiso entre puntos y ruido		
			distance1[j] = TMath::Abs(x[j-5])-TMath::Abs(x[j]);
			distance2[j] = TMath::Abs(y[j-5])-TMath::Abs(y[j]);
			distance3[j] = TMath::Abs(z[j-1])-TMath::Abs(z[j]);//distancia aceptable entre un punto y el siguiente 10º, se evalúa punto a punto
			difcharge[j] = TMath::Abs(Char[j-1]-Char[j]);
			Double_t px;
			Double_t py;
			Double_t pz;
			Double_t pcarga;
			if ( Char[j]>0 && z[j]<1000){//Filtrito para que no se repitan y quitarme ceros de la matriz  && z[j]<920 && que siempre sean hacia adelante (elásticos)
				//std::cout<< "lugar en la matriz::"<<j<<"primer elemento"<<calculate[counter]<<"\n";
			
				
				//Si sigue un proceso ascendente en carga forma parte de la misma traza; sino tiene un retorno de carro
					
					counteraux=counteraux+1;
					
			pcarga=Char[j];px=x[j];py=y[j];pz=z[j];

			//cout << "\t" << j << " " << px << endl;
			
			puntox.push_back(px);
			puntoy.push_back(py);
			puntoz.push_back(pz);
			TBcarga.push_back(pcarga);		
			r[j]=sqrt((x[j]*x[j])+(y[j]*y[j]));
					fPadPlane->Fill(x[j],y[j],100000000.-Char[j]);
						outputultimo<< x[j]<<"	"<< y[j] <<"	"<< r[j]<<"	" <<std::endl;
						outputDataMatlab <<i<< "	"<< x[j]<<"	"<< y[j] <<"	"<< z[j]<<"	" <<Char[j]<<"\n";
	
						xzfiltrado->Fill( z[j],x[j]); yzfiltrado->Fill( z[j],y[j]); xyfiltrado->Fill( x[j],y[j]); DISTRIBUCIONQ->Fill(z[j],100000000.-Char[j]); calor->Fill(x[j],y[j],Char[j]);
		
					//else {numtrazas=numtrazas+1;outputultimo<< 0.<<"	"<< 0. <<"	"<< 0.<<"	" <<std::endl; outputDataMatlab<<"     "  <<std::endl;}
					
				
					
			}//end of filtrito
				
			}//end for counter
		      
		     
		     std::cout<<"Número de pads con carga positiva en el evento ::"<<"	"<<i<<"	==	"<<counteraux<<"\n";
                     std::cout<<"Número de trazas encontradas en el evento ::"<<"	"<<i<<"	=="<<numtrazas+1<<"\n";
                     std::cout<<"**************************************************************************************************************"<<"\n";         
//Reordeno con respecto a la carga

				
				outputultimo<< 0.<<"	"<< 0. <<"	"<< 0.<<"	" <<std::endl;
				outputDataMatlab<<"     "  <<std::endl;		
				
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////FILTRADO 2////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			for(int indTB=0;indTB<512;++indTB)
			  {
				// Aquí tenemos la respuesta de la mesh para cada uno de los eventos en total (sin filtrar) en caso de tener 10 eventos tenemos 10 grupos de 512)
				PadWaveFormaTotal<<i<<"	"<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<"\n";	
			  }	
    T-> Fill();	
    	
     }//end for nEvents
   //T->Write();
   f->Write();
   f->Close();
    
   delete f;
/* Acumulación de todos los eventos de un run
        mesh->Scale(1.0/nEvents);
	for(int indTB=0;indTB<512;++indTB)
	{ PadWaveFormaTotal<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<mesh->GetBinError(indTB)<<"\n";	}
*/
// open and write the figures for the analysis
		
     TCanvas *c1 = new TCanvas();
	c1->Divide(2,2);
	c1->cd(1);
     	Acumulation->Draw("zcol l");
    	c1->cd(3);
    	Chargeall->Draw("zcol l");
	c1->cd(2);
    	AcumulationHC->Draw("zcol l");
	c1->cd(4);
    	ChargeHC->Draw("zcol l");
     TCanvas *c2 = new TCanvas();
	TimeStamp->Draw("zcol"); 
     TCanvas *c5 = new TCanvas();
	mesh->Draw("hist");
     TCanvas *c6 = new TCanvas();
	fPadPlane -> Draw("COL L");
        fPadPlane -> SetMinimum(1.0);
     TCanvas *c8 = new TCanvas();
	c8->Divide(2,2);
	c8->cd(1);
     	xztotal->Draw("zcol l");
    	c8->cd(3);
    	yztotal->Draw("zcol l");
	c8->cd(2);
    	xz_ordenado_z->Draw("zcol l");
	c8->cd(4);
    	yz_ordenado_z->Draw("zcol l");
     TCanvas *c9 = new TCanvas();
	c9->Divide(3,1);
	c9->cd(1);
	xzfiltrado->Draw("zcol");
	c9->cd(2);
	yzfiltrado->Draw("zcol");
	c9->cd(3);
	xyfiltrado->Draw("zcol");
     
     TCanvas *c15 = new TCanvas();
	FFTim->Draw("zcol");
    // TCanvas *c16 = new TCanvas();
	//ADC->Draw("zcol");
     TCanvas *c16 = new TCanvas();
	FFTreal->Draw("zcol");
     TCanvas *c17 = new TCanvas();
	FFTordenada->Draw("zcol");
     TCanvas *c18 = new TCanvas();
	ADCFILTR->Draw("zcol");
     TCanvas *c19 = new TCanvas();
	ADCSINFILTR->Draw("zcol");

     TCanvas *c20 = new TCanvas();
        ADCFILTRhisto->Draw("hist");

     TCanvas *c21 = new TCanvas();
	FFTconvolute->Draw("zcol");
     TCanvas *c22 = new TCanvas();
	DISTRIBUCIONQ->Draw("zcol");
    TCanvas *c23 = new TCanvas();
	calor->Draw("COLZ");
        
     outputData.close();
     outputFile.close();
     PadWaveFormaTotal.close();
     PadWaveFormaEventos.close();
     outputDataHierar.close();
     outputultimo.close();
     outputDataMatlab.close(); 	
    //gStyle->SetStatX(0.85);
     gStyle->SetStatW(0.3);
     gStyle->SetLabelOffset(1.2);
     gStyle->SetLabelFont(190);
     gStyle->SetOptStat(1);
     gStyle->SetPalette(55);
     gPad ->Update();
// Time of CPU-analisys
 	    std::cout << std::endl << std::endl;
            std::cout << "Macro finished succesfully."  << std::endl << std::endl;
            // -----   Finish   -------------------------------------------------------
            timer.Stop();
            Double_t rtime = timer.RealTime();
            Double_t ctime = timer.CpuTime();
            cout << endl << endl;
            cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
            cout << endl;

}

