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

void baselineana()
{
 	TStopwatch timer;
  	timer.Start();



    TFile *newfile2 = new TFile("./cousa.root", "recreate"); 
     TTree *t2       = new TTree("PhysicsTree", " my little tree ") ;
		

    TFile *f = new TFile("points.root","RECREATE");
    	TTree *T = new TTree("points","Projection");
     TBranch *meu    = t2->Branch("meu",&meu, "meu/F" ) ;
     
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
    TH2F* Acumulation = new TH2F("Acumulation","Acumulation",400,-300,300,400,-300,300);
    TH2F* AcumulationHC = new TH2F("AcumulationHC","AcumulationHC",400,-300,300,400,-300,300);
    TH1F* mesh = new TH1F("mesh","mesh",512,0,511);
    TH1F* mesh2 = new TH1F("mesh2","mesh2",512,0,511);
    TH1F* ADCFILTRhistoyproy = new TH1F("ADCFILTRhistoyproy","ADCFILTRhistoyproy",512,-350,1100);
    TH2F* ADCFILTR = new TH2F("ADCFILTR","ADCFILTR",300,0,512,300,-2000,4500);
    TH2F* ADCSINFILTR = new TH2F("ADCSINFILTR","ADCSINFILTR",300,0,512,300,-2000,4500);
    TH2F* xztotal = new TH2F("xztotal","xztotal",200,0,1000,200,-300,300);
    TH2F* yztotal = new TH2F("yztotal","yztotal",200,0,1000,200,-300,300);
    TH2F* xz_ordenado_z = new TH2F("xz_ordenado_z","xz_ordenado_z",200,0,1000,200,-300,300);
    TH2F* yz_ordenado_z = new TH2F("yz_ordenado_z","yz_ordenado_z",200,0,1000,200,-300,300);
    TH2F* xzfiltrado = new TH2F("xzfiltrado","xzfiltrado",200,0,1000,200,-300,300);
    TH2F* yzfiltrado = new TH2F("yzfiltrado","yzfiltrado",200,0,1000,200,-300,300);
    TH2F* xyfiltrado = new TH2F("xyfiltrado","xyfiltrado",400,-300,300,400,-300,300);

       TClonesArray *events = nullptr;
       TBranch *eventsBranch = tree->GetBranch("ATEventH");
       eventsBranch->SetAddress(&events);
       Int_t firstEvent = 0;
       Int_t eventCount = std::numeric_limits<Int_t>::max();
       Int_t const lastEvent = (eventCount < (eventsBranch->GetEntries() - firstEvent) ? eventCount : (eventsBranch->GetEntries() - firstEvent)) + firstEvent - 0;
      //std::cout<<" piiiiiiiiiiiiiiiiiiiiiiiiiiiiii :: "<<TMath::Pi()<<"\n";
      size_t size = 10000;
      	
	Double_t z[size];Double_t y[size];Double_t x[size];Double_t r[size];
	Double_t Charge[size];
	Double_t Qhit[size];
	Double_t Char[size];
	Double_t firstfourier[size];
	Double_t Amplitudefourier[size];
	Double_t phasefourier[size];
	Double_t thirdfourier[size];
	Double_t firstphasefourier[size];
	Double_t aconvolute[size];
	Double_t bconvolute[size];
	Double_t adcfil[size];
	Double_t adcfiltrada[size];
	Double_t difcharge[size];
	Int_t counteraux; Double_t baseproy;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////baseline analisis////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
			int maxAdcIdx=0;
			Double_t basecorr;
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
					
	              	    }
				Double_t maximo= GetMaximum(adc);//aquí obtengo el máximo para cada uno de los tracks de los eventos (512 TimeBuckets) 
				Double_t minimo= GetMinimum(adc);//aquí obtengo el mínimo para cada uno de los tracks de los eventos (512 TimeBuckets) 
				//std::cout<<"MÁXIMO::"<<maximo<<"	"<<"MÍNIMO::"<<minimo<<"\n";

	
			//Compute the transform and look at the magnitude of the output
   			  TH1 *fourier =0;
				
   			 	TVirtualFFT::SetTransform(0);
			 
   			 fourier = mesh2->FFT(fourier, "MAG");
	                 fourier->SetStats(kTRUE);
   			 //Look at the phase of the output
				delete fourier;
   			 TH1 *phase = 0;
   			 phase = mesh2->FFT(phase, "PH");
  		
			delete phase;
			
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
   			 
   			 fft_back=0;
			  delete fft_back;
			 
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

///obtengo la transformada filtrada en forma binomica para poder realizar la transformada inversa
// Multipilico uno a uno por la funcion rect que es la FFT de la sinc; si quiero otra función para el filtrado se multiplica uno a uno realizando la FFT		
			
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////FILTRADO///////////////////////////////////////////////////////////////
			if (indTB<25) { //Así mantiene la integridad del pulso
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
			//Integro la correccion a la carga de Yassid
			if (indTB>20 && adcfil[indTB]>0 && adcfil[indTB]>adcfil[indTB-1] && adcfil[indTB]>adcfil[indTB+1]){//si tengo un máximo
				for (Int_t j=0;j<10;j++){
				basecorr+=adcfil[indTB-8-j]; //realizo la corrección a la base para ese máximo (los 10 pads anteriores a -8)
				}}
			//std::cout<<"basecorr:"<<"	"<<basecorr<<"	"<<"\n";

			 if(indTB>11){
        			for(Int_t i=0;i<11;i++){

          				if(adcfil[indTB-i+10] > 0 && adcfil[indTB-i+10] < 4000){
           				 adcfiltrada[indTB]=(adcfil[indTB]-basecorr/10.)*(1/sqrt(n_size)); //Substract the baseline correction
         				}else adcfiltrada[indTB]=(adcfil[indTB])*(1/sqrt(n_size));
      				 }
   				  }

			ADCFILTR->Fill(indTB,adcfiltrada[indTB]);
			ADCFILTRhistoyproy->Fill(adcfiltrada[indTB]);
		
			      
			    }
		
				
	             	}//if
			 
	             }//for
			
		      //std::cout<<" Raw event number : "<<i<<"\n";
	              std::cout<<" Number of pads Totales:: "<<padArray->size()<<"\n";


			//indTB es dt en la transformada
			
				
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

				sumacarga+=ADCFILTRhistoyproy->GetBinContent(indTB);
					
				//ADCFILTR->Fill(indTB,adcfiltrada[indTB]);
			        //ADCFILTRhisto->Fill(adcfiltrada[indTB]);
				
				//std::cout<<i<< "	"<<indTB<< "	"<<"adcfiltrada::"<<adcfiltrada[indTB]<<"	"<<"\n";
				
			  }	
				
				// Si hay mas de 5 tracks por hit y el numero de tracks separados es 'tracks.size()' encontrados por evento ******************************************************************************************************************************************************************
//con este if me quito la mayoría del cross-talcking y window. Si pongo hit.GetCharge puedo recuperar por donde va el haz y algo de cross-talking en trazas; con track.GetHitArray()->size()<150 me quito la mayoría de eventos 'ruido', track.GetHitArray()->size() son el numero de tracks en una traza identificada por HC
// Con hit.GetCharge() pongo una primera cota al ruido fino teniendo un compromiso entre el ruido que me cargo y el número de puntos en los tracks
//hitGetCharge es el máximo del pulso de cada pad activado con cada track encontrado
//hitGetQHit es la señal integrada para cada TimeBucket

                                
				
				if( track.GetHitArray()->size()>5  && track.GetHitArray()->size()<200 ){//( )|| hit.GetCharge()<0 para ver el haz // && track.GetHitArray()->size()>5//     &&  hit.GetCharge()>0 
//tengo que realizar otro if si calculatezgeo(1) es menor que el rango máximo del 22Mg en ATTPC

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

	        
				//Double_t* adc = GetPadWaveForm(hit.GetHitPadNum(),padArray);
				
				//PadWaveForma <<i<< "	"<< adc[i]<<"	"<< hit.GetHitPadNum()<<"       "<< track.GetHitArray()->size()<<"       "<< tracks.size()<< "\n";
		 	
                 		}//end if charge
				
 				
				}//end of HITS event
                 }//end of for Tracks

	      }//end of if NULLevents

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////FILTRADO 2////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			      
				
                        counteraux=0;
			int numtrazas=0;

			puntox.clear();
			puntoy.clear();
			puntoz.clear();
			TBcarga.clear();
			
			for (int j=0;j<counter;++j){
				
			Double_t px;
			Double_t py;
			Double_t pz;
			Double_t pcarga;
			if ( Char[j]>0){//Filtrito para que no se repitan y quitarme ceros de la matriz  && z[j]<920 && que siempre sean hacia adelante (elásticos)
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
						
	
						xzfiltrado->Fill( z[j],x[j]); yzfiltrado->Fill( z[j],y[j]); xyfiltrado->Fill( x[j],y[j]); 
		
					
				
					
			}//end of filtrito
				
			}//end for counter
		      
		     
		     std::cout<<"Número de pads con carga positiva en el evento ::"<<"	"<<i<<"	==	"<<counteraux<<"\n";
                     std::cout<<"Número de trazas encontradas en el evento ::"<<"	"<<i<<"	=="<<numtrazas+1<<"\n";
                     std::cout<<"**************************************************************************************************************"<<"\n";         
//Reordeno con respecto a la carga
					
				
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////FILTRADO 2////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			
    T-> Fill();	
    	
     }//end for nEvents


   f->Write();
   f->Close();
    
   delete f;


   //ADCFILTRhistoyproy->Write();
   //newfile2->Write();
   


// open and write the figures for the analysis
/*		
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
     
*/     
    // TCanvas *c16 = new TCanvas();
	//ADC->Draw("zcol");

     TCanvas *c18 = new TCanvas();
	ADCFILTR->Draw("zcol");
     TCanvas *c19 = new TCanvas();
	ADCSINFILTR->Draw("zcol");

     TCanvas *c20 = new TCanvas();
        ADCFILTRhistoyproy->Draw("hist");
    
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
      newfile2->Close();
}

