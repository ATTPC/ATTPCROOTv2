#include "ATTrigger.hh"

#include <string>
#include <sstream>



ClassImp(ATTrigger)

ATTrigger::ATTrigger()
{
}


ATTrigger::~ATTrigger()
{
}

void ATTrigger::SetAtMap(TString mapPath) {
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try
  {
    file.open(mapPath);

    // Int_t intCoboNum, intpad;
    Int_t nLines = 0;
    Int_t nPoints = 0;
    // fPadNum = 0;
    // for(int ii=0;ii<512;ii++) fDummy[ii] = 0.0;
    // memset(fCoboNumArray, 0, sizeof(fCoboNumArray));
    // fCoboNumArray[10240] = {0};
    // fPadNumArray[10241] = {0};

    while (!file.eof())
    {
      // read a single line
      std::string line;
      std::getline(file, line);

      try
      {
        Float_t CoboNum, asad, aget, channel, pad;

        // read with default seperator (space) seperated elements
        {
          std::istringstream iss(line);
          iss.exceptions(std::ifstream::failbit | std::ifstream::badbit);
          iss >> CoboNum >> asad >> aget >> channel >> pad;
        }

        printf("Successfully read point: CoboNum=%8f, asad=%8f, aget=%8f, channel=%8f, pad=%8f\n", CoboNum, asad, aget, channel, pad);

        ++nPoints;
      }
      catch (...)
      {
        std::cout << "Malformed point in line " << (nLines + 1) << "!" << std::endl;
      }

      // intCoboNum = (int) CoboNum;
      // intpad     = (int) pad;
      // fCoboNumArray[nLines] = CoboNum;
      // fPadNumArray[nLines] = pad;
      ++nLines;
    }

    file.close();

    std::cout << "Successfully read " << nPoints << " points in " << nLines << " lines!" << std::endl;
  }
  catch (...)
  {
    std::cout << "Something bad happened while reading the file!" << std::endl;
  }
}



void ATTrigger::SetTriggerParameters(Double_t read, Double_t write, Double_t MSB, Double_t LSB,
   Double_t width, Double_t fraction, Double_t threshold, Double_t window, Double_t height){

  fMultiplicity_threshold         = threshold;
  fMultiplicity_window            = window;
  fTrigger_height                 = height;

  fTime_factor = read/write;
  fTrigger_width = width * write;
	fPad_threshold = ((MSB*pow(2,4) + LSB)/128)*fraction*4096;
	fTime_window = fMultiplicity_window / (read*write);
	std::cout << "====Parameters for Trigger======"<<std::endl;
  std::cout << " ===Time Factor:   "<<fTime_factor<<std::endl;
  std::cout << " ===Trigger Width: "<<fTrigger_width<<" us?"<<std::endl;
  std::cout << " ===Pad Threshold: "<<fPad_threshold<<std::endl;
  std::cout << " ===Time Window:   "<<fTime_window<<" us?"<<std::endl;
}


Bool_t ATTrigger::ImplementTrigger(ATRawEvent *rawEvent, ATEvent *event){

  fEvent    = event;
  fRawEvent = rawEvent;
  fTrigger  = kFALSE;

  TMatrixD triggerSignal(10,512);  // The trigger signal is a vector with 522 positions. The first 10 one will be filled with CoBo number and the rest will be the trigger signal. [(0,10),(0,512)]
  TMatrixD result(10,512);

  Int_t numHits = fEvent->GetNumHits();
  Int_t numPads = fRawEvent->GetNumPads();

  //Loop over the NUMBER of SIGNALS in the EVENT
  for(Int_t iHit=0; iHit < numHits; iHit++){
    fHit = fEvent ->GetHitArray()->at(iHit);
    Int_t PadNumHit = fHit.GetHitPadNum();

    fPad = fRawEvent -> GetPad(PadNumHit,fValidPad);
    fRawAdc = fPad->GetRawADC();

    fPadNum = fPad->GetPadNum();


    //if (iHit<100){fHRawPulse.push_back( new TH2I(Form("fHRawPulse_Pad%i",iHit),Form("fHRawPulse_Pad%i",iHit),512,0,512,500,0,4100));}

    //LOOP over the TIME distribution of each signal of the EVENT
//  for (int j = 0; j<512; j++){
  //  if(fMaxRawADC <= fRawAdc[j]) fMaxRawADC =fRawAdc[j];
  //  if (iHit<100){ fHRawPulse.back()->Fill(j,fRawAdc[j]);}

  //  }

  //*************************************************************************************************
  //Calculation of the trigger signal for the given event
  //*************************************************************************************************

    double trigSignal[512];			//Empty Trigger Signal for each signal of the event
    fTbIdx = 0;

    while (fTbIdx < 512){
      if (fRawAdc[fTbIdx] > fPad_threshold) {
        for (Int_t ii = fTbIdx; ii < std::min(fTbIdx+fTrigger_width,512.); ii++){trigSignal[ii] += fTrigger_height;}
        fTbIdx+= fTrigger_width;

        }

      else fTbIdx +=1;


  }//While

    for(Int_t j = 0; j < 512; j++){
      if (trigSignal [j] != 48) {trigSignal[j] = 0;}
      if (trigSignal[j] > 0 ){
        //triggerSignal(j+10) = trigSignal[j];

            for (Int_t l = 0; l <10240; l++){
              if (fPadNumArray[l] == fPadNum){fCobo = fCoboNumArray[l];}
                }//Loop to check with Cobo is hitting
              triggerSignal(fCobo,j) = trigSignal[j];
              }//Positive signal if

            }//3rd foor loop
      triggerSignal(fCobo,0) = fCobo;



  //*************************************************************************************************
  //Calculation of the multiplicity signals from the trigger signals
  //*************************************************************************************************

    for (Int_t l =0; l < 512; l++){
      Int_t triggerWindow = l - fMultiplicity_window;
      fMinIdx = std::max(0,triggerWindow);
      fMaxIdx = l;

        for (Int_t ll =0; ll <10; ll++){
            fAccum = 0;
            result(ll,0)= fCobo;
          for(Int_t jj = fMinIdx; jj < fMaxIdx; jj++){
            fAccum += triggerSignal(ll,jj);
            result(ll,l) = fAccum * fTime_factor;

              }

            }//ll for

          }//l for

    //std::cerr << "Cobo -> " << result(3,0) << std::endl;

  //*************************************************************************************************
  //Determines if the given multiplicity signals rose above the multiplicity threshold in any CoBo.
  //*************************************************************************************************

    for (Int_t kk =0; kk < 10; kk++){
        if (result(kk,0) > 0){
          for (Int_t k =0; k < 512; k++){
            if(result(kk,k) >  fMultiplicity_threshold) {fTrigger = kTRUE;}

              }

            }
          }


        }//Loop on entries of the event

        return fTrigger;
}
