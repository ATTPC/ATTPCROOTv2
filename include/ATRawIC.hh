#ifndef ATRAWIC_H
#define ATRAWIC_H

#include "TObject.h"
#include "TROOT.h"

class ATRawIC : public TObject  {

   public:
    ATRawIC();
    ~ATRawIC();

    void Initialize();
    void SetRawfADC(Int_t idx, Int_t val);
    
    Int_t *GetRawfADC();
    Int_t GetRawfADC(Int_t idx);
   
   private:

    Int_t fRawAdc[512];

 ClassDef(ATRawIC, 1);

};

#endif
