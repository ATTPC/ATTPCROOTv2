#ifndef AtRAWIC_H
#define AtRAWIC_H

#include "TObject.h"
#include "TROOT.h"

class AtRawIC : public TObject  {

   public:
    AtRawIC();
    ~AtRawIC();

    void Initialize();
    void SetRawfADC(Int_t idx, Int_t val);
    
    Int_t *GetRawfADC();
    Int_t GetRawfADC(Int_t idx);
   
   private:

    Int_t fRawAdc[512];

 ClassDef(AtRawIC, 1);

};

#endif
