#ifndef AtSIMULAtEDPOINT_H
#define AtSIMULAtEDPOINT_H

#include "TROOT.h"
#include "TObject.h"
#include "TVector3.h"


class AtSimulatedPoint : public TObject {
  public:
    AtSimulatedPoint();
    AtSimulatedPoint(std::size_t id,Int_t electronNumber, Double_t x, Double_t y, Double_t atime);
    ~AtSimulatedPoint();

    //!< Track ID setter
    void SetElectronNumber(Int_t electronNumber);
    //!< Position setter
    void SetPoint(std::size_t id,Int_t electronNumber, Double_t x, Double_t y, Double_t atime);
    //!< Position setter
    void SetPosition(Double_t x, Double_t y, Double_t atime);
    //!< ID setter
    void SetPointID(std::size_t id);
    //!< MCID setter
    void SetMCEventID(std::size_t mcid);


    //!< Track ID getter
    Int_t GetElectronNumber();
    //!< Position getter
    TVector3 GetPosition();
    //!< ID getter
    std::size_t GetPointID();
    //!<MC ID getter  
    std::size_t GetMCEventID();    

  private:
    //!< Track ID having this hit
    Int_t fElectronNumber;
    //!< Position
    TVector3 fPosition;
    //!< Point ID
    std::size_t fPointID;
    //!< MC Event ID;
    std::size_t fMCEventID;


  ClassDef(AtSimulatedPoint, 1);
};

#endif
