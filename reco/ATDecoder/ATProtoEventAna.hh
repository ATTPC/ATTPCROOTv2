#ifndef ATPROTOEVENTANA_H
#define ATPROTOEVENTANA_H

#include "TROOT.h"
#include "TObject.h"

#include <vector>
#include <map>

class ATProtoEventAna : public TNamed {
  public:
    ATProtoEventAna();
    ~ATProtoEventAna();

    std::vector<Double_t> fPar0_fit;
    std::vector<Double_t> fPar1_fit;
    std::vector<Double_t> fAngle;
    std::vector<Double_t> fAngle_fit;
    std::vector<Double_t> fRange;
    std::vector<std::pair<Double_t,Double_t>> fHoughPar;
    std::vector<std::pair<Double_t,Double_t>> fELossHitPattern;
    std::vector<std::vector<std::pair<Double_t,Double_t>>> fQELossHitPattern;

    // Getters
    std::vector<Double_t>* GetAngleFit();
    std::vector<Double_t>* GetAngle();
    std::vector<Double_t>* GetPar0();
    std::vector<Double_t>* GetPar1();
    std::vector<Double_t>* GetRange();
    std::vector<std::pair<Double_t,Double_t>>* GetHoughPar();
    std::vector<std::pair<Double_t,Double_t>>* GetELossHitPattern();
    std::vector<std::vector<std::pair<Double_t,Double_t>>>* GetQELossHitPattern();

    //Setters
    void SetAngleFit(std::vector<Double_t> Angle_fit);
    void SetAngle(std::vector<Double_t> Angle);
    void SetPar0(std::vector<Double_t> Par0_fit);
    void SetPar1(std::vector<Double_t> Par1_fit);
    void SetRange(std::vector<Double_t> Range);
    void SetHoughPar(std::vector<std::pair<Double_t,Double_t>> HoughPar );
    void SetELHitPattern(std::vector<std::pair<Double_t,Double_t>> ELHitPattern );
    void SetQELHitPattern(std::vector<std::vector<std::pair<Double_t,Double_t>>> QELHitPattern );



   private:

    ClassDef(ATProtoEventAna, 2);

};

#endif
