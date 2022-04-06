#include "AtProtoEventAna.h"

#include <Rtypes.h>

ClassImp(AtProtoEventAna);

AtProtoEventAna::AtProtoEventAna() : TNamed("AtProtoEventAna", "Proto Event Analysis")
{
   fPar0_fit.clear();
   fPar1_fit.clear();
   fAngle.clear();
   fAngle_fit.clear();
   fRange.clear();
   fHoughPar.clear();
   fELossHitPattern.clear();
   fQELossHitPattern.clear();
   fQVertex.clear();
   fChi2.clear();
   fNDF.clear();
   fVertex02 = -500.0;
   fVertex13 = -500.0;
}

AtProtoEventAna::~AtProtoEventAna() {}

std::vector<Double_t> *AtProtoEventAna::GetAngleFit()
{
   return &fAngle_fit;
}
std::vector<Double_t> *AtProtoEventAna::GetAngle()
{
   return &fAngle;
}
std::vector<Double_t> *AtProtoEventAna::GetPar0()
{
   return &fPar0_fit;
}
std::vector<Double_t> *AtProtoEventAna::GetPar1()
{
   return &fPar1_fit;
}
std::vector<Double_t> *AtProtoEventAna::GetRange()
{
   return &fRange;
}
std::vector<std::pair<Double_t, Double_t>> *AtProtoEventAna::GetHoughPar()
{
   return &fHoughPar;
}
std::vector<std::pair<Double_t, Double_t>> *AtProtoEventAna::GetELossHitPattern()
{
   return &fELossHitPattern;
}
std::vector<std::vector<std::pair<Double_t, Double_t>>> *AtProtoEventAna::GetQELossHitPattern()
{
   return &fQELossHitPattern;
}
std::vector<Double_t> *AtProtoEventAna::GetVertex()
{
   return &fQVertex;
}
std::vector<Double_t> *AtProtoEventAna::GetChi2()
{
   return &fChi2;
}
std::vector<Int_t> *AtProtoEventAna::GetNDF()
{
   return &fNDF;
}

// Setters
void AtProtoEventAna::SetAngleFit(std::vector<Double_t> Angle_fit)
{
   fAngle_fit = Angle_fit;
}
void AtProtoEventAna::SetAngle(std::vector<Double_t> Angle)
{
   fAngle = Angle;
}
void AtProtoEventAna::SetPar0(std::vector<Double_t> Par0_fit)
{
   fPar0_fit = Par0_fit;
}
void AtProtoEventAna::SetPar1(std::vector<Double_t> Par1_fit)
{
   fPar1_fit = Par1_fit;
}
void AtProtoEventAna::SetRange(std::vector<Double_t> Range)
{
   fRange = Range;
}
void AtProtoEventAna::SetHoughPar(std::vector<std::pair<Double_t, Double_t>> HoughPar)
{
   fHoughPar = HoughPar;
}
void AtProtoEventAna::SetELHitPattern(std::vector<std::pair<Double_t, Double_t>> ELHitPattern)
{
   fELossHitPattern = ELHitPattern;
}
void AtProtoEventAna::SetQELHitPattern(std::vector<std::vector<std::pair<Double_t, Double_t>>> QELHitPattern)
{
   fQELossHitPattern = QELHitPattern;
}
void AtProtoEventAna::SetVertex02(Double_t Vertex02)
{
   fVertex02 = Vertex02;
}
void AtProtoEventAna::SetVertex13(Double_t Vertex13)
{
   fVertex13 = Vertex13;
}
void AtProtoEventAna::SetVertex(std::vector<Double_t> Vertex)
{
   fQVertex = Vertex;
}
void AtProtoEventAna::SetChi2(std::vector<Double_t> Chi2)
{
   fChi2 = Chi2;
}
void AtProtoEventAna::SetNDF(std::vector<Int_t> NDF)
{
   fNDF = NDF;
}
