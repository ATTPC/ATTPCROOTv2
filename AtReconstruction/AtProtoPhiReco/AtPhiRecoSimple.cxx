#include "AtPhiRecoSimple.h"

ClassImp(AtPhiRecoSimple)

   AtPhiRecoSimple::AtPhiRecoSimple()
{
   PhiDist = new TH1D("PhiDist", "PhiDist", 90.0, 0.0, 90.0);
}

AtPhiRecoSimple::~AtPhiRecoSimple() {}

void AtPhiRecoSimple::PhiAnalyze(AtEvent *event, AtProtoEvent *protoevent)
{

   event->SortHitArray();
   Int_t nHits = event->GetNumHits();

   AtProtoQuadrant *ProtoQuad1 = new AtProtoQuadrant(1); // Quadrant ID : 1
   ProtoQuad1->SetEventID(event->GetEventID());
   AtProtoQuadrant *ProtoQuad2 = new AtProtoQuadrant(2); // Quadrant ID : 2
   ProtoQuad2->SetEventID(event->GetEventID());
   AtProtoQuadrant *ProtoQuad3 = new AtProtoQuadrant(3); // Quadrant ID : 3
   ProtoQuad3->SetEventID(event->GetEventID());
   AtProtoQuadrant *ProtoQuad4 = new AtProtoQuadrant(4); // Quadrant ID : 4
   ProtoQuad4->SetEventID(event->GetEventID());

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit hit = event->GetHitArray()->at(iHit);
      AtHit *phit = &hit;
      Int_t PadNum = hit.GetHitPadNum();
      // std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNum<<std::endl;

      // Dividing into quadrants
      if (PadNum > 0 && PadNum < 64) {

         // std::cout<<" Quadrant 1 "<<std::endl;
         // std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNum<<std::endl;
         ProtoQuad1->AddHit(phit);

      } else if (PadNum > 63 && PadNum < 127) {

         // std::cout<<" Quadrant 2 "<<std::endl;
         // std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNum<<std::endl;
         ProtoQuad2->AddHit(phit);

      } else if (PadNum > 126 && PadNum < 190) {

         // std::cout<<" Quadrant 3 "<<std::endl;
         // std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNum<<std::endl;
         ProtoQuad3->AddHit(phit);

      } else if (PadNum > 189 && PadNum < 253) {

         // std::cout<<" Quadrant 4 "<<std::endl;
         // std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNum<<std::endl;
         ProtoQuad4->AddHit(phit);

      } else if (PadNum == 0) {

         // TODO: Central Pad
      }
   }

   PhiCalc(ProtoQuad1, event);
   PhiCalc(ProtoQuad2, event);
   PhiCalc(ProtoQuad3, event);
   PhiCalc(ProtoQuad4, event);

   // DO NOT ENABLE!
   // fQuadArray.push_back(*ProtoQuad1);
   // fQuadArray.push_back(*ProtoQuad2);
   // fQuadArray.push_back(*ProtoQuad3);
   // fQuadArray.push_back(*ProtoQuad4);

   protoevent->AddQuadrant(*ProtoQuad1);
   protoevent->AddQuadrant(*ProtoQuad2);
   protoevent->AddQuadrant(*ProtoQuad3);
   protoevent->AddQuadrant(*ProtoQuad4);

   delete ProtoQuad1;
   delete ProtoQuad2;
   delete ProtoQuad3;
   delete ProtoQuad4;
}

void AtPhiRecoSimple::PhiCalc(AtProtoQuadrant *quadrant, AtEvent *event)
{

   PhiDist->Reset();
   Double_t a = 0.5 + 0.125; // Small size of the strip plus half the dead area between strips
   Double_t da;              //
   std::vector<AtHit> fast_HArray;
   std::vector<AtHit> slow_HArray;

   // type A: a + da - Phi/90
   // type B: a + Phi/90

   // TODO This function is tuned for prototype micromegas,
   Int_t nHits = quadrant->GetNumHits();
   if (nHits > 1) {
      // std::cout<<" Number of hits in quadrant  : "<<nHits<<std::endl;
      for (Int_t iHit = 0; iHit < nHits - 1; iHit++) {

         Double_t phi = 0.0;
         AtHit qhit_f = quadrant->GetHitArray()->at(iHit);     // First strip
         AtHit qhit_s = quadrant->GetHitArray()->at(iHit + 1); // Second strip
         Int_t PadNum_qf = qhit_f.GetHitPadNum();
         Int_t PadNum_qs = qhit_s.GetHitPadNum();
         Double_t Q_f = qhit_f.GetCharge(); // TODO: For the moment we asume the charge to be the amplitude
         Double_t Q_s = qhit_s.GetCharge();
         Int_t M_f = event->GetHitPadMult(PadNum_qf);
         Int_t M_s = event->GetHitPadMult(PadNum_qs);
         Int_t T_f = qhit_f.GetTimeStamp();
         Int_t T_s = qhit_s.GetTimeStamp();

         /* if(quadrant->GetEventID()==12154){
                std::cout<<" ======================================================================= "<<std::endl;
                std::cout<<" Prototype quadrant : "<<quadrant->GetQuadrantID()<<std::endl;
                               std::cout<<" First Hit Pad : "<<PadNum_qf<<" First Pad Charge : "<<Q_f<<" First Pad
            TimeStamp : "<<T_f<<std::endl; std::cout<<" Second Hit Pad : "<<PadNum_qs<<" Second Pad Charge : "<<Q_s<<"
            Second Pad TimeStamp : "<<T_s<<std::endl; std::cout<<" Multiplicity first : "<<M_f<<" Multiplicity second :
            "<<M_s<<" Charge First (Amplitude) : "<<Q_f<<" Charge Second (Amplitude) : "<<Q_s<<std::endl;
                std::cout<<M_s<<" Charge First (Integral) : "<<qhit_f.GetQHit()<<" Charge Second (Integral) :
            "<<qhit_s.GetQHit()<<std::endl;
                            }*/

         // if(M_f==1) Q_f = qhit_f.GetQHit();
         // if(M_s==1) Q_s = qhit_s.GetQHit();

         if (PadNum_qf < 11)
            da = 1.0 - a;
         else
            da = 3.2 - a;
         if (PadNum_qs < 11)
            da = 1.0 - a;
         else
            da = 3.2 - a;

         if (M_f == 2) {                                // Check if the first Pad in the vector has multihit
            if (M_s == 2 && (PadNum_qf == PadNum_qs)) { // Check if the next hit in the vector is the same pad and same
                                                        // multiplicity (self-consistent)
               if (T_f > T_s) {                         // Check which hit arrives first
                  fast_HArray.push_back(qhit_f);
                  slow_HArray.push_back(qhit_s);
               } else if (T_f < T_s) {
                  fast_HArray.push_back(qhit_s);
                  slow_HArray.push_back(qhit_f);
               }
               continue; // TODO: If there is an isoleted pad with multihits, we are removing it. Fix this part.
            }

         } else if (M_f > 2)
            continue;

         /*if(quadrant->GetEventID()==12382  && quadrant->GetQuadrantID()==1){
           if(PadNum_qf==PadNum_qs && M_f==M_s){std::cout<<" Multihit "<<M_f<<" in Pads Num : "<<PadNum_qf<<" -
           "<<PadNum_qs<<std::endl;}
                             }*/

         if (PadNum_qs == PadNum_qf + 1) {

            if (quadrant->GetQuadrantID() % 2 == 1) {
               // std::cout<<" "<<std::endl;
               // std::cout<<" Inside an odd quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
               phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               /*TYPE A*/ if (PadNum_qf % 2 == 1 && PadNum_qs % 2 == 0)
                  phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               /*TYPE B*/ else if (PadNum_qf % 2 == 0 && PadNum_qs % 2 == 1)
                  phi = (((Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da + (Q_f - Q_s) / (Q_f + Q_s) - 1) * 45) +
                        90; // phi = 90.0-phi;
               else {
                  std::cout << " ======================================================================= " << std::endl;
                  std::cout << " Warning, even-odd sttagering not found. " << std::endl;
               }

            }

            else if (quadrant->GetQuadrantID() % 2 == 0) {

               // std::cout<<" Inside an even quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
               phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               if (PadNum_qf % 2 == 0 && PadNum_qs % 2 == 1)
                  phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               else if (PadNum_qf % 2 == 1 && PadNum_qs % 2 == 0)
                  phi = (((Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da + (Q_f - Q_s) / (Q_f + Q_s) - 1) * 45) +
                        90; // phi = 90.0-phi;
               else {
                  std::cout << " ======================================================================= " << std::endl;
                  std::cout << " Warning, even-odd sttagering not found. " << std::endl;
               }

            } else
               std::cout << " Invalid Quadrant ID : " << quadrant->GetQuadrantID() << std::endl;

         } else {
            // std::cout<<" ======================================================================= "<<std::endl;
            // std::cout<<" Warning, Pads are not adjacent. "<<std::endl;
            phi = 0.0;

         } // Adjacent strips

         // if(phi>90 || phi<0) std::cout<<" Phi out of boundaries!: "<<phi<<std::endl;
         PhiDist->Fill(phi);
         quadrant->AddPhiVal(phi);

         //  if(quadrant->GetEventID()==12382  && quadrant->GetQuadrantID()==1){
         //  std::cout<<" ======================================================================= "<<std::endl;
         // std::cout<<" Prototype quadrant : "<<quadrant->GetQuadrantID()<<std::endl;
         // std::cout<<" First Hit Pad : "<<PadNum_qf<<" First Pad Charge : "<<Q_f<<" First Pad TimeStamp :
         // "<<T_f<<std::endl;
         //  std::cout<<" Second Hit Pad : "<<PadNum_qs<<" Second Pad Charge : "<<Q_s<<" Second Pad TimeStamp :
         //  "<<T_s<<std::endl;
         // std::cout<<" Phi : "<<phi<<std::endl;
         // }

      } // nHits
   }    // nHits>1
   // PhiDist->Draw();

   // Send the vectors with multihit to the Phi overloaded Calculation function
   PhiCalcMulti(&fast_HArray, quadrant);
   PhiCalcMulti(&slow_HArray, quadrant);
   quadrant->SetPhiDistribution(PhiDist);
}

void AtPhiRecoSimple::PhiCalcMulti(std::vector<AtHit> *multihit_Array, AtProtoQuadrant *quadrant)
{

   Double_t a = 0.5 + 0.125; // Small size of the strip plus half the dead area between strips
   Double_t da;              //
   Double_t phi = 0.0;

   Int_t nHits = multihit_Array->size();

   if (nHits > 1) {
      for (Int_t iHit = 0; iHit < nHits - 1; iHit++) {
         AtHit qhit_f = multihit_Array->at(iHit);
         AtHit qhit_s = multihit_Array->at(iHit + 1);
         Int_t PadNum_qf = qhit_f.GetHitPadNum();
         Int_t PadNum_qs = qhit_s.GetHitPadNum();
         Double_t Q_f = qhit_f.GetCharge(); // TODO: For the moment we asume the charge to be the amplitude
         Double_t Q_s = qhit_s.GetCharge();
         Int_t T_f = qhit_f.GetTimeStamp();
         Int_t T_s = qhit_s.GetTimeStamp();

         if (PadNum_qf < 11)
            da = 1.0 - a;
         else
            da = 3.2 - a;
         if (PadNum_qs < 11)
            da = 1.0 - a;
         else
            da = 3.2 - a;

         if (PadNum_qs == PadNum_qf + 1) {

            if (quadrant->GetQuadrantID() % 2 == 1) {
               // std::cout<<" "<<std::endl;
               // std::cout<<" Inside an odd quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
               phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               /*TYPE A*/ if (PadNum_qf % 2 == 1 && PadNum_qs % 2 == 0)
                  phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               /*TYPE B*/ else if (PadNum_qf % 2 == 0 && PadNum_qs % 2 == 1)
                  phi = (((Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da + (Q_f - Q_s) / (Q_f + Q_s) - 1) * 45) +
                        90; // phi = 90.0-phi;
               else {
                  std::cout << " ======================================================================= " << std::endl;
                  std::cout << " Warning, even-odd sttagering not found. " << std::endl;
               }

            }

            else if (quadrant->GetQuadrantID() % 2 == 0) {

               // std::cout<<" Inside an even quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
               phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               if (PadNum_qf % 2 == 0 && PadNum_qs % 2 == 1)
                  phi = (1.0 - (Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da - (Q_f - Q_s) / (Q_f + Q_s)) * 45;
               else if (PadNum_qf % 2 == 1 && PadNum_qs % 2 == 0)
                  phi = (((Q_f - Q_s) / (Q_f + Q_s) * 2.0 * a / da + (Q_f - Q_s) / (Q_f + Q_s) - 1) * 45) +
                        90; // phi = 90.0-phi;
               else {
                  std::cout << " ======================================================================= " << std::endl;
                  std::cout << " Warning, even-odd sttagering not found. " << std::endl;
               }

            } else
               std::cout << " Invalid Quadrant ID : " << quadrant->GetQuadrantID() << std::endl;

         } else {
            // std::cout<<" ======================================================================= "<<std::endl;
            // std::cout<<" Warning, Pads are not adjacent. "<<std::endl;
            phi = 0.0;

         } // Adjacent strips

         PhiDist->Fill(phi);
         quadrant->AddPhiVal(phi);

         //  if(quadrant->GetEventID()==12382  && quadrant->GetQuadrantID()==1){
         // std::cout<<" ======================================================================= "<<std::endl;
         // std::cout<<" Prototype quadrant : "<<quadrant->GetQuadrantID()<<std::endl;
         // std::cout<<" First Hit Pad : "<<PadNum_qf<<" First Pad Charge : "<<Q_f<<" First Pad TimeStamp :
         // "<<T_f<<std::endl; std::cout<<" Second Hit Pad : "<<PadNum_qs<<" Second Pad Charge : "<<Q_s<<" Second Pad
         // TimeStamp : "<<T_s<<std::endl;
         // std::cout<<" Phi : "<<phi<<std::endl;
         // }

      } // nHits

   } // nHit>1
}
