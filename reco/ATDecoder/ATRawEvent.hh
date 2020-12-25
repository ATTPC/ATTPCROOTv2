/*********************************************************************
 *   ATTPC ATRawEvent Stores a RawEvent composed by the ATTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *                                                                   *
 *********************************************************************/

#ifndef ATRAWEVENT_H
#define ATRAWEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include "ATPad.hh"

#include <vector>
#include <iostream>
#include <map>
#include <iterator>

class ATRawEvent : public TNamed
{
public:
  ATRawEvent();
  ATRawEvent(ATRawEvent *object);
  ~ATRawEvent();

  void Clear();

  // setters
  void SetEventID(ULong_t evtid);
  void SetPad(ATPad *pad);
  void SetIsGood(Bool_t value);
  void RemovePad(Int_t padNo);
  void SetTimestamp(ULong_t timestamp);
  void SetIsExtGate(Bool_t value);
  void SetSimMCPointMap(std::multimap<Int_t,std::size_t> map);

  // getters
  ULong_t GetEventID();
  Int_t GetNumPads();
  ULong_t GetTimestamp();
  Bool_t IsGood();
  Bool_t GetIsExtGate();

  std::vector<ATPad> *GetPads();

  ATPad *GetPad(Int_t padNo);
  ATPad *GetPad(Int_t PadNum,Bool_t& IsValid);

  std::multimap<Int_t,std::size_t>& GetSimMCPointMap();


private:
  ULong_t fEventID;
  std::vector<ATPad> fPadArray;
  ULong_t fTimestamp;

  Bool_t fIsGood;
  Bool_t fIsinGate;

  std::multimap<Int_t,std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

  ClassDef(ATRawEvent, 2);
};

#endif
