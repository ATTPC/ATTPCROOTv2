#include "S800.h"

int S800::DecodeS800(unsigned short *pevent, unsigned short twords)
{
   long long int n;
   unsigned short *p = pevent, sublength, plength, ptag, ID;
   unsigned short nwords, words;
   //++p;
   bool found = false;

   // twords--; // total event length is self-inclusive

   // Loop over sub-events until the right one is found
   while (twords > 0) {
      nwords = *p;
      ++p;
      // cout << "nwords: " << nwords << ".   twords: " << twords << endl;
      sublength = nwords;
      // cout << "sublength: " << sublength << endl;
      nwords--;
      if (*p++ != S800_PACKET) {
         twords -= sublength;
         p += sublength - 2;
      } else {
         nwords--;
         found = true;
         // fhasdata = true;
         break;
      }
   }
   if (!found)
      return 0;

   // Unpack S800 data in tree
   if (*p++ != S800_VERSION) {
      std::cout << "DecodedEvent: "
                << "Wrong version of S800 sub-event. Aborting ..." << std::endl;
      return 1;
   }
   nwords--;
   while (nwords > 0) {
      plength = *p;
      ++p;
      ptag = *p;
      ++p;
      switch (ptag) {
      case S800_TRIGGER_PACKET:
         // cout << "Trigger: " << hex << S800_TRIGGER_PACKET << dec << endl;
         p = DecodeS800Trigger(p);
         break;

      case S800_TOF_PACKET:
         // cout << "Tofr: " << hex << S800_TOF_PACKET << dec << endl;
         p = DecodeS800TimeOfFlight(p);
         break;

      case S800_FP_SCINT_PACKET:
         // cout << "FP SCINT: " << hex << S800_FP_SCINT_PACKET << dec << endl;
         words = plength - 2;
         while (words > 0) {
            ID = ((*p) & 0xF000) >> 12;
            p = DecodeS800Scintillator(p, ID, ID / 2);
            words -= 2;
         }
         break;

      case S800_FP_CRDC_PACKET:
         // cout << "CRDC: " << hex << S800_FP_CRDC_PACKET << dec << endl;
         ID = *p;
         ++p;
         // std::cout << "ID " << ID << std::endl;
         p = DecodeS800Crdc(p, ID);
         break;

      case S800_FP_IC_PACKET:
         // cout << "FP_IC: " << hex << S800_FP_IC_PACKET << dec << endl;
         p = DecodeS800IonChamber(p);
         break;

      case S800_TIMESTAMP_PACKET:
         // cout << "TIMESTAMP: " << hex << S800_TIMESTAMP_PACKET << dec << endl;
         n = *p++;
         n = (*p++ << 16 | n);
         n = (*p++ << 16 | n);
         n = (*p++ << 16 | n);
         this->SetInternalTS(n);
         break;

      case S800_EVENT_NUMBER_PACKET:
         // cout << "EVENT_NUMBER: " << hex << S800_EVENT_NUMBER_PACKET << dec << endl;
         n = *p++;
         n = (*p++ << 16 | n);
         n = (*p++ << 16 | n);
         this->SetEvtNr(n);
         break;

      case S800_FP_HODO_PACKET:
         // cout << "HODO: " << hex << S800_FP_HODO_PACKET << dec << endl;
         p = DecodeS800HodoScope(p);
         break;
      case S800_VME_TDC_PACKET:
         // cout << "VME_TDC: " << hex << S800_VME_TDC_PACKET << dec << endl;
         p = DecodeS800NewMultiHitTDC(p);
         break;
      default: // S800_II_CRDC_PACKET, S800_II_TRACK_PACKET...
         p += plength - 2;
         break;
      }
      nwords -= plength;
   }
   // this->SetTS(ts);
   // if(ffirst_ts<1){
   //   ffirst_ts = ts;
   // }

   return 0;
}

unsigned short *S800::DecodeS800TimeOfFlight(unsigned short *p)
{

   UShort_t words = (*(p - 2)) - 2, ch, dum;
   Short_t rf = -1;
   Short_t obj = -1;
   Short_t xfp = -1;
   Short_t si = -1;
   Short_t tac_obj = -1;
   Short_t tac_xfp = -1;
   while (words > 0) {
      ch = ((*p) & 0xf000) >> 12;
      int tmp = *p;
      ++p;
      if (ch == 12)
         rf = (tmp)&0xfff;
      else if (ch == 13)
         obj = (tmp)&0xfff;
      else if (ch == 14)
         xfp = (tmp)&0xfff;
      else if (ch == 15)
         si = (tmp)&0xfff;
      else if (ch == 5)
         tac_obj = (tmp)&0xfff;
      else if (ch == 4)
         tac_xfp = (tmp)&0xfff;
      else if (ch > 0 && ch < 8)
         dum = (tmp)&0xfff;
      words--;
   }
   this->GetTimeOfFlight()->Set(rf, obj, xfp, si);
   this->GetTimeOfFlight()->SetTAC(tac_obj, tac_xfp);
   return p;
}

unsigned short *S800::DecodeS800Trigger(unsigned short *p)
{
   UShort_t words = (*(p - 2)) - 2, ch;
   int registr = -1;
   int s800 = -1;
   int external1 = -1;
   int external2 = -1;
   int secondary = -1;
   registr = *p++;
   words--;
   while (words > 0) {
      ch = ((*p) & 0xf000) >> 12;
      if (ch == 8)
         s800 = (*p++) & 0xfff;
      if (ch == 9)
         external1 = (*p++) & 0xfff;
      if (ch == 10)
         external2 = (*p++) & 0xfff;
      if (ch == 11)
         secondary = (*p++) & 0xfff;
      words--;
   }
   this->GetTrigger()->Set(registr, s800, external1, external2, secondary);
   return p;
}

unsigned short *S800::DecodeS800Scintillator(unsigned short *p, unsigned short updown, int id)
{
   int de_up = -1;
   int time_up = -1;
   int de_down = -1;
   int time_down = -1;

   // Even updown: UP channels.  Odd ID: DOWN channels
   if (updown % 2 == 0) {
      de_up = (*p++) & 0xfff;
      time_up = (*p++) & 0xfff;
   } else {
      de_down = (*p++) & 0xfff;
      time_down = (*p++) & 0xfff;
   }
   this->GetScintillator(id)->SetID(id);
   this->GetScintillator(id)->Set(de_up, time_up, de_down, time_down);
   return p;
}

unsigned short *S800::DecodeS800HodoScope(unsigned short *p)
{
   UShort_t words = (*(p - 2)) - 2;
   UShort_t id;
   UShort_t ch;
   UShort_t energy;
   while (words > 0) {
      id = *p;
      if (id == 0) {
         p++;
         words--;
         while (words > 0) {
            ch = (((*p) & 0xF000) >> 12);
            energy = ((*p) & 0x0FFF);
            this->GetHodoscope(ch)->SetEnergy((Int_t)energy);
            p++;
            words--;
         }
      } else if (id == 1) {
         p++;
         words--;
         while (words > 0) {
            ch = (((*p) & 0xF000) >> 12) + 16;
            energy = ((*p) & 0x0FFF);
            this->GetHodoscope(ch)->SetEnergy((Int_t)energy);
            p++;
            words--;
         }
      } else if (id == 2) {
         p++;
         words--;
         while (words > 0) {
            // coincidence register A (for the first  16 channels)
            p++;
            words--;
            // coincidence register B (for the second 16 channels)
            p++;
            words--;
            // TAC time
            p++;
            words--;
         }
      } else {
         p++;
         words--;
      }
   }
   return p;
}

unsigned short *S800::DecodeS800Crdc(unsigned short *p, int id)
{
   UShort_t anode = -1;
   UShort_t tac = -1;
   this->GetCrdc(id)->SetID(id);

   Int_t tag;
   tag = S800_FP_CRDC_PACKET;
   if (*(p + 1) == tag + 1) {
      p = DecodeS800CrdcRaw(p, id);
   }
   if (*(p + 1) == tag + 5) {
      anode = *(p + 2);
      tac = *(p + 3);
      p += 4;
   }
   this->GetCrdc(id)->SetAnodeTAC(anode, tac);

   return p;
}

unsigned short *S800::DecodeS800IonChamber(unsigned short *p)
{
   UShort_t ch = -1;
   UShort_t raw = -1;
   if (*(p + 1) == S800_FP_IC_ENERGY_PACKET) {
      // IC packet with times
      UShort_t length;
      length = *p++;
      p++;
      length -= 2;
      while (length > 0) {
         ch = ((*p) & 0xf000) >> 12;
         raw = (*p++) & 0xfff;
         length--;
         this->GetIonChamber()->Set(ch, raw);
      }
   } else {
      // Old style IC packet
      UShort_t words = (*(p - 2)) - 2;
      while (words > 0) {
         ch = ((*p) & 0xf000) >> 12;
         raw = (*p++) & 0xfff;
         words--;
         this->GetIonChamber()->Set(ch, raw);
      }
   }

   return p;
}

unsigned short *S800::DecodeS800CrdcRaw(unsigned short *p, int id)
{
   static ULong_t total = 0, failed = 0;
   Short_t sampleBegin = 0, sampleWidth, isample, ichannel, cdata[4], connector, previous_sample = 0, ch, sindex = 0,
           previous_channel = 0, m_sampleWidth = 0; // Added Simon
   Short_t maxwidth = S800_CRDC_MAXWIDTH;
   Short_t channels;
   channels = S800_FP_CRDC_CHANNELS;

   unsigned short *pStore = p;
   bool mes1 = true, mes2 = true, mes3 = true, mes4 = true;
   bool debug = S800_DEBUG;
   UShort_t length = *p++;
   short i = length - 3;
   p++; // skip packet id
   UShort_t threshold = *p++;

   unsigned short data_test[256][32];       // Added Simon
   memset(data_test, 0, sizeof(data_test)); // Added Simon

   while (i > 0) {
      if ((*p) >> 15 != 1) {
         std::cout << "DecodedEvent: "
                   << "CRDC data is corrupted!" << std::endl;
         p++;
         i--;
         continue;
      } else {
         isample = ((*p) & 0x7FC0) >> 6;
         ichannel = (*p) & 0x003F;
         if (i == length - 3) {
            sampleBegin = isample;
            previous_sample = isample;
         }
         if (previous_channel > ichannel)
            sindex++;                 // Added Simon
         previous_channel = ichannel; // Added Simon
      }
      p++;
      i--;
      memset(cdata, 0, sizeof(cdata));
      while ((*p) >> 15 == 0) {
         connector = ((*p) & 0x0C00) >> 10;
         cdata[connector] = (*p) & 0x3FF;
         p++;
         i--;
         if (i == 0)
            break;
      }
      if (isample < sampleBegin || isample > sampleBegin + maxwidth) {
         if (debug)
            printf("Warning in Crdc Unpack: inconsistent sample number: %d (first: %d)\n", isample, sampleBegin);
         mes1 = false;
         // continue;//Commented Simon
      }
      if (isample < previous_sample) {
         if (debug)
            printf("Warning in Crdc Unpack: sample number lower than previous: %d (previous: %d)\n", isample,
                   previous_sample);
         mes2 = false;
         // continue;//Commented Simon
      }
      previous_sample = isample;
      for (int j = 0; j < 4; j++) {
         ch = ichannel + j * 64;
         if (cdata[j] != 0 && ch < channels) {
            if (cdata[j] < threshold) {
               if (debug)
                  printf("Warning in Crdc Unpack: data lower than threshold: %d (threshold: %d)\n", cdata[j],
                         threshold);
               mes3 = false;
            } else {
               // std::cout << "ch " << ch << " cdata[j]" << cdata[j] << " isample " << isample << std::endl;
               this->GetCrdc(id)->Set(cdata[j], isample, ch);
               data_test[ch][sindex] = cdata[j]; // Added Simon
            }
         } else if (cdata[j] != 0 && ch >= channels) {
            if (debug) {
               printf("Warning in Crdc Unpack: channel greater than limit: %d (limit: %d)\n", ch, channels);
            }
            mes4 = false;
         }
      }
      m_sampleWidth = sindex + 1; // Added Simon
      sampleWidth = isample - sampleBegin + 1;
      this->GetCrdc(id)->SetSampleWidth(sampleWidth);
   }

   /////////////////////////////// Added Simon
   // Integrate samples into pads (and subtract pedestals)
   for (int q = 0; q < channels; q++) {
      this->GetCrdc_test()->Set_raw(id, q, -1);
      int nsamples = 0; // added to fix bug
      for (int s = 0; s < m_sampleWidth; s++) {
         if (data_test[q][s] != 0) {
            nsamples++; // added to fix bug
            // raw[q] += (data[q][s] - ped[q]) / sampleWidth;
            // raw_test[q] += (data[q][s] - ped[q]);
            this->GetCrdc_test()->Set_raw(
               id, q, this->GetCrdc_test()->Get_raw(id, q) + (data_test[q][s])); // ! have to remove the pedestals
         }
      }
      if (nsamples > 0) {
         this->GetCrdc_test()->Set_raw(id, q, (this->GetCrdc_test()->Get_raw(id, q) + 1) / nsamples);
         std::cout << "raw " << this->GetCrdc_test()->Get_raw(id, q) << " nsamples " << nsamples << std::endl;
         // raw_test[q] = (raw_test[q] + 1 ) / nsamples; //added to fix bug
      }
   }
   ///////////////////////////////

   if (!mes1 || !mes2 || !mes3 || !mes4)
      failed++;
   total++;
   if (failed == 1000) {
      if (debug)
         printf("Errors in Crdc Unpackings: %g%%\n", 1.0 * failed / total * 100);
      total = 0;
      failed = 0;
   }
   return (pStore + length);
}

unsigned short *S800::DecodeS800NewMultiHitTDC(unsigned short *p)
{
   // Data should be interpreted in 16-bit words

   // Declare temporay arrays to hold the raw data
   unsigned short data[32][32];
   signed short hits[32];
   unsigned short raw[32];
   for (int i = 0; i < 32; i++) {
      hits[i] = -1;
      raw[i] = 0;
      for (int j = 0; j < 32; j++) {
         data[i][j] = 0;
      }
   }

   UShort_t length, ch, hit;
   length = *(p - 2);
   length -= 2;
   while (length > 0) {
      ch = (*p) & 0xFF;
      hit = (*p++) >> 8;
      if (hit < 32)
         data[ch][hit] = *p++;
      else
         p++;
      if (hit == 0)
         raw[ch] = data[ch][0];
      if (hit > hits[ch])
         hits[ch] = hit;
      length -= 2;
   }

   if (raw[15] != 0) {
      for (int i = 0; i < 13; i++) {
         switch (i) {
         case 0: // e1up
            if (hits[0] >= 0) {
               fMultiHitTOF.fE1Up.push_back((data[0][0] - raw[15]) * 0.0625);
            }
            break;
         case 1: // e1down
            if (hits[1] >= 0) {
               fMultiHitTOF.fE1Down.push_back((data[1][0] - raw[15]) * 0.0625);
            }
            break;
         case 2: // xf
            if (hits[2] >= 0) {
               for (int j = 0; j <= hits[2]; j++) {
                  fMultiHitTOF.fXf.push_back((data[2][j] - raw[15]) * 0.0625);
               }
            }
            break;
         case 3: // obj
            if (hits[3] >= 0) {
               for (int j = 0; j <= hits[3]; j++) {
                  fMultiHitTOF.fObj.push_back((data[3][j] - raw[15]) * 0.0625);
               }
            }
            break;
         case 4: // galotte
            if (hits[4] >= 0) {
               for (int j = 0; j <= hits[4]; j++) {
                  fMultiHitTOF.fGalotte.push_back((data[4][j] - raw[15]) * 0.0625);
               }
            }
            break;
         case 5: // rf
            if (hits[5] >= 0) {
               for (int j = 0; j <= hits[5]; j++) {
                  fMultiHitTOF.fRf.push_back((data[5][j] - raw[15]) * 0.0625);
               }
            }
            break;
         case 12: // hodoscope
            if (hits[12] >= 0) {
               for (int j = 0; j <= hits[12]; j++) {
                  fMultiHitTOF.fHodoscope.push_back((data[12][j] - raw[15]) * 0.0625);
               }
            }
            break;
         default: break;
         } // end Switch i
      }    // end for i
   }       // end if raw[15]!=0

   // cout<<"fNewTOF.fE1Up.size() "<<fNewTOF.fE1Up.size()<<endl;
   // cout<<"fNewTOF.fE1Down.size() "<<fNewTOF.fE1Down.size()<<endl;
   // cout<<"fNewTOF.obj.size() "<<fNewTOF.fObj.size()<<endl;

   return p;
}
