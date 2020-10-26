/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtMagnet  file                               -----
// -----                Created 26/03/14  by M. Al-Turany              -----
// -------------------------------------------------------------------------

#include "AtMagnet.h"

#include "TGeoManager.h"
#include "FairRun.h"                    // for FairRun
#include "FairRuntimeDb.h"              // for FairRuntimeDb
#include "TList.h"                      // for TListIter, TList (ptr only)
#include "TObjArray.h"                  // for TObjArray
#include "TString.h"                    // for TString
#include "TGeoBBox.h"
#include "TGeoCompositeShape.h"
#include "TGeoTube.h"
#include "TGeoMaterial.h"
#include "TGeoElement.h"
#include "TGeoMedium.h"
#include <stddef.h>                     // for NULL
#include <iostream>                     // for operator<<, basic_ostream, etc



AtMagnet::~AtMagnet()
{
}
AtMagnet::AtMagnet()
  : FairModule("AtMagnet", "")
{
}

AtMagnet::AtMagnet(const char* name, const char* Title)
  : FairModule(name ,Title)
{
}

void AtMagnet::ConstructGeometry()
{
   
    TGeoVolume *top=gGeoManager->GetTopVolume();
    
    // define some materials
    TGeoMaterial *matFe     = new TGeoMaterial("Fe", 55.84, 26, 7.9);

    // define some media
    TGeoMedium *Fe     = new TGeoMedium("Fe", 100, matFe);
    

    // magnet yoke
    TGeoBBox *magyoke1 = new TGeoBBox("magyoke1", 261/2.0, 221/2.0, 278/2.0);
    TGeoBBox *magyoke2 = new TGeoBBox("magyoke2", 242/2.0, 202/2.0, 279/2.0);
    
    TGeoCompositeShape *magyokec = new TGeoCompositeShape("magyokec", "magyoke1-magyoke2");
    TGeoVolume *magyoke = new TGeoVolume("magyoke", magyokec, Fe);
    magyoke->SetLineColor(kViolet+2);
    magyoke->SetTransparency(50);
    top->AddNode(magyoke, 1, new TGeoTranslation(0, 6.079, 90));
    
    // magnet
     TGeoTube *SolenoidGeo = new TGeoTube("SolenoidGeo",125./4.0,274./4.0,229.0/2.0);// Radius divided by 2.0 
     TGeoVolume *SolenoidVol = new TGeoVolume("SolenoidVol", SolenoidGeo, Fe);
     SolenoidVol->SetLineColor(kWhite);
     SolenoidVol->SetTransparency(50);
     top->AddNode(SolenoidVol,1,new TGeoTranslation(0, 0, 110));

  /*  TGeoTubeSeg *magnet1a = new TGeoTubeSeg("magnet1a", 250, 300, 35, 45, 135);
    TGeoTubeSeg *magnet1b = new TGeoTubeSeg("magnet1b", 250, 300, 35, 45, 135);
    TGeoTubeSeg *magnet1c = new TGeoTubeSeg("magnet1c", 250, 270, 125, 45, 60);
    TGeoTubeSeg *magnet1d = new TGeoTubeSeg("magnet1d", 250, 270, 125, 120, 135);
    
    // magnet composite shape matrices
    TGeoTranslation *m1 = new TGeoTranslation(0, 0, 160);
    m1->SetName("m1");
    m1->RegisterYourself();
    TGeoTranslation *m2 = new TGeoTranslation(0, 0, -160);
    m2->SetName("m2");
    m2->RegisterYourself();
    
    TGeoCompositeShape *magcomp1 = new TGeoCompositeShape("magcomp1", "magnet1a:m1+magnet1b:m2+magnet1c+magnet1d");
    TGeoVolume *magnet1 = new TGeoVolume("magnet1", magcomp1, Fe);
    magnet1->SetLineColor(kYellow);
    top->AddNode(magnet1, 1, new TGeoTranslation(0, 0, 0));
    
    TGeoRotation m3;
    m3.SetAngles(180, 0, 0);
    TGeoTranslation m4(0, 0, 0);
    TGeoCombiTrans m5(m4, m3);
    TGeoHMatrix *m6 = new TGeoHMatrix(m5);
    top->AddNode(magnet1, 2, m6);*/
    
    
}



ClassImp(AtMagnet)














