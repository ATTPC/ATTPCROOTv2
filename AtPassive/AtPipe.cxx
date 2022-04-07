/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtPipe  file                               -----
// -----                Created by M. Al-Turany  June 2014             -----
// -------------------------------------------------------------------------

#include "AtPipe.h"

#include <FairModule.h>
#include <TGeoVolume.h>
#include <TGeoPcon.h>
#include <TGeoMaterial.h>
#include <TGeoMedium.h>
#include <TGeoManager.h>

AtPipe::~AtPipe() = default;
AtPipe::AtPipe() : FairModule() {}

AtPipe::AtPipe(const char *name, const char *title) : FairModule(name, title) {}

// -----  ConstructGeometry  --------------------------------------------------
void AtPipe::ConstructGeometry()
{
   TGeoVolume *top = gGeoManager->GetTopVolume();

   // define some materials
   auto *matCarbon = new TGeoMaterial("C", 12.011, 6.0, 2.265);
   auto *matVacuum = new TGeoMaterial("Vacuum", 0, 0, 0);

   // define some media
   auto *Carbon = new TGeoMedium("C", 50, matCarbon);
   auto *Vacuum = new TGeoMedium("Vacuum", 60, matVacuum);

   Int_t nSects = 2;
   Double_t z[] = {-50, 0};   // in cm
   Double_t r[] = {2.5, 2.5}; // in cm
   Double_t Thickness = 0.05; // thickness of beam pipe [cm]
   auto *shape = new TGeoPcon(0., 360., nSects);
   for (Int_t iSect = 0; iSect < nSects; iSect++) {
      shape->DefineSection(iSect, z[iSect], r[iSect], r[iSect] + Thickness);
   }

   // ---> Voluwme
   auto *pipe = new TGeoVolume("AtPipe", shape, Carbon);

   // --Now create the same but diameter less by Thikness and vacuum instead of Carbon
   auto *Vshape = new TGeoPcon(0., 360., nSects);
   for (Int_t iSect = 0; iSect < nSects; iSect++) {
      Vshape->DefineSection(iSect, z[iSect], 0, r[iSect]);
   }

   // ---> Volume
   auto *Vpipe = new TGeoVolume("AtPipe", Vshape, Vacuum);

   // TGeoVolume *TPipe=gGeoManager->MakeTubs("TPipe",Vacuum,0,2.5,3.0,10,0);

   top->AddNode(pipe, 1);
   top->AddNode(Vpipe, 1);
   // top->AddNode(TPipe,1);
   // top->AddNodeOverlap(TPipe,1,new TGeoCombiTrans(0,0,0,new TGeoRotation("TPipe",0,-7,0)));
}
// ----------------------------------------------------------------------------

ClassImp(AtPipe)
