/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                  AtDetectorList header file               -----
// -----                  M. Al-Turany   June 2014                     -----
// -------------------------------------------------------------------------

/** Defines unique identifier for all detectors system **/
/** THis is needed for stack filtring **/

#ifndef AtDetectorList_H
#define AtDetectorList_H 1
// kSTOPHERE is needed for iteration over the enum. All detectors have to be put before.
enum DetectorId { kAtTpc, kAtSiArray, kAtApollo, kSTOPHERE, kAtSeGA };

#endif
