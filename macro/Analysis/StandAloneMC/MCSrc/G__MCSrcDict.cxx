// Do NOT change. Changes will be lost next time file is generated

#define R__DICTIONARY_FILENAME \
   dIhomedIayyadlimdIfair_install_ROOT6dIATTPCROOTv2dImacrodIAnalysisdIStandAloneMCdIMCSrcdIG__MCSrcDict

/*******************************************************************/
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define G__DICTIONARY
#include <RConfig.h>
#include <TBuffer.h>
#include <TClass.h>
#include <TDictAttributeMap.h>
#include <TError.h>
#include <TInterpreter.h>
#include <TMemberInspector.h>
#include <TROOT.h>
#include <TVirtualMutex.h>

#ifndef G__ROOT
#define G__ROOT
#endif

#include <RtypesImp.h>
#include <TCollectionProxyInfo.h>
#include <TFileMergeInfo.h>
#include <TIsAProxy.h>

#include <algorithm>
/*******************************************************************/

#include <TDataMember.h>

// Since CINT ignores the std namespace, we need to do so in this file.
namespace std {
}
using namespace std;

// Header files passed as explicit arguments
#include "MCSrc.hh"

// Header files passed via #pragma extra_include

namespace {
void TriggerDictionaryInitialization_G__MCSrcDict_Impl()
{
   static const char *headers[] = {0};
   static const char *includePaths[] = {
      "/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include/root",
      "/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include",
      "/home/ayyadlim/fair_install_ROOT6/FairRoot.v15.07.sp/include",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/include",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/macro/Analysis/StandAloneMC/MCSrc",
      "/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include/root",
      "/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include",
      "/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include/root",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/macro/Analysis/StandAloneMC/MCSrc/",
      0};
   static const char *fwdDeclCode =
      R"DICTFWDDCLS(
#pragma clang diagnostic ignored "-Wkeyword-compat"
#pragma clang diagnostic ignored "-Wignored-attributes"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
extern int __Cling_Autoloading_Map;
)DICTFWDDCLS";
   static const char *payloadCode = R"DICTPAYLOAD(

#ifndef G__VECTOR_HAS_CLASS_ITERATOR
  #define G__VECTOR_HAS_CLASS_ITERATOR 1
#endif

#define _BACKWARD_BACKWARD_WARNING_H
#ifdef _OPENMP
#include <omp.h>
#endif

#undef  _BACKWARD_BACKWARD_WARNING_H
)DICTPAYLOAD";
   static const char *classesHeaders[] = {nullptr};

   static bool isInitialized = false;
   if (!isInitialized) {
      TROOT::RegisterModule("G__MCSrcDict", headers, includePaths, payloadCode, fwdDeclCode,
                            TriggerDictionaryInitialization_G__MCSrcDict_Impl, {}, classesHeaders);
      isInitialized = true;
   }
}
static struct DictInit {
   DictInit() { TriggerDictionaryInitialization_G__MCSrcDict_Impl(); }
} __TheDictionaryInitializer;
} // namespace
void TriggerDictionaryInitialization_G__MCSrcDict()
{
   TriggerDictionaryInitialization_G__MCSrcDict_Impl();
}
