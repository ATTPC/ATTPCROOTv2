// Do NOT change. Changes will be lost next time file is generated

#define R__DICTIONARY_FILENAME G__MCsrc_cuda

/*******************************************************************/
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#define G__DICTIONARY
#include "RConfig.h"
#include <TClass.h>
#include <TDictAttributeMap.h>
#include <TInterpreter.h>
#include <TROOT.h>
#include <TBuffer.h>
#include <TMemberInspector.h>
#include <TInterpreter.h>
#include <TVirtualMutex.h>
#include <TError.h>

#ifndef G__ROOT
#define G__ROOT
#endif

#include "RtypesImp.h"
#include <TIsAProxy.h>
#include <TFileMergeInfo.h>
#include <algorithm>
#include <TCollectionProxyInfo.h>
/*******************************************************************/

#include <TDataMember.h>

// Since CINT ignores the std namespace, we need to do so in this file.
namespace std {
}
using namespace std;

// Header files passed as explicit arguments
#include "MCsrc_cuda.hh"

// Header files passed via #pragma extra_include

namespace {
void TriggerDictionaryInitialization_G__MCsrc_cuda_Impl()
{
   static const char *headers[] = {"MCsrc_cuda.hh", 0};
   static const char *includePaths[] = {
      "/home/ayyadlim/fair_install_ROOT6/FairSoftInst.nov15p3/include/root",
      "/home/ayyadlim/fair_install_ROOT6/FairSoftInst.nov15p3/include",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/include",
      "/home/ayyadlim/fair_install_ROOT6/FairRootInst.v15.11a/include",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_cuda/MCsrc_cuda/MCsrc_cuda",
      "/home/ayyadlim/fair_install_ROOT6/FairSoftInst.nov15p3/include/root",
      "/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_cuda/MCsrc_cuda/",
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
#ifndef R__HAVE_CONFIG
  #define R__HAVE_CONFIG 1
#endif

#define _BACKWARD_BACKWARD_WARNING_H
#include "MCsrc_cuda.hh"

#undef  _BACKWARD_BACKWARD_WARNING_H
)DICTPAYLOAD";
   static const char *classesHeaders[] = {nullptr};

   static bool isInitialized = false;
   if (!isInitialized) {
      TROOT::RegisterModule("G__MCsrc_cuda", headers, includePaths, payloadCode, fwdDeclCode,
                            TriggerDictionaryInitialization_G__MCsrc_cuda_Impl, {}, classesHeaders);
      isInitialized = true;
   }
}
static struct DictInit {
   DictInit() { TriggerDictionaryInitialization_G__MCsrc_cuda_Impl(); }
} __TheDictionaryInitializer;
} // namespace
void TriggerDictionaryInitialization_G__MCsrc_cuda()
{
   TriggerDictionaryInitialization_G__MCsrc_cuda_Impl();
}
