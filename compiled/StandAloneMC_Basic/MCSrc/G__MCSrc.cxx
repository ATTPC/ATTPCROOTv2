// Do NOT change. Changes will be lost next time file is generated

#define R__DICTIONARY_FILENAME G__MCSrc

/*******************************************************************/
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#define G__DICTIONARY
#include "RConfig.h"
#include "TClass.h"
#include "TDictAttributeMap.h"
#include "TInterpreter.h"
#include "TROOT.h"
#include "TBuffer.h"
#include "TMemberInspector.h"
#include "TInterpreter.h"
#include "TVirtualMutex.h"
#include "TError.h"

#ifndef G__ROOT
#define G__ROOT
#endif

#include "RtypesImp.h"
#include "TIsAProxy.h"
#include "TFileMergeInfo.h"
#include <algorithm>
#include "TCollectionProxyInfo.h"
/*******************************************************************/

#include "TDataMember.h"

// Since CINT ignores the std namespace, we need to do so in this file.
namespace std {} using namespace std;

// Header files passed as explicit arguments
#include "/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_Basic/MCSrc/MCSrc.hh"
#include "/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_Basic/MCSrc/MCMinimization.hh"
#include "/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_Basic/MCSrc/MCQMinimization.hh"

// Header files passed via #pragma extra_include

namespace ROOT {
   static TClass *MCMinimization_Dictionary();
   static void MCMinimization_TClassManip(TClass*);
   static void *new_MCMinimization(void *p = 0);
   static void *newArray_MCMinimization(Long_t size, void *p);
   static void delete_MCMinimization(void *p);
   static void deleteArray_MCMinimization(void *p);
   static void destruct_MCMinimization(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const ::MCMinimization*)
   {
      ::MCMinimization *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(::MCMinimization));
      static ::ROOT::TGenericClassInfo 
         instance("MCMinimization", "MCMinimization.hh", 20,
                  typeid(::MCMinimization), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &MCMinimization_Dictionary, isa_proxy, 4,
                  sizeof(::MCMinimization) );
      instance.SetNew(&new_MCMinimization);
      instance.SetNewArray(&newArray_MCMinimization);
      instance.SetDelete(&delete_MCMinimization);
      instance.SetDeleteArray(&deleteArray_MCMinimization);
      instance.SetDestructor(&destruct_MCMinimization);
      return &instance;
   }
   TGenericClassInfo *GenerateInitInstance(const ::MCMinimization*)
   {
      return GenerateInitInstanceLocal((::MCMinimization*)0);
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const ::MCMinimization*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *MCMinimization_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const ::MCMinimization*)0x0)->GetClass();
      MCMinimization_TClassManip(theClass);
   return theClass;
   }

   static void MCMinimization_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   static TClass *MCMinimizationcLcLFitPar_Dictionary();
   static void MCMinimizationcLcLFitPar_TClassManip(TClass*);
   static void *new_MCMinimizationcLcLFitPar(void *p = 0);
   static void *newArray_MCMinimizationcLcLFitPar(Long_t size, void *p);
   static void delete_MCMinimizationcLcLFitPar(void *p);
   static void deleteArray_MCMinimizationcLcLFitPar(void *p);
   static void destruct_MCMinimizationcLcLFitPar(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const ::MCMinimization::FitPar*)
   {
      ::MCMinimization::FitPar *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(::MCMinimization::FitPar));
      static ::ROOT::TGenericClassInfo 
         instance("MCMinimization::FitPar", "MCMinimization.hh", 34,
                  typeid(::MCMinimization::FitPar), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &MCMinimizationcLcLFitPar_Dictionary, isa_proxy, 4,
                  sizeof(::MCMinimization::FitPar) );
      instance.SetNew(&new_MCMinimizationcLcLFitPar);
      instance.SetNewArray(&newArray_MCMinimizationcLcLFitPar);
      instance.SetDelete(&delete_MCMinimizationcLcLFitPar);
      instance.SetDeleteArray(&deleteArray_MCMinimizationcLcLFitPar);
      instance.SetDestructor(&destruct_MCMinimizationcLcLFitPar);
      return &instance;
   }
   TGenericClassInfo *GenerateInitInstance(const ::MCMinimization::FitPar*)
   {
      return GenerateInitInstanceLocal((::MCMinimization::FitPar*)0);
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const ::MCMinimization::FitPar*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *MCMinimizationcLcLFitPar_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const ::MCMinimization::FitPar*)0x0)->GetClass();
      MCMinimizationcLcLFitPar_TClassManip(theClass);
   return theClass;
   }

   static void MCMinimizationcLcLFitPar_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   static TClass *MCQMinimization_Dictionary();
   static void MCQMinimization_TClassManip(TClass*);
   static void *new_MCQMinimization(void *p = 0);
   static void *newArray_MCQMinimization(Long_t size, void *p);
   static void delete_MCQMinimization(void *p);
   static void deleteArray_MCQMinimization(void *p);
   static void destruct_MCQMinimization(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const ::MCQMinimization*)
   {
      ::MCQMinimization *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(::MCQMinimization));
      static ::ROOT::TGenericClassInfo 
         instance("MCQMinimization", "MCQMinimization.hh", 29,
                  typeid(::MCQMinimization), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &MCQMinimization_Dictionary, isa_proxy, 4,
                  sizeof(::MCQMinimization) );
      instance.SetNew(&new_MCQMinimization);
      instance.SetNewArray(&newArray_MCQMinimization);
      instance.SetDelete(&delete_MCQMinimization);
      instance.SetDeleteArray(&deleteArray_MCQMinimization);
      instance.SetDestructor(&destruct_MCQMinimization);
      return &instance;
   }
   TGenericClassInfo *GenerateInitInstance(const ::MCQMinimization*)
   {
      return GenerateInitInstanceLocal((::MCQMinimization*)0);
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const ::MCQMinimization*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *MCQMinimization_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const ::MCQMinimization*)0x0)->GetClass();
      MCQMinimization_TClassManip(theClass);
   return theClass;
   }

   static void MCQMinimization_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   // Wrappers around operator new
   static void *new_MCMinimization(void *p) {
      return  p ? new(p) ::MCMinimization : new ::MCMinimization;
   }
   static void *newArray_MCMinimization(Long_t nElements, void *p) {
      return p ? new(p) ::MCMinimization[nElements] : new ::MCMinimization[nElements];
   }
   // Wrapper around operator delete
   static void delete_MCMinimization(void *p) {
      delete ((::MCMinimization*)p);
   }
   static void deleteArray_MCMinimization(void *p) {
      delete [] ((::MCMinimization*)p);
   }
   static void destruct_MCMinimization(void *p) {
      typedef ::MCMinimization current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class ::MCMinimization

namespace ROOT {
   // Wrappers around operator new
   static void *new_MCMinimizationcLcLFitPar(void *p) {
      return  p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) ::MCMinimization::FitPar : new ::MCMinimization::FitPar;
   }
   static void *newArray_MCMinimizationcLcLFitPar(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) ::MCMinimization::FitPar[nElements] : new ::MCMinimization::FitPar[nElements];
   }
   // Wrapper around operator delete
   static void delete_MCMinimizationcLcLFitPar(void *p) {
      delete ((::MCMinimization::FitPar*)p);
   }
   static void deleteArray_MCMinimizationcLcLFitPar(void *p) {
      delete [] ((::MCMinimization::FitPar*)p);
   }
   static void destruct_MCMinimizationcLcLFitPar(void *p) {
      typedef ::MCMinimization::FitPar current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class ::MCMinimization::FitPar

namespace ROOT {
   // Wrappers around operator new
   static void *new_MCQMinimization(void *p) {
      return  p ? new(p) ::MCQMinimization : new ::MCQMinimization;
   }
   static void *newArray_MCQMinimization(Long_t nElements, void *p) {
      return p ? new(p) ::MCQMinimization[nElements] : new ::MCQMinimization[nElements];
   }
   // Wrapper around operator delete
   static void delete_MCQMinimization(void *p) {
      delete ((::MCQMinimization*)p);
   }
   static void deleteArray_MCQMinimization(void *p) {
      delete [] ((::MCQMinimization*)p);
   }
   static void destruct_MCQMinimization(void *p) {
      typedef ::MCQMinimization current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class ::MCQMinimization

namespace ROOT {
   static TClass *vectorlEintgR_Dictionary();
   static void vectorlEintgR_TClassManip(TClass*);
   static void *new_vectorlEintgR(void *p = 0);
   static void *newArray_vectorlEintgR(Long_t size, void *p);
   static void delete_vectorlEintgR(void *p);
   static void deleteArray_vectorlEintgR(void *p);
   static void destruct_vectorlEintgR(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const vector<int>*)
   {
      vector<int> *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(vector<int>));
      static ::ROOT::TGenericClassInfo 
         instance("vector<int>", -2, "vector", 477,
                  typeid(vector<int>), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &vectorlEintgR_Dictionary, isa_proxy, 0,
                  sizeof(vector<int>) );
      instance.SetNew(&new_vectorlEintgR);
      instance.SetNewArray(&newArray_vectorlEintgR);
      instance.SetDelete(&delete_vectorlEintgR);
      instance.SetDeleteArray(&deleteArray_vectorlEintgR);
      instance.SetDestructor(&destruct_vectorlEintgR);
      instance.AdoptCollectionProxyInfo(TCollectionProxyInfo::Generate(TCollectionProxyInfo::Pushback< vector<int> >()));
      return &instance;
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const vector<int>*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *vectorlEintgR_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const vector<int>*)0x0)->GetClass();
      vectorlEintgR_TClassManip(theClass);
   return theClass;
   }

   static void vectorlEintgR_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   // Wrappers around operator new
   static void *new_vectorlEintgR(void *p) {
      return  p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<int> : new vector<int>;
   }
   static void *newArray_vectorlEintgR(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<int>[nElements] : new vector<int>[nElements];
   }
   // Wrapper around operator delete
   static void delete_vectorlEintgR(void *p) {
      delete ((vector<int>*)p);
   }
   static void deleteArray_vectorlEintgR(void *p) {
      delete [] ((vector<int>*)p);
   }
   static void destruct_vectorlEintgR(void *p) {
      typedef vector<int> current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class vector<int>

namespace ROOT {
   static TClass *vectorlEdoublegR_Dictionary();
   static void vectorlEdoublegR_TClassManip(TClass*);
   static void *new_vectorlEdoublegR(void *p = 0);
   static void *newArray_vectorlEdoublegR(Long_t size, void *p);
   static void delete_vectorlEdoublegR(void *p);
   static void deleteArray_vectorlEdoublegR(void *p);
   static void destruct_vectorlEdoublegR(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const vector<double>*)
   {
      vector<double> *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(vector<double>));
      static ::ROOT::TGenericClassInfo 
         instance("vector<double>", -2, "vector", 477,
                  typeid(vector<double>), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &vectorlEdoublegR_Dictionary, isa_proxy, 0,
                  sizeof(vector<double>) );
      instance.SetNew(&new_vectorlEdoublegR);
      instance.SetNewArray(&newArray_vectorlEdoublegR);
      instance.SetDelete(&delete_vectorlEdoublegR);
      instance.SetDeleteArray(&deleteArray_vectorlEdoublegR);
      instance.SetDestructor(&destruct_vectorlEdoublegR);
      instance.AdoptCollectionProxyInfo(TCollectionProxyInfo::Generate(TCollectionProxyInfo::Pushback< vector<double> >()));
      return &instance;
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const vector<double>*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *vectorlEdoublegR_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const vector<double>*)0x0)->GetClass();
      vectorlEdoublegR_TClassManip(theClass);
   return theClass;
   }

   static void vectorlEdoublegR_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   // Wrappers around operator new
   static void *new_vectorlEdoublegR(void *p) {
      return  p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<double> : new vector<double>;
   }
   static void *newArray_vectorlEdoublegR(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<double>[nElements] : new vector<double>[nElements];
   }
   // Wrapper around operator delete
   static void delete_vectorlEdoublegR(void *p) {
      delete ((vector<double>*)p);
   }
   static void deleteArray_vectorlEdoublegR(void *p) {
      delete [] ((vector<double>*)p);
   }
   static void destruct_vectorlEdoublegR(void *p) {
      typedef vector<double> current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class vector<double>

namespace ROOT {
   static TClass *vectorlEATHitgR_Dictionary();
   static void vectorlEATHitgR_TClassManip(TClass*);
   static void *new_vectorlEATHitgR(void *p = 0);
   static void *newArray_vectorlEATHitgR(Long_t size, void *p);
   static void delete_vectorlEATHitgR(void *p);
   static void deleteArray_vectorlEATHitgR(void *p);
   static void destruct_vectorlEATHitgR(void *p);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const vector<ATHit>*)
   {
      vector<ATHit> *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(vector<ATHit>));
      static ::ROOT::TGenericClassInfo 
         instance("vector<ATHit>", -2, "vector", 477,
                  typeid(vector<ATHit>), ::ROOT::Internal::DefineBehavior(ptr, ptr),
                  &vectorlEATHitgR_Dictionary, isa_proxy, 0,
                  sizeof(vector<ATHit>) );
      instance.SetNew(&new_vectorlEATHitgR);
      instance.SetNewArray(&newArray_vectorlEATHitgR);
      instance.SetDelete(&delete_vectorlEATHitgR);
      instance.SetDeleteArray(&deleteArray_vectorlEATHitgR);
      instance.SetDestructor(&destruct_vectorlEATHitgR);
      instance.AdoptCollectionProxyInfo(TCollectionProxyInfo::Generate(TCollectionProxyInfo::Pushback< vector<ATHit> >()));
      return &instance;
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const vector<ATHit>*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static TClass *vectorlEATHitgR_Dictionary() {
      TClass* theClass =::ROOT::GenerateInitInstanceLocal((const vector<ATHit>*)0x0)->GetClass();
      vectorlEATHitgR_TClassManip(theClass);
   return theClass;
   }

   static void vectorlEATHitgR_TClassManip(TClass* ){
   }

} // end of namespace ROOT

namespace ROOT {
   // Wrappers around operator new
   static void *new_vectorlEATHitgR(void *p) {
      return  p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<ATHit> : new vector<ATHit>;
   }
   static void *newArray_vectorlEATHitgR(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::Internal::TOperatorNewHelper*)p) vector<ATHit>[nElements] : new vector<ATHit>[nElements];
   }
   // Wrapper around operator delete
   static void delete_vectorlEATHitgR(void *p) {
      delete ((vector<ATHit>*)p);
   }
   static void deleteArray_vectorlEATHitgR(void *p) {
      delete [] ((vector<ATHit>*)p);
   }
   static void destruct_vectorlEATHitgR(void *p) {
      typedef vector<ATHit> current_t;
      ((current_t*)p)->~current_t();
   }
} // end of namespace ROOT for class vector<ATHit>

namespace {
  void TriggerDictionaryInitialization_G__MCSrc_Impl() {
    static const char* headers[] = {
"MCSrc.hh",
"MCMinimization.hh",
"MCQMinimization.hh",
0
    };
    static const char* includePaths[] = {
"/Users/Yassid/fair_install_ROOT6/FairSoftInst.nov15p3/include/root",
"/Users/Yassid/fair_install_ROOT6/FairSoftInst.nov15p3/include",
"/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/include",
"/Users/Yassid/fair_install_ROOT6/FairRootInst.v15.11a/include",
"/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_Basic/MCSrc/MCSrc",
"/Users/Yassid/fair_install_ROOT6/FairSoftInst.nov15p3/include/root",
"/Users/Yassid/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC_Basic/MCSrc/",
0
    };
    static const char* fwdDeclCode = R"DICTFWDDCLS(
#line 1 "G__MCSrc dictionary forward declarations' payload"
#pragma clang diagnostic ignored "-Wkeyword-compat"
#pragma clang diagnostic ignored "-Wignored-attributes"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
extern int __Cling_Autoloading_Map;
class __attribute__((annotate("$clingAutoload$MCMinimization.hh")))  MCMinimization;
class __attribute__((annotate("$clingAutoload$MCQMinimization.hh")))  MCQMinimization;
)DICTFWDDCLS";
    static const char* payloadCode = R"DICTPAYLOAD(
#line 1 "G__MCSrc dictionary payload"

#ifndef G__VECTOR_HAS_CLASS_ITERATOR
  #define G__VECTOR_HAS_CLASS_ITERATOR 1
#endif
#ifndef R__HAVE_CONFIG
  #define R__HAVE_CONFIG 1
#endif

#define _BACKWARD_BACKWARD_WARNING_H
#include "MCSrc.hh"
#include "MCMinimization.hh"
#include "MCQMinimization.hh"

#undef  _BACKWARD_BACKWARD_WARNING_H
)DICTPAYLOAD";
    static const char* classesHeaders[]={
"MCMinimization", payloadCode, "@",
"MCMinimization::FitPar", payloadCode, "@",
"MCQMinimization", payloadCode, "@",
nullptr};

    static bool isInitialized = false;
    if (!isInitialized) {
      TROOT::RegisterModule("G__MCSrc",
        headers, includePaths, payloadCode, fwdDeclCode,
        TriggerDictionaryInitialization_G__MCSrc_Impl, {}, classesHeaders);
      isInitialized = true;
    }
  }
  static struct DictInit {
    DictInit() {
      TriggerDictionaryInitialization_G__MCSrc_Impl();
    }
  } __TheDictionaryInitializer;
}
void TriggerDictionaryInitialization_G__MCSrc() {
  TriggerDictionaryInitialization_G__MCSrc_Impl();
}
