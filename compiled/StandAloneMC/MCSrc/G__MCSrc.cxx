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
#include "MCSrc.hh"
#include "MCMinimization.hh"

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
                  typeid(::MCMinimization), DefineBehavior(ptr, ptr),
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
                  typeid(::MCMinimization::FitPar), DefineBehavior(ptr, ptr),
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
      return  p ? ::new((::ROOT::TOperatorNewHelper*)p) ::MCMinimization::FitPar : new ::MCMinimization::FitPar;
   }
   static void *newArray_MCMinimizationcLcLFitPar(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::TOperatorNewHelper*)p) ::MCMinimization::FitPar[nElements] : new ::MCMinimization::FitPar[nElements];
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
         instance("vector<int>", -2, "vector", 214,
                  typeid(vector<int>), DefineBehavior(ptr, ptr),
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
      return  p ? ::new((::ROOT::TOperatorNewHelper*)p) vector<int> : new vector<int>;
   }
   static void *newArray_vectorlEintgR(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::TOperatorNewHelper*)p) vector<int>[nElements] : new vector<int>[nElements];
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
         instance("vector<double>", -2, "vector", 214,
                  typeid(vector<double>), DefineBehavior(ptr, ptr),
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
      return  p ? ::new((::ROOT::TOperatorNewHelper*)p) vector<double> : new vector<double>;
   }
   static void *newArray_vectorlEdoublegR(Long_t nElements, void *p) {
      return p ? ::new((::ROOT::TOperatorNewHelper*)p) vector<double>[nElements] : new vector<double>[nElements];
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

namespace {
  void TriggerDictionaryInitialization_G__MCSrc_Impl() {
    static const char* headers[] = {
"MCSrc.hh",
"MCMinimization.hh",
0
    };
    static const char* includePaths[] = {
"/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC/MCSrc/inc",
"/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include/root",
"/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include",
"/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/include",
"/home/ayyadlim/fair_install_ROOT6/FairRoot.v15.07.sp/include",
"/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC/MCSrc/MCSrc",
"/home/ayyadlim/fair_install_ROOT6/FairSoft.jul15.sp/include/root",
"/home/ayyadlim/fair_install_ROOT6/ATTPCROOTv2/compiled/StandAloneMC/MCSrc/",
0
    };
    static const char* fwdDeclCode = 
R"DICTFWDDCLS(
#pragma clang diagnostic ignored "-Wkeyword-compat"
#pragma clang diagnostic ignored "-Wignored-attributes"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
extern int __Cling_Autoloading_Map;
class __attribute__((annotate("$clingAutoload$MCMinimization.hh")))  MCMinimization;
)DICTFWDDCLS";
    static const char* payloadCode = R"DICTPAYLOAD(

#ifndef G__VECTOR_HAS_CLASS_ITERATOR
  #define G__VECTOR_HAS_CLASS_ITERATOR 1
#endif
#ifndef R__HAVE_CONFIG
  #define R__HAVE_CONFIG 1
#endif

#define _BACKWARD_BACKWARD_WARNING_H
#include "MCSrc.hh"
#include "MCMinimization.hh"

#undef  _BACKWARD_BACKWARD_WARNING_H
)DICTPAYLOAD";
    static const char* classesHeaders[]={
"MCMinimization", payloadCode, "@",
"MCMinimization::FitPar", payloadCode, "@",
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
