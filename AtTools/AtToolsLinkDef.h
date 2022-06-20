#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;
#pragma link C++ namespace AtTools;
#pragma link C++ namespace RandomSample;

#pragma link C++ class AtTools::AtELossManager + ;
#pragma link C++ class AtTools::AtParsers + ;
#pragma link C++ class AtEulerTransformation + ;
#pragma link C++ class AtTools::AtTrackTransformer - !;

#pragma link C++ class AtSpaceChargeModel + ;
#pragma link C++ class AtLineChargeModel + ;
#pragma link C++ class AtRadialChargeModel - !;

#pragma link C++ class AtTools::AtKinematics + ;
#pragma link C++ class AtTools::AtVirtualTerminal + ;

#pragma link C++ class RandomSample::AtSample - !;
#pragma link C++ class RandomSample::AtIndependentSample - !;
#pragma link C++ class RandomSample::AtUniform - !;
#pragma link C++ class RandomSample::AtChargeWeighted - !;
#pragma link C++ class RandomSample::AtGaussian - !;
#pragma link C++ class RandomSample::AtWeightedGaussian - !;
#pragma link C++ class RandomSample::AtWeightedY - !;

#endif
