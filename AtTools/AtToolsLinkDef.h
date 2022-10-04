#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;
#pragma link C++ namespace AtTools;
#pragma link C++ namespace RandomSample;
#pragma link C++ namespace ElectronicResponse;

#pragma link C++ class AtTools::AtELossManager + ;
#pragma link C++ class AtTools::AtParsers + ;
#pragma link C++ class AtEulerTransformation + ;
#pragma link C++ class AtTools::AtTrackTransformer - !;

#pragma link C++ class AtSpaceChargeModel - !;
#pragma link C++ class AtLineChargeModel - !;
#pragma link C++ class AtRadialChargeModel - !;
#pragma link C++ class AtEDistortionModel - !;

#pragma link C++ class AtTools::AtKinematics + ;
#pragma link C++ class AtTools::AtVirtualTerminal + ;

#pragma link C++ class RandomSample::AtSample - !;
#pragma link C++ class RandomSample::AtIndependentSample - !;
#pragma link C++ class RandomSample::AtUniform - !;
#pragma link C++ class RandomSample::AtChargeWeighted - !;
#pragma link C++ class RandomSample::AtGaussian - !;
#pragma link C++ class RandomSample::AtWeightedGaussian - !;
#pragma link C++ class RandomSample::AtWeightedY - !;

#pragma link C++ enum RandomSample::SampleMethod;
#pragma link C++ function RandomSample::CreateSampler;

#pragma link C++ class ElectronicResponse::AtElectronicResponse - !;
#pragma link C++ class ElectronicResponse::AtNominalResponse - !;
#pragma link C++ class ElectronicResponse::AtReducedTimeResponse - !;
#pragma link C++ class ElectronicResponse::AtVectorResponse - !;
#pragma link C++ class ElectronicResponse::AtFileResponse - !;

#endif
