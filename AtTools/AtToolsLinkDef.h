#ifdef __CINT__

#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ nestedclass;
#pragma link C++ nestedtypedef;
#pragma link C++ namespace AtTools;
#pragma link C++ namespace RandomSample;
#pragma link C++ namespace ElectronicResponse;
#pragma link C++ namespace tk;
#pragma link C++ namespace AtTools::Kinematics;

#pragma link C++ class AtTools::AtELossManager + ;
#pragma link C++ class AtTools::AtParsers + ;
#pragma link C++ class AtEulerTransformation + ;
#pragma link C++ class AtTools::AtTrackTransformer - !;
#pragma link C++ class AtTools::AtELossModel - !;
#pragma link C++ class AtTools::AtELossTable - !;

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
#pragma link C++ class RandomSample::AtWeightedGaussianTrunc - !;
#pragma link C++ class RandomSample::AtY - !;

#pragma link C++ enum RandomSample::SampleMethod;
#pragma link C++ function RandomSample::CreateSampler;

#pragma link C++ class ElectronicResponse::AtElectronicResponse - !;
#pragma link C++ class ElectronicResponse::AtNominalResponse - !;
#pragma link C++ class ElectronicResponse::AtReducedTimeResponse - !;
#pragma link C++ class ElectronicResponse::AtVectorResponse - !;
#pragma link C++ class ElectronicResponse::AtFileResponse - !;

#pragma link C++ class AtCutHEIST - !;
#pragma link C++ class CSVRow < int> - !;
#pragma link C++ class CSVIterator < int> - !;
#pragma link C++ class CSVRange < int> - !;
#pragma link C++ class CSVRange < std::string> - !;
#pragma link C++ class tk::spline - !;

#pragma link C++ class AtFindVertex - !;

#pragma link C++ function AtTools::GetHitFunctionTB;
#pragma link C++ function AtTools::GetHitFunction;
#pragma link C++ function AtTools::GetTB;
#pragma link C++ function AtTools::GetDriftTB;
#pragma link C++ function AtTools::SplitString;

#pragma link C++ function AtTools::Kinematics::GetGamma;
#pragma link C++ function AtTools::Kinematics::GetVelocity;
#pragma link C++ function AtTools::Kinematics::GetBeta;
#pragma link C++ function AtTools::Kinematics::GetRelMom;
#pragma link C++ function AtTools::Kinematics::AtoE;
#pragma link C++ function AtTools::Kinematics::EtoA;

#endif
