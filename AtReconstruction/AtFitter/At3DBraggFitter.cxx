#include "At3DBraggFitter.h"

#include <FairLogger.h>

#include <TDirectory.h>
#include <TF1.h>
#include <TMinuit.h>
#include <TSystem.h>

#include <chrono>
#include <thread>

ClassImp(AtFITTER::At3DBraggFitter);

void AtFITTER::At3DBraggFitter::Init()
{

   AtFITTER::gMaterial = new catima::Material();
   AtFITTER::gMaterial->density(fMaterialDensity);

   for (auto materialComponent : fMaterialComponents)
      AtFITTER::gMaterial->add_element(std::get<0>(materialComponent), std::get<1>(materialComponent),
                                       std::get<2>(materialComponent));
}

std::unique_ptr<At3DBraggFitResult> AtFITTER::At3DBraggFitter::ProcessTrack(AtTrack &track)
{

   gSystem->Load("libMinuit");

   // Get the scattering angle to set the bin size accordingly.
   auto *pattern = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
   Double_t theta =
      TMath::ATan2(std::abs(pattern->GetDirection().Z()),
                   TMath::Sqrt(std::pow(pattern->GetDirection().X(), 2) + std::pow(pattern->GetDirection().Y(), 2)));
   AtFITTER::gDS = fDZ / TMath::Cos(theta);

   Double_t maxLength = TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2));
   int nSBins = std::ceil(maxLength / AtFITTER::gDS);

   // Initialize the Bragg curve histogram.
   AtFITTER::gHistExpBraggCurve =
      new TH1F("gHistExpBraggCurve", "gHistExpBraggCurve", nSBins, 0, nSBins * AtFITTER::gDS);
   AtFITTER::gHistExpBraggCurve->SetDirectory(0);

   // Fill the Bragg curve histogram.
   std::vector<std::pair<Double_t, Double_t>> archLengthVsELossValues = *(track.Get3DBraggCurveValues());
   for (std::pair<Double_t, Double_t> pairBragg : archLengthVsELossValues)
      AtFITTER::gHistExpBraggCurve->Fill(pairBragg.first, pairBragg.second);

   for (Int_t i = 1; i <= AtFITTER::gHistExpBraggCurve->GetNbinsX(); i++)
      // AtFITTER::gHistExpBraggCurve->SetBinError(i, TMath::Sqrt(AtFITTER::gHistExpBraggCurve->GetBinContent(i)));
      AtFITTER::gHistExpBraggCurve->SetBinError(i, 0.1 * AtFITTER::gHistExpBraggCurve->GetBinContent(i));

   // Initialize an At3DBraggResult object to store the fit results.
   std::unique_ptr<At3DBraggFitResult> fitResult = std::make_unique<At3DBraggFitResult>();

   while (fParticleIdx < fAZPairs.size()) {

      auto AZPair = fAZPairs[fParticleIdx];
      AtFITTER::gMassUma = fParticleMassesUma[fParticleIdx];

      catima::Projectile *p = new catima::Projectile(AZPair.first, AZPair.second);
      SetProjectile(p);

      // Set up Minuit fitter
      TMinuit minuit(2);
      minuit.SetFCN(BraggFCN);

      // Configure fitting parameters
      /*minuit.DefineParameter(0, "kinE", fKEGuess, 0.1, fMinKE, fMaxKE);      // MeV
      //minuit.DefineParameter(1, "amplFactor", 3e3, 1e1, 2e3, 3e3);
      minuit.DefineParameter(1, "amplFactor", 3e3, 1e1, 1e0, 1e4);*/

      Double_t KEGuess = EstimateEnergyFromRange(
         AtFITTER::gHistExpBraggCurve->GetXaxis()->GetBinCenter(AtFITTER::gHistExpBraggCurve->GetMaximumBin()));
      // Double_t amplFactorGuess = EstimateAmplitudeFactor(KEGuess);

      // minuit.DefineParameter(0, "kinE", KEGuess, 0.25, 0, 0);      // MeV
      minuit.DefineParameter(0, "kinE", KEGuess, 0.25, KEGuess - 0.5, KEGuess + 0.5); // MeV
      // minuit.DefineParameter(0, "kinE", fKEGuess, 0.1, 0, 0);      // MeV
      minuit.DefineParameter(1, "amplFactor", 3.5e3, 3e2, 3.2e3, 3.8e3);
      // minuit.DefineParameter(1, "amplFactor", amplFactorGuess, 3e2, 0, 0);
      // minuit.DefineParameter(1, "amplFactor", amplFactorGuess, 3e2, amplFactorGuess - 6e2, amplFactorGuess + 6e2);

      // Minimize first KE.
      double arglist[10];
      int ierflg = 0;
      arglist[0] = 5000000; // Max calls
      arglist[1] = 0.1;     // Tolerance
      // arglist[1] = 0.1; // Tolerance

      // minuit.FixParameter(1);
      // minuit.mnexcm("MIGRAD", arglist, 2, ierflg);
      minuit.mnexcm("SIMPLEX", arglist, 2, ierflg);
      /*
               // Now minimize amplFactor.
               minuit.mnfree(1);
               minuit.FixParameter(0);
               minuit.mnexcm("MIGRAD", arglist, 2, ierflg);

               // Last fit to tune both.
               minuit.mnfree(0);
               minuit.mnexcm("MIGRAD", arglist, 2, ierflg);
      */
      // Retrieve results
      double fitPar[2], fitErr[2];
      for (int i = 0; i < 2; i++) {
         minuit.GetParameter(i, fitPar[i], fitErr[i]);
      }

      // Store fitted parameters for current particle.
      fitResult->AddAZPair(AZPair);
      fitResult->AddKineticEnergy(fitPar[0]);
      fitResult->AddAmplitudeFactor(fitPar[1]);

      // Get the CATIMA Bragg curve with the fitted parameters.
      Double_t kineticEnergy = fitPar[0];
      Double_t amplitudeFactor = fitPar[1];

      // std::cout << "La puta madre" << std::endl;
      std::vector<std::pair<Double_t, Double_t>> eLossModel = GetELossModel(kineticEnergy);
      Int_t iteModel{0};

      // Get chi2 value
      Double_t fval = 0;
      for (int i = 1; i <= AtFITTER::gHistExpBraggCurve->GetNbinsX(); i++) {
         double range = AtFITTER::gHistExpBraggCurve->GetBinCenter(i);
         double charge = AtFITTER::gHistExpBraggCurve->GetBinContent(i);
         double err = AtFITTER::gHistExpBraggCurve->GetBinError(i);

         if (range <= 30)
            continue;

         if (charge == 0)
            continue;

         for (Int_t j = iteModel; j < eLossModel.size(); j++) {
            auto modelPair = eLossModel[j];
            /*std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "Greetings good sir." << std::endl;
            std::cout << "range = " << range << std::endl;
            std::cout << "modelPair.first = " << modelPair.first << std::endl;*/
            if (std::abs(modelPair.first - range) < 0.0001) {
               // if (modelPair.first == range) {
               // std::cout << "Hewo! :3" << std::endl;
               double model = amplitudeFactor * modelPair.second;
               if (err > 0)
                  fval += pow((charge - model) / err, 2);
               iteModel = j;
               break;
            }
         } // For loop model values.
      }    // For loop experimental values.

      fitResult->AddELoss(eLossModel);
      fitResult->AddChi2(fval);

      // std::vector<std::pair<Double_t, Double_t>> dEdx = GetDeDx(kineticEnergy);
      // fitResult->AddDeDx(dEdx);

      fParticleIdx++;
   }

   fParticleIdx = 0;
   int bestFitIdx{0};
   double bestChi2{999999};
   while (fParticleIdx < fAZPairs.size()) {
      double chi2 = fitResult->GetChi2(fParticleIdx);
      if (chi2 < bestChi2) {
         bestChi2 = chi2;
         bestFitIdx = fParticleIdx;
      }
      fParticleIdx++;
   }
   fitResult->SetBestFitIndex(bestFitIdx);

   fParticleIdx = 0;
   return std::move(fitResult);
}

std::vector<std::unique_ptr<AtFittedTrack>> AtFITTER::At3DBraggFitter::ProcessTracks(std::vector<AtTrack> &tracks)
{
   std::vector<std::unique_ptr<AtFittedTrack>> letMeCompilePls;
   return letMeCompilePls;
}

void AtFITTER::At3DBraggFitter::SetProjectileIndex(Int_t i)
{

   auto AZPair = fAZPairs[i];
   catima::Projectile *p = new catima::Projectile(AZPair.first, AZPair.second);
   SetProjectile(p);
   AtFITTER::gMassUma = fParticleMassesUma[i];
}

Double_t AtFITTER::At3DBraggFitter::EstimateEnergyFromRange(Double_t range)
{

   Double_t kineticEnergy{0.1};

   AtFITTER::gProjectile->T = kineticEnergy / AtFITTER::gMassUma;
   Double_t catimaRange =
      catima::range(*AtFITTER::gProjectile, *AtFITTER::gMaterial) / AtFITTER::gMaterial->density() * 10;

   while (catimaRange < range) {

      kineticEnergy += 0.01;

      AtFITTER::gProjectile->T = kineticEnergy / AtFITTER::gMassUma;
      catimaRange = catima::range(*AtFITTER::gProjectile, *AtFITTER::gMaterial) / AtFITTER::gMaterial->density() * 10;
   }

   return kineticEnergy;
}

Double_t AtFITTER::At3DBraggFitter::EstimateAmplitudeFactor(Double_t kineticEnergy)
{

   std::vector<std::pair<Double_t, Double_t>> eLossModel = GetELossModel(kineticEnergy);

   Double_t estimatedAmplitudeFactor{0};
   Int_t entries{0};

   Int_t iteModel{0};
   for (int i = 1; i <= AtFITTER::gHistExpBraggCurve->GetNbinsX(); i++) {
      double range = AtFITTER::gHistExpBraggCurve->GetBinCenter(i);
      double charge = AtFITTER::gHistExpBraggCurve->GetBinContent(i);
      double err = AtFITTER::gHistExpBraggCurve->GetBinError(i);

      if (range <= 30)
         continue;

      if (charge == 0)
         continue;

      for (Int_t j = iteModel; j < eLossModel.size(); j++) {
         auto modelPair = eLossModel[j];
         if (std::abs(modelPair.first - range) < 0.0001 && modelPair.second != 0) {
            estimatedAmplitudeFactor += charge / modelPair.second;
            entries++;
            break;
         }
      } // For loop model values.
   }    // For loop experimental values.

   if (entries)
      return estimatedAmplitudeFactor / entries;
   return 3e3;
}

std::vector<std::pair<Double_t, Double_t>> AtFITTER::At3DBraggFitter::GetDeDx(Double_t kineticEnergy)
{

   std::vector<std::pair<Double_t, Double_t>> dEdxVector;

   double remainingKineticEnergy = kineticEnergy;

   Double_t range{0};
   Double_t maxLength = TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2));

   while (range < maxLength) {

      catima::Result result =
         catima::calculate(*AtFITTER::gProjectile, *AtFITTER::gMaterial, remainingKineticEnergy / AtFITTER::gMassUma);

      Double_t dEdx = result.dEdxi * AtFITTER::gMaterial->density();

      Double_t ELoss = dEdx * (AtFITTER::gDS / 10.) / 200;
      remainingKineticEnergy -= ELoss;

      dEdxVector.push_back(std::make_pair(range / 10., dEdx));

      range += AtFITTER::gDS / 10. / 200;
   }

   return dEdxVector;
}

std::vector<std::pair<double, double>> AtFITTER::At3DBraggFitter::GetELossModel(Double_t kineticEnergy)
{

   std::vector<std::pair<double, double>> ELossModel;

   double remainingKineticEnergy = kineticEnergy;

   int ite{0};

   Double_t range{AtFITTER::gDS / 2.};
   Double_t maxLength = TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2));

   while (range < maxLength) {

      Double_t totalBinELoss{0};
      for (Int_t i = 0; i < 200; i++) {
         catima::Result result = catima::calculate(*AtFITTER::gProjectile, *AtFITTER::gMaterial,
                                                   remainingKineticEnergy / AtFITTER::gMassUma);

         double ELoss = result.dEdxi * AtFITTER::gMaterial->density() * (AtFITTER::gDS / 10.) / 200;
         remainingKineticEnergy -= ELoss;
         totalBinELoss += ELoss;
      }

      ELossModel.push_back(std::make_pair(range, totalBinELoss));

      range += AtFITTER::gDS;

      ite++;
   }

   return ELossModel;
}

void AtFITTER::At3DBraggFitter::BraggFCN(int &npar, double *gin, double &fval, double *par, int iflag)
{
   // Fitting parameters.
   Double_t kineticEnergy = par[0];
   Double_t amplitudeFactor = par[1];

   // Get the CATIMA Bragg curve.
   std::vector<std::pair<Double_t, Double_t>> eLossModel = AtFITTER::At3DBraggFitter::GetELossModel(kineticEnergy);
   Int_t iteModel{0};

   fval = 0;
   for (int i = 1; i <= AtFITTER::gHistExpBraggCurve->GetNbinsX(); i++) {
      double range = AtFITTER::gHistExpBraggCurve->GetBinCenter(i);
      double charge = AtFITTER::gHistExpBraggCurve->GetBinContent(i);
      double err = AtFITTER::gHistExpBraggCurve->GetBinError(i);

      if (range <= 30)
         continue;

      if (charge == 0)
         continue;

      for (Int_t j = iteModel; j < eLossModel.size(); j++) {
         auto modelPair = eLossModel[j];
         // std::cout << "Greetings good sir." << std::endl;
         // std::cout << "range = " << range << std::endl;
         // std::cout << "modelPair.first = " << modelPair.first << std::endl;
         if (std::abs(modelPair.first - range) < 0.0001) {
            // std::cout << "Hewo! :3" << std::endl;
            double model = amplitudeFactor * modelPair.second;
            if (err > 0)
               fval += pow((charge - model) / err, 2);
            iteModel = j;
            break;
         }
         // std::this_thread::sleep_for(std::chrono::seconds(1));
      } // For loop model values.
   }    // For loop experimental values.
   // std::cout << "Iteration chi2 = " << fval << std::endl;
   // std::this_thread::sleep_for(std::chrono::seconds(1));
}
