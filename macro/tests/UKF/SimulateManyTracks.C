std::string getEnergyPath()
{
   auto env = std::getenv("VMCWORKDIR");
   if (env == nullptr) {
      return "../../resources/energy_loss/HinH.txt"; // Default path assuming cwd is build/AtTools
   }
   return std::string(env) + "/resources/energy_loss/HinH.txt"; // Use environment variable
}
using ROOT::Math::XYZPoint;
using ROOT::Math::XYZVector;

const double mass_p = 938.272;           // Mass of proton in MeV/c^2
const double charge_p = 1.602176634e-19; // Charge of proton

// Vectors to store the simulated points to compare to
std::vector<double> x_sim, y_sim, z_sim, Eloss_sim;
TH1F *hMom = nullptr;
TH1F *hMomSampled = nullptr;
TH1F *hMomError = nullptr;
TH1F *hMomError2 = nullptr;

TCanvas *c1 = new TCanvas();
TCanvas *c2 = new TCanvas();

// Parameters for model
const int pointsToCluster = 5;
const double sigma_pos = 1;                // Position uncertainty of 10 mm
const double sigma_mom = 0.1;              // Momentum uncertainty in percentage
const double sigma_mom_sample = 0.05;      // Sampled momentum uncertainty in percentage
const double sigma_theta = 1 * M_PI / 180; // Angular uncertainty of 1 degree
const double sigma_phi = 1 * M_PI / 180;   // Angular uncertainty of 1 degree
const double gasDensity = 3.553e-5;        // g/cm^3
const double fAlpha = 1e-1;
const double fBeta = 2;
const double fKappa = 0;

// Global variables
kf::TrackFitterUKF *ukf = nullptr;

// Functions in file
/// Loads hits from an input file
void LoadHits();
/// Run a single UKF for the passed initial state
void SingleUKF(XYZPoint initialPos, XYZVector initialMom, TMatrixD initialCov);
/// @brief Create the ukf pointer to reuse.
void CreateUKF();

TMatrixD CalculateInitialCov(double p);
TMatrixD CalculatePosCov();

/// @brief  Test many tracks, saving stat properties
/// @param n
/// @param bias
void TestManyTracks(int n, double bias = 0)
{
   LoadHits();
   CreateUKF();

   XYZPoint fTruePos(-3.40046e-04, -1.49863e-04, 1.0018);
   XYZVector fTrueMom(0.00935463, -0.0454279, 0.00826042);
   fTrueMom *= 1e3;
   double prevKE = AtTools::Kinematics::KE(fTrueMom.R(), mass_p) - Eloss_sim[0];
   double fMomPrev = AtTools::Kinematics::GetRelMomFromKE(prevKE, mass_p);
   std::cout << "Initial mom: " << fTrueMom.R() << " past mom " << fMomPrev << std::endl;

   double fSigmaMom = fTrueMom.R() * sigma_mom_sample;

   hMom = new TH1F("hMom", "Reconstructed Momentum (MeV/c)", 100, fTrueMom.R() - 4 * fSigmaMom,
                   fTrueMom.R() + 4 * fSigmaMom);
   hMomSampled = new TH1F("hMom", "Reconstructed Momentum (MeV/c)", 100, fTrueMom.R() - 4 * fSigmaMom,
                          fTrueMom.R() + 4 * fSigmaMom);
   hMomError = new TH1F("hMomError", "Error (%)", 100, -2, 2);
   hMomError2 =
      new TH1F("hMomError2", "Error (%)", 100, -4 * fSigmaMom / fTrueMom.R() * 100, 4 * fSigmaMom / fTrueMom.R() * 100);

   for (int i = 0; i < n; ++i) {

      if (i % 100 == 0)
         std::cout << "On iteration " << i << std::endl;

      double pSampled = gRandom->Gaus(fTrueMom.R(), sigma_mom_sample * fTrueMom.R());
      pSampled += bias;
      hMomSampled->Fill(pSampled);

      // pSampled = fTrueMom.R();

      ROOT::Math::Polar3DVector sampledMom(pSampled, fTrueMom.Theta(), fTrueMom.Phi());
      SingleUKF(fTruePos, XYZVector(sampledMom), CalculateInitialCov(pSampled));

      auto filtState = ukf->GetFilteredStates()[0];
      auto smoothState = ukf->GetSmoothedStates()[0];
      auto smoothStatePrev = ukf->GetSmoothedStates()[1];

      double pReco = (smoothState[3] + smoothStatePrev[3]) / 2; // Average of current and previous state
      hMom->Fill(pReco);
      double error = (pReco - fTrueMom.R()) / fTrueMom.R() * 100;
      hMomError->Fill(error);
      double errorPrev = (ukf->GetSmoothedStates()[1][3] - fMomPrev) / fMomPrev * 100;
      hMomError2->Fill(errorPrev);

      std::cout << std::endl
                << std::endl
                << "With initial momentum " << pSampled << " reconstructed " << pReco << " Error: " << error << "%"
                << std::endl
                << std::endl;
   }

   // Draw results
   c1->cd();
   hMomSampled->Scale(10);
   hMomSampled->SetStats(0);
   hMom->SetStats(0);
   hMom->Draw("hist");
   hMomSampled->SetLineColor(kRed);
   hMomSampled->Draw("same hist");

   auto legend = new TLegend(0.65, 0.75, 0.88, 0.88);
   legend->AddEntry(hMom, "Reconstructed", "l");
   legend->AddEntry(hMomSampled, "Sampled (scale x10)", "l");
   legend->Draw();

   c2->cd();
   hMomError->Draw("hist");
   // hMomError2->SetLineColor(kRed);
   // hMomError2->Draw("same hist");
}

/*********** Function implementations **************/
void LoadHits()
{
   if (x_sim.size() != 0)
      return;
   Eloss_sim.clear();
   std::ifstream infile("hits.txt");
   double xi, yi, zi, Ei;
   int i = 0;
   double eLoss = 0;

   // Save first point.
   infile >> xi >> yi >> zi >> Ei;
   eLoss = Ei;               // Initialize energy loss
   x_sim.push_back(xi * 10); // Convert to mm
   y_sim.push_back(yi * 10); // Convert to mm
   z_sim.push_back(zi * 10); // Convert to mm

   while (infile >> xi >> yi >> zi >> Ei) {
      // Ei *= 1e3; // Convert to MeV

      if (++i % pointsToCluster != 0) {
         eLoss += Ei;
         continue; // Skip every 5th point
      }
      x_sim.push_back(xi * 10);
      y_sim.push_back(yi * 10);
      z_sim.push_back(zi * 10);
      Eloss_sim.push_back(eLoss);
      eLoss = 0; // Reset energy loss for the next segment
   }
}

void CreateUKF()
{
   auto elossModel2 = std::make_unique<AtTools::AtELossCATIMA>(gasDensity);
   elossModel2->SetProjectile(1, 1, 1);
   std::vector<std::tuple<int, int, int>> mat;
   mat.push_back({1, 1, 1});
   elossModel2->SetMaterial(mat);

   auto elossModel = std::make_unique<AtTools::AtELossTable>(0);
   elossModel->LoadSrimTable(getEnergyPath()); // Use the function to get the path
   elossModel->SetDensity(gasDensity);

   AtTools::AtPropagator propagator(charge_p, mass_p, std::move(elossModel));
   propagator.SetEField({0, 0, 0});    // No electric field
   propagator.SetBField({0, 0, 2.85}); // Magnetic field

   // Setup stepper for UKF
   auto stepper = std::make_unique<AtTools::AtRK4Stepper>();

   // Setup UKF
   ukf = new kf::TrackFitterUKF(std::move(propagator), std::move(stepper));
   ukf->setParameters(fAlpha, fBeta, fKappa);
}

TMatrixD CalculateInitialCov(double p)
{
   TMatrixD cov(6, 6);
   cov.Zero();
   for (int i = 0; i < 3; ++i) {

      cov(i, i) = sigma_pos * sigma_pos; // Set diagonal covariance to some small number
   }
   cov(3, 3) = p * p * sigma_mom * sigma_mom; // Momentum uncertainty
   cov(4, 4) = sigma_theta * sigma_theta;     // Angular uncertainty
   cov(5, 5) = sigma_phi * sigma_phi;         // Angular uncertainty

   return cov;
}
TMatrixD CalculatePosCov()
{
   TMatrixD cov(3, 3);
   cov.Zero();
   for (int i = 0; i < 3; ++i) {

      cov(i, i) = sigma_pos * sigma_pos; // Set diagonal covariance to some small number
   }
   return cov;
}

void SingleUKF(XYZPoint intitialPos, XYZVector initialMom, TMatrixD intitialCov)
{
   ukf->SetInitialState(intitialPos, initialMom, intitialCov);

   for (int i = 1; i < x_sim.size(); ++i) {
      ukf->SetMeasCov(CalculatePosCov());
      XYZPoint point(x_sim[i], y_sim[i], z_sim[i]);

      ukf->predictUKF(point);
      ukf->correctUKF(point);
   }

   ukf->smoothUKF();
}