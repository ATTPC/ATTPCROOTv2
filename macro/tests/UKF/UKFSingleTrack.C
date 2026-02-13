std::string getEnergyPath()
{
   auto env = std::getenv("VMCWORKDIR");
   if (env == nullptr) {
      return "../../resources/energy_loss/HinH.txt"; // Default path assuming cwd is build/AtTools
   }
   return std::string(env) + "/resources/energy_loss/HinH.txt"; // Use environment variable
}

const double mass_p = 938.272;           // Mass of proton in MeV/c^2
const double charge_p = 1.602176634e-19; // Charge of proton

// Simulated (measurement) hits
std::vector<double> x, y, z, Eloss;
int pointsToCluster = 5;
void LoadHits()
{
   std::ifstream infile("hits.txt");
   double xi, yi, zi, Ei;
   int i = 0;
   double eLoss = 0;

   // Save first point.
   infile >> xi >> yi >> zi >> Ei;
   eLoss = Ei;           // Initialize energy loss
   x.push_back(xi * 10); // Convert to mm
   y.push_back(yi * 10); // Convert to mm
   z.push_back(zi * 10); // Convert to mm

   while (infile >> xi >> yi >> zi >> Ei) {
      // Ei *= 1e3; // Convert to MeV

      if (++i % pointsToCluster != 0) {
         eLoss += Ei;
         continue; // Skip every 5th point
      }

      double dx = x.back() - xi;
      double dy = y.back() - yi;
      double dz = z.back() - zi;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      x.push_back(xi * 10);
      y.push_back(yi * 10);
      z.push_back(zi * 10);
      Eloss.push_back(eLoss);
      eLoss = 0; // Reset energy loss for the next segment
   }

   std::cout << "Finished loading hits. Total points: " << x.size() << std::endl;
   std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
}

// This test should plot the trajectory of a particle in a magnetic field using
// the output from GEANT and the AtPropagator class.
void UKFSingleTrack()
{
   LoadHits(); // Load hits from file

   std::cout << " Creating the UKF class" << std::endl;
   using namespace AtTools;

   std::vector<double> x2, y2, z2, Eloss2, p2, sigmap2, lambda2, sigmalambda2, residual;
   std::vector<double> xSmooth, ySmooth, zSmooth, pSmooth, sigmapSmooth, residualSmooth, eLossSmooth;

   // Setup the Propagator for UKF
   auto elossModel = std::make_unique<AtTools::AtELossTable>(0);
   elossModel->LoadSrimTable(getEnergyPath()); // Use the function to get the path
   elossModel->SetDensity(3.553e-5);           // Set density in g/cm^3 for 300 torr H2

   auto elossModel2 = std::make_unique<AtTools::AtELossCATIMA>(3.553e-5);
   elossModel2->SetProjectile(1, 1, 1);
   std::vector<std::tuple<int, int, int>> mat;
   mat.push_back({1, 1, 1});
   elossModel2->SetMaterial(mat);

   AtTools::AtPropagator propagator(charge_p, mass_p, std::move(elossModel2));
   propagator.SetEField({0, 0, 0});    // No electric field
   propagator.SetBField({0, 0, 2.85}); // Magnetic field

   // Setup stepper for UKF
   auto stepper = std::make_unique<AtTools::AtRK4Stepper>();

   // Setup UKF
   kf::TrackFitterUKF ukf(std::move(propagator), std::move(stepper));

   XYZPoint startPos(-3.40046e-05, -1.49863e-05, 0.10018); // Start position in cm
   startPos *= 10;                                         // Convert to mm
   XYZVector startMom(0.00935463, -0.0454279, 0.00826042); // Start momentum in GeV/c
   startMom *= 1e3;
   double beginMom = startMom.R(); // Initial momentum in MeV/c
   std::cout << " Initial momentum: " << beginMom << " MeV/c" << std::endl;

   XYZPoint nextPos(x[1], y[1], z[1]);
   startMom = startMom.R() * (nextPos - startPos).Unit(); // Set momentum direction towards the first hit

   // Initial uncertainties
   double sigma_pos = 1;                  // Position uncertainty of 10 mm
   double sigma_mom = 0.1 * startMom.R(); // Momentum uncertainty of 10% MeV/c
   double sigma_theta = 1 * M_PI / 180;   // Angular uncertainty of 1 degree
   double sigma_phi = 1 * M_PI / 180;     // Angular uncertainty of 1 degree
   ukf.fEnableEnStraggling = true;        // Enable energy straggling
   ukf.setParameters(1e-3, 2, 0);         // alpha, beta, kappa

   TMatrixD cov(6, 6);
   cov.Zero();
   for (int i = 0; i < 3; ++i) {

      cov(i, i) = sigma_pos * sigma_pos; // Set diagonal covariance to some small number
   }
   cov(3, 3) = sigma_mom * sigma_mom;     // Momentum uncertainty
   cov(4, 4) = sigma_theta * sigma_theta; // Angular uncertainty
   cov(5, 5) = sigma_phi * sigma_phi;     // Angular uncertainty

   // Set the initial state
   std::cout << "Setting initial state" << std::endl;

   ukf.SetInitialState(startPos, startMom, cov);
   TMatrixD cov_meas(3, 3);
   cov_meas.Zero();
   for (int i = 0; i < 3; ++i) {
      cov_meas(i, i) = sigma_pos * sigma_pos;
   }
   // Create the covariance for measurement points. Assume constant

   x2.push_back(startPos.X());
   y2.push_back(startPos.Y());
   z2.push_back(startPos.Z());
   p2.push_back(startMom.R());
   sigmap2.push_back(sigma_mom);
   residual.push_back(0); // Initial residual is zero

   ROOT::Math::XYZVector lastMom = ROOT::Math::XYZVector(startMom.X(), startMom.Y(), startMom.Z());

   // Skip the first point since it is the initial state.
   // Stop when things break (point 21).
   for (size_t i = 1; i < x.size() && i < 100; ++i) {
      std::cout << "Processing hit " << i << " of " << x.size() << std::endl;
      XYZPoint point(x[i], y[i], z[i]); // measurement point in mm
      ukf.SetMeasCov(cov_meas);         // Set measurement noise covariance

      ukf.predictUKF(point);
      auto augState = ukf.GetAugStateVector();
      auto augCov = ukf.GetAugStateCovariance();
      auto covP = ukf.matP();

      if (i == 1) {
         LOG(info) << "P-: " << std::endl << covP;
         Eigen::SelfAdjointEigenSolver<decltype(covP)> es(covP); // float version
         float λmin = es.eigenvalues()(0);
         auto vmin = es.eigenvectors().col(0); // length-6

         std::printf("λmin = %.3e  eigvec = [", λmin);
         for (int i = 0; i < 6; ++i)
            std::printf(" %.2e", vmin(i));
         std::printf(" ]\n");
      }
      ukf.correctUKF(point);

      auto state = ukf.vecX();
      auto cov = ukf.matP();

      ROOT::Math::XYZPoint pos(state[0], state[1], state[2]);
      ROOT::Math::Polar3DVector momPolar(state[3], state[4], state[5]);
      ROOT::Math::XYZVector mom(momPolar);

      std::cout << "Predicted position: " << pos << std::endl;
      std::cout << "Predicted momentum: " << mom << std::endl;
      std::cout << "Measurement point: " << point << std::endl;

      auto KE_in = Kinematics::KE(lastMom, mass_p);
      auto KE_out = Kinematics::KE(mom, mass_p);
      lastMom = mom;

      double residualValue = (point - pos).R();

      x2.push_back(pos.X());
      y2.push_back(pos.Y());
      z2.push_back(pos.Z());
      Eloss2.push_back((KE_in - KE_out));
      p2.push_back(mom.R());
      sigmap2.push_back(std::sqrt(cov(3, 3)));         // Propagate momentum uncertainty
      lambda2.push_back(augState[6]);                  // Energy straggling factor
      sigmalambda2.push_back(std::sqrt(augCov(6, 6))); // Propagate energy straggling uncertainty
      residual.push_back(residualValue);               // Store the residual for this hit
   }

   LOG(info) << "After forward pass " << ukf.nTouch << " touches.";

   // At this point we have the full trajectory of the particle
   ukf.smoothUKF(); // Perform smoothing
   auto smoothedStates = ukf.GetSmoothedStates();
   auto smoothedCovariances = ukf.GetSmoothedCovariances();
   auto filteredStates = ukf.GetFilteredStates();
   auto filteredCovariances = ukf.GetFilteredCovariances();
   for (int i = 0; i < smoothedStates.size(); ++i) {
      auto &state = smoothedStates[i];
      auto &filteredState = filteredStates[i];
      xSmooth.push_back(state[0]);
      ySmooth.push_back(state[1]);
      zSmooth.push_back(state[2]);
      pSmooth.push_back(state[3]);
      sigmapSmooth.push_back(std::sqrt(smoothedCovariances[i](3, 3))); // Momentum uncertainty
      residualSmooth.push_back(
         (XYZPoint(state[0], state[1], state[2]) - XYZPoint(filteredState[0], filteredState[1], filteredState[2])).R());
      if (i > 0) {
         auto lastMom = smoothedStates[i - 1][3];
         auto mom = smoothedStates[i][3];
         auto KE_in = Kinematics::KE(lastMom, mass_p);
         auto KE_out = Kinematics::KE(mom, mass_p);
         if (i > 1)
            eLossSmooth.push_back(KE_in - KE_out); // Energy loss between smoothed states
      } else {
         eLossSmooth.push_back(0); // First point has no previous state to compare
      }
   }
   LOG(info) << "Initial smoothed momentum: " << smoothedStates[0][3];
   LOG(info) << "Starting momentum: " << beginMom;
   LOG(info) << "Error in momentum reconstruction: " << (smoothedStates[0][3] - beginMom) / beginMom * 100 << "%";

   TGraph2D *track = new TGraph2D(x.size(), x.data(), y.data(), z.data());
   track->SetTitle("Particle Track;X [mm];Y [mm];Z [mm]");
   track->SetMarkerStyle(20);
   track->SetMarkerSize(0.8);

   TGraph2D *track2 = new TGraph2D(x2.size(), x2.data(), y2.data(), z2.data());
   track2->SetTitle("Propagated Particle Track;X [mm];Y [mm];Z [mm]");
   track2->SetMarkerStyle(21);
   track2->SetMarkerSize(0.8);
   track2->SetMarkerColor(kRed);

   TGraph2D *smoothedTrack = new TGraph2D(xSmooth.size(), xSmooth.data(), ySmooth.data(), zSmooth.data());
   smoothedTrack->SetTitle("Smoothed Particle Track;X [mm];Y [mm];Z [mm]");
   smoothedTrack->SetMarkerStyle(22);
   smoothedTrack->SetMarkerSize(0.8);
   smoothedTrack->SetMarkerColor(kGreen + 2);

   TCanvas *c1 = new TCanvas("c1", "Particle Track", 800, 600);

   // Set axis ranges based on track and track2
   double xmin = std::min(*std::min_element(x.begin(), x.end()), *std::min_element(x2.begin(), x2.end()));
   double xmax = std::max(*std::max_element(x.begin(), x.end()), *std::max_element(x2.begin(), x2.end()));
   double ymin = std::min(*std::min_element(y.begin(), y.end()), *std::min_element(y2.begin(), y2.end()));
   double ymax = std::max(*std::max_element(y.begin(), y.end()), *std::max_element(y2.begin(), y2.end()));
   double zmin = std::min(*std::min_element(z.begin(), z.end()), *std::min_element(z2.begin(), z2.end()));
   double zmax = std::max(*std::max_element(z.begin(), z.end()), *std::max_element(z2.begin(), z2.end()));

   track->GetXaxis()->SetLimits(xmin, xmax);
   track->GetYaxis()->SetLimits(ymin, ymax);
   track->GetZaxis()->SetLimits(zmin, zmax);

   track->Draw("P");
   // track2->Draw("PSAME");
   smoothedTrack->Draw("PSAME");

   TGraph *elossGraph = new TGraph(Eloss.size());
   for (size_t i = 0; i < Eloss.size(); ++i) {
      elossGraph->SetPoint(i, i, Eloss[i]);
   }
   elossGraph->SetTitle("Energy Loss per Hit;Hit Number;Energy Loss [MeV]");
   elossGraph->SetMarkerStyle(20);

   TGraph *eloss2Graph = new TGraph(Eloss2.size());
   for (size_t i = 0; i < Eloss2.size(); ++i) {
      eloss2Graph->SetPoint(i, i, Eloss2[i]);
   }
   eloss2Graph->SetTitle("Propagated Energy Loss per Hit;Hit Number;Energy Loss [MeV]");
   eloss2Graph->SetMarkerStyle(21);
   eloss2Graph->SetMarkerColor(kRed);
   TGraph *elossSmoothGraph = new TGraph(eLossSmooth.size());
   for (size_t i = 0; i < eLossSmooth.size(); ++i) {
      elossSmoothGraph->SetPoint(i, i, eLossSmooth[i]);
   }
   elossSmoothGraph->SetTitle("Smoothed Energy Loss per Hit;Hit Number;Energy Loss [MeV]");
   elossSmoothGraph->SetMarkerStyle(22);
   elossSmoothGraph->SetMarkerColor(kGreen + 2);

   TGraphErrors *pGraph = new TGraphErrors(p2.size());
   for (size_t i = 0; i < p2.size(); ++i) {
      pGraph->SetPoint(i, i, p2[i]);
      pGraph->SetPointError(i, 0,
                            sigmap2[i] * 1); // Error bars from sigmap2, converted to GeV/c
   }
   pGraph->SetTitle("Momentum per Hit;Hit Number;Momentum [MeV/c]");
   pGraph->SetMarkerStyle(20);
   pGraph->SetMarkerColor(kRed);

   TGraphErrors *lambdaGraph = new TGraphErrors(lambda2.size());
   for (size_t i = 0; i < lambda2.size(); ++i) {
      lambdaGraph->SetPoint(i, i, lambda2[i] * 0.03 * pointsToCluster / 5.);
      lambdaGraph->SetPointError(i, 0, sigmalambda2[i] * 0.03 * pointsToCluster / 5.);
      // std::cout << "Lambda: " << lambda2[i] << ", Error: " << sigmalambda2[i] << std::endl;
   }
   lambdaGraph->SetTitle("Lambda per Hit (scaled);Hit Number;Lambda [scaled]");
   lambdaGraph->SetMarkerStyle(22);
   lambdaGraph->SetMarkerColor(kGreen + 2);

   TGraph *residualGraph = new TGraph(residual.size());
   for (size_t i = 0; i < residual.size(); ++i) {
      residualGraph->SetPoint(i, i, residual[i] * .1);
   }
   residualGraph->SetTitle("Residual per Hit;Hit Number;Residual [cm]");
   residualGraph->SetMarkerStyle(23);
   residualGraph->SetMarkerColor(kMagenta);

   TGraphErrors *pSmoothGraph = new TGraphErrors(pSmooth.size());
   for (size_t i = 0; i < pSmooth.size(); ++i) {
      pSmoothGraph->SetPoint(i, i, pSmooth[i]);
      pSmoothGraph->SetPointError(i, 0,
                                  sigmapSmooth[i] * 1); // Error bars from sigmapSmooth, converted to GeV/c
   }
   pSmoothGraph->SetTitle("Smoothed Momentum per Hit;Hit Number;Momentum [MeV/c]");
   pSmoothGraph->SetMarkerStyle(22);
   pSmoothGraph->SetMarkerColor(kGreen + 2);

   TGraph *residualSmoothGraph = new TGraph(residualSmooth.size());
   for (size_t i = 0; i < residualSmooth.size(); ++i) {
      residualSmoothGraph->SetPoint(i, i, residualSmooth[i] * .1);
   }
   residualSmoothGraph->SetTitle("Smoothed Residual per Hit;Hit Number;Residual [cm]");
   residualSmoothGraph->SetMarkerStyle(24);
   residualSmoothGraph->SetMarkerColor(kOrange + 7);

   TCanvas *c2 = new TCanvas("c2", "Energy Loss per Hit", 800, 600);
   elossGraph->Draw("AP");
   eloss2Graph->Draw("PSAME");
   elossSmoothGraph->Draw("PSAME");
   // pGraph->Draw("PSAME");
   //  lambdaGraph->Draw("PSAME");
   // residualGraph->Draw("PSAME");
   // pSmoothGraph->Draw("PSAME");
   // residualSmoothGraph->Draw("PSAME");

   TCanvas *c3 = new TCanvas("c3", "Momentum at Hit (error bars 5X)", 800, 600);
   pGraph->Draw("AP");
   pSmoothGraph->Draw("PSAME");

   TCanvas *c4 = new TCanvas("c4", "Residual at Hit", 800, 600);
   residualGraph->Draw("AP");
   residualSmoothGraph->Draw("PSAME");

   double sumEloss = std::accumulate(Eloss.begin(), Eloss.end(), 0.0);
   double sumEloss2 = std::accumulate(Eloss2.begin(), Eloss2.end(), 0.0);
   std::cout << "Sum of Eloss: " << sumEloss << std::endl;
   std::cout << "Sum of Eloss2: " << sumEloss2 << std::endl;
   std::cout << "Initial energy: " << Kinematics::KE(beginMom, mass_p) << " MeV" << std::endl;
}