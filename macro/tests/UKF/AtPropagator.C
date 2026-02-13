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

// This test should plot the trajectory of a particle in a magnetic field using
// the output from GEANT and the AtPropagator class.
void AtPropagator()
{
   using namespace AtTools;

   std::vector<double> x, y, z;
   std::vector<double> x2, y2, z2;

   std::ifstream infile("hits.txt");
   double xi, yi, zi, Ei;
   while (infile >> xi >> yi >> zi >> Ei) {
      x.push_back(xi * 10);
      y.push_back(yi * 10);
      z.push_back(zi * 10);
   }

   // Our propagator setup
   double charge = charge_p; // Charge in Coulombs
   double mass = mass_p;     // Mass in MeV/c^2
   auto elossModel = std::make_unique<AtTools::AtELossTable>(0);
   elossModel->LoadSrimTable(getEnergyPath()); // Use the function to get the path
   elossModel->SetDensity(3.553e-5);           // Set density in g/cm^3 for 300 torr H2

   auto elossModel2 = std::make_unique<AtTools::AtELossCATIMA>(3.553e-5);
   elossModel2->SetProjectile(1, 1, 1);
   std::vector<std::tuple<int, int, int>> mat;
   mat.push_back({1, 1, 1});
   elossModel2->SetMaterial(mat);

   AtTools::AtPropagator propagator(charge, mass, std::move(elossModel2));
   propagator.SetEField({0, 0, 0});    // No electric field
   propagator.SetBField({0, 0, 2.85}); // Magnetic field
   AtTools::AtRK4Stepper stepper;

   XYZPoint startPos(-3.40046e-05, -1.49863e-05, 0.10018); // Start position in cm
   startPos *= 10;                                         // Convert to mm
   XYZVector startMom(0.00935463, -0.0454279, 0.00826042); // Start momentum in GeV/c
   startMom *= 1e3;                                        // Convert to MeV/c

   propagator.SetState(startPos, startMom);

   // Loop through until the particle is stopped
   while (Kinematics::KE(propagator.GetState().fMom, propagator.GetState().fMass) > 0.1) {
      // Propagate to the next point
      propagator.PropagateOneStep(stepper);

      // Get the current position and momentum
      auto pos = propagator.GetPosition();

      // Store the position for plotting
      x2.push_back(pos.X());
      y2.push_back(pos.Y());
      z2.push_back(pos.Z());
   }

   TGraph2D *track = new TGraph2D(x.size(), x.data(), y.data(), z.data());
   track->SetTitle("Particle Track;X [mm];Y [mm];Z [mm]");
   track->SetMarkerStyle(20);
   track->SetMarkerSize(0.8);

   TGraph2D *track2 = new TGraph2D(x2.size(), x2.data(), y2.data(), z2.data());
   track2->SetTitle("Propagated Particle Track;X [mm];Y [mm];Z [mm]");
   track2->SetMarkerStyle(21);
   track2->SetMarkerSize(0.8);
   track2->SetMarkerColor(kRed);

   TCanvas *c1 = new TCanvas("c1", "Particle Track", 800, 600);
   track->Draw("P");
   track2->Draw("PSAME");
}