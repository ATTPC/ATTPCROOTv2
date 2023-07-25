double EField(double rho, double z)
{
   double lambda = 1.54e-8;               // SI
   constexpr double rBeam = 2.0 / 100;    // in *m*
   constexpr double eps = 8.85418782E-12; // SI
   constexpr double pi = 3.14159265358979;
   constexpr double eps2pi = 2 * pi * eps;
   rho /= 100.; // Convert units from cm to m
   z /= 100.;   // Convert units from cm to m

   double field;
   if (rho > rBeam)
      field = lambda / eps2pi / rho * (z / 1.); // v/m
   else
      // field = lambda / eps2pi / rBeam / rBeam * rho * (z/1.); // v/m
      field = 0;
   return field / 100.; // V/cm
}
AtRadialChargeModel *scModel = nullptr;
AtLineChargeModel *scModel2 = nullptr;
void testSC()
{

   scModel = new AtRadialChargeModel(&EField);
   scModel2 = new AtLineChargeModel();
}

// Input units are mm
double dist(double rho, double z)
{
   rho /= 1000;
   z /= 1000;
   double dZ = z;
   constexpr double lambda = 1.54e-8;     // SI
   constexpr double eps = 8.85418782E-12; // SI
   constexpr double pi = 3.14159265358979;
   constexpr double eps2pi = 2 * pi * eps;
   constexpr double Ez = 70000;
   constexpr double prefactor = lambda / pi / eps / Ez;

   double dRho2 = prefactor * dZ * (dZ / 2);
   cout << "rhoi " << rho * rho << " corr " << dRho2 << endl;
   return sqrt(rho * rho - dRho2);
}
