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
