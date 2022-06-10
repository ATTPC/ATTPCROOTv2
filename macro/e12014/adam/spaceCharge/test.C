
double noField(double r, double z)
{
   return 0;
}

double constField(double r, double z)
{
   return 20;
}

double lineField(double r, double z)
{
   double lambda = 5.28e-8;               // SI
   constexpr double eps = 8.85418782E-12; // SI
   constexpr double pi = 3.14159265358979;
   constexpr double eps2pi = 2 * pi * eps;
   r /= 100.;                        // Convert units from cm to m
   auto field = lambda / eps2pi / r; // v/m
   return field / 100.;              // V/cm
}

AtRadialChargeModel model(&lineField);

AtLineChargeModel oldModel;

void test()
{

   for (int i = -2; i <= 5; ++i) {
      auto dT = pow(10, -i);
      model.SetStepSize(dT);
      ROOT::Math::XYZPoint p(100, 0, 100);
      auto pRad = model.CorrectSpaceCharge(p);
      auto pAna = oldModel.CorrectSpaceCharge(p);
      auto diff = pRad.X() - pAna.X();
      std::cout << "Time step (us) " << dT << " Difference in corrected point " << diff << " % diff "
                << diff / pAna.X() * 100 << std::endl;
   }

   ROOT::Math::XYZPoint p(100, 0, 100);
   auto pCorr = model.CorrectSpaceCharge(p);
   auto pApply = model.ApplySpaceCharge(p);

   std::cout << p << " Corrected to " << pCorr << std::endl;
   std::cout << p << " Applied to " << pApply << std::endl;
   std::cout << p << " Corrected to " << oldModel.CorrectSpaceCharge(p) << " with old" << std::endl;
   std::cout << p << " Applied to " << oldModel.ApplySpaceCharge(p) << " with old" << std::endl;
}
