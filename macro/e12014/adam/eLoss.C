AtTools::AtELossTable *model = nullptr;

void eLoss()
{

   model = new AtTools::AtELossTable();

   std::ifstream in("PbinHe.txt");

   double energy;
   double elec;
   double nucl;
   double range;
   double longi;
   double lat;

   std::string energy_unit;
   std::string range_unit;
   std::string long_strag_unit;
   std::string lat_strag_unit;

   std::vector<double> fdEdX, fEnergy;

   while (in >> energy >> energy_unit >> elec >> nucl >> range >> range_unit >> longi >> long_strag_unit >> lat >>
          lat_strag_unit) {
      if (energy_unit == "eV") {
         fEnergy.push_back(energy * (1.e-6));

      } else if (energy_unit == "keV") {
         fEnergy.push_back(energy * (1.e-3));

      } else if (energy_unit == "MeV") {
         fEnergy.push_back(energy);

      } else if (energy_unit == "GeV") {
         fEnergy.push_back(energy * (1.e3));
      }

      fdEdX.push_back(elec + nucl);
   }

   model = new AtTools::AtELossTable(fEnergy, fdEdX);
}
