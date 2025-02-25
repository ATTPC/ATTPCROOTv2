/*Bool_t compareEventName(std::string &getname, std::string &fribname)
{
   // Parsing FRIB event number
   std::regex fribregex("evt(\\d+)_\\d+");
   std::string result = std::regex_replace(fribname, fribregex, "$1\n");

   int fribnumber;
   std::istringstream iss(result);
   while (iss >> fribnumber) {
      // std::cout << fribnumber << std::endl;
   }

   // Parsing GET event name
   std::regex getregex("evt(\\d+)_data");
   result = std::regex_replace(getname, getregex, "$1\n");

   int getnumber;
   std::istringstream isss(result);
   while (isss >> getnumber) {
      // std::cout << getnumber << std::endl;
   }

   return (fribnumber == getnumber) ? 1 : 0;

   return 0;
}
*/
Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
{

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E);

void C14_ana_genfit()
{

   TH2F *energy_angle_cross_section = new TH2F("energy_angle_cross_section", "Energy vs Angle vs Cross Section", 500, 0, 180, 1000, 0, 10);

std::string filename = "/home/david/Desktop/Azure2/AZURE2/all_angles.txt"; // Update with your file path

std::ifstream infile(filename);
if (!infile.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
}

std::string line1;
// Skip the first line (header)
std::getline(infile, line1);

// Read the rest of the file
while (std::getline(infile, line1)) {
    std::istringstream iss(line1);
    double energy, angle, crossSection;
    std::string dummy; // To skip the second column
    if (!(iss >> energy >> dummy >> angle >> crossSection)) {
        std::cerr << "Error: Invalid line format: " << line1 << std::endl;
        continue;
    }
    // Fill the histogram
    energy_angle_cross_section->Fill(angle, energy, crossSection);
}

infile.close();
   std::ofstream outFiletxt("energy_angle_cm.txt");
   outFiletxt << "Energy CM (MeV)\tAngle CM (deg)\n";
   double_t convert = 0.0065688;
   std::ifstream file("/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/be10_range.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::string line;
    std::vector<std::string> ener;
    std::vector<std::string> range;
    std::vector<std::string> stopping_power;
    vector<double> Ebeam_tb_all;
    vector<double> Ebeam_a_all;
    vector<double> Ebeam_cm_values;
    vector<double> angles_cm_values;
    // Skip the header row
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        // Read each cell in the row
        while (std::getline(ss, cell, '\t')) {
            row.push_back(cell);
        }

        // Ensure the row has at least 4 columns
        if (row.size() >= 4) {
            ener.push_back(row[0]);
            double stopping_pwr = (std::stod(row[1]) + std::stod(row[2]))*convert;
            stopping_power.push_back(std::to_string(stopping_pwr));
            range.push_back(row[3]);
        }
    }
    

    file.close();  

    int n = range.size();
    std::vector<double> x(n), y(n), y2(n);
    for (int i = 0; i < n; ++i) {
        x[i] = std::stod(range[i]);
        y[i] = std::stod(ener[i]);
        y2[i] = std::stod(stopping_power[i]);
    }
   
    // Create TGraph
    TGraph *interpsrim = new TGraph(n, x.data(), y.data()); 
    TGraph *stopping_power_vs_energy = new TGraph(n, y.data(), y2.data());
    TGraph *interpsrimenergy = new TGraph(n, y.data(), x.data());
    double_t ener_0 = 7.35; //MeV
    double_t energy_i = ener_0;
    std::vector<double> energy_vs_distance;
    energy_vs_distance.push_back(energy_i);

    while(energy_i > 0.01){
      double_t stp = stopping_power_vs_energy->Eval(energy_i);
      energy_i = energy_i - stp;
      energy_vs_distance.push_back(energy_i);
    }

   std::vector<double> distances(energy_vs_distance.size());
    for (size_t i = 0; i < distances.size(); ++i) {
        distances[i] = static_cast<double>(i);
    }

   TGraph *energy_vs_distance_gr = new TGraph(distances.size(), distances.data(), energy_vs_distance.data());


   FairRunAna *run = new FairRunAna();

   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_nocoin = new TH2F("Ang_Ener_nocoin", "Ang_Ener_nocoin", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_nocoin_cut = new TH2F("Ang_Ener_nocoin_cut", "Ang_Ener_nocoin_cut", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_cut = new TH2F("Ang_Ener_cut", "Ang_Ener_cut", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_03 = new TH2F("Ang_Ener_03", "Ang_Ener_03", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_36 = new TH2F("Ang_Ener_36", "Ang_Ener_36", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_69 = new TH2F("Ang_Ener_69", "Ang_Ener_69", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_912 = new TH2F("Ang_Ener_912", "Ang_Ener_912", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_1215 = new TH2F("Ang_Ener_1215", "Ang_Ener_1215", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_1518 = new TH2F("Ang_Ener_1518", "Ang_Ener_1518", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_1821 = new TH2F("Ang_Ener_1821", "Ang_Ener_1821", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_2124 = new TH2F("Ang_Ener_2124", "Ang_Ener_2124", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_2427 = new TH2F("Ang_Ener_2427", "Ang_Ener_2427", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_2730 = new TH2F("Ang_Ener_2730", "Ang_Ener_2730", 720, 0, 100, 500, 0, 10.0);
   TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 20.0);
   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   TH2F *ELoss_vs_bro_alpha = new TH2F("ELoss_vs_bro_alpha", "ELoss_vs_bro_alpha", 4000, 0, 25000, 1000, 0, 4);
   TH2F *ELoss_vs_bro_be = new TH2F("ELoss_vs_bro_be", "ELoss_vs_bro_be", 4000, 0, 25000, 1000, 0, 4);
   TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 4000000, 1000, 0, 4);
   TH2F *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 10, 1000, 0, 10);
   TH2F *hVxVy_cut = new TH2F("hVxVy_cut", "hVxVy_cut", 1000, 0, 10, 1000, 0, 10);
   TH2F *angle_vs_angle = new TH2F("angle_vs_angle", "angle_vs_angle", 720, 0, 90, 720, 0, 90);
   TH2F *ecm_anglecm = new TH2F("ecm_anglecm", "ecm_anglecm", 500, 0, 180, 1000, 0, 5);
   TH2F *ecm_anglecm_nocoin = new TH2F("ecm_anglecm_nocoin", "ecm_anglecm_nocoin", 500, 0, 180, 1000, 0, 5);
   TH2F *ecm_anglecm_nocoin_cut = new TH2F("ecm_anglecm_nocoin_cut", "ecm_anglecm_nocoin_cut", 500, 0, 180, 1000, 0, 5);
   TH2F *ecm_anglecm_cut = new TH2F("ecm_anglecm_cut", "ecm_anglecm_cut", 500, 0, 180, 1000, 0, 5);
   TH2F *ecm_anglecm_03 = new TH2F("ecm_anglecm_03", "ecm_anglecm_03", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_36 = new TH2F("ecm_anglecm_36", "ecm_anglecm_36", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_69 = new TH2F("ecm_anglecm_69", "ecm_anglecm_69", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_912 = new TH2F("ecm_anglecm_912", "ecm_anglecm_912", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_1215 = new TH2F("ecm_anglecm_1215", "ecm_anglecm_1215", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_1518 = new TH2F("ecm_anglecm_1518", "ecm_anglecm_1518", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_1821 = new TH2F("ecm_anglecm_1821", "ecm_anglecm_1821", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_2124 = new TH2F("ecm_anglecm_2124", "ecm_anglecm_2124", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_2427 = new TH2F("ecm_anglecm_2427", "ecm_anglecm_2427", 500, 0, 180, 1000, 0, 10);
   TH2F *ecm_anglecm_2730 = new TH2F("ecm_anglecm_2730", "ecm_anglecm_2730", 500, 0, 180, 1000, 0, 10);
   TH2F *ealpha_vs_vertex = new TH2F("ealpha_vs_vertex", "ealpha_vs_vertex", 1000, -10, 100, 1000, 0, 10);
   TH2F *energy_vs_length = new TH2F("energy_vs_length", "energy_vs_length", 1000, -100, 10, 200, 0, 20);
   TH2F *energyPRA_vs_length = new TH2F("energyPRA_vs_length", "energyPRA_vs_length", 1000, -100, 10, 200, 0, 20);
   TH2F *data_vs_sim_alphaener = new TH2F("data_vs_sim_alphaener", "data_vs_sim_alphaener", 100, 0, 10, 100, 0, 10);
   TH1F *henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);
   TH1F *vertex_distribution_gf = new TH1F("vertex_distribution_gf", "vertex_distribution_gf", 200, 0, 200);
   TH1F *vertex_distribution_pra = new TH1F("vertex_distribution_pra", "vertex_distribution_pra", 200, 0, 200);
   TH1F *vertex_distribution_xtr = new TH1F("vertex_distribution_xtr", "vertex_distribution_xtr", 200, 0, 200);
   TH1F *ex_ener_120 = new TH1F("ex_ener_120", "ex_ener_120", 100, 0, 15.0);
   TH1F *ex_ener_110 = new TH1F("ex_ener_110", "ex_ener_110", 100, 0, 15.0);
   TH1F *ex_ener_100 = new TH1F("ex_ener_100", "ex_ener_100", 100, 0, 15.0);
   TH1F *ex_ener_90 = new TH1F("ex_ener_90", "ex_ener_90", 100, 0, 15.0);
   TH1F *ex_ener_80 = new TH1F("ex_ener_80", "ex_ener_80", 100, 0, 15.0);
   TH1F *ex_ener_70 = new TH1F("ex_ener_70", "ex_ener_70", 100, 0, 15.0);
   TH1F *ex_ener_60 = new TH1F("ex_ener_60", "ex_ener_60", 100, 0, 15.0);
   TH1F *ang_distribution = new TH1F("ang_distribution", "ang_distribution", 360, 0, 180);
   auto *hex = new TH1F("hex", "hex", 600, -5, 55);
   auto *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);

   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_t = 3.016049281 * 931.49401;
   Double_t m_He3 = 3.016029 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_Li9 = 9.026790 * 931.49401;
   Double_t m_beam = m_Be10;
   Float_t aMass = 4.00260325415;
   Float_t O16Mass = 15.99491461956;
   Double_t m_C14 = 14.003242 * 931.49401;
   Double_t m_C13 = 13.00335484 * 931.49401;
   Double_t m_C12 = 12.00 * 931.49401;
   Double_t m_C16 = 16.0147 * 931.49401;
   Double_t m_C17 = 17.0226 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 10.5; // 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_d;
   m_B = m_C16;

   Double_t Am = 1.0;

   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString dataDir = dir + "/macro/Unpack_HDF5/a1975/";
   //TString dataDir = dir + "/macro/Simulation/ATTPC/10Be_aa/data/";
   

   //std::vector<TString> files{"output_digi.root"};
   //std::ifstream alphaEnergyFile("/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/macro/Simulation/ATTPC/10Be_aa/alphaenergy_sim.txt");

   std::vector<double> alphaEnergies_sim;
   double energy_sim;
  // while (alphaEnergyFile >> energy_sim) {
    //alphaEnergies_sim.push_back(energy_sim);
  // }

// Close the file
  // alphaEnergyFile.close();


   std::vector<TString> files{"run_0062.root","run_0063.root","run_0064.root","run_0065.root","run_0066.root","run_0067.root", "run_0070.root",
   "run_0071.root", "run_0072.root","run_0073.root","run_0074.root","run_0075.root","run_0076.root","run_0077.root","run_0078.root","run_0079.root",
   "run_0080.root","run_0081.root","run_0082.root","run_0083.root","run_0084.root","run_0085.root","run_0086.root","run_0087.root","run_0089.root",
   "run_0090.root","run_0091.root","run_0092.root","run_0093.root","run_0094.root","run_0095.root","run_0096.root","run_0097.root","run_0098.root",
   "run_0099.root","run_0100.root","run_0101.root","run_0102.root","run_0103.root","run_0104.root","run_0105.root"};

   TCutG *cut1 = new TCutG("CUT1",8);
   cut1->SetVarX("ecm_anglecm_nocoin_cut");
   cut1->SetVarY("");
   cut1->SetTitle("Graph");
   cut1->SetFillStyle(1000);
   cut1->SetPoint(0,87.6119,2.85786);
   cut1->SetPoint(1,92.9045,2.85786);
   cut1->SetPoint(2,99.617,2.65625);
   cut1->SetPoint(3,98.8425,2.54284);
   cut1->SetPoint(4,93.8081,2.54284);
   cut1->SetPoint(5,86.9664,2.66885);
   cut1->SetPoint(6,86.4501,2.80746);
   cut1->SetPoint(7,87.6119,2.85786);

   TCutG *cut2 = new TCutG("CUT2",11);
   cut2->SetVarX("ecm_anglecm_nocoin_cut");
   cut2->SetVarY("");
   cut2->SetTitle("Graph");
   cut2->SetFillStyle(1000);
   cut2->SetPoint(0,91.3554,1.9506);
   cut2->SetPoint(1,100.004,1.8372);
   cut2->SetPoint(2,109.299,1.62298);
   cut2->SetPoint(3,117.56,1.42137);
   cut2->SetPoint(4,117.431,1.18196);
   cut2->SetPoint(5,109.17,1.10635);
   cut2->SetPoint(6,92.1299,1.43397);
   cut2->SetPoint(7,89.0318,1.7994);
   cut2->SetPoint(8,94.7117,2.01361);
   cut2->SetPoint(9,93.4208,1.938);
   cut2->SetPoint(10,91.3554,1.9506);

   TCutG *cut3 = new TCutG("CUT3",9);
   cut3->SetVarX("ecm_anglecm_nocoin_cut");
   cut3->SetVarY("");
   cut3->SetTitle("Graph");
   cut3->SetFillStyle(1000);
   cut3->SetPoint(0,73.5413,0.94254);
   cut3->SetPoint(1,81.1575,0.866935);
   cut3->SetPoint(2,85.2883,0.740927);
   cut3->SetPoint(3,80.8993,0.539314);
   cut3->SetPoint(4,76.3812,0.677923);
   cut3->SetPoint(5,71.0886,0.728327);
   cut3->SetPoint(6,70.3141,0.854335);
   cut3->SetPoint(7,73.9286,0.929939);
   cut3->SetPoint(8,73.5413,0.94254);

   TCutG *cut4 = new TCutG("CUT4",8);
   cut4->SetVarX("ecm_anglecm_nocoin_cut");
   cut4->SetVarY("");
   cut4->SetTitle("Graph");
   cut4->SetFillStyle(1000);
   cut4->SetPoint(0,136.02,2.85786);
   cut4->SetPoint(1,123.886,2.90827);
   cut4->SetPoint(2,122.982,2.53024);
   cut4->SetPoint(3,125.176,2.36643);
   cut4->SetPoint(4,131.244,2.25302);
   cut4->SetPoint(5,138.214,2.21522);
   cut4->SetPoint(6,141.054,2.84526);
   cut4->SetPoint(7,136.02,2.85786);

   TCutG *cutbkg = new TCutG("CUTBKG",6);
   cutbkg->SetVarX("ecm_anglecm_nocoin_cut");
   cutbkg->SetVarY("");
   cutbkg->SetTitle("Graph");
   cutbkg->SetFillStyle(1000);
   cutbkg->SetPoint(0,162.354,7.50756);
   cutbkg->SetPoint(1,152.672,0.110887);
   cutbkg->SetPoint(2,176.553,0.199093);
   cutbkg->SetPoint(3,171.39,7.99899);
   cutbkg->SetPoint(4,164.161,8.0494);
   cutbkg->SetPoint(5,162.354,7.50756);


   TCutG *cutbkgcoin = new TCutG("CUTbkg_coin",6);
   cutbkgcoin->SetVarX("");
   cutbkgcoin->SetVarY("");
   cutbkgcoin->SetTitle("Graph");
   cutbkgcoin->SetFillStyle(1000);
   cutbkgcoin->SetPoint(0,159.772,2.78226);
   cutbkgcoin->SetPoint(1,145.701,0.211693);
   cutbkgcoin->SetPoint(2,167.001,0.388105);
   cutbkgcoin->SetPoint(3,165.452,2.79486);
   cutbkgcoin->SetPoint(4,161.708,2.71925);
   cutbkgcoin->SetPoint(5,159.772,2.78226);

   TCutG *cut_threshold = new TCutG("CUT_threshold",8);
   cut_threshold->SetVarX("");
   cut_threshold->SetVarY("");
   cut_threshold->SetTitle("Graph");
   cut_threshold->SetFillStyle(1000);
   cut_threshold->SetPoint(0,52.761,2.98387);
   cut_threshold->SetPoint(1,50.4662,2.69405);
   cut_threshold->SetPoint(2,51.757,1.96321);
   cut_threshold->SetPoint(3,57.7094,1.73639);
   cut_threshold->SetPoint(4,61.1518,2.10181);
   cut_threshold->SetPoint(5,53.0479,3.07208);
   cut_threshold->SetPoint(6,52.1156,2.89567);
   cut_threshold->SetPoint(7,52.761,2.98387);






   /*TCutG *cutt = new TCutG("CUTT", 25);
   cutt->SetVarX("ELossvsBrho");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 195.8704, 2.17006);
   cutt->SetPoint(1, 168.9599, 1.823906);
   cutt->SetPoint(2, 203.9435, 1.67006);
   cutt->SetPoint(3, 308.8943, 1.443564);
   cutt->SetPoint(4, 478.4301, 1.140145);
   cutt->SetPoint(5, 677.5675, 0.9307436);
   cutt->SetPoint(6, 1145.809, 0.7598034);
   cutt->SetPoint(7, 1581.759, 0.6572393);
   cutt->SetPoint(8, 1595.214, 0.7298889);
   cutt->SetPoint(9, 1544.084, 0.798265);
   cutt->SetPoint(10, 1342.256, 0.8538205);
   cutt->SetPoint(11, 978.9645, 0.9307436);
   cutt->SetPoint(12, 747.5346, 1.123051);
   cutt->SetPoint(13, 618.3645, 1.349547);
   cutt->SetPoint(14, 421.9182, 1.793991);
   cutt->SetPoint(15, 273.9107, 2.110231);
   cutt->SetPoint(16, 198.5614, 2.17006);
   cutt->SetPoint(17, 195.8704, 2.17006);*/
   /*cutt->SetPoint(0, 165.5517, 2.378476);
   cutt->SetPoint(1, 188.9105, 1.8812);
   cutt->SetPoint(2, 305.7046, 1.523348);
   cutt->SetPoint(3, 461.4301, 1.249149);
   cutt->SetPoint(4, 640.5143, 1.002835);
   cutt->SetPoint(5, 887.0796, 0.8448221);
   cutt->SetPoint(6, 1201.126, 0.7379311);
   cutt->SetPoint(7, 1551.508, 0.6217452);
   cutt->SetPoint(8, 1813.646, 0.5892131);
   cutt->SetPoint(9, 2096.547, 0.5706234);
   cutt->SetPoint(10, 2158.838, 0.5706234);
   cutt->SetPoint(11, 2184.792, 0.6589247);
   cutt->SetPoint(12, 2153.647, 0.7100465);
   cutt->SetPoint(13, 1517.768, 0.8355272);
   cutt->SetPoint(14, 1089.523, 0.9424183);
   cutt->SetPoint(15, 876.6979, 1.086489);
   cutt->SetPoint(16, 656.0869, 1.272386);
   cutt->SetPoint(17, 549.6745, 1.583764);
   cutt->SetPoint(18, 422.4987, 2.006681);
   cutt->SetPoint(19, 329.0634, 2.332002);
   cutt->SetPoint(20, 271.9641, 2.582963);
   cutt->SetPoint(21, 227.8419, 2.596905);
   cutt->SetPoint(22, 162.9563, 2.564373);
   cutt->SetPoint(23, 165.5517, 2.378476);
   cutt->SetPoint(24, 165.5517, 2.378476);*/

   /*TCutG *cutd = new TCutG("CUTD", 20);
   cutd->SetVarX("ELossvsBrho");
   cutd->SetVarY("");
   cutd->SetTitle("Graph");
   cutd->SetFillStyle(1000);
   cutd->SetPoint(0, 132.3103, 1.817738);
   cutd->SetPoint(1, 191.8601, 1.676979);
   cutd->SetPoint(2, 318.7288, 1.404543);
   cutd->SetPoint(3, 488.0121, 1.113945);
   cutd->SetPoint(4, 688.6722, 0.9005369);
   cutd->SetPoint(5, 1197.102, 0.7325347);
   cutd->SetPoint(6, 1883.43, 0.5736138);
   cutd->SetPoint(7, 3924.598, 0.4510176);
   cutd->SetPoint(8, 12408.35, 0.3011779);
   cutd->SetPoint(9, 21374.5, 0.2239877);
   cutd->SetPoint(10, 22365.46, 0.1059321);
   cutd->SetPoint(11, 20990.52, 0.05598556);
   cutd->SetPoint(12, 9892.469, 0.1195539);
   cutd->SetPoint(13, 4415.379, 0.2058253);
   cutd->SetPoint(14, 1783.738, 0.3147997);
   cutd->SetPoint(15, 774.7924, 0.5100454);
   cutd->SetPoint(16, 232.0857, 0.955024);
   cutd->SetPoint(17, 114.4489, 1.218379);
   cutd->SetPoint(18, 88.796, 1.663357);
   cutd->SetPoint(19, 129.9334, 1.817738);
   cutd->SetPoint(20, 132.3103, 1.817738);*/
   /*cutd->SetPoint(0, 48.7938, 1.961132);
   cutd->SetPoint(1, 923.042, 0.9194652);
   cutd->SetPoint(2, 1360.166, 0.7639925);
   cutd->SetPoint(3, 3737.028, 0.4841418);
   cutd->SetPoint(4, 8053.629, 0.3364428);
   cutd->SetPoint(5, 16905.39, 0.289801);
   cutd->SetPoint(6, 22888.53, 0.2664801);
   cutd->SetPoint(7, 24309.18, 0.2198383);
   cutd->SetPoint(8, 24746.3, 0.1576492);
   cutd->SetPoint(9, 24555.06, 0.04881838);
   cutd->SetPoint(10, 10731.01, 0.04104475);
   cutd->SetPoint(11, 8217.55, 0.04881838);
   cutd->SetPoint(12, 6523.694, 0.1343283);
   cutd->SetPoint(13, 3299.904, 0.2276119);
   cutd->SetPoint(14, 1360.166, 0.4375);
   cutd->SetPoint(15, 458.5976, 0.6707089);
   cutd->SetPoint(16, 158.0748, 1.028296);
   cutd->SetPoint(17, -33.16696, 1.510261);
   cutd->SetPoint(18, 48.7938, 1.922264);
   cutd->SetPoint(19, 48.7938, 1.961132);*/

   // dedx cuts
   /*auto cuttdedx = new TCutG("CUTTDEDEX", 19);
   cuttdedx->SetVarX("dedxvsBrho");
   cuttdedx->SetVarY("");
   cuttdedx->SetTitle("Graph");
   cuttdedx->SetFillStyle(1000);
   cuttdedx->SetPoint(0, 9474.773, 2.854373);
   cuttdedx->SetPoint(1, 20398.5, 1.965816);
   cuttdedx->SetPoint(2, 37174.23, 1.468925);
   cuttdedx->SetPoint(3, 67604.63, 1.042184);
   cuttdedx->SetPoint(4, 83600.09, 0.9077313);
   cuttdedx->SetPoint(5, 118321.9, 0.8375821);
   cuttdedx->SetPoint(6, 130416.1, 0.8025075);
   cuttdedx->SetPoint(7, 130025.9, 0.6212885);
   cuttdedx->SetPoint(8, 93743.55, 0.685592);
   cuttdedx->SetPoint(9, 54340.1, 0.8317363);
   cuttdedx->SetPoint(10, 21568.9, 1.270169);
   cuttdedx->SetPoint(11, 11425.44, 1.585841);
   cuttdedx->SetPoint(12, 2842.508, 2.100269);
   cuttdedx->SetPoint(13, 1281.975, 2.345791);
   cuttdedx->SetPoint(14, 2062.242, 2.813453);
   cuttdedx->SetPoint(15, 2062.242, 2.918677);
   cuttdedx->SetPoint(16, 9084.64, 2.971289);
   cuttdedx->SetPoint(17, 9474.773, 2.866065);
   cuttdedx->SetPoint(18, 9474.773, 2.854373);

   // Kinematics cut
   auto *cutk = new TCutG("cutk", 13);
   cutk->SetVarX("Ang_Ener");
   cutk->SetVarY("");
   cutk->SetTitle("Graph");
   cutk->SetFillStyle(1000);
   cutk->SetPoint(0, 48.55834, 3.726572);
   cutk->SetPoint(1, 42.74863, 3.512897);
   cutk->SetPoint(2, 39.03687, 2.421627);
   cutk->SetPoint(3, 27.49815, 1.521139);
   cutk->SetPoint(4, 16.84702, 1.025107);
   cutk->SetPoint(5, 15.39459, 0.4222375);
   cutk->SetPoint(6, 23.38294, 0.231456);
   cutk->SetPoint(7, 37.66514, 0.3153999);
   cutk->SetPoint(8, 48.80041, 1.055632);
   cutk->SetPoint(9, 54.12597, 2.185058);
   cutk->SetPoint(10, 53.56114, 3.428953);
   cutk->SetPoint(11, 48.71972, 3.734203);
   cutk->SetPoint(12, 48.55834, 3.726572);
   */

   std::ofstream file1("check_event1.txt");
   std::ofstream file2("check_event2.txt");
   std::ofstream file3("check_event3.txt");
   std::ofstream file4("check_event4.txt");
   std::ofstream filebkg("check_eventbkg.txt");
   std::ofstream filebkgcoin("check_eventbkgcoin.txt");
   std::ofstream filethreshold("check_eventthreshold.txt");

   for (auto iFile : files) {

      // GET Data
      TString filePath = dataDir + iFile;
      TFile *file = new TFile(filePath.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim", file);
      TTreeReaderValue<TClonesArray> trackingArray(ReaderTracking, "AtTrackingEvent");
      

      //TTreeReaderValue<TClonesArray> eventHArray(ReaderTracking, "AtEventH");

      // FRIB data
     /* TFile *fileFRIB = new TFile((dataDir + iFile.second).Data(), "READ");
      TTree *treeFRIB = (TTree *)fileFRIB->Get("FRIB_output_tree");
      Int_t nEventsFRIB = treeFRIB->GetEntries();
      std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;

      TTreeReader Reader2("FRIB_output_tree", fileFRIB);
      TTreeReaderValue<ULong64_t> ts(Reader2, "timestamp");
      TTreeReaderValue<std::vector<Float_t>> energyIC(Reader2, "energy");
      TTreeReaderValue<std::vector<Float_t>> timeIC(Reader2, "time");
      TTreeReaderValue<UInt_t> multIC(Reader2, "mult");
      TTreeReaderValue<std::string> fribEvName(Reader2, "eventName");

      if (nEvents != nEventsFRIB + 1) {
         std::cerr << " Error, incompatible number of events! Exiting... "
                   << "\n";
         // std::exit(0);
      }*/

      //ULong64_t fribTSRef = 0;
      //ULong64_t getTSRef = 0;

      //ULong64_t fribDTS = 0;
      //ULong64_t getDTS = 0;

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         //if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         ReaderTracking.Next();
         //Reader2.Next();

         AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);
         //AtEvent *event = (AtEvent *)eventHArray->At(0);


         if (trackingEvent) {
            
            auto &fittedTracks = trackingEvent->GetFittedTracks();
            // std::cout<<" Number of fitted tracks "<<fittedTracks.size()<<"\n";
            //auto eventName = event->GetEventName();
            //auto getTS = event->GetTimestamp(1);

           /* if (i == 0) {
               fribDTS = 0;
               getDTS = 0;
            } else {

               fribDTS = *ts - fribTSRef;
               getDTS = getTS - getTSRef;
            }

            if (fribDTS > (getDTS + 5) || fribDTS < (getDTS - 5)) {
               std::cerr << i << "  " << fribDTS << "  " << getDTS << "\n";
               std::exit(0);
            }

            getTSRef = getTS;
            fribTSRef = *ts;
            */
            // 900 - 1300
            /*Bool_t goodBeam = false;
            for (auto ener : *energyIC) {
               if (ener > 900 && ener < 1300) {
                  goodBeam = true;
                  henergyIC->Fill(ener);
               }
            }
            if (!goodBeam)
               continue;
            */
            // if(*multIC!=1)
            // continue;
            // Find track with largets angle
            auto itMax = std::max_element(fittedTracks.begin(), fittedTracks.end(), [](const auto &a, const auto &b) {
               auto [energyb, energyXtrb, thetab, phib, energyPRAb, thetaPRAb, phiPRAb] = b.get()->GetEnergyAngles();
               auto [energya, energyXtra, thetaa, phia, energyPRAa, thetaPRAa, phiPRAa] = a.get()->GetEnergyAngles();
               return (thetab > thetaa) && (thetab < 90 && thetaa < 90);
            });

            /*auto itMax = std::max_element(fittedTracks.begin(), fittedTracks.end(), [](const auto &a, const auto &b) {
               auto [chargeb, brhob, eLossADCb, dEdxADCb, pdgb, trackPointsb] = b.get()->GetTrackProperties();
               auto [chargea, brhoa, eLossADCa, dEdxADCa, pdga, trackPointsa] = a.get()->GetTrackProperties();
               return (trackPointsb > trackPointsa);
            });*/


            Int_t maxAIndex = std::distance(fittedTracks.begin(), itMax);

            //if(fittedTracks.size() > 1 && i%2 != 0){
            if(fittedTracks.size() > 1){
               
               //std::cout << "Event number: " << i << std::endl;
               auto &track1 = fittedTracks.at(0);
               auto &track2 = fittedTracks.at(1);
               auto [energy1, energyXtr1, theta1, phi1, energyPRA1, thetaPRA1, phiPRA1] = track1->GetEnergyAngles();
               auto [energy2, energyXtr2, theta2, phi2, energyPRA2, thetaPRA2, phiPRA2] = track2->GetEnergyAngles();
               auto [iniPos1, iniPosPRA1, iniPosXtr1] = track1->GetVertices();
               auto [iniPos2, iniPosPRA2, iniPosXtr2] = track2->GetVertices();
               auto [pValue1, chi21, bChi21, ndf1, bNdf1, fitConverged1] = track1->GetStats();
               auto [pValue2, chi22, bChi22, ndf2, bNdf2, fitConverged2] = track2->GetStats();
               auto [charge1, brho1, eLossADC1, dEdxADC1, pdg1, trackPoints1] = track1->GetTrackProperties();
               auto [charge2, brho2, eLossADC2, dEdxADC2, pdg2, trackPoints2] = track2->GetTrackProperties();
               auto [exEnergy1, exEnergyXtr1] = track1->GetExcitationEnergy();
               auto [exEnergy2, exEnergyXtr2] = track2->GetExcitationEnergy();
               auto [distanceXtr1, trackLength1, POCAXtr1] = track1->GetDistances();
               auto [distanceXtr2, trackLength2, POCAXtr2] = track2->GetDistances();
               //std::cout << " Track 1 : " << energy1 << "  " << theta1 << "  " << phi1 << "  " << energyPRA1 << "  " << thetaPRA1 << "  " << phiPRA1 << "\n";
               if(theta1 > 90.0) 
                  theta1 = 180.0 - theta1;
               if(theta2 > 90.0) 
                  theta2 = 180.0 - theta2;    

               //if(std::abs(iniPos1.z() - iniPos2.z()) <= 20.0){
               if(theta1 >= theta2){
                  Double_t vx = TMath::Sin(theta1 * TMath::DegToRad()) * TMath::Sqrt(energy1 * Am);
                  Double_t vy = TMath::Cos(theta1 * TMath::DegToRad()) * TMath::Sqrt(energy1 * Am);
                  hVxVy->Fill(vx, vy);

                  if(trackLength1 < -20.0)
                  hVxVy_cut->Fill(vx, vy);
                  angle_vs_angle->Fill(theta1, theta2);
                  ELoss_vs_bro_alpha->Fill(eLossADC1, brho1);              
                  ELoss_vs_bro_be->Fill(eLossADC2, brho2);
                  ELossvsBrho->Fill(eLossADC1, brho1);
                  ELossvsBrho->Fill(eLossADC2, brho2);
                  double_t ener_bro = 0.0;
                  GetEnergy(4,2,brho1,ener_bro);
                  //ener_bro = TMath::Sqrt((brho1/0.1439 * 2/4)*(brho1/0.1439 * 2/4)*2*931.5 + 931.5*931.5) - 931.5;
                  //Ang_Ener->Fill(theta1, ener_bro*4);
                  if(trackLength1 < -20.0)
                  Ang_Ener_cut->Fill(theta1, energy1);
                  /*if(trackLength1 < 0.0 && trackLength1 > -3.0)
                  Ang_Ener_03->Fill(theta1, energy1);
                  if(trackLength1 < -3.0 && trackLength1 > -6.0)
                  Ang_Ener_36->Fill(theta1, energy1);
                  if(trackLength1 < -6.0 && trackLength1 > -9.0)
                  Ang_Ener_69->Fill(theta1, energy1);
                  if(trackLength1 < -9.0 && trackLength1 > -12.0)
                  Ang_Ener_912->Fill(theta1, energy1);
                  if(trackLength1 < -12.0 && trackLength1 > -15.0)
                  Ang_Ener_1215->Fill(theta1, energy1);
                  if(trackLength1 < -15.0 && trackLength1 > -18.0)
                  Ang_Ener_1518->Fill(theta1, energy1);
                  if(trackLength1 < -18.0 && trackLength1 > -21.0)
                  Ang_Ener_1821->Fill(theta1, energy1);
                  if(trackLength1 < -21.0 && trackLength1 > -24.0)
                  Ang_Ener_2124->Fill(theta1, energy1);
                  if(trackLength1 < -24.0 && trackLength1 > -27.0)
                  Ang_Ener_2427->Fill(theta1, energy1);
                  if(trackLength1 < -27.0 && trackLength1 > -30.0)
                  Ang_Ener_2730->Fill(theta1, energy1);*/

                  Ang_Ener->Fill(theta1, energy1*Am);
                  Ang_Ener_nocoin->Fill(theta1, energy1*Am);
                  //if(trackLength1 > -20.0)
                  //energy_vs_length->Fill(trackLength1, energy1);
                  if (i == 15){
                  std::cout << "Energy: " << energy1 << "   " << "Angle: " << theta1 << std::endl;
                  std::cout << "Energy: " << energyPRA1 << "   " << "Angle: " << theta1 << std::endl;
                  std::cout << "Energy: " << energyXtr1 << "   " << "Angle: " << theta1 << std::endl;
                  }
                  double_t covered_range = 1000 - iniPos1.z()*10;
                  ealpha_vs_vertex->Fill(energy_vs_distance_gr->Eval(covered_range), energy1);
                  
               }
                     
               if(theta1 < theta2){
                  Double_t vx = TMath::Sin(theta2 * TMath::DegToRad()) * TMath::Sqrt(energy2 * Am);
                  Double_t vy = TMath::Cos(theta2 * TMath::DegToRad()) * TMath::Sqrt(energy2 * Am);
                  hVxVy->Fill(vx, vy);

                  if(trackLength2 < -20.0)
                  hVxVy_cut->Fill(vx, vy);
                  angle_vs_angle->Fill(theta2, theta1);  
                  ELoss_vs_bro_alpha->Fill(eLossADC2, brho2);
                  ELoss_vs_bro_be->Fill(eLossADC1, brho1);
                  ELossvsBrho->Fill(eLossADC1, brho1);
                  ELossvsBrho->Fill(eLossADC2, brho2);
                  double_t ener_bro_2 = 0.0;
                  //GetEnergy(4,2,brho2,ener_bro_2);
                  //ener_bro_2 = TMath::Sqrt((brho2/0.1439 * 2/4)*(brho1/0.1439 * 2/4)*2*931.5 + 931.5*931.5) - 931.5;
                  //Ang_Ener->Fill(theta2, ener_bro_2*4);
                  if(trackLength2 < -20.0)
                  Ang_Ener_cut->Fill(theta2, energy2);
                  /*if(trackLength2 < 0.0 && trackLength2 > -3.0)
                  Ang_Ener_03->Fill(theta2, energy2);
                  if(trackLength2 < -3.0 && trackLength2 > -6.0)
                  Ang_Ener_36->Fill(theta2, energy2);
                  if(trackLength2 < -6.0 && trackLength2 > -9.0)
                  Ang_Ener_69->Fill(theta2, energy2);
                  if(trackLength2 < -9.0 && trackLength2 > -12.0)
                  Ang_Ener_912->Fill(theta2, energy2);
                  if(trackLength2 < -12.0 && trackLength2 > -15.0)
                  Ang_Ener_1215->Fill(theta2, energy2);
                  if(trackLength2 < -15.0 && trackLength2 > -18.0)
                  Ang_Ener_1518->Fill(theta2, energy2);
                  if(trackLength2 < -18.0 && trackLength2 > -21.0)
                  Ang_Ener_1821->Fill(theta2, energy2);
                  if(trackLength2 < -21.0 && trackLength2 > -24.0)
                  Ang_Ener_2124->Fill(theta2, energy2);
                  if(trackLength2 < -24.0 && trackLength2 > -27.0)
                  Ang_Ener_2427->Fill(theta2, energy2);
                  if(trackLength2 < -27.0 && trackLength2 > -30.0)
                  Ang_Ener_2730->Fill(theta2, energy2);*/

                  Ang_Ener->Fill(theta2, energy2);
                  Ang_Ener_nocoin->Fill(theta2, energy2);
                  //if(trackLength2 > -20.0)
                  //energy_vs_length->Fill(trackLength2, energy2);
                  if(i == 15){
                  std::cout << "Energy: " << energy2 << "   " << "Angle: " << theta2 << std::endl;
                  std::cout << "Energy: " << energyPRA2 << "   " << "Angle: " << theta2 << std::endl;
                  std::cout << "Energy: " << energyXtr2 << "   " << "Angle: " << theta2 << std::endl;
                  }
                  double_t covered_range = 1000 - iniPos2.z()*10;
                  ealpha_vs_vertex->Fill(energy_vs_distance_gr->Eval(covered_range), energy2);
                  
               }      


               if(theta1 > theta2){
                  double a = (m_a*(energy1 + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - theta1*TMath::DegToRad())));
                  a = (m_a*(2.5 + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - theta1*TMath::DegToRad())));
                                     
                  double Ebeam = ((TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a))*(TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a)) - (m_a + m_Be10)*(m_a + m_Be10))/(2*m_a);
                  
                  std::cout << "ECM min = " << Ebeam * 2.0/7.0 << std::endl;
                  if(trackLength1 < -20.0){
                  ecm_anglecm_cut->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(cutbkgcoin->IsInside(2*(TMath::Pi()/2 - theta1*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     filebkgcoin << iFile << "\t" << i << "\t" << trackLength1 << "\t" << energy1 << std::endl;
                  }
                  /*if(trackLength1 < 0.0 && trackLength1 > -3.0)
                  ecm_anglecm_03->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -3.0 && trackLength1 > -6.0)
                  ecm_anglecm_36->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -6.0 && trackLength1 > -9.0)
                  ecm_anglecm_69->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -9.0 && trackLength1 > -12.0) 
                  ecm_anglecm_912->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -12.0 && trackLength1 > -15.0)
                  ecm_anglecm_1215->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -15.0 && trackLength1 > -18.0)
                  ecm_anglecm_1518->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -18.0 && trackLength1 > -21.0)
                  ecm_anglecm_1821->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -21.0 && trackLength1 > -24.0)
                  ecm_anglecm_2124->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -24.0 && trackLength1 > -27.0)
                  ecm_anglecm_2427->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -27.0 && trackLength1 > -30.0)
                  ecm_anglecm_2730->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);*/

                  ecm_anglecm->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  ecm_anglecm_nocoin->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  outFiletxt << std::fixed << std::setprecision(4) << Ebeam * 2.0/7.0 << "\t" << (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg() << std::endl;
                  if(trackLength1 < -20.0){
                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>115. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<125.)
                           ex_ener_120->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>105. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<115.)
                           ex_ener_110->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>95. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<105.)
                           ex_ener_100->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>85. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<95.)
                           ex_ener_90->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>75. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<85.)
                           ex_ener_80->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>65. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<75.)
                           ex_ener_70->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()>55. && (2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg()<65.)
                           ex_ener_60->Fill(Ebeam * 2.0/7.0);   
                  
                  if(Ebeam * 2.0/7.0 > 2.4 && Ebeam * 2.0/7.0 < 3.2)
                     ang_distribution->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg());
                  }
               }

               if(theta2 > theta1){
                  double a = (m_a*(energy2 + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - theta2*TMath::DegToRad())));
                                     
                  double Ebeam = ((TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a))*(TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a)) - (m_a + m_Be10)*(m_a + m_Be10))/(2*m_a);
                  if(trackLength2 < -20.0){
                  ecm_anglecm_cut->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0); 
                  if(cutbkgcoin->IsInside(2*(TMath::Pi()/2 - theta2*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     filebkgcoin << iFile << "\t" << i << "\t" << trackLength2 << "\t" << energy2 << std::endl;
                  }
                 /* if(trackLength2 < 0.0 && trackLength2 > -3.0)
                  ecm_anglecm_03->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -3.0 && trackLength2 > -6.0)
                  ecm_anglecm_36->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -6.0 && trackLength2 > -9.0)
                  ecm_anglecm_69->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -9.0 && trackLength2 > -12.0) 
                  ecm_anglecm_912->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -12.0 && trackLength2 > -15.0)
                  ecm_anglecm_1215->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -15.0 && trackLength2 > -18.0)
                  ecm_anglecm_1518->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -18.0 && trackLength2 > -21.0)
                  ecm_anglecm_1821->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -21.0 && trackLength2 > -24.0)
                  ecm_anglecm_2124->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -24.0 && trackLength2 > -27.0)
                  ecm_anglecm_2427->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength2 < -27.0 && trackLength2 > -30.0)
                  ecm_anglecm_2730->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);*/
                  
                  ecm_anglecm->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  ecm_anglecm_nocoin->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  outFiletxt << std::fixed << std::setprecision(4) << Ebeam * 2.0/7.0 << "\t" << (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg() << std::endl;
                  if(trackLength2 < -20.0){
                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>115. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<125.)
                           ex_ener_120->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>105. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<115.)
                           ex_ener_110->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>95. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<105.)
                           ex_ener_100->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>85. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<95.)
                           ex_ener_90->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>75. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<85.)
                           ex_ener_80->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>65. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<75.)
                           ex_ener_70->Fill(Ebeam * 2.0/7.0);

                  if((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()>55. && (2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg()<65.)
                           ex_ener_60->Fill(Ebeam * 2.0/7.0); 

                  if(Ebeam * 2.0/7.0 > 2.4 && Ebeam * 2.0/7.0 < 3.2)
                     ang_distribution->Fill((2*(TMath::Pi()/2 - theta2*TMath::DegToRad())) * TMath::RadToDeg());   
                  }      
               }
               //}
            }









            //for (auto index = 0; index < fittedTracks.size(); ++index) {

              // if (index != maxAIndex)
                //  continue;
               //if(fittedTracks.size() == 1 && i % 2 != 0){
               if(fittedTracks.size() == 1){
               auto [energy, energyXtr, theta, phi, energyPRA, thetaPRA, phiPRA] =
                  fittedTracks.at(0)->GetEnergyAngles();
               auto [iniPos, iniPosPRA, iniPosXtr] = fittedTracks.at(0)->GetVertices();
               auto [pValue, chi2, bChi2, ndf, bNdf, fitConverged] = fittedTracks.at(0)->GetStats();
               auto [charge, brho, eLossADC, dEdxADC, pdg, trackPoints] = fittedTracks.at(0)->GetTrackProperties();
               // auto [ICEnergy,ICTime] = fittedTracks.at(0)->GetIonChamber(); //TODO
               auto [exEnergy, exEnergyXtr] = fittedTracks.at(0)->GetExcitationEnergy();
               auto [distanceXtr, trackLength, POCAXtr] = fittedTracks.at(0)->GetDistances();
               energy_vs_length->Fill(trackLength, energy);
               energyPRA_vs_length->Fill(trackLength, energyPRA);
               if(theta > 90.0) 
                  theta = 180.0 - theta;
               // Conditions
               //if (!cutd->IsInside(eLossADC, brho))
                //continue;

               // if (!cutt->IsInside(eLossADC, brho))
               // continue;

               //if (cuttdedx->IsInside(dEdxADC, brho))
                 // continue;

               //if(!cutk->IsInside(theta, energy * Am))
                  //continue;

               //if (theta > 90.0 || theta < 10.0)
                 // continue;

                //if(eLossADC<600)
                  //  continue;

                // if(dEdxADC<20000 || dEdxADC>110000)
                  //  continue; 

               // Histograms
               //std::cout << "Energy: " << energy << "   " << "Angle: " << theta << std::endl;
               Ang_Ener_nocoin->Fill(theta, energy * Am);
               
               if(trackLength < -20.0){
               Ang_Ener_nocoin_cut->Fill(theta,energy);
               
               
               if(cut_threshold->IsInside(theta, energy * Am))
                  filethreshold << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;
               }
               //Ang_Ener_PRAC->Fill(thetaPRA, energyPRA * Am);
               //ELossvsBrho->Fill(eLossADC, brho);
               //dedxvsBrho->Fill(dEdxADC, brho);
               //std::cout << iniPos.z() << "  " << iniPosPRA.z() << "  " << iniPosXtr.z() << "\n";
               
               vertex_distribution_gf->Fill(iniPos.z());
               vertex_distribution_pra->Fill(iniPosPRA.z()*0.1);
               vertex_distribution_xtr->Fill(iniPosXtr.z()*0.1);
               
               double a = (m_a*(energy + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - theta*TMath::DegToRad())));
                                     
                  double Ebeam = ((TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a))*(TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a)) - (m_a + m_Be10)*(m_a + m_Be10))/(2*m_a);
                  //if(trackPoints > 50){
                  ecm_anglecm_nocoin_cut->Fill((2*(TMath::Pi()/2 - theta*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(cut1->IsInside(2*(TMath::Pi()/2 - theta*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     file1 << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;

                  if(cut2->IsInside(2*(TMath::Pi()/2 - theta*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     file2 << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;

                  if(cut3->IsInside(2*(TMath::Pi()/2 - theta*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     file3 << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;

                  if(cut4->IsInside(2*(TMath::Pi()/2 - theta*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))
                     file4 << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;

                  if(cutbkg->IsInside(2*(TMath::Pi()/2 - theta*TMath::DegToRad()) * TMath::RadToDeg(), Ebeam * 2.0/7.0))   
                     filebkg << iFile << "\t" << i << "\t" << trackLength << "\t" << energy << std::endl;
                  
                  //}
                  /*if(trackLength1 < 0.0 && trackLength1 > -3.0)
                  ecm_anglecm_03->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -3.0 && trackLength1 > -6.0)
                  ecm_anglecm_36->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -6.0 && trackLength1 > -9.0)
                  ecm_anglecm_69->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -9.0 && trackLength1 > -12.0) 
                  ecm_anglecm_912->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -12.0 && trackLength1 > -15.0)
                  ecm_anglecm_1215->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -15.0 && trackLength1 > -18.0)
                  ecm_anglecm_1518->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -18.0 && trackLength1 > -21.0)
                  ecm_anglecm_1821->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -21.0 && trackLength1 > -24.0)
                  ecm_anglecm_2124->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -24.0 && trackLength1 > -27.0)
                  ecm_anglecm_2427->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                  if(trackLength1 < -27.0 && trackLength1 > -30.0)
                  ecm_anglecm_2730->Fill((2*(TMath::Pi()/2 - theta1*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);*/

                  ecm_anglecm_nocoin->Fill((2*(TMath::Pi()/2 - theta*TMath::DegToRad())) * TMath::RadToDeg(), Ebeam * 2.0/7.0);

                  if (i < alphaEnergies_sim.size()) {
                  data_vs_sim_alphaener->Fill(alphaEnergies_sim[i], energy);
                  }


                  energy = 0.0;
      
               //Double_t vx = TMath::Sin(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);
               //Double_t vy = TMath::Cos(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);

               //hVxVy->Fill(vx, vy);
                  
               }
           // }
         }

      } // events

   } // Files

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "10Be_a_gs_lowener.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         // numKin++;

         // MomLabRec[numKin] =( pow(EnerLabRec[numKin] + M_Ener,2) - TMath::Power(M_Ener, 2))/1000.0;
         // std::cout<<" Momentum : " <<MomLabRec[numKin]<<"\n";
         // Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *kine_gs = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   kine_gs->SetLineColor(kRed);

   /*TCanvas *c13 = new TCanvas();
   c13->Divide(2, 2);
   c13->Draw();
   c13->cd(1);
   Ang_Ener_03->Draw("col");
   c13->cd(2);
   Ang_Ener_36->Draw("col");
   c13->cd(3);
   Ang_Ener_69->Draw("col");
   c13->cd(4);
   Ang_Ener_912->Draw("col");

   TCanvas *c14 = new TCanvas();
   c14->Divide(2, 2);
   c14->Draw();
   c14->cd(1);
   Ang_Ener_1215->Draw("col");
   c14->cd(2);
   Ang_Ener_1518->Draw("col");
   c14->cd(3);
   Ang_Ener_1821->Draw("col");
   c14->cd(4);
   Ang_Ener_2124->Draw("col");

   TCanvas *c15 = new TCanvas();
   c15->Divide(2, 2);
   c15->Draw();
   c15->cd(1);
   Ang_Ener_2427->Draw("col");
   c15->cd(2);
   Ang_Ener_2730->Draw("col");

   TCanvas *c16 = new TCanvas();
   c16->Divide(2, 2);
   c16->Draw();
   c16->cd(1);
   ecm_anglecm_03->Draw("col");
   c16->cd(2);
   ecm_anglecm_36->Draw("col");
   c16->cd(3);
   ecm_anglecm_69->Draw("col");
   c16->cd(4);
   ecm_anglecm_912->Draw("col");

   TCanvas *c17 = new TCanvas();
   c17->Divide(2, 2);
   c17->Draw();
   c17->cd(1);
   ecm_anglecm_1215->Draw("col");
   c17->cd(2);
   ecm_anglecm_1518->Draw("col");
   c17->cd(3);
   ecm_anglecm_1821->Draw("col");
   c17->cd(4);
   ecm_anglecm_2124->Draw("col");

   TCanvas *c18 = new TCanvas();
   c18->Divide(2, 2);
   c18->Draw();
   c18->cd(1);
   ecm_anglecm_2427->Draw("col");
   c18->cd(2);
   ecm_anglecm_2730->Draw("col");
*/

   TCanvas *c22 = new TCanvas();
   data_vs_sim_alphaener->Draw("colz");
   
   TCanvas *c21 = new TCanvas();
   energyPRA_vs_length->Draw("colz");
   TCanvas *c19_cut = new TCanvas();
   Ang_Ener_nocoin_cut->SetMarkerStyle(20);
   Ang_Ener_nocoin_cut->SetMarkerSize(0.5);
   Ang_Ener_nocoin_cut->Draw("colz");
   Ang_Ener_nocoin_cut->GetXaxis()->SetTitle("Theta (deg)");
   Ang_Ener_nocoin_cut->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");
   TCanvas *c19 = new TCanvas();
   Ang_Ener_nocoin->SetMarkerStyle(20);
   Ang_Ener_nocoin->SetMarkerSize(0.5);
   Ang_Ener_nocoin->Draw("colz");
   Ang_Ener_nocoin->GetXaxis()->SetTitle("Theta (deg)");
   Ang_Ener_nocoin->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");



   TCanvas *c20 = new TCanvas();
   ecm_anglecm_nocoin->Draw("colz");

   TCanvas *c20_cut = new TCanvas();
   ecm_anglecm_nocoin_cut->Draw("colz");
   cutbkg->SetLineColor(kRed);
   cutbkg->Draw("same l");


   TCanvas *c12 = new TCanvas();
   energy_angle_cross_section->Draw("colz");

   /*TCanvas *c11 = new TCanvas();
   hVxVy->Draw("zcol");

   TCanvas *c11_cut = new TCanvas();
   hVxVy_cut->Draw("zcol");*/

   TCanvas *c10 = new TCanvas();
   energy_vs_length->Draw("colz");

   /*TCanvas *c9 = new TCanvas();
   ealpha_vs_vertex->Draw("colz");*/

   TCanvas *c7 = new TCanvas();
   c7->Divide(2, 2);
   c7->Draw();
   c7->cd(1);
   ex_ener_60->Draw();
   c7->cd(2);
   ex_ener_70->Draw();
   c7->cd(3);
   ex_ener_80->Draw();
   c7->cd(4);
   ex_ener_90->Draw();

   TCanvas *c8 = new TCanvas();
   c8->Divide(2, 2);
   c8->Draw();
   c8->cd(1);
   ex_ener_100->Draw();
   c8->cd(2);
   ex_ener_110->Draw();
   c8->cd(3);
   ex_ener_120->Draw();
   c8->cd(4);
   ang_distribution->Draw();

  /* TCanvas *c5 = new TCanvas();
   ELoss_vs_bro_alpha->Draw("colz");

   TCanvas *c6 = new TCanvas();
   ELoss_vs_bro_be->Draw("colz");*/
   
   TCanvas *c4 = new TCanvas();
   ecm_anglecm->Draw("colz");

   TCanvas *c4_cut = new TCanvas();
   ecm_anglecm_cut->Draw("colz");
   cutbkgcoin->SetLineColor(kRed);
   cutbkgcoin->Draw("same l");
   
   TCanvas *c3 = new TCanvas();
   angle_vs_angle->Draw("colz");

   TCanvas *c2 = new TCanvas();
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw("col");
   Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");

   TCanvas *c2_cut = new TCanvas();
   Ang_Ener_cut->SetMarkerStyle(20);
   Ang_Ener_cut->SetMarkerSize(0.5);
   Ang_Ener_cut->Draw("col");
   Ang_Ener_cut->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener_cut->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");

  /* TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw("col");
   Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");
   c1->cd(2);
   Ang_Ener_PRAC->Draw("col");
   kine_gs->Draw("same");
   c1->cd(3);
   hVxVy->Draw("zcol");*/

   TCanvas *cvertex = new TCanvas();
   cvertex->Divide(2, 2);
   cvertex->Draw();
   cvertex->cd(1);
   vertex_distribution_gf->Draw();
   cvertex->cd(2);
   vertex_distribution_pra->Draw();
   cvertex->cd(3);
   vertex_distribution_xtr->Draw();
   


   /*TCanvas *cpid = new TCanvas();
   cpid->Divide(2, 2);
   cpid->Draw("zcol");
   cpid->cd(1);
   ELossvsBrho->Draw("zcol");
   //cutt->Draw("l");
   //cutd->Draw("l");
   cpid->cd(2);
   dedxvsBrho->Draw("zcol");

   TCanvas *c_IC = new TCanvas();
   henergyIC->Draw();*/
   outFiletxt.close();
   file1.close();
   file2.close();
   file3.close();
   file4.close();
   filebkg.close();
   filebkgcoin.close();
   filethreshold.close();
}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E)
{

   // Energy per nucleon
   Float_t AM = 931.5;
   Float_t X = BRO / 0.1439 * IZ / M;
   X = pow(X, 2);
   X = 2. * AM * X;
   X = X + pow(AM, 2);
   E = TMath::Sqrt(X) - AM;
}
