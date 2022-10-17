#include <unistd.h>
using namespace std;

void sumCS()
{
   Double_t ExEje = 0;
   size_t N_cross = 630;
   Double_t sum = 0;

   for (size_t iEx = 0; iEx <= 20; iEx++) {
      sum = 0;
      if (iEx % 2 != 0)
         continue;

      string filename = Form("dl0_dj1_dOmega/wrapAll_dl0dj1_%dMeV.dat", (Int_t)iEx);
      // string filename= Form("dl2_dj1_dOmega/wrapAll_dl2dj1_%dMeV.dat",(Int_t)iEx);
      // string filename= Form("dl1_dj2_dOmega/wrapAll_dl1dj2_%dMeV.dat",(Int_t)iEx);
      ifstream inputfile;
      inputfile.open(filename.c_str());
      if (inputfile.fail()) {
         cerr << "file not found " << filename << endl;
         exit(1);
      }
      for (Int_t i = 0; i < N_cross; i++) {
         Double_t c1 = 0, c2 = 0, c3 = 0;
         inputfile >> c1 >> c2 >> c3;
         // sum += c3;
         if (c1 <= 1 && c2 == 0)
            sum += c3;
         // if(c1<=1 && c2==5)sum += c3;
      }
      inputfile.close();
      cout << iEx << " " << sum << endl;
   }

   return 0;
}
