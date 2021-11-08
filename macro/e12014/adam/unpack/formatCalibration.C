#include <fstream>
#include <iostream>

using namespace std;
void formatCalibration()
{
   ifstream calFile("calibration.txt");
   if (!calFile.is_open()) {
      cout << "Can't open input calibration file" << endl;
   }

   ofstream outFile("calibrationFormated.rxt");
   if (!outFile.is_open()) {
      cout << "Can't open output calibration file" << endl;
   }

   while (!calFile.eof()) {
      int padNum;
      float a, b, c, d;
      calFile >> padNum >> a >> b >> c >> d;
      if (b != 0)
         outFile << padNum << "\t" << -a / b << "\t" << 1.0 / b << endl;
   }
}
