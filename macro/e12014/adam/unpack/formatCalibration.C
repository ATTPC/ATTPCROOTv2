#include <fstream>
#include <iostream>

using namespace std;
void formatCalibration()
{
   ifstream calFile("output/calibration.csv");
   if (!calFile.is_open()) {
      cout << "Can't open input calibration file" << endl;
   }

   ofstream outFile("output/calibrationFormated.txt");
   if (!outFile.is_open()) {
      cout << "Can't open output calibration file" << endl;
   }

   while (!calFile.eof()) {
      int padNum;
      float a, b, c, d;
      char t1, t2;
      calFile >> padNum >> t1 >> a >> t2 >> b;
      outFile << padNum << "\t" << 0 << "\t" << b << endl;
   }
}
