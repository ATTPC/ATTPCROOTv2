#include "AtElectronicResponse.h"

#include <cmath>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <string>
#include <utility>
using namespace ElectronicResponse;

/**
 * @param[in] peakingTime Peaking time of the electronics in us.
 */
AtNominalResponse::AtNominalResponse(double peakingTime) : fPeakingTime(peakingTime) {}

double AtNominalResponse::GetResponse(double time) const
{
   double reducedTime = time / fPeakingTime;
   return pow(2.718, -3 * reducedTime) * sin(reducedTime) * pow(reducedTime, 3);
}

/**
 * @param[in] peakingTime Peaking time of the electronics in us.
 * @param[in] response Function pointer (or callable object) where the input is the reduced time
 * and output is the response function value at that reduced time.
 */
AtReducedTimeResponse::AtReducedTimeResponse(double peakingTime, ReducedResponse response)
   : fPeakingTime(peakingTime), fResponse(std::move(response))
{
}

double AtReducedTimeResponse::GetResponse(double time) const
{
   return fResponse(time / fPeakingTime);
}

/**
 * @param[in] tbTime Time between time samples in us.
 */
AtVectorResponse::AtVectorResponse(double tbTime, std::vector<double> trace) : fTBTime(tbTime), fTrace(std::move(trace))
{
}

double AtVectorResponse::GetResponse(double time) const
{
   // int tb = (double(time) / fTBTime + 0.5);
   int tb = time / fTBTime;
   if (tb >= fTrace.size())
      return 0;
   return fTrace.at(tb);
}

/**
 * The text file is assumed to have the response function at the following points:
 * (0.5 * tbTime, 1.5 * tbTime, 2.5 * tbTime, ...), with each point on a new line
 */
AtFileResponse::AtFileResponse(double tbTime, std::string filePath) : AtVectorResponse(tbTime, {})
{
   std::ifstream input(filePath);
   if (!input.is_open())
      std::cout << "wave input not open" << std::endl;
   else {
      std::cout << "wave input is open" << std::endl;
      while (!input.eof()) {
         double value = 0;
         input >> value;
         fTrace.push_back(value);
      }
   }
   input.close();

   for (int i = 0; i < fTrace.size(); i++) {
      std::cout << "fWaveSample[" << i << "]: " << fTrace[i] << std::endl;
   }
}
