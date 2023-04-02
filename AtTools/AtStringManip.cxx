#include "AtStringManip.h"

#include <sstream>

namespace AtTools {
std::vector<std::string> SplitString(const std::string &s, char delim)
{
   using namespace std;
   vector<string> res;
   stringstream ss(s);
   string token;
   while (getline(ss, token, delim)) {
      if (token != "")
         res.push_back(token);
   }
   return res;
}
} // namespace AtTools
