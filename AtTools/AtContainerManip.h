#ifndef ATCONTAINERMANIP_H
#define ATCONTAINERMANIP_H

#include <algorithm> // IWYU pragma: keep
#include <vector>

namespace AtTools {

template <typename T>
T GetMedian(std::vector<T> &vec)
{
   if (vec.empty())
      return 0;

   auto n = vec.size() / 2;

   std::nth_element(vec.begin(), vec.begin() + n, vec.end());
   auto med = vec[n];
   if (vec.size() % 2 == 0) { // if even, get average of middle two entries
      med = (*std::max_element(vec.begin(), vec.begin() + n) + med) / 2.0;
   }
   return med;
}

} // namespace AtTools

#endif //#ifndef ATCONTAINERMANIP_H
