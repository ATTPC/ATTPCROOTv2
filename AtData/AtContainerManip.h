#ifndef ATCONTAINERMANIP_H
#define ATCONTAINERMANIP_H

#include <FairLogger.h>

#include <TH1.h> // for TH1, TH1D (ptr only)

#include <algorithm> // IWYU pragma: keep
#include <iterator>  // for make_move_iterator
#include <memory>
#include <string>      // for string
#include <type_traits> // for remove_const_t
#include <vector>

namespace ContainerManip {

/**
 * @brief Use contents of data to set the bin contents of hist.
 *
 * Will resize the historgram to contain data.size() bins, setting the axis
 * according to xMin and xMax. If both are 0, then sets the axis to [0,data.size()]
 */
template <typename T>
void SetHistFromData(TH1 &hist, const T &data, double xMin = 0, double xMax = 0)
{
   if (xMin == 0 && xMax == 0)
      xMax = data.size();
   hist.Reset();
   hist.SetBins(data.size(), xMin, xMax);
   for (int i = 0; i < hist.GetNbinsX(); ++i)
      hist.SetBinContent(i + 1, data.at(i));
}

/**
 * @brief Use contents of data to create a histogram with the given name
 *
 * Will resize the historgram to contain data.size() bins, setting the axis
 * according to xMin and xMax. If both are 0, then sets the axis to [0,data.size()].
 * The returned unique_ptr owns the memory and it will not be cleaned up by root.
 */
template <typename T>
std::unique_ptr<TH1D> CreateHistFromData(const std::string &name, const T &data, double xMin = 0, double xMax = 0)
{
   if (xMin == 0 && xMax == 0)
      xMax = data.size();

   auto ret = std::make_unique<TH1D>(name.data(), name.data(), data.size(), 0, data.size());
   for (int i = 0; i < ret->GetNbinsX(); ++i)
      ret->SetBinContent(i + 1, data.at(i));
   ret->SetDirectory(nullptr); // Pass ownership to the pointer instead of current ROOT directory
   return ret;
}

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

template <typename T>
T *GetPointer(T &t)
{
   return &t;
}

template <typename T>
std::remove_const_t<T> *GetPointerNonConst(T &t)
{
   return const_cast<T *>(&t);
}

template <typename T>
std::vector<const T *> GetConstPointerVector(const std::vector<T> &vec)
{
   LOG(info) << "Transforming object -> pointer.";
   std::vector<const T *> ret;
   ret.resize(vec.size());
   std::transform(vec.begin(), vec.end(), ret.begin(), [](const T &a) { return GetPointer(a); });
   return ret;
}

template <typename T>
std::vector<const T *> GetConstPointerVector(const std::vector<std::unique_ptr<T>> &vec)
{
   LOG(info) << "Transforming unique pointer -> pointer.";
   std::vector<const T *> ret;
   ret.resize(vec.size());
   std::transform(vec.begin(), vec.end(), ret.begin(), [](const std::unique_ptr<T> &a) { return a.get(); });
   return ret;
}

template <typename T>
std::vector<T *> GetPointerVector(const std::vector<T> &vec)
{
   LOG(info) << "Transforming object -> pointer.";
   std::vector<T *> ret;
   ret.resize(vec.size());
   std::transform(vec.begin(), vec.end(), ret.begin(), [](const T &a) { return GetPointer(a); });
   return ret;
}

template <typename T>
std::vector<T *> GetPointerVector(const std::vector<std::unique_ptr<T>> &vec)
{
   LOG(info) << "Transforming unique pointer -> pointer.";
   std::vector<T *> ret;
   ret.resize(vec.size());
   std::transform(vec.begin(), vec.end(), ret.begin(), [](const std::unique_ptr<T> &a) { return a.get(); });
   return ret;
}

template <typename T>
std::vector<T> GetObjectVector(const std::vector<std::unique_ptr<T>> &vec)
{
   LOG(info) << "Transforming unique pointer -> object.";
   std::vector<T> ret;
   ret.resize(vec.size());
   std::transform(vec.begin(), vec.end(), ret.begin(), [](const std::unique_ptr<T> &a) { return *a; });
   return ret;
}

/**
 * Will move elements from the input vector to the output vector if op(elem) returns true
 * Suggested to make op() take a const referance to T so Operator is type bool(const T&)
 *
 * @param[in] op Operator must have the form bool(T)
 */
template <typename T, typename Operator>
std::vector<T> MoveFromVector(std::vector<T> &vec, Operator op)
{
   std::vector<T> retVec;
   auto itStartEqualRange = vec.end();

   for (auto it = vec.begin(); it != vec.end(); ++it) {

      bool isInPattern = op(*it);

      // Start of sub-vector with vec in pattern
      if (isInPattern && itStartEqualRange == vec.end()) {
         itStartEqualRange = it;
         continue;
      }

      // End of sub-vector with vec in pattern.
      // Move vec in this range to retVec then delete the empty entries
      if (itStartEqualRange != vec.end() && !isInPattern) {
         retVec.insert(retVec.end(), std::make_move_iterator(itStartEqualRange), std::make_move_iterator(it));
         vec.erase(itStartEqualRange, it);
         it = itStartEqualRange;
         itStartEqualRange = vec.end();
         continue;
      }
   }

   // If the last chunk of the array was in the pattern, move it and delete empty entries
   if (itStartEqualRange != vec.end()) {
      auto it = vec.end();
      retVec.insert(retVec.end(), std::make_move_iterator(itStartEqualRange), std::make_move_iterator(it));
      vec.erase(itStartEqualRange, it);
   }
   return retVec;
}

} // namespace ContainerManip

#endif //#ifndef ATCONTAINERMANIP_H
