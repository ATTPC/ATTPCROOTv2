#ifndef ATCSVREADER_H
#define ATCSVREADER_H
// A simple CSV reader w/ modern cpp features
// Assumes no commas or newlines in the file except as deliminators
// ie you can't escape them
// Will work for strings or basic numbers (basically anything that stringstream is overloaded for
// Skips over entries that don't match the type is CSVRow<int> will parse "0,tr,-2" into {0,-2}
// Supports ranged loops.
//
// Based on: https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
//
// Adam Anthony 5/24/21

#include <string>
#include <iterator>
#include <vector>
#include <iostream>
#include <sstream>

// Represents a row of CSV file of type T
template <typename T>
class CSVRow {
public:
   T operator[](std::size_t index) const { return fData.at(index); }
   std::size_t size() const { return fData.size(); }
   const std::string &getLine() const { return fLine; }
   void readNextLine(std::istream &stream)
   {
      fData.clear(); // Clear old data

      std::getline(stream, fLine);
      std::istringstream str(fLine); // Get next line
      for (std::string elem; std::getline(str, elem, ',');) {
         std::istringstream ss(elem);
         T obj;
         ss >> obj;
         if (!ss.fail())
            fData.emplace_back(obj);
      }
   }

private:
   std::string fLine;
   std::vector<T> fData;
};

// Method to stream into this type of object
template <typename T>
std::istream &operator>>(std::istream &stream, CSVRow<T> &data)
{
   data.readNextLine(stream);
   return stream;
}

// Iterator
template <typename T>
class CSVIterator {
private:
   std::istream *fStream;
   CSVRow<T> fRow;

public:
   CSVIterator(std::istream &stream) : fStream(stream.good() ? &stream : nullptr) { ++(*this); }
   CSVIterator() : fStream(nullptr) {}

   // Pre increment
   CSVIterator &operator++()
   {
      if (fStream) {
         if (!((*fStream) >> fRow)) {
            fStream = nullptr;
         }
      }
      return *this;
   }
   // Post increment
   CSVIterator &operator++(int)
   {
      CSVIterator tmp(*this);
      ++(*this);
      return tmp;
   }
   CSVRow<T> const &operator*() const { return fRow; }
   CSVRow<T> const *operator->() const { return &fRow; }

   bool operator==(CSVIterator const &rhs)
   {
      return ((this == &rhs) || ((this->fStream == nullptr) && (rhs.fStream == nullptr)));
   }
   bool operator!=(CSVIterator const &rhs) { return !((*this) == rhs); }
};

// Add a range class
template <typename T>
class CSVRange {
private:
   std::istream &fStream;

public:
   CSVRange(std::istream &str) : fStream(str) {}
   CSVIterator<T> begin() const { return CSVIterator<T>{fStream}; }
   CSVIterator<T> end() const { return CSVIterator<T>{}; }
};

#endif //#ifndef ATCSVREADER_H
