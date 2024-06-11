#ifndef ATSTRINGMANIP_H
#define ATSTRINGMANIP_H

#include <string>
#include <vector>

namespace AtTools {

/**
 * @brief split string, s, into vector of tokens split by delim.
 */
std::vector<std::string> SplitString(const std::string &s, char delim);

} // namespace AtTools
#endif // #ifndef ATSTRINGMANIP_H
