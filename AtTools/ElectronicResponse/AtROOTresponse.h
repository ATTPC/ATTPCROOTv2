#ifndef ATROOTRESPONSE_H
#define ATROOTRESPONSE_H

#include "AtElectronicResponse.h"
#include "AtRawEvent.h"

#include <stdexcept>
#include <string>

namespace ElectronicResponse {
/**
 * @brief Response function is represended by an AtEvent in a root file.
 * @ingroup elecResponse
 */
class AtRootResponse : public AtElectronicResponse {
protected:
   AtRawEvent fResponse;
   double fTBTime;

public:
   AtRootResponse(double tbTime, std::string filePath, std::string objectName);

   virtual double GetResponse(double time) const override
   {
      throw std::invalid_argument("A pad number must be specified to use this response function");
   }
   virtual double GetResponse(int padNum, double time) const override;
};
} // namespace ElectronicResponse

#endif //#ifndef ATROOTRESPONSE_H
