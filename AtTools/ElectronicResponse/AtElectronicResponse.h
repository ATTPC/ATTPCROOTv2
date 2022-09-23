#ifndef ATELECTRONICRESPONSE_H
#define ATELECTRONICRESPONSE_H

#include <functional>
#include <string>
#include <vector>

/**
 * @defgroup elecResponse Electronic Response
 *
 * Group of classes for describing the response of electronics to charge collected. Used both by
 * the simulation and decovolution PSA method.
 */
namespace ElectronicResponse {

/**
 * @brief Base class for describing the response of electronics to charge.
 *
 * Tracks the response function as a function of time (in us). This is a FunctionObject that takes
 * either the padnumber and time to get the response function, or just the time. Deriving classes that
 * only implement padnumber and time should throw an exception if the time only version is called.
 * Because these meet the interface of a callable object they can be passed directly to an AtPulseTask
 * as the response.
 *
 * There is no normalization intrinsic to this class because that is also true in the code as written.
 * IMO there should be and I think the best way is to normalize the functions so the maximum value is
 * always 1. That way in simulation or analysis you can swap response functions arbitrarily without
 * also chaning the gain or calibration.
 *
 * @ingroup elecResponse
 */
class AtElectronicResponse {
protected:
   virtual double GetResponse(double time) const = 0;
   virtual double GetResponse(int padNum, double time) const = 0;

public:
   virtual ~AtElectronicResponse() = default;
   /**
    * Returns the response of the electronics to a delta-function of charge at t=0 after time (us).
    */
   double operator()(double time) const { return GetResponse(time); }
   /**
    * Returns the response of the electronics to a delta-function of charge at t=0 after time (us).
    */
   double operator()(int padNum, double time) const { return GetResponse(padNum, time); }
};

/**
 * @brief Nominal response of GET electronics.
 * @ingroup elecResponse
 */
class AtNominalResponse : public AtElectronicResponse {
protected:
   double fPeakingTime; //! Electric peaking time in us

public:
   AtNominalResponse(double peakingTime);

protected:
   virtual double GetResponse(double time) const override;
   virtual double GetResponse(int padNum, double time) const override { return GetResponse(time); }
};

/**
 * @brief Response of GET electronics given by a function of the reduced time.
 * @ingroup elecResponse
 */
class AtReducedTimeResponse : public AtElectronicResponse {
protected:
   using ReducedResponse = std::function<double(double)>;
   double fPeakingTime; //! Electric peaking time in us
   ReducedResponse fResponse;

public:
   AtReducedTimeResponse(double peakingTime, ReducedResponse response);

protected:
   virtual double GetResponse(double time) const override;
   virtual double GetResponse(int padNum, double time) const override { return GetResponse(time); }
};

/**
 * @brief Response function speficied as a trace.
 * @ingroup elecResponse
 */
class AtVectorResponse : public AtElectronicResponse {
protected:
   double fTBTime; //! Width of a time bucket in us
   std::vector<double> fTrace;

public:
   /**
    * The vector is assumed to have the response function at the following points in time:
    * (0.5 * tbTime, 1.5 * tbTime, 2.5 * tbTime, ...)
    */
   AtVectorResponse(double tbTime, std::vector<double> trace);

protected:
   virtual double GetResponse(double time) const override;
   virtual double GetResponse(int padNum, double time) const override { return GetResponse(time); }
};

/**
 * @brief Response function speficied as a trace in a txt file.
 *
 * @ingroup elecResponse
 */
class AtFileResponse : public AtVectorResponse {

public:
   AtFileResponse(double tbTime, std::string filePath);
};

} // namespace ElectronicResponse

#endif //#ifndef ATELECTRONICRESPONSE_H
