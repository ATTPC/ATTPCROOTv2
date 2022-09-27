#ifndef ATPADBASE_H
#define ATPADBASE_H

#include <Rtypes.h>
#include <TObject.h>

#include <memory>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 @defgroup Pads Pads
 *
 * Classes for storing information on a channel (pad) basis. Pads follow the composition design pattern
 * with additional information added through "augments" which all extent AtPadBase.
 *
 * Each augment added to the container class AtPad is referenced through a string. The following is a
 * table of augments used in the code.
 * Augment Name | Class Type | Description
 * -------------|------------|-------------
 * "fft" | AtPadFFT | Representation of ADC in fourier space (256 complex numbers)
 * "Q" | AtPadArray | # of electrons collected on pad per TB (512 doubles)
 * "Qreco" | AtPadArray | Reconstructed charge collected on pad per TB (512 doubles)
 * "Qreco-fft" | AtPadFFT | Representation of reconstructed charge in fourier space (256 complex numbers)
*/

/**
 * @brief Base class for AtPad composition pattern.
 *
 * This should stay very lightweight. A seperate copy of the data in this class
 * will be stored for every pad augment added to an AtPad object.
 *
 * @ingroup Pads
 */
class AtPadBase : public TObject {
public:
   virtual ~AtPadBase() = default;
   virtual std::unique_ptr<AtPadBase> Clone() const = 0;
   ClassDefOverride(AtPadBase, 1);
};

#endif //#ifndef ATPADBASE_H
